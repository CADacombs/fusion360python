"""
This code is intended as a solution to the problem presented on the thread,
https://forums.autodesk.com/t5/fusion-api-and-scripts-forum/how-to-reliably-detect-concave-vs-convex-face-transitions-in/td-p/14107920

Developed with assistance from Autodesk Assistant (AI).

260519: First released version.
260521: Added support for co-surfaces. Improved debug output and graphics for face indices.
"""

import adsk
import adsk.core as ac
import adsk.fusion as af
import math
import traceback

_handlers = []
_shared   = {}
CUSTOM_EVENT_ID = 'faceFloodFillPostSelect'

_app = ac.Application.get()
_ui  = _app.userInterface


def _dbg(msg):
    if _shared.get("debug"):
        _app.log(msg)


def is_face_concave(face: af.BRepFace):
    """
    Returns True if concave, False if convex, None if planar/saddle.
    Uses analytic dot-product for analytic surface types,
    and corrected curvature sign for NurbsSurface.
    """
    ST   = ac.SurfaceTypes
    geom = face.geometry
    st   = geom.surfaceType

    # -- Plane ----------------------------------------------------------------
    if st == ST.PlaneSurfaceType:
        return None

    # -- NurbsSurface ---------------------------------------------------------
    # negative mean => CONVEX, positive mean => CONCAVE
    elif st == ST.NurbsSurfaceType:
        srfEvaluator = face.evaluator
        prange = srfEvaluator.parametricRange()
        param  = ac.Point2D.create(
            (prange.minPoint.x + prange.maxPoint.x) / 2.0,
            (prange.minPoint.y + prange.maxPoint.y) / 2.0)
        ok, _, maxCurv, minCurv = srfEvaluator.getCurvature(param)
        if not ok:
            raise RuntimeError("getCurvature failed.")
        if face.isParamReversed:
            maxCurv, minCurv = -maxCurv, -minCurv
        mean = (maxCurv + minCurv) / 2.0
        if abs(mean) < 1e-10:
            return None
        return mean > 0

    # -- All analytic types ---------------------------------------------------
    # Outward face normal dot radial/center vector:
    #   > 0 => normal points away from axis/center => CONVEX
    #   < 0 => normal points toward axis/center   => CONCAVE
    else:
        pt = face.pointOnFace
        ok, normal = face.evaluator.getNormalAtPoint(pt)
        if not ok:
            raise RuntimeError("getNormalAtPoint failed.")

        if st == ST.SphereSurfaceType:
            center = ac.Sphere.cast(geom).origin
            ref    = ac.Vector3D.create(
                pt.x - center.x, pt.y - center.y, pt.z - center.z)
        elif st == ST.TorusSurfaceType:
            ref = _torus_radial(ac.Torus.cast(geom), pt)
        elif st == ST.CylinderSurfaceType:
            cyl = ac.Cylinder.cast(geom)
            ref = _axis_radial(pt, cyl.origin, cyl.axis)
        elif st == ST.ConeSurfaceType:
            cone = ac.Cone.cast(geom)
            ref  = _axis_radial(pt, cone.origin, cone.axis)
        elif st == ST.EllipticalCylinderSurfaceType:
            ec  = ac.EllipticalCylinder.cast(geom)
            ref = _axis_radial(pt, ec.origin, ec.axis)
        elif st == ST.EllipticalConeSurfaceType:
            ec       = ac.EllipticalCone.cast(geom)
            axis, _  = ec.getAxes()
            ref      = _axis_radial(pt, ec.origin, axis)
        else:
            raise TypeError(f'Unrecognised surface type: {st}')

        return normal.dotProduct(ref) < 0


def _axis_radial(pt, axis_origin, axis_vector):
    """Radial vector from an axis to a point (axis component removed)."""
    to_pt = ac.Vector3D.create(
        pt.x - axis_origin.x,
        pt.y - axis_origin.y,
        pt.z - axis_origin.z)
    axial = to_pt.dotProduct(axis_vector)
    return ac.Vector3D.create(
        to_pt.x - axial * axis_vector.x,
        to_pt.y - axial * axis_vector.y,
        to_pt.z - axial * axis_vector.z)


def _torus_radial(tor, pt):
    """Vector from the nearest tube-center point on the ring to the surface point."""
    in_plane = _axis_radial(pt, tor.origin, tor.axis)
    length   = in_plane.length
    if length < 1e-10:
        raise RuntimeError('Point is on torus axis.')
    scale       = tor.majorRadius / length
    tube_center = ac.Point3D.create(
        tor.origin.x + in_plane.x * scale,
        tor.origin.y + in_plane.y * scale,
        tor.origin.z + in_plane.z * scale)
    return ac.Vector3D.create(
        pt.x - tube_center.x,
        pt.y - tube_center.y,
        pt.z - tube_center.z)


def is_curved_face(face):
    """True if face has non-zero curvature (fillet/round)."""
    ev   = face.evaluator
    bbox = ev.parametricRange()
    param = ac.Point2D.create(
        (bbox.minPoint.x + bbox.maxPoint.x) / 2.0,
        (bbox.minPoint.y + bbox.maxPoint.y) / 2.0)
    ok, _, max_curv, min_curv = ev.getCurvature(param)
    return ok and (abs(max_curv) > 1e-10 or abs(min_curv) > 1e-10)


def _curv_ratio(c1, c2):
    """Relative curvature difference. Returns 0.0 when both are near-zero (co-planar)."""
    denom = max(abs(c1), abs(c2))
    if denom < 1e-10:
        return 0.0
    return abs(c1 - c2) / denom


def _is_g2_edge(edge, f1, f2, tangent_tol_cos):
    """
    True if f1 and f2 are G2-continuous across edge (co-surface).
    Tests at 3 points: quarter, midpoint, three-quarter.
    G2 requires:
      1. Normal directions within tangent_tol (G1 already confirmed by caller)
      2. Both principal curvature ratios <= 0.02 at all 3 sample points
      3. Max-curvature tangent directions within tangent_tol
    """
    ev = edge.evaluator
    _, t0, t1 = ev.getParameterExtents()
    ST = ac.SurfaceTypes

    for t in [t0*0.75 + t1*0.25, (t0 + t1)*0.5, t0*0.25 + t1*0.75]:
        _, pt = ev.getPointAtParameter(t)

        ok1, n1 = f1.evaluator.getNormalAtPoint(pt)
        ok2, n2 = f2.evaluator.getNormalAtPoint(pt)
        if not ok1 or not ok2:
            return False

        # Surface type matching: NurbsSurface is wildcard
        st1 = f1.geometry.surfaceType
        st2 = f2.geometry.surfaceType
        nurbs = ST.NurbsSurfaceType
        if st1 != st2 and st1 != nurbs and st2 != nurbs:
            return False

        # Get parametric coords for curvature evaluation
        ok1p, param1 = f1.evaluator.getParameterAtPoint(pt)
        ok2p, param2 = f2.evaluator.getParameterAtPoint(pt)
        if not ok1p or not ok2p:
            return False

        ok1c, tan1, maxC1, minC1 = f1.evaluator.getCurvature(param1)
        ok2c, tan2, maxC2, minC2 = f2.evaluator.getCurvature(param2)
        if not ok1c or not ok2c:
            return False

        # Curvature magnitude ratio test
        if _curv_ratio(maxC1, maxC2) > 0.02:
            return False
        if _curv_ratio(minC1, minC2) > 0.02:
            return False

        # Max-curvature direction test (only meaningful when curvature is significant)
        if abs(maxC1) > 1e-10 and abs(maxC2) > 1e-10:
            dir_dot = abs(tan1.x*tan2.x + tan1.y*tan2.y + tan1.z*tan2.z)
            dir_dot = max(-1.0, min(1.0, dir_dot))
            if dir_dot < tangent_tol_cos:
                return False

    return True


def build_edge_sets(body, tangent_tol_cos):
    """
    Build three disjoint sets of edge tokens: concave, convex, tangent.
    tangent_tokens includes G1-tangent edges AND G2-continuous (co-surface) edges.
    Starts from body.concaveEdges / body.convexEdges, then:
      1. Promotes edges within tangent_tol to tangent_tokens (G1)
      2. Promotes remaining concave/convex edges that are G2-continuous to tangent_tokens
    Raises if the three sets do not account for all edges.
    """
    concave_tokens = {e.entityToken for e in body.concaveEdges}
    convex_tokens  = {e.entityToken for e in body.convexEdges}

    overlap = concave_tokens & convex_tokens
    if overlap:
        raise RuntimeError(f'{len(overlap)} edges appear in both concave and convex sets')

    tangent_tokens = set()
    all_edges      = list(body.edges)
    total          = len(all_edges)

    # Build an edge -> faces lookup once
    edge_faces = {}
    for edge in all_edges:
        edge_faces[edge.entityToken] = list(edge.faces)

    for edge in all_edges:
        tok   = edge.entityToken
        faces = edge_faces[tok]
        if len(faces) < 2:
            continue
        f1, f2 = faces[0], faces[1]
        ev = edge.evaluator
        _, t0, t1 = ev.getParameterExtents()
        _, pt     = ev.getPointAtParameter((t0 + t1) / 2.0)
        ok1, n1   = f1.evaluator.getNormalAtPoint(pt)
        ok2, n2   = f2.evaluator.getNormalAtPoint(pt)
        if not ok1 or not ok2:
            continue
        n_dot = n1.x*n2.x + n1.y*n2.y + n1.z*n2.z
        if abs(n_dot) >= tangent_tol_cos:
            # G1 tangent
            tangent_tokens.add(tok)
            concave_tokens.discard(tok)
            convex_tokens.discard(tok)


    boundary  = sum(1 for e in all_edges if len(list(e.faces)) < 2)
    accounted = len(concave_tokens) + len(convex_tokens) + len(tangent_tokens)
    if accounted + boundary != total:
        raise RuntimeError(f'Edge count mismatch: {accounted}+{boundary} != {total}')

    # Build g2_tokens: subset of tangent_tokens where faces are G2-continuous (co-surface)
    edge_by_tok = {e.entityToken: e for e in all_edges}
    g2_tokens   = set()
    for tok in tangent_tokens:
        edge  = edge_by_tok.get(tok)
        if edge is None:
            continue
        faces = edge_faces.get(tok, list(edge.faces))
        if len(faces) < 2:
            continue
        if _is_g2_edge(edge, faces[0], faces[1], tangent_tol_cos):
            g2_tokens.add(tok)

    return concave_tokens, convex_tokens, tangent_tokens, g2_tokens


def get_adj_face(edge, ref_face):
    adj = [f for f in edge.faces if f != ref_face]
    return adj[0] if adj else None


def is_extrusion_cylinder(adj_face, edge):
    """
    True if adj_face is an extrusion-type fillet tangent to a concave/convex fillet.
    Sphere corner connectors always qualify.
    Cylinders qualify only if the shared edge is perpendicular to the cylinder axis
    (i.e. the cylinder is extruded along its axis, not curving away).
    """
    ST = ac.SurfaceTypes
    st = adj_face.geometry.surfaceType
    if st == ST.SphereSurfaceType:
        return True
    if st == ST.CylinderSurfaceType:
        cyl  = ac.Cylinder.cast(adj_face.geometry)
        axis = cyl.axis
        ev   = edge.evaluator
        _, t0, t1 = ev.getParameterExtents()
        _, tan    = ev.getTangent((t0 + t1) / 2.0)
        dot   = abs(tan.x*axis.x + tan.y*axis.y + tan.z*axis.z)
        dot   = max(-1.0, min(1.0, dot))
        return math.degrees(math.acos(dot)) > 80.0
    return False

def flood_fill(seed_face, mode, tangent_tol_cos, lineage=None):
    """
    BFS flood fill across BRep faces using pre-classified edge sets.
    lineage: optional dict {face_token: (parent_token, rule, depth)} for debug tree output.
    Concave mode rules (applied from every queued face):
      1. G2 co-surface edge
      2. Tangent edge + adjacent face is concave curved
      3. Concave sharp edge
      4. From a concave curved face: tangent edge into flat or extrusion-type convex curved face
    Convex mode: mirror of Concave.
    Tangent mode: cross any tangent edge.
    """
    body      = seed_face.body
    body_tok  = body.entityToken
    cache_key = (body_tok, tangent_tol_cos)
    # Per-body cache persists for entire script run; keyed by (body_token, tol)
    edge_cache = _shared.setdefault('_edge_cache', {})
    if cache_key not in edge_cache:
        concave_tokens, convex_tokens, tangent_tokens, g2_tokens = build_edge_sets(body, tangent_tol_cos)
        edge_cache[cache_key] = (concave_tokens, convex_tokens, tangent_tokens, g2_tokens)
        _dbg(f"Edge sets [{body_tok[:8]}]: {len(concave_tokens)} concave, "
             f"{len(convex_tokens)} convex, {len(tangent_tokens)} tangent, "
             f"{len(g2_tokens)} g2 = {body.edges.count} total")
    concave_tokens, convex_tokens, tangent_tokens, g2_tokens = edge_cache[cache_key]

    visited_tokens = {seed_face.entityToken}
    visited_faces  = [seed_face]
    queue          = [(seed_face, 0)]  # (face, depth)
    if lineage is not None:
        lineage[seed_face.entityToken] = (None, 'seed', 0)

    while queue:
        face, depth = queue.pop(0)  # FIFO for tree order

        for edge in face.edges:
            tok      = edge.entityToken
            adj_face = get_adj_face(edge, face)
            if adj_face is None or adj_face.entityToken in visited_tokens:
                continue

            is_tangent       = tok in tangent_tokens
            is_sharp_concave = tok in concave_tokens
            is_sharp_convex  = tok in convex_tokens
            adj_curved       = is_curved_face(adj_face)
            adj_concavity    = is_face_concave(adj_face) if adj_curved else None

            match = False

            face_concavity = is_face_concave(face) if is_curved_face(face) else None
            is_g2          = tok in g2_tokens
            rule           = ''

            if mode == 'tangent':
                rule = 'R1-edge-tangent'
                match = is_tangent

            elif mode == 'concave':
                if is_g2:
                    rule = 'R1-G2-cosurface'
                    match = True
                elif is_tangent and adj_concavity == True:
                    rule = 'R2-edge-tangent-adj-face-concave'
                    match = True
                elif is_sharp_concave:
                    rule = 'R3-edge-concave'
                    match = True
                elif is_tangent and face_concavity == True:
                    rule = 'R4-from-concave-face'
                    if adj_concavity is None:
                        match = True
                    elif adj_concavity == False:
                        match = is_extrusion_cylinder(adj_face, edge)

            elif mode == 'convex':
                if is_g2:
                    rule = 'R1-G2-cosurface'
                    match = True
                elif is_tangent and adj_concavity == False:
                    rule = 'R2-edge-tangent-adj-face-convex'
                    match = True
                elif is_sharp_convex:
                    rule = 'R3-edge-convex'
                    match = True
                elif is_tangent and face_concavity == False:
                    rule = 'R4-from-convex-face'
                    if adj_concavity is None:
                        match = True
                    elif adj_concavity == True:
                        match = is_extrusion_cylinder(adj_face, edge)

            if match:
                visited_tokens.add(adj_face.entityToken)
                visited_faces.append(adj_face)
                queue.append((adj_face, depth + 1))
                if lineage is not None:
                    lineage[adj_face.entityToken] = (face.entityToken, rule, depth + 1)

    return visited_faces


# ---------- helpers ----------

_SURF_NAMES = {
    ac.SurfaceTypes.PlaneSurfaceType: 'Plane',
    ac.SurfaceTypes.CylinderSurfaceType: 'Cylinder',
    ac.SurfaceTypes.ConeSurfaceType: 'Cone',
    ac.SurfaceTypes.SphereSurfaceType: 'Sphere',
    ac.SurfaceTypes.TorusSurfaceType: 'Torus',
    ac.SurfaceTypes.NurbsSurfaceType: 'Nurbs',
    ac.SurfaceTypes.EllipticalCylinderSurfaceType: 'EllCyl',
    ac.SurfaceTypes.EllipticalConeSurfaceType: 'EllCone',
}

def _face_desc(face):
    """Short description of a face for debug logging."""
    st      = face.geometry.surfaceType
    name    = _SURF_NAMES.get(st, str(st))
    curved  = is_curved_face(face)
    conc    = is_face_concave(face) if curved else None
    conc_s  = 'concave' if conc == True else 'convex' if conc == False else 'flat'
    faces   = list(face.body.faces)
    try:
        idx = next(i for i,f in enumerate(faces) if f.entityToken == face.entityToken)
    except StopIteration:
        idx = -1
    return f'{name}[{idx}]({conc_s})'


def _build_face_index_graphics(body):
    """Draw face index numbers at pointOnFace for each face in body.
    Called once per body per run when debug is enabled.
    """
    design = af.Design.cast(_app.activeProduct)
    root   = design.rootComponent
    # Clear any existing debug graphics group for this body
    grp_name = f'face_idx_{body.entityToken[:8]}'
    grps = root.customGraphicsGroups
    for gi in range(grps.count - 1, -1, -1):
        grps.item(gi).deleteMe()
    grp = grps.add()
    bb = af.CustomGraphicsBillBoard.create(ac.Point3D.create(0,0,0))
    bb.billBoardStyle = af.CustomGraphicsBillBoardStyles.ScreenBillBoardStyle
    for idx, face in enumerate(body.faces):
        pt = face.pointOnFace
        m  = ac.Matrix3D.create()
        m.translation = ac.Vector3D.create(pt.x, pt.y, pt.z)
        txt = grp.addText(f'.{idx}', 'Arial', 0.3, m)
        txt.billBoarding = bb
    _app.activeViewport.refresh()
    return grp


# ---------- command handlers ----------

class CommandExecuteHandler(ac.CommandEventHandler):
    def __init__(self): super().__init__()
    def notify(self, args): pass

class CommandDestroyHandler(ac.CommandEventHandler):
    def __init__(self): super().__init__()
    def notify(self, args):
        try:
            # Clear face index debug graphics
            design = af.Design.cast(_app.activeProduct)
            if design:
                grps = design.rootComponent.customGraphicsGroups
                for gi in range(grps.count - 1, -1, -1):
                    grps.item(gi).deleteMe()
                _app.activeViewport.refresh()
        except:
            pass
        try:
            if args.terminationReason == ac.CommandTerminationReason.CompletedTerminationReason:
                faces = _shared.get('faces', [])
                if faces:
                    try:
                        _app.unregisterCustomEvent(CUSTOM_EVENT_ID)
                    except:
                        pass
                    custom_event = _app.registerCustomEvent(CUSTOM_EVENT_ID)
                    class OnPostSelect(ac.CustomEventHandler):
                        def notify(self, args):
                            try:
                                _ui.activeSelections.clear()
                                for f in faces:
                                    _ui.activeSelections.add(f)
                            except:
                                _ui.messageBox(traceback.format_exc())
                            _app.unregisterCustomEvent(CUSTOM_EVENT_ID)
                    handler = OnPostSelect()
                    custom_event.add(handler)
                    _handlers.append(handler)
                    _app.fireCustomEvent(CUSTOM_EVENT_ID, '')
        except:
            _ui.messageBox(traceback.format_exc())
        adsk.terminate()

class PreSelectHandler(ac.SelectionEventHandler):
    def __init__(self): super().__init__()
    def notify(self, args):
        try:
            if _shared.get('locked'): return
            face = af.BRepFace.cast(args.selection.entity)
            if not face: return
            mode  = _shared.get('mode', 'concave')
            tol   = _shared.get('tol', math.cos(math.radians(0.1)))
            lineage = {} if _shared.get('debug') else None
            # Build face index graphics once per body when debug is on
            if _shared.get('debug'):
                body_tok = face.body.entityToken
                gfx_cache = _shared.setdefault('_gfx_cache', set())
                if body_tok not in gfx_cache:
                    _build_face_index_graphics(face.body)
                    gfx_cache.add(body_tok)
            faces = flood_fill(face, mode, tol, lineage)
            _shared['faces']   = faces
            _shared['lineage'] = lineage
            col = ac.ObjectCollection.create()
            for f in faces:
                if f.entityToken != face.entityToken:
                    col.add(f)
            args.additionalEntities = col
        except:
            _ui.messageBox(traceback.format_exc())

class PreSelectEndHandler(ac.SelectionEventHandler):
    def __init__(self): super().__init__()
    def notify(self, args):
        try:
            if not _shared.get('locked'):
                _shared['faces'] = []
        except:
            _ui.messageBox(traceback.format_exc())

class SelectHandler(ac.SelectionEventHandler):
    def __init__(self): super().__init__()
    def notify(self, args):
        try:
            if not _shared.get('locked'):
                _shared['locked'] = True
                _shared['inputs'].itemById('lbl').text = ''
                if _shared.get("debug"):
                    lineage = _shared.get('lineage', {})
                    faces = _shared.get('faces', [])
                    mode  = _shared.get('mode', '')
                    seed  = faces[0] if faces else af.BRepFace.cast(args.selection.entity)
                    _app.log('---')
                    _app.log(f'Mode: {mode}  Seed: {_face_desc(seed)}  Found: {len(faces)} faces')
                    body_faces = list(seed.body.faces)
                    def face_idx(f):
                        try: return next(i for i,bf in enumerate(body_faces) if bf.entityToken == f.entityToken)
                        except StopIteration: return -1
                    def short_desc(f):
                        st   = _SURF_NAMES.get(f.geometry.surfaceType, str(f.geometry.surfaceType))
                        curv = is_curved_face(f)
                        conc = is_face_concave(f) if curv else None
                        cs   = 'concave' if conc == True else 'convex' if conc == False else 'flat'
                        return f'{st}({cs})'
                    from collections import defaultdict
                    children = defaultdict(list)
                    for f in faces:
                        parent_tok, rule, _ = lineage.get(f.entityToken, (None, "?", 0))
                        children[parent_tok].append((f, rule))
                    seed_idx = face_idx(seed)
                    _app.log(f"[{seed_idx}] {short_desc(seed)} [seed]")
                    def print_tree(f, ind):
                        pidx = face_idx(f)
                        for child, rule in children.get(f.entityToken, []):
                            cidx = face_idx(child)
                            _app.log(f"{ind}[{pidx}]->[{cidx}] {short_desc(child)} [{rule}]")
                            print_tree(child, ind + "        ")
                    print_tree(seed, "        ")
        except:
            _ui.messageBox(traceback.format_exc())

class InputChangedHandler(ac.InputChangedEventHandler):
    def __init__(self): super().__init__()
    def notify(self, args):
        try:
            changed_id = args.input.id
            inputs     = _shared['inputs']
            mode_map   = {'Concave':'concave','Convex':'convex','Tangent':'tangent'}
            if changed_id in ('mode', 'tol'):
                _shared['mode'] = mode_map[inputs.itemById('mode').selectedItem.name]
                _shared['tol']  = math.cos(inputs.itemById('tol').value)
                _shared.pop('_edge_cache', None)  # invalidate all body caches on tol change
            if changed_id == 'debug':
                _shared['debug'] = inputs.itemById('debug').value
            if changed_id in ('mode', 'tol', 'seed'):
                sel = inputs.itemById('seed')
                if sel.selectionCount == 0:
                    _shared['locked'] = False
                    _shared['faces']  = []
                    inputs.itemById('lbl').text = 'Select Seed Face'
        except:
            _ui.messageBox(traceback.format_exc())

class CommandCreatedHandler(ac.CommandCreatedEventHandler):
    def __init__(self): super().__init__()
    def notify(self, args):
        try:
            cmd    = ac.Command.cast(args.command)
            inputs = cmd.commandInputs

            _shared['locked'] = False
            _shared['faces']  = []
            _shared['mode']   = 'concave'
            _shared['tol']    = math.cos(math.radians(0.1))
            _shared['debug']  = False
            _shared['inputs'] = inputs

            onExecute     = CommandExecuteHandler()
            onDestroy     = CommandDestroyHandler()
            onPreSelect   = PreSelectHandler()
            onPreSelEnd   = PreSelectEndHandler()
            onSelect      = SelectHandler()
            onInputChange = InputChangedHandler()
            cmd.execute.add(onExecute)
            cmd.destroy.add(onDestroy)
            cmd.preSelect.add(onPreSelect)
            cmd.preSelectEnd.add(onPreSelEnd)
            cmd.select.add(onSelect)
            cmd.inputChanged.add(onInputChange)
            _handlers.extend([onExecute, onDestroy, onPreSelect,
                              onPreSelEnd, onSelect, onInputChange])

            inputs.addTextBoxCommandInput('lbl', '', 'Select Seed Face', 1, True)
            sel = inputs.addSelectionInput('seed', '', 'Hover to preview, click to lock')
            sel.addSelectionFilter('SolidFaces')
            sel.setSelectionLimits(1, 0)

            rb = inputs.addRadioButtonGroupCommandInput('mode', 'Selection Mode')
            rb.listItems.add('Concave', True)
            rb.listItems.add('Convex',  False)
            rb.listItems.add('Tangent', False)

            inputs.addValueInput(
                'tol', 'Tangent Tolerance', 'deg',
                ac.ValueInput.createByString('0.1 deg'))

            inputs.addBoolValueInput('debug', 'Debug', True, '', False)

        except:
            _ui.messageBox(traceback.format_exc())


# ---------- entry point ----------

def run(_context):
    global _handlers, _shared
    _handlers.clear()
    _shared.clear()

    try:
        cmd_def = _ui.commandDefinitions.itemById('faceFloodFillCmd')
        if cmd_def: cmd_def.deleteMe()
        cmd_def = _ui.commandDefinitions.addButtonDefinition(
            'faceFloodFillCmd', 'Face Flood Fill', '', '')
        onCreate = CommandCreatedHandler()
        cmd_def.commandCreated.add(onCreate)
        _handlers.append(onCreate)
        cmd_def.execute()
        adsk.autoTerminate(False)
    except:
        _ui.messageBox(traceback.format_exc())