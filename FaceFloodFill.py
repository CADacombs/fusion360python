"""
This code is intended as a solution to the problem presented on the thread,
https://forums.autodesk.com/t5/fusion-api-and-scripts-forum/how-to-reliably-detect-concave-vs-convex-face-transitions-in/td-p/14107920

Developed with assistance from Autodesk Assistant (AI).

260519: First released version.
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


def build_edge_sets(body, tangent_tol_cos):
    """
    Build three disjoint sets of edge tokens: concave, convex, tangent.
    Starts from body.concaveEdges / body.convexEdges, then promotes
    any edge whose face normals are within tangent_tol_cos to the tangent set.
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

    for edge in all_edges:
        tok   = edge.entityToken
        faces = list(edge.faces)
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
            tangent_tokens.add(tok)
            concave_tokens.discard(tok)
            convex_tokens.discard(tok)

    boundary  = sum(1 for e in all_edges if len(list(e.faces)) < 2)
    accounted = len(concave_tokens) + len(convex_tokens) + len(tangent_tokens)
    if accounted + boundary != total:
        raise RuntimeError(f'Edge count mismatch: {accounted}+{boundary} != {total}')

    return concave_tokens, convex_tokens, tangent_tokens


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

def flood_fill(seed_face, mode, tangent_tol_cos):
    """
    BFS flood fill across BRep faces using pre-classified edge sets.
    Concave mode rules (applied from every queued face):
      1. Tangent edge + adjacent face is concave curved
      2. Concave edge (any adjacent face type)
      3. From a concave curved face: tangent edge into flat face
         (included but not propagated) or into extrusion-type convex curved face
    Convex mode: mirror of Concave.
    Tangent mode: cross any tangent edge.
    """
    body      = seed_face.body
    body_tok  = body.entityToken
    cache_key = (body_tok, tangent_tol_cos)
    if _shared.get('_edge_cache_key') != cache_key:
        concave_tokens, convex_tokens, tangent_tokens = build_edge_sets(body, tangent_tol_cos)
        _shared['_edge_cache']     = (concave_tokens, convex_tokens, tangent_tokens)
        _shared['_edge_cache_key'] = cache_key
        _shared['_edge_cache_log'] = (f"Edge sets: {len(concave_tokens)} concave, "
                                      f"{len(convex_tokens)} convex, "
                                      f"{len(tangent_tokens)} tangent = {body.edges.count} total")
    else:
        concave_tokens, convex_tokens, tangent_tokens = _shared['_edge_cache']

    visited_tokens = {seed_face.entityToken}
    visited_faces  = [seed_face]
    queue          = [seed_face]
    no_propagate   = set()  # faces added but not queued for further traversal

    while queue:
        face = queue.pop()
        if face.entityToken in no_propagate:
            continue

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

            match     = False
            propagate = True

            if mode == 'tangent':
                match = is_tangent

            elif mode == 'concave':
                if is_tangent and adj_concavity == True:
                    match = True
                elif is_sharp_concave:
                    match = True
                elif is_curved_face(face) and is_face_concave(face) == True and is_tangent:
                    if adj_concavity is None:
                        match = True; propagate = False
                    elif adj_concavity == False:
                        match = is_extrusion_cylinder(adj_face, edge)

            elif mode == 'convex':
                if is_tangent and adj_concavity == False:
                    match = True
                elif is_sharp_convex:
                    match = True
                elif is_curved_face(face) and is_face_concave(face) == False and is_tangent:
                    if adj_concavity is None:
                        match = True; propagate = False
                    elif adj_concavity == True:
                        match = is_extrusion_cylinder(adj_face, edge)

            if match:
                visited_tokens.add(adj_face.entityToken)
                visited_faces.append(adj_face)
                if propagate:
                    queue.append(adj_face)
                else:
                    no_propagate.add(adj_face.entityToken)

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


# ---------- command handlers ----------

class CommandExecuteHandler(ac.CommandEventHandler):
    def __init__(self): super().__init__()
    def notify(self, args): pass

class CommandDestroyHandler(ac.CommandEventHandler):
    def __init__(self): super().__init__()
    def notify(self, args):
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
            faces = flood_fill(face, mode, tol)
            _shared['faces'] = faces
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
                    faces = _shared.get('faces', [])
                    mode  = _shared.get('mode', '')
                    seed  = faces[0] if faces else af.BRepFace.cast(args.selection.entity)
                    _app.log('---')
                    if '_edge_cache_log' in _shared:
                        _app.log(_shared['_edge_cache_log'])
                    _app.log(f'Mode: {mode}  Seed: {_face_desc(seed)}  Found: {len(faces)} faces')
                    for fi, ff in enumerate(faces):
                        _app.log(f'  {fi+1}. {_face_desc(ff)}')
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
                _shared.pop('_edge_cache_key', None)  # invalidate cache on tol change
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