"""
When a SketchCurve or BrepEdge is selected, report the curve type
and additional info for NURBS curves.
When a BrepFace is selected, report the underlying surface type
and additional info for NURBS surfaces.

Bug in NurbsSurface.getData as reported here
https://forums.autodesk.com/t5/fusion-360-api-and-scripts/incorrect-propertiesu-and-propertiesv-from-nurbssurface-getdata/m-p/10828454#M15191
Workaround is to ignore values of propertiesU and propertiesV.
Instead, get values of relevant properties from 
"""

"""
211214: Added getKnotInfo.
...
220427: Combined two scripts into this one.
220621: Bug fix in getting name of SurfaceTypes Enumerator.
220708: Added app.log of control point locations.
221215: Added ProceduralToNURBSConversion of NurbSurfaces.
221216: Now checks whether ProceduralToNURBSConversion created a NurbsSurface not epsilon equal to the original.
221217: For easier reading, removed report of CP locations.
230530: Now also reports geometry of BRepCoEdges when a BRepEdges is selected.
"""

import adsk.core as ac
import adsk.fusion as af
import traceback


_app = ac.Application.get()
_ui = _app.userInterface


def _log(*printMe):
    try: myList = list(printMe)
    except: myList = [printMe]
    _app.log(" ".join(str(_) for _ in myList))


def enumNameFromInteger_NotBitwise(sEnum, theInteger):
    enum = eval(sEnum)
    for sAttr in dir(enum):
        if sAttr[:2] == '__': continue
        if sAttr == 'thisown': continue
        if eval(f"{sEnum}.{sAttr}") == theInteger:
            return sAttr


def enumNamesFromInteger_Bitwise(sEnum, theInteger, trimFromRight):
    enum = eval(sEnum)
    sAttrs = []
    for sAttr in dir(enum):
        if sAttr[:2] == '__': continue
        if sAttr == 'thisown': continue
        iEnumValue = eval(f"{sEnum}.{sAttr}")
        if iEnumValue & theInteger:
            sAttrs.append(sAttr[:-trimFromRight])
    return sAttrs


def getKnotInfo(knots):
    """
    Returns a string.
    """

    iK = 0
    ts_Unique = []
    ms = []
    iKsPerM = []
    deltas_ts = []

    while iK < len(knots):
        k = knots[iK]
        ts_Unique.append(k)
        m = knots.count(k)
        ms.append(m)
        iKsPerM.append(range(iK, iK+m))
        if iK > 0: deltas_ts.append(ts_Unique[-1]-ts_Unique[-2])
        iK += m

    s  = f"Count:{len(knots)}"
    s += f" SpanCt:{len(ms)-1}"
    s += f" Multiplicities: {','.join(str(i) for i in ms)}"

    # Record knot parameters to at least 3 decimal places but more if
    # required to display all unique values.
    for dec_plcs in range(15, 3-1, -1):
        LIST = [" {: .{}f}".format(t, dec_plcs) for t in ts_Unique]
        if len(set(LIST)) != len(ts_Unique):
            dec_plcs += 1
            break


    # Unique knot parameters.
    if False:
        s += "\n   "

        zipped = zip(iKsPerM, ts_Unique)
        s_JoinUs = []
        for idxs, t in zipped:
            if len(idxs) == 1:
                s_JoinUs.append(f"[{idxs[0]}]{t:.{dec_plcs}f}")#.format(t, dec_plcs, idxs[0]))
            else:
                s_JoinUs.append(f"[{idxs[0]},{idxs[-1]}]{t:.{dec_plcs}f}")#.format(t, dec_plcs, idxs[0], idxs[-1]))
        s += " ".join(s_JoinUs)

    else:
        s += "  Range:{0:.{2}f},{1:.{2}f}".format(
            min(ts_Unique), max(ts_Unique), dec_plcs)

    if abs(min(deltas_ts) - max(deltas_ts)) <= 10.0**(-dec_plcs):
        s += "  Deltas:{0:.{1}f}".format(
            deltas_ts[0], dec_plcs)
    else:
        s += "  DeltaMin,Max: {0:.{2}f},{1:.{2}f}".format(
            min(deltas_ts), max(deltas_ts), dec_plcs)

    return s


def getNurbsCrvInfo(nc, iCt_MaxCPs: int=0):
    """
    nc: NurbsCurve2D or NurbsCurve3D
    """

    (
        bSuccess,
        cps,
        degree,
        knots,
        isRational_getData,
        weights,
        isPeriodic,
        ) = nc.getData()

    if not bSuccess: return

    s  = f"\ndegree: {degree}"
    s += f"  CP count: {len(cps)}"
    s += f"\nknots: {getKnotInfo(knots)}"
    s += f"\nisClosed: {nc.isClosed}"
    s += f"  isPeriodic: {isPeriodic}"
    # s += f"\nisRational (per property): {nc.isRational}"
    s += f"  isRational:  {isRational_getData}"
    s += f"  weights: {weights if weights else None}"

    if iCt_MaxCPs == 0:
        return s

    s += "\nControl points:"

    # cps is adsk.core.Point3DVector but acts like a list.

    if (iCt_MaxCPs < 0) or (len(cps.size) <= iCt_MaxCPs):
        for i, cp in enumerate(cps):
            s += f"\n  {i}: {cp.x}, {cp.y}, {cp.z}"
    else:
        for i, cp in enumerate(list(cps)[:iCt_MaxCPs//2]):
            s += f"\n  {i}: {cp.x}, {cp.y}, {cp.z}"
        s += "\n..."
        start = len(cps)-iCt_MaxCPs//2
        for i, cp in enumerate(list(cps)[start:], start=start):
            s += f"\n  {i}: {cp.x}, {cp.y}, {cp.z}"

    return s


def getCrvInfo(crv: ac.Curve3D):
    s = f"\nCurve type: {crv.objectType[12:]}"

    if isinstance(crv, (ac.NurbsCurve3D, ac.NurbsCurve2D)):
        s += getNurbsCrvInfo(crv, iCt_MaxCPs=0)

    return s


def getNurbsSrfInfo(ns: ac.NurbsSurface, iCt_MaxCPs: int=0):
    (
        bSuccess,
        degreeU,
        degreeV,
        cpCt_U,
        cpCt_V,
        cps,
        knotsU,
        knotsV,
        weights,
        propertiesU_Per_getData,
        propertiesV_Per_getData,
        ) = ns.getData()

    if not bSuccess: return

    propertiesU_Per_prop = ns.propertiesU
    propertiesV_Per_prop = ns.propertiesV

    if propertiesU_Per_getData != propertiesU_Per_prop:
        _log(
            "propertiesU from NurbsSurface.getData doesn't match NurbsSurfce.propertiesU."
            "  This means that bug reported to forum on 12-16-2021 still exists.")

    if propertiesV_Per_getData != propertiesV_Per_prop:
        _log(
            "propertiesV from NurbsSurface.getData doesn't match NurbsSurfce.propertiesV."
            "  The means that bug reported to forum on 12-16-2021 still exists.")

    #log(rc)

    s  = f"\nDegrees (U x V): {degreeU} x {degreeV}"
    s += f"\nCP Counts (U x V): {cpCt_U} x {cpCt_V}"
    s += f"\nKnotsU  {getKnotInfo(knotsU)}"
    s += f"\nKnotsV  {getKnotInfo(knotsV)}"
    s += f"\nWeights: {weights if weights else None}"
    # for weight in weights:
    #     s += f"\n{weight:.30f}"
    # s += f"\n{propertiesU}"
    # s += f"\n{propertiesV}"
    s += "\nProperties (U x V): {} x {}".format(
        ",".join(enumNamesFromInteger_Bitwise('ac.NurbsSurfaceProperties', propertiesU_Per_prop, 12)),
        ",".join(enumNamesFromInteger_Bitwise('ac.NurbsSurfaceProperties', propertiesV_Per_prop, 12)))


    if iCt_MaxCPs == 0:
        return s

    s += "\nControl points:"

    # cps is adsk.core.Point3DVector but acts like a list.

    if (iCt_MaxCPs < 0) or (cps.size() <= iCt_MaxCPs):
        for i, cp in enumerate(cps):
            s += f"\n  {i}: {cp.x}, {cp.y}, {cp.z}"
    else:
        for i, cp in enumerate(list(cps)[:iCt_MaxCPs//2]):
            s += f"\n  {i}: {cp.x}, {cp.y}, {cp.z}"
        s += "\n..."
        start = cps.size()-iCt_MaxCPs//2
        for i, cp in enumerate(list(cps)[start:], start=start):
            s += f"\n  {i}: {cp.x}, {cp.y}, {cp.z}"

    return s


def getSrfInfo(surface: ac.Surface):

    # s = "\nSurface type: {}".format(
    #     enumNameFromInteger_NotBitwise('ac.SurfaceTypes', surface.surfaceType))
    # [:-11] removes "SurfaceType"

    if surface.surfaceType == ac.SurfaceTypes.NurbsSurfaceType:
        ns = surface
        s_nsInfo = getNurbsSrfInfo(ns)
        if s_nsInfo:
            s += s_nsInfo

    return s


def areNurbsSurfacesEquivalent(ns_A, ns_B):
    getData_A = ns_A.getData()
    if not getData_A[0]:
        raise Exception("getData failed!")
    getData_B = ns_B.getData()
    if not getData_A[0]:
        raise Exception("getData failed!")

    for i in 1,2,3,4,6,7,8,9,10:
        if getData_A[i] != getData_B[i]:
            return False

    # Now check all control points.
    for j in range(getData_A[3] * getData_A[4]):
        if not getData_A[5][j].isEqualTo(getData_B[5][j]):
            return False

    return True


def getGeomInfo(ent):

    s = f"\nSelected: {ent.objectType[14:]}"

    if (isinstance(ent, af.BRepFace)):
        face = ent
        surface = face.geometry

        s += "\nSurface type: {}".format(
            enumNameFromInteger_NotBitwise('ac.SurfaceTypes', surface.surfaceType)[:-11])

        if surface.surfaceType == ac.SurfaceTypes.NurbsSurfaceType:
            ns = surface

            s_nsInfo = getNurbsSrfInfo(ns, iCt_MaxCPs=0)
            if s_nsInfo:
                s += s_nsInfo
            body_Converted = face.convert(0)#af.BRepConvertOptions.ProceduralToNURBSConversion)

            s_fromProcedural = ""

            for face_converted in body_Converted.faces:
                ns_converted = face_converted.geometry
                if areNurbsSurfacesEquivalent(ns, ns_converted):
                    s += "\n\nSurface is not procedurally calculated."
                    continue

                s_nsInfo = getNurbsSrfInfo(ns_converted, iCt_MaxCPs=0)
                if s_nsInfo:
                    s_fromProcedural += s_nsInfo

            if s_fromProcedural:
                s += "\n\nProceduralToNURBSConversion:"
                s += s_fromProcedural

        return s
    elif isinstance(ent, af.BRepEdge):
        rc = getCrvInfo(ent.geometry)
        if rc: s += rc
        for ic, coedge in enumerate(ent.coEdges):
            s += f"\n\nBRepCoEdge {ic+1} of {ent.coEdges.count} of selected BRepEdge"
            rc = getCrvInfo(coedge.geometry)
            if rc: s += rc
        return s
    elif isinstance(ent, af.SketchCurve):
        rc = getCrvInfo(ent.geometry)
        if rc: return s + rc
    else:
        raise ValueError("Wrong selection entity type passed to printInfo.  Try again.")
    

def main():
    while True:
        try:
            sel = _ui.selectEntity(
                "Select a sketch curve, brep edge, or brep face",
                filter="Edges,Faces,SketchCurves")
        except:
            return

        sInfo = getGeomInfo(sel.entity)
        _log(sInfo)


def run(context):
    try:
        main()
    except:
        _log(f"\nFailed:\n{traceback.format_exc()}")
    finally:
        _log("\nEnd of script.")
