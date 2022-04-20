"""
Report the curve type of a SketchCurve or BrepEdge and additional info
for NURBS curves.
"""

"""
220320: Created.
220419: Modified printed output.
"""

import adsk.core as ac
import adsk.fusion as af
import traceback


_app = ac.Application.get()
_ui = _app.userInterface


def log(printMe): _app.log(str(printMe))


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

    s += "\n   "

    zipped = zip(iKsPerM, ts_Unique)
    s_JoinUs = []
    for idxs, t in zipped:
        if len(idxs) == 1:
            s_JoinUs.append(f"[{idxs[0]}]{t:.{dec_plcs}f}")#.format(t, dec_plcs, idxs[0]))
        else:
            s_JoinUs.append(f"[{idxs[0]},{idxs[-1]}]{t:.{dec_plcs}f}")#.format(t, dec_plcs, idxs[0], idxs[-1]))
    s += " ".join(s_JoinUs)

    if abs(min(deltas_ts) - max(deltas_ts)) <= 10.0**(-dec_plcs):
        s += "  Deltas:{0:.{1}f}".format(
            deltas_ts[0], dec_plcs)
    else:
        s += "  DeltaRange:[{0:.{2}f},{1:.{2}f}]".format(
            min(deltas_ts), max(deltas_ts), dec_plcs)

    return s


def getNurbsCrvInfo(nc: ac.NurbsCurve3D):

    (
        bSuccess,
        controlPoints,
        degree,
        knots,
        isRational_getData,
        weights,
        isPeriodic,
        ) = nc.getData()

    if not bSuccess: return

    s  = f"\ndegree: {degree}"
    s += f"\nCP count: {len(controlPoints)}"
    s += f"\nknots: {getKnotInfo(knots)}"
    # s += f"\nisRational (per property): {nc.isRational}"
    s += f"\nisRational:  {isRational_getData}"
    s += f"\nweights: {weights if weights else None}"
    s += f"\nisPeriodic: {isPeriodic}"

    return s


def getCrvInfo(sel: ac.Selection):
    ent = sel.entity

    s = '\n'

    s += f"Selected: {ent.objectType[14:]}"

    if isinstance(ent, af.BRepEdge):
        crv = ent.geometry
    elif isinstance(ent, af.SketchCurve):
        crv = ent.geometry
    else:
        raise ValueError("Wrong selection entity type passed to printInfo.  Try again.")
    
    s += f"\nGeometry: {crv.objectType[12:]}"

    if isinstance(crv, (ac.NurbsCurve3D)):
        s += getNurbsCrvInfo(crv)

    return s


def main():
    while True:
        try:
            sel = _ui.selectEntity(
                "Select a sketch curve or brep edge",
                filter="Edges,SketchCurves")
        except:
            return

        sInfo = getCrvInfo(sel)
        log(sInfo)


def run(context):
    try:
        main()
    except:
        log(f"\nFailed:\n{traceback.format_exc()}")
    finally:
        log("\nEnd of script.")
