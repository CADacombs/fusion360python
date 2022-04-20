"""
Bug in NurbsSurface.getData as reported here
https://forums.autodesk.com/t5/fusion-360-api-and-scripts/incorrect-propertiesu-and-propertiesv-from-nurbssurface-getdata/m-p/10828454#M15191
Workaround is to ignore values of propertiesU and propertiesV.
Instead, get values of relevant properties from 
"""

"""
Modification of a sample script.
211214: Added getKnotInfo.
220319: ...
220407: Now, repeats face selection until Esc is pressed.  Now, all printed output goes to Text Commands panel.
220413: Modified printed output.
220417: Bug fix for when there are multiple NurbsSurfaceProperties enum values.
220419: Refactored to match a similar script for curves.
"""

import adsk.core as ac
import adsk.fusion as af
import traceback


_app = ac.Application.get()
_ui = _app.userInterface


def log(printMe): _app.log(str(printMe))


def enumNameFromInteger_NotBitwise(sEnum, theInteger):
    enum = eval(sEnum)
    for sAttr in dir(enum):
        if sAttr[:2] == '__': continue
        if sAttr == 'thisown': continue
        if eval(f"{sEnum}.{sAttr}") == theInteger:
            return sAttr[:-11]


def enumNamesFromInteger_Bitwise(sEnum, theInteger):
    enum = eval(sEnum)
    sAttrs = []
    for sAttr in dir(enum):
        if sAttr[:2] == '__': continue
        if sAttr == 'thisown': continue
        iEnumValue = eval(f"{sEnum}.{sAttr}")
        if iEnumValue & theInteger:
            sAttrs.append(sAttr[:-12])
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


def getNurbsSrfInfo(ns: ac.NurbsSurface):
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
        log(
            "propertiesU from NurbsSurface.getData doesn't match NurbsSurfce.propertiesU."
            "  Bug reported to forum on 12-16-2021 still exists.")

    if propertiesV_Per_getData != propertiesV_Per_prop:
        log(
            "propertiesV from NurbsSurface.getData doesn't match NurbsSurfce.propertiesV."
            "  Bug reported to forum on 12-16-2021 still exists.")

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
        ",".join(enumNamesFromInteger_Bitwise('ac.NurbsSurfaceProperties', propertiesU_Per_prop)),
        ",".join(enumNamesFromInteger_Bitwise('ac.NurbsSurfaceProperties', propertiesV_Per_prop)))

    return s


def getSrfInfo(sel: ac.Selection):
    ent = sel.entity

    if (isinstance(ent.geometry, ac.Surface)):
        #log("Surface picked.")
        surface = ent.geometry
        s = "\nSurface type: {}".format(
            enumNameFromInteger_NotBitwise('ac.SurfaceTypes', surface.surfaceType))

        if surface.surfaceType == ac.SurfaceTypes.NurbsSurfaceType:
            ns = surface
            s_nsInfo = getNurbsSrfInfo(ns)
            if s_nsInfo:
                s += s_nsInfo

        return s

def main():
    while True:
        try:
            sel = _ui.selectEntity("Select a face", "Faces")
        except:
            return

        sInfo = getSrfInfo(sel)
        log(sInfo)


def run(context):
    try:
        main()
    except:
        log(f"\nFailed:\n{traceback.format_exc()}")
    finally:
        log("\nEnd of script.")
