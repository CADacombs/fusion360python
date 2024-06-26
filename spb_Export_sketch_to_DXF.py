"""
This script is an alternative to 'Save as DXF' command accessed by
right clicking on a sketch.

Features not found in native command:
    . Multiple sketches can be exported to the same .dxf. (See limitation in next section.)
    . Points are supported.
    . Option to export per World coordinates instead of default to transform sketch coordinates to the World XY plane.
    . Option to include points with connectedEntities.
    . Option to include points that are not deletable.
    . Option to include construction curves.
    . Option to include linked points and curves.
    . Option to include reference points and curves.
    . DXF entities are on layer names described with parent component name, sketch name, and construction, etc.

Limitations:
    . Sketches from different occurrences of the same component cannot be included.
    . Ellipses are currently exported as NURBS curves.

Send any questions, comments, or script development service needs to @CADacombs on Autodesk's forums.
"""

"""
240209-17: Created basic working version.
240229-0301: Changed temporary folder path. Added dialog box for input.
240301: Added saving of options to an .ini in same folder as this script.
        Added option to export World coordinates vs. sketch plane transformation to World XY plane.
240304: Bug fix of arcs per Sketch coordinates.
        Added option to include SketchPoints with connectedEntities. Origin point has none.
        Added option to include SketchPoints per isDeletable. Origin point is False
        Added option to include construction curves.
        Added option to include linked points and curves.
        Added option to include reference points and curves.
240308: Added option to set DXF path. Default may be Desktop but chosen one will be saved in .ini.
240403: Bug fix. Renamed template file.
240404: Layer names in DXF now include the sketch's parent component name and the sketch name.
240405: Now, multiple sketches can be exported to one .dxf.

Notes:
    Right-click-'Save as DXF' output always includes normal of arcs and circles:
        210
        0
        220
        0
        230
        1
    Points at the center of rectangles and polygons are not  Adding an option for points with geometric constraints

TODO:
    WIP:


    Next up:

    Probably:
        Project points and curves to sketch plane. Add options to dialog.

    Maybe:
        When SketchPt.connectedEntities points are not included for export, add an option to
            create points at center of all circles.
        Export ellipses as ELLIPSE instead of SPLINE.
"""

import adsk
import adsk.core as ac
import adsk.fusion as af

import configparser
import os.path
import math


_app = ac.Application.get()
_ui = _app.userInterface

_strCmdID = os.path.splitext(os.path.basename(__file__))[0]
_strCmdLabel = 'Export sketch to DXF'

_handlers = []

_bDebug = False


def normalize_DXF_path(sPath_Full):
    if os.path.normpath(sPath_Full):
        sPath_DXFFolder, sFileName_DXF = os.path.split(sPath_Full)
        if not os.path.isdir(sPath_DXFFolder):
            sPath_DXFFolder = Ins.sPath_DXF_folder_Default
    else:
        sPath_DXFFolder = Ins.sPath_DXF_folder_Default
        sFileName_DXF = Ins.sFileName_DXF_Default
        # Ins.sPath_DXFFolder = os.path.expanduser('~\\Desktop')
        # return os.path.join(sPath_Folder, Ins.sFileName_DXF_Default)

    Ins.sPath_DXF_Full = os.path.join(sPath_DXFFolder, sFileName_DXF)
    

class Ins:
    """Inputs"""
    
    # Defaults before the values in .ini exist.
    sketches = []
    bSketchCoords = True
    bIncludePointsWithConnectedEntities = False
    bIncludeNonDeletablePoints = False
    bIncludeConstructionCurves = False
    bIncludeLinked = False
    bIncludeRef = False
    sPath_DXF_folder_Default = os.path.expanduser('~\\Desktop')
    sFileName_DXF_Default = "from_Fusion_sketch.dxf"
    sPath_DXF_Full = os.path.join(
        sPath_DXF_folder_Default, sFileName_DXF_Default)


def _log(*printMe):
    if len(printMe) == 0:
        _app.log(" ")
        return
    _app.log(" ".join(str(_) for _ in printMe))


def _onFail():
    import traceback
    sFail = 'Failed:\n{}'.format(traceback.format_exc())
    _log(sFail)
    _ui.messageBox(sFail)


_s_inifile = os.path.join(os.path.dirname(os.path.realpath(__file__)), _strCmdID+".ini")
_config = configparser.ConfigParser()
_config['DialogOptionValues'] = {}
_configOpts = _config['DialogOptionValues']

_list_DXF_entity_lines = []
_list_DXF_Layers = []


def dxf(s=""):
    """Accumulate strings in list(_list_CodeLines) to eventually be lines in DXF."""
    _list_DXF_entity_lines.append(str(s))


def generate_entity_handle():
    i=4096 # 0x1000
    while True:
        yield hex(i)[2:]
        i += 1

_new_entity_handle = generate_entity_handle()


def build_DXF_code_for_entities(sketch: af.Sketch, bSketchCoords: bool, bIncludePointsWithConnectedEntities: bool, bIncludeNonDeletablePoints: bool, bIncludeConstructionCurves: bool, bIncludeLinked: bool, bIncludeRef: bool):
    """
    Will be using worldGeometry and creating a reference of the sketch's 'CPlane' with 2 lines.
    """

    _unitsMgr = _app.activeProduct.unitsManager
    cm_dlu = _unitsMgr.distanceDisplayUnits == af.DistanceUnits.CentimeterDistanceUnits


    def from_cm_coordinate(x):
        if cm_dlu:
            return x
        return _unitsMgr.convert(
            valueInInputUnits=x,
            inputUnits='cm',
            outputUnits=_unitsMgr.defaultLengthUnits)
    

    def scaleForUnit(a):
        """
        Scale value to defaultLengthUnits.
        Parameters:
            a: float or Point3D
        Returns: Same type as a but scaled from centimeters to defaultLengthUnits.
        """
        if cm_dlu:
            return a
        if isinstance(a, float):
            return from_cm_coordinate(a)
        if isinstance(a, ac.Point3D):
            return ac.Point3D.create(from_cm_coordinate(a.x), from_cm_coordinate(a.y), from_cm_coordinate(a.z))


    def are_Vector3Ds_epsilon_equal(vA: ac.Vector3D, vB=ac.Vector3D.create(0.0,0.0,1.0), epsilon=1e-9):

        if abs(vA.x - vB.x) > epsilon: return False
        if abs(vA.y - vB.y) > epsilon: return False
        return abs(vA.z - vB.z) <= epsilon


    def calculate_arbitrary_X_axis(normal: ac.Vector3D):

        N = normal

        if (abs(N.x) < 1.0/64.0) and (abs(N.y) < 1.0/64.0):
            Wy = ac.Vector3D.create(0.0,1.0,0.0)
            Ax = Wy.crossProduct(N)
        else:
            Wz = ac.Vector3D.create(0.0,0.0,1.0)
            Ax = Wz.crossProduct(N)

        Ax.normalize()

        return Ax


    def create_transformation_for_OCS(normal: ac.Vector3D):

        Az = normal
        Ax = calculate_arbitrary_X_axis(normal)
        Ay = Az.crossProduct(Ax)

        xform = ac.Matrix3D.create()
        xform.setWithArray(
            [
                Ax.x, Ax.y, Ax.z, 0.0,
                Ay.x, Ay.y, Ay.z, 0.0,
                Az.x, Az.y, Az.z, 0.0, 
                0.0, 0.0, 0.0, 1.0
            ])

        return xform


    def calc_arc_start_and_end_angles_per_X_dir(startAngle, endAngle, xDirection: ac.Vector3D):

        angleBetween = referenceVector.angleTo(xDirection)
        # angleTo result is always >= 0.0 and <= pi.
        if _bDebug: sEval="math.degrees(angleBetween)"; _log(sEval+':',eval(sEval))

        if abs(angleBetween) <= 1e-9:
            # Start of arc is on positive side of sketch's X axis.
            if _bDebug: sEval="abs(angleBetween) <= 1e-9"; _log(sEval+':',eval(sEval))
            return startAngle, endAngle
        
        if abs(angleBetween-math.pi) <= 1e-9:
            # Start of arc is on negative side of sketch's X axis.
            if _bDebug: sEval="abs(angleBetween-math.pi) <= 1e-9"; _log(sEval+':',eval(sEval))
            startAngle_Out = math.pi
            endAngle_Out = (math.pi + endAngle) % (2*math.pi)
            return startAngle_Out, endAngle_Out
        
        if _bDebug: sEval="xDirection.crossProduct(referenceVector).angleTo(normal)"; _log(sEval+':',eval(sEval))
        if xDirection.crossProduct(referenceVector).angleTo(normal) < math.pi/4.0:
            startAngle_Out = startAngle + angleBetween
            endAngle_Out = endAngle + angleBetween
        else:
            startAngle_Out = startAngle - angleBetween
            endAngle_Out = endAngle - angleBetween

        # Force angles to be >= 0.0 and < 360.0.
        if (startAngle_Out < 0.0) or (startAngle_Out >= 2*math.pi):
            startAngle_Out %= 2*math.pi
        if (endAngle_Out < 0.0) or (endAngle_Out >= 2*math.pi):
            endAngle_Out %= 2*math.pi
        if _bDebug:
            sEval="math.degrees(startAngle_Out)"; _log(sEval+':',eval(sEval))
            sEval="math.degrees(endAngle_Out)"; _log(sEval+':',eval(sEval))

        return startAngle_Out, endAngle_Out
        

    sLayBase = f"{sketch.parentComponent.name}${sketch.name}"
    sLays = []
    sLay_Normal = f"fromFusion${sLayBase}"; sLays.append(sLay_Normal)
    sLay_Constr = f"fromFusion${sLayBase}$Constr"; sLays.append(sLay_Constr)
    sLay_SkPts_Connected = f"fromFusion${sLayBase}$Pts_with_connected_entities"; sLays.append(sLay_SkPts_Connected)

    dict_sLay_usage = {
        sLay_Normal: 0,
        sLay_Constr: 0,
        sLay_SkPts_Connected: 0,
    }

    def addCodeForSketchPoint(sketchPt: af.SketchPoint):

        if (
            not bIncludePointsWithConnectedEntities and
            sketchPt.connectedEntities
        ):
            return
        
        if (
            not bIncludeNonDeletablePoints and
            not sketchPt.isDeletable
        ):
            return

        if not bIncludeLinked and sketchPt.isLinked:
            return

        if not bIncludeRef and sketchPt.isReference:
            return

        # if _bDebug:
            # sEval="sketchPt.connectedEntities"; _log(sEval+':',eval(sEval))
            # sEval="sketchPt.isDeletable"; _log(sEval+':',eval(sEval))

        # connectedEntities is None for the sketch's origin point and any exclusively added SketchPoints.
        # if sketchPt.connectedEntities is not None:
        #     return
        dxf(' 0')
        dxf('POINT')
        dxf(' 5')
        dxf(next(_new_entity_handle))
        # Not created by Fusion's Save As DXF.
        # dxf(' 330')
        # dxf('A02')
        dxf(' 100')
        dxf('AcDbEntity')
        dxf(' 8')
        if not sketchPt.isDeletable and not sketchPt.connectedEntities:
            # This include the origin point.
            dxf(sLay_Constr)
            dict_sLay_usage[sLay_Constr] += 1
        elif sketchPt.connectedEntities:
            # Center points of circles and arcs.
            dxf(sLay_SkPts_Connected)
            dict_sLay_usage[sLay_SkPts_Connected] += 1
        else:
            # Origin, center of polygons, and exclusively added SketchPoints.
            dxf(sLay_Normal)
            dict_sLay_usage[sLay_Normal] += 1
        # if sketchPt.connectedEntities is None:
        #     # Origin, center of polygons, and exclusively added SketchPoints.
        #     dxf('From_Fusion_sketch')
        # else:
        #     # Center points of circles and arcs.
        #     dxf('From_Fusion_sketch_Pts_with_connected_entities')
        geom = sketchPt.geometry if bSketchCoords else sketchPt.worldGeometry
        dxf(' 100')
        dxf('AcDbPoint')
        dxf(' 10')
        dxf(str(scaleForUnit(geom.x)))
        dxf(' 20')
        dxf(str(scaleForUnit(geom.y)))
        dxf(' 30')
        dxf(str(scaleForUnit(geom.z)))


    def addCodeForNurbsCurve3d(nc: ac.NurbsCurve3D):
        (returnValue, controlPoints, degree, knots, isRational, weights, isPeriodic) = nc.getData()

        if _bDebug: _log()

        dxf(' 0')
        dxf('SPLINE')
        dxf(' 5')
        dxf(next(_new_entity_handle))
        dxf(' 100')
        dxf('AcDbEntity')
        dxf(' 8')
        if sketchCrv.isConstruction:
            dxf(sLay_Constr)
            dict_sLay_usage[sLay_Constr] += 1
        else:
            dxf(sLay_Normal)
            dict_sLay_usage[sLay_Normal] += 1
        dxf(' 100')
        dxf('AcDbSpline')

        # Redetermine value of isRational due to the existence of false positives
        # or when the weight span is neglible.
        isRational = bool(weights and (max(weights) - min(weights)) > 1e-9)

        dxf(' 70')
        i70 = 0
        if nc.isClosed: i70 += 1
        if nc.isPeriodic: i70 += 2
        if nc.isRational: i70 += 4
        # At least, now won't both setting for Planar (8) and Linear (16)
        # Fusion doesn't record Planar. Linear?
        dxf(i70)

        dxf(' 71')
        dxf(degree)

        dxf(' 72')
        dxf(len(knots))

        dxf(' 73')
        dxf(len(controlPoints))

        dxf(' 74')
        dxf(0)

        # 42, 43, & 44, tolerances, taken from Fusion.
        dxf(' 42')
        dxf("0.000000001")
        dxf(' 43')
        dxf("0.0000000001")
        dxf(' 44')
        dxf("0.0000000001")

        for knot in knots:
            dxf(' 40')
            dxf(knot)

        if isRational:
            for weight in weights:
                dxf(' 41')
                dxf(weight)

        for cp in controlPoints:
            dxf(' 10')
            dxf(cp.x)
            dxf(' 20')
            dxf(cp.y)
            dxf(' 30')
            dxf(cp.z)



    # There is always(?) at least one sketch point in each sketch - the read-only one at origin.
    for sketchPt in sketch.sketchPoints:
        addCodeForSketchPoint(sketchPt)


    if sketch.sketchCurves:
        for sketchCrv in sketch.sketchCurves:
            geom = sketchCrv.geometry if bSketchCoords else sketchCrv.worldGeometry

            if not bIncludeConstructionCurves and sketchCrv.isConstruction:
                continue

            if not bIncludeLinked and sketchCrv.isLinked:
                continue

            if not bIncludeRef and sketchCrv.isReference:
                continue

            if isinstance(geom, ac.Arc3D):
                (returnValue, center, normal, referenceVector, radius, startAngle, endAngle) = geom.getData()

                if _bDebug: 
                    _log()
                    sEval="radius"; _log(sEval+':',eval(sEval))
                    sEval="referenceVector.asArray()"; _log(sEval+':',eval(sEval))
           
                    sEval="sketch.xDirection.asArray()"; _log(sEval+':',eval(sEval))
                    # negVector = ac.Vector3D.create(); negVector.subtract(sketch.xDirection)

                    sEval="math.degrees(startAngle)"; _log(sEval+':',eval(sEval))
                    sEval="math.degrees(endAngle)"; _log(sEval+':',eval(sEval))

                dxf(' 0')
                dxf('ARC')
                dxf(' 5')
                dxf(next(_new_entity_handle))
                # dxf(' 330')
                # dxf('A02')
                dxf(' 100')
                dxf('AcDbEntity')
                dxf(' 8')
                if sketchCrv.isConstruction:
                    dxf(sLay_Constr)
                    dict_sLay_usage[sLay_Constr] += 1
                else:
                    dxf(sLay_Normal)
                    dict_sLay_usage[sLay_Normal] += 1
                dxf(' 100')
                dxf('AcDbCircle')

                # Failed because direction of X axis of sketch also needs to be incorporated, and why bother?
                # if are_Vector3Ds_epsilon_equal(normal):
                #     dxf(' 10')
                #     dxf(str(scaleForUnit(center.x)))
                #     dxf(' 20')
                #     dxf(str(scaleForUnit(center.y)))
                #     dxf(' 30')
                #     dxf(str(scaleForUnit(center.z)))
                #     dxf(' 40')
                #     dxf(str(scaleForUnit(radius)))
                #     dxf(' 100')
                #     dxf('AcDbArc')

                #     startAngle_Adj, endAngle_Adj = calc_arc_start_and_end_angles_per_X_dir(startAngle, endAngle, sketch.xDirection)

                #     dxf(' 50')
                #     dxf(str(math.degrees(startAngle_Adj))) # Always <= 0.0 and < 360.0.
                #     dxf(' 51')
                #     dxf(str(math.degrees(endAngle_Adj))) # Always <= 0.0 and < 360.0, regardless of value of '50'.
                #     continue


                xform = create_transformation_for_OCS(normal)

                center.transformBy(xform)

                dxf(' 10')
                dxf(str(scaleForUnit(center.x)))
                dxf(' 20')
                dxf(str(scaleForUnit(center.y)))
                dxf(' 30')
                dxf(str(scaleForUnit(center.z)))
                dxf(' 40')
                dxf(str(scaleForUnit(radius)))

                # 210, 220, & 230 must be before 100 (AcDbArc).
                dxf(' 210')
                dxf(str(scaleForUnit(normal.x)))
                dxf(' 220')
                dxf(str(scaleForUnit(normal.y)))
                dxf(' 230')
                dxf(str(scaleForUnit(normal.z)))

                dxf(' 100')
                dxf('AcDbArc')

                xDirection = calculate_arbitrary_X_axis(normal)

                startAngle_Adj, endAngle_Adj = calc_arc_start_and_end_angles_per_X_dir(startAngle, endAngle, xDirection)

                dxf(' 50')
                dxf(str(math.degrees(startAngle_Adj)))
                dxf(' 51')
                dxf(str(math.degrees(endAngle_Adj)))

                continue


            if isinstance(geom, ac.Circle3D):
                (returnValue, center, normal, radius) = geom.getData()

                dxf(' 0')
                dxf('CIRCLE')
                dxf(' 5')
                dxf(next(_new_entity_handle))
                # dxf('330')
                # dxf('A02')
                dxf(' 100')
                dxf('AcDbEntity')
                dxf(' 8')
                if sketchCrv.isConstruction:
                    dxf(sLay_Constr)
                    dict_sLay_usage[sLay_Constr] += 1
                else:
                    dxf(sLay_Normal)
                    dict_sLay_usage[sLay_Normal] += 1
                dxf(' 100')
                dxf('AcDbCircle')

                if are_Vector3Ds_epsilon_equal(normal):
                    dxf(' 10')
                    dxf(str(scaleForUnit(center.x)))
                    dxf(' 20')
                    dxf(str(scaleForUnit(center.y)))
                    dxf(' 30')
                    dxf(str(scaleForUnit(center.z)))
                    dxf(' 40')
                    dxf(str(scaleForUnit(radius)))
                    continue


                if _bDebug:
                    sEval="center.asArray()"; _log(sEval+':',eval(sEval))
                    sEval="normal.asArray()"; _log(sEval+':',eval(sEval))

                # geom = sketchCrv.worldGeometry
                # (returnValue, center, normal, radius) = geom.getData()
                # sEval="center.asArray()"; _log(sEval+':',eval(sEval))
                # sEval="normal.asArray()"; _log(sEval+':',eval(sEval))

                # zDir_Sketch = sketch.xDirection.crossProduct(sketch.yDirection)
                # _log(zDir_Sketch.asArray())
                #xform.setToRotateTo(zDir_Sketch, normal)
                # _log(xform.setToRotateTo(normal, zDir_Sketch))
                # xform.setToRotateTo(normal, zDir_Sketch)

                xform = create_transformation_for_OCS(normal)
                if _bDebug:
                    sEval="xform.asArray()"; _log(sEval+':',eval(sEval))
                    sEval="center.transformBy(xform)"; _log(sEval+':',eval(sEval))
                    sEval="center.asArray()"; _log(sEval+':',eval(sEval))

                dxf(' 10')
                dxf(str(scaleForUnit(center.x)))
                dxf(' 20')
                dxf(str(scaleForUnit(center.y)))
                dxf(' 30')
                dxf(str(scaleForUnit(center.z)))
                dxf(' 40')
                dxf(str(scaleForUnit(radius)))

                dxf(' 210')
                dxf(str(scaleForUnit(normal.x)))
                dxf(' 220')
                dxf(str(scaleForUnit(normal.y)))
                dxf(' 230')
                dxf(str(scaleForUnit(normal.z)))

                continue


            if isinstance(geom, ac.Line3D):
                (returnValue, startPoint, endPoint) = geom.getData()
                dxf(' 0')
                dxf('LINE')
                dxf(' 5')
                dxf(next(_new_entity_handle))
                # dxf('330')
                # dxf('A02')
                dxf(' 100')
                dxf('AcDbEntity')
                dxf(' 8')
                if sketchCrv.isConstruction:
                    dxf(sLay_Constr)
                    dict_sLay_usage[sLay_Constr] += 1
                else:
                    dxf(sLay_Normal)
                    dict_sLay_usage[sLay_Normal] += 1
                dxf(' 100')
                dxf('AcDbLine')
                dxf(' 10')
                dxf(str(scaleForUnit(startPoint.x)))
                dxf(' 20')
                dxf(str(scaleForUnit(startPoint.y)))
                dxf(' 30')
                dxf(str(scaleForUnit(startPoint.z)))
                dxf(' 11')
                dxf(str(scaleForUnit(endPoint.x)))
                dxf(' 21')
                dxf(str(scaleForUnit(endPoint.y)))
                dxf(' 31')
                dxf(str(scaleForUnit(endPoint.z)))
                continue


            if isinstance(geom, ac.NurbsCurve3D):
                addCodeForNurbsCurve3d(geom)
                continue

            if isinstance(geom, (ac.Ellipse3D, ac.EllipticalArc3D)):
                addCodeForNurbsCurve3d(geom.asNurbsCurve)
                continue

            _log(f"Geometry type, {geom}, not supported.")


    list_sLayers_Out = [_ for _ in sLays if dict_sLay_usage[_] > 0]

    return list_sLayers_Out


def generate_layer_handle():
    i=80 # 0x50
    while True:
        yield hex(i)[2:]
        i += 1

_new_lay_handle = generate_layer_handle()

def build_DXF_code_for_layers(sLays):

    for sLay in sLays:
        _list_DXF_Layers.append(" 0")
        _list_DXF_Layers.append("LAYER")
        _list_DXF_Layers.append(" 5")
        _list_DXF_Layers.append(next(_new_lay_handle))
        _list_DXF_Layers.append(" 100")
        _list_DXF_Layers.append("AcDbSymbolTableRecord")
        _list_DXF_Layers.append(" 100")
        _list_DXF_Layers.append("AcDbLayerTableRecord")
        _list_DXF_Layers.append(" 2")
        _list_DXF_Layers.append(sLay)
        _list_DXF_Layers.append(" 70")
        _list_DXF_Layers.append("0")
        _list_DXF_Layers.append(" 62")
        _list_DXF_Layers.append("7")
        _list_DXF_Layers.append(" 6")
        _list_DXF_Layers.append("CONTINUOUS")


def get_INSUNITS_value():
    distanceUnits = _app.activeProduct.unitsManager.distanceDisplayUnits
    return (4, 5, 6, 1, 2)[distanceUnits]


def main(sketches: list[af.Sketch]|None, bSketchCoords: bool|None, bIncludePointsWithConnectedEntities: bool|None, bIncludeNonDeletablePoints: bool|None, bIncludeConstructionCurves: bool|None, bIncludeLinked: bool|None, bIncludeRef: bool|None):
    
    if sketches is None: sketches = Ins.sketches
    if bSketchCoords is None: bSketchCoords = Ins.bSketchCoords
    if bIncludePointsWithConnectedEntities is None: bIncludePointsWithConnectedEntities = Ins.bIncludePointsWithConnectedEntities
    if bIncludeNonDeletablePoints is None: bIncludeNonDeletablePoints = Ins.bIncludeNonDeletablePoints
    if bIncludeConstructionCurves is None: bIncludeConstructionCurves = Ins.bIncludeConstructionCurves
    if bIncludeLinked is None: bIncludeLinked = Ins.bIncludeLinked
    if bIncludeRef is None: bIncludeRef = Ins.bInclbIncludeRefudeConstructionCurves

    sLays_toBuild = []

    for sketch in sketches:

        sLays_toAdd = build_DXF_code_for_entities(
            sketch,
            bSketchCoords=bSketchCoords,
            bIncludePointsWithConnectedEntities=bIncludePointsWithConnectedEntities,
            bIncludeNonDeletablePoints=bIncludeNonDeletablePoints,
            bIncludeConstructionCurves=bIncludeConstructionCurves,
            bIncludeLinked=bIncludeLinked,
            bIncludeRef=bIncludeRef,
            )
        
        if not sLays_toAdd: return

        sLays_toBuild.extend([_ for _ in sLays_toAdd if _ not in sLays_toBuild])

    build_DXF_code_for_layers(sLays_toBuild)


    with open(os.path.join(os.path.dirname(__file__), "spb_Export_sketch_to_DXF-template.txt"), 'r') as fIn:
        sDXFCode = fIn.read()

        sPath_Out = Ins.sPath_DXF_Full

        with open(sPath_Out, 'w') as fOut:
            fOut.write(sDXFCode.format(
                insunits=get_INSUNITS_value(),
                layers="\n".join(_list_DXF_Layers),
                entities="\n".join(_list_DXF_entity_lines),
                ))
            fOut.write('\n')

            _log('"{}" was created/overwritten.'.format(sPath_Out))
            
        _log("^"*40, "\nEnd of script.")


class MyCommand_Execute_Handler(ac.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            command: ac.Command = args.firingEvent.sender
            inputs = command.commandInputs
         
            if _bDebug: _log(inputs.itemById('sketches').selectionCount)
            #Ins.sketches = [inputs.itemById('sketches').selection(_).entity for _ in range(inputs.itemById('sketches').selectionCount)]
            bSketchCoords = inputs.itemById('bSketchCoords').value
            bIncludePointsWithConnectedEntities = inputs.itemById('bIncludePointsWithConnectedEntities').value
            bIncludeNonDeletablePoints = inputs.itemById('bIncludeNonDeletablePoints').value
            bIncludeConstructionCurves = inputs.itemById('bIncludeConstructionCurves').value
            bIncludeLinked = inputs.itemById('bIncludeLinked').value
            bIncludeRef = inputs.itemById('bIncludeRef').value

            # _configOpts['bSketchCoords'] = str(bSketchCoords)


            if _bDebug: _log(Ins.sketches)

            with open(_s_inifile, 'w') as configfile:
                _config.write(configfile)


            main(
                sketches=Ins.sketches,
                bSketchCoords=bSketchCoords,
                bIncludePointsWithConnectedEntities=bIncludePointsWithConnectedEntities,
                bIncludeNonDeletablePoints=bIncludeNonDeletablePoints,
                bIncludeConstructionCurves=bIncludeConstructionCurves,
                bIncludeLinked=bIncludeLinked,
                bIncludeRef=bIncludeRef,
                )
            
            # Force the termination of the command.
            adsk.terminate()

        except:
            _onFail()


class MyCommand_InputChanged_Handler(ac.InputChangedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            eventArgs = ac.InputChangedEventArgs.cast(args)
            inputs = eventArgs.inputs
            cmdInput = eventArgs.input
            if _bDebug: sEval="cmdInput.id"; _log(sEval+':',eval(sEval))
            if cmdInput.id == 'sketches':
                Ins.sketches = [inputs.itemById('sketches').selection(_).entity for _ in range(inputs.itemById('sketches').selectionCount)]
                #_log(len(Ins.sketches))
                # try:
                #     Ins.sketches = inputs.itemById('sketches').selection(0).entity
                # except:
                #     # Handles when sketches is unselected.
                #     Ins.sketches = None
            elif cmdInput.id == 'bSketchCoords':
                Ins.bSketchCoords = inputs.itemById('bSketchCoords').value
                _configOpts['bSketchCoords'] = str(Ins.bSketchCoords)
                with open(_s_inifile, 'w') as configfile:
                    _config.write(configfile)
            elif cmdInput.id == 'bIncludePointsWithConnectedEntities':
                Ins.bIncludePointsWithConnectedEntities = inputs.itemById('bIncludePointsWithConnectedEntities').value
                _configOpts['bIncludePointsWithConnectedEntities'] = str(Ins.bIncludePointsWithConnectedEntities)
                with open(_s_inifile, 'w') as configfile:
                    _config.write(configfile)
            elif cmdInput.id == 'bIncludeNonDeletablePoints':
                Ins.bIncludeNonDeletablePoints = inputs.itemById('bIncludeNonDeletablePoints').value
                _configOpts['bIncludeNonDeletablePoints'] = str(Ins.bIncludeNonDeletablePoints)
                with open(_s_inifile, 'w') as configfile:
                    _config.write(configfile)
            elif cmdInput.id == 'bIncludeConstructionCurves':
                Ins.bIncludeConstructionCurves = inputs.itemById('bIncludeConstructionCurves').value
                _configOpts['bIncludeConstructionCurves'] = str(Ins.bIncludeConstructionCurves)
                with open(_s_inifile, 'w') as configfile:
                    _config.write(configfile)
            elif cmdInput.id == 'bIncludeLinked':
                Ins.bIncludeLinked = inputs.itemById('bIncludeLinked').value
                _configOpts['bIncludeLinked'] = str(Ins.bIncludeLinked)
                with open(_s_inifile, 'w') as configfile:
                    _config.write(configfile)
            elif cmdInput.id == 'bIncludeRef':
                Ins.bIncludeRef = inputs.itemById('bIncludeRef').value
                _configOpts['bIncludeRef'] = str(Ins.bIncludeRef)
                with open(_s_inifile, 'w') as configfile:
                    _config.write(configfile)
            elif cmdInput.id == 'sv_FilePath':
                sPath_DXF_Full_Input = inputs.itemById('sv_FilePath').value
                normalize_DXF_path(sPath_DXF_Full_Input)
                if _bDebug: sEval="Ins.sPath_DXF_Full"; _log(sEval+':',eval(sEval))
                inputs.itemById('tb_FilePath').text = Ins.sPath_DXF_Full
                inputs.itemById('sv_FilePath').value = Ins.sPath_DXF_Full
                _configOpts['sPath_DXF_Full'] = str(Ins.sPath_DXF_Full)
                with open(_s_inifile, 'w') as configfile:
                    _config.write(configfile)
            elif cmdInput.id == 'buttonClick':
                fileDlg = _ui.createFileDialog()
                # fileDlg.isMultiSelectEnabled = True
                fileDlg.title = 'Save DXF File Dialog'
                fileDlg.filter = 'DXF files (*.dxf)'
                if _bDebug: sEval="Ins.sPath_DXF_Full"; _log(sEval+':',eval(sEval))
                fileDlg.initialDirectory = os.path.dirname(Ins.sPath_DXF_Full)
                if _bDebug: sEval="fileDlg.initialDirectory"; _log(sEval+':',eval(sEval))
                if _bDebug: sEval="Ins.sPath_DXF_Full"; _log(sEval+':',eval(sEval))
                fileDlg.initialFilename = os.path.split(Ins.sPath_DXF_Full)[1] #Ins.sPath_DXF_Full
                if _bDebug: sEval="fileDlg.initialFilename"; _log(sEval+':',eval(sEval))
                dlgResult = fileDlg.showSave()
                if dlgResult != ac.DialogResults.DialogOK: return
                
                if _bDebug: sEval="fileDlg.filename"; _log(sEval+':',eval(sEval))
                normalize_DXF_path(fileDlg.filename)
                inputs.itemById('tb_FilePath').text = Ins.sPath_DXF_Full
                inputs.itemById('sv_FilePath').value = Ins.sPath_DXF_Full
                _configOpts['sPath_DXF_Full'] = str(Ins.sPath_DXF_Full)
                with open(_s_inifile, 'w') as configfile:
                    _config.write(configfile)
        except:
            _onFail()


class MyCommand_Destroy_Handler(ac.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            # when the command is done, terminate the script
            # this will release all globals which will remove all event handlers
            adsk.terminate()
        except:
            _onFail()


class MyCommand_Created_Handler(ac.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
 
            cmd = ac.Command.cast(args.command)
    
            # Connect up to the command executed event.
            onExecute = MyCommand_Execute_Handler()
            cmd.execute.add(onExecute)
            _handlers.append(onExecute)

            onDestroy = MyCommand_Destroy_Handler()
            cmd.destroy.add(onDestroy)
            _handlers.append(onDestroy)

            onInputChanged = MyCommand_InputChanged_Handler()
            cmd.inputChanged.add(onInputChanged)
            _handlers.append(onInputChanged)   

            try:
                _config.read(_s_inifile)
                # All are == 'True' regardless of intended default value.
                Ins.bSketchCoords = (_configOpts['bSketchCoords'] == 'True')
                Ins.bIncludePointsWithConnectedEntities = (_configOpts['bIncludePointsWithConnectedEntities'] == 'True')
                Ins.bIncludeNonDeletablePoints = (_configOpts['bIncludeNonDeletablePoints'] == 'True')
                Ins.bIncludeConstructionCurves = (_configOpts['bIncludeConstructionCurves'] == 'True')
                Ins.bIncludeLinked = (_configOpts['bIncludeLinked'] == 'True')
                Ins.bIncludeRef = (_configOpts['bIncludeRef'] == 'True')
                Ins.sPath_DXF_Full = _configOpts['sPath_DXF_Full']
                normalize_DXF_path(Ins.sPath_DXF_Full)
            except:
                # _onFail()
                _configOpts['bSketchCoords'] = str(Ins.bSketchCoords)
                _configOpts['bIncludePointsWithConnectedEntities'] = str(Ins.bIncludePointsWithConnectedEntities)
                _configOpts['bIncludeNonDeletablePoints'] = str(Ins.bIncludeNonDeletablePoints)
                _configOpts['bIncludeConstructionCurves'] = str(Ins.bIncludeConstructionCurves)
                _configOpts['bIncludeLinked'] = str(Ins.bIncludeLinked)
                _configOpts['bIncludeRef'] = str(Ins.bIncludeRef)
                _configOpts['sPath_DXF_Full'] = Ins.sPath_DXF_Full

                with open(_s_inifile, 'w') as configfile:
                    _config.write(configfile)


            inputs = cmd.commandInputs

            unitsMgr = _app.activeProduct.unitsManager


            id = 'sketches'
            selectInput = inputs.addSelectionInput(
                id='sketches',
                name="Sketches",
                commandPrompt="Select sketches to export")
            selectInput.addSelectionFilter(ac.SelectionCommandInput.Sketches)
            selectInput.setSelectionLimits(minimum=1, maximum=0)

            inputs.addBoolValueInput(
                'bSketchCoords',
                "Transform sketch coordinates to World XY plane",
                True,
                "",
                Ins.bSketchCoords)

            inputs.addBoolValueInput(
                'bIncludePointsWithConnectedEntities',
                "Include points that have connected entities",
                True,
                "",
                Ins.bIncludePointsWithConnectedEntities)

            inputs.addBoolValueInput(
                'bIncludeNonDeletablePoints',
                "Include points that are not directly deletable",
                True,
                "",
                Ins.bIncludeNonDeletablePoints)

            inputs.addBoolValueInput(
                'bIncludeConstructionCurves',
                "Include construction curves",
                True,
                "",
                Ins.bIncludeConstructionCurves)

            inputs.addBoolValueInput(
                'bIncludeLinked',
                "Include linked points and curves",
                True,
                "",
                Ins.bIncludeLinked)

            inputs.addBoolValueInput(
                'bIncludeRef',
                "Include reference points and curves",
                True,
                "",
                Ins.bIncludeRef)

            inputs.addStringValueInput(
                'sv_FilePath',
                "File path edit",
                Ins.sPath_DXF_Full,
                )
            
            inputs.addBoolValueInput('buttonClick', "SaveAs Dialog", False, '', False)

            inputs.addTextBoxCommandInput(
                'tb_FilePath',
                "File path that will be used",
                Ins.sPath_DXF_Full,
                1,
                True)
            
        except:
            _onFail()


def run(context):
    try:
        _log("\nStart of script.\n", "V"*40, )

        if _ui.commandDefinitions.itemById(_strCmdID):
            _ui.commandDefinitions.itemById(_strCmdID).deleteMe()            
        cmdDef = _ui.commandDefinitions.addButtonDefinition(_strCmdID, _strCmdLabel, '', '')

        onCommandCreated = MyCommand_Created_Handler()
        cmdDef.commandCreated.add(onCommandCreated)
        _handlers.append(onCommandCreated)

        cmdDef.execute()

        adsk.autoTerminate(False) # Otherwise, dialog doesn't even display.

    except:
        _onFail()
        _log("^"*40, "\nEnd of script.")
