"""
240209-17: Created.

TODO:
    WIP:
        ...
    Up next:
        Add dialog box to add options (below).
    Add option: Export to World coordinates vs. sketch plane transformation to World XY plane.
    Add option: Project to sketch plane.
    Add option: Skip construction curves.
    Add option: Add point at center of all circles since SketchPt.connectedEntities points are ignored.
                Leave the latter ignored to prevent other points from exporting.
    Add option: Export each sketch to a unique layer.
"""

import adsk.core as ac
import adsk.fusion as af
import os.path
import math


_app = ac.Application.get()
_ui = _app.userInterface
# _des: af.Design = _app.activeDocument.products.itemByProductType("DesignProductType")

_sFileName = "from_Fusion_sketch.dxf"
_sPath_TanslationFolder = os.path.expanduser('~/Desktop/Translations')
if os.path.isdir(_sPath_TanslationFolder):
    _sPath_Out = _sPath_TanslationFolder + "/" + _sFileName
else:
    _sPath_Out = os.path.expanduser('~/Desktop') + "/" + _sFileName


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


_list_DXF_entity_lines = []


def dxf(s=""):
    """Accumulate strings in list(_list_CodeLines) to eventually be lines in DXF."""
    _list_DXF_entity_lines.append(str(s))


def build_DXF_code_for_entities(sketch: af.Sketch):
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
            return from_cm_coordinate(a.x)
        if isinstance(a, ac.Point3D):
            return ac.Point3D.create(from_cm_coordinate(a.x), from_cm_coordinate(a.y), from_cm_coordinate(a.z))


    def generate_entity_handle():
        i=4096 # 0x1000
        while True:
            yield hex(i)[2:]
            i += 1

    new_entity_handle = generate_entity_handle()

    bSketchCoords = True
    bConnectedEntityPts = False


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
        sEval="math.degrees(angleBetween)"; _log(sEval+':',eval(sEval))

        if abs(angleBetween) <= 1e-9:
            # Start of arc is on positive side of sketch's X axis.
            sEval="abs(angleBetween) <= 1e-9"; _log(sEval+':',eval(sEval))
            return startAngle, endAngle
        
        if abs(angleBetween-math.pi) <= 1e-9:
            # Start of arc is on negative side of sketch's X axis.
            sEval="abs(angleBetween-math.pi) <= 1e-9"; _log(sEval+':',eval(sEval))
            startAngle_Out = math.pi
            endAngle_Out = (math.pi + endAngle) % (2*math.pi)
            return startAngle_Out, endAngle_Out
        
        sEval="xDirection.crossProduct(referenceVector).angleTo(normal)"; _log(sEval+':',eval(sEval))
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
        sEval="math.degrees(startAngle_Out)"; _log(sEval+':',eval(sEval))
        sEval="math.degrees(endAngle_Out)"; _log(sEval+':',eval(sEval))

        return startAngle_Out, endAngle_Out
        

    # There is (always? /) at least one sketch point, the read-only one at origin.
    for sketchPt in sketch.sketchPoints:
        if not bConnectedEntityPts and sketchPt.connectedEntities:
            continue
        # connectedEntities is None for the sketch's origin point and any exclusively added SketchPoints.
        # if sketchPt.connectedEntities is not None:
        #     continue
        dxf(' 0')
        dxf('POINT')
        dxf(' 5')
        dxf(next(new_entity_handle))
        # dxf(' 330')
        # dxf('A02')
        dxf(' 100')
        dxf('AcDbEntity')
        dxf(' 8')
        if sketchPt.connectedEntities is None:
            # Origin, center of polygons, and exclusively added SketchPoints.
            dxf('From_Fusion_sketch')
        else:
            # Center points of circles and arcs.
            dxf('From_Fusion_sketch_Pts_with_connected_entities')
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

        _log()

        dxf(' 0')
        dxf('SPLINE')
        dxf(' 5')
        dxf(next(new_entity_handle))
        dxf(' 100')
        dxf('AcDbEntity')
        dxf(' 8')
        if sketchCrv.isConstruction:
            dxf('From_Fusion_sketch_Constr')
        else:
            dxf('From_Fusion_sketch')
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


    if sketch.sketchCurves:
        for sketchCrv in sketch.sketchCurves:
            geom = sketchCrv.geometry if bSketchCoords else sketchCrv.worldGeometry

            if sketchCrv.isConstruction:
                # _log("Skipped isConstruction==True for {}".format(type(geom)))
                continue


            if isinstance(geom, ac.Arc3D):
                (returnValue, center, normal, referenceVector, radius, startAngle, endAngle) = geom.getData()

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
                dxf(next(new_entity_handle))
                # dxf(' 330')
                # dxf('A02')
                dxf(' 100')
                dxf('AcDbEntity')
                dxf(' 8')
                if sketchCrv.isConstruction:
                    dxf('From_Fusion_sketch_Constr')
                else:
                    dxf('From_Fusion_sketch')
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
                    dxf(' 100')
                    dxf('AcDbArc')

                    startAngle_Adj, endAngle_Adj = calc_arc_start_and_end_angles_per_X_dir(startAngle, endAngle, sketch.xDirection)

                    dxf(' 50')
                    dxf(str(math.degrees(startAngle_Adj))) # Always <= 0.0 and < 360.0.
                    dxf(' 51')
                    dxf(str(math.degrees(endAngle_Adj))) # Always <= 0.0 and < 360.0, regardless of value of '50'.
                    continue


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
                dxf(next(new_entity_handle))
                # dxf('330')
                # dxf('A02')
                dxf(' 100')
                dxf('AcDbEntity')
                dxf(' 8')
                if sketchCrv.isConstruction:
                    dxf('From_Fusion_sketch_Constr')
                else:
                    dxf('From_Fusion_sketch')
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
                dxf(next(new_entity_handle))
                # dxf('330')
                # dxf('A02')
                dxf(' 100')
                dxf('AcDbEntity')
                dxf(' 8')
                if sketchCrv.isConstruction:
                    dxf('From_Fusion_sketch_Constr')
                else:
                    dxf('From_Fusion_sketch')
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


def get_INSUNITS_value():
    distanceUnits = _app.activeProduct.unitsManager.distanceDisplayUnits
    return (4, 5, 6, 1, 2)[distanceUnits]


def main():
    while True:
        try:
            sel = _ui.selectEntity(
                "Select a sketch to export",
                filter="Sketches")
        except:
            break
        
        build_DXF_code_for_entities(sel.entity)
        break
        # _log(sel.entity)
        # _log(type(sel))
        # _log(type(sel.entity))

        # sInfo = getGeomInfo(sel.entity)
        # _log(sInfo)
    

    with open(os.path.join(os.path.dirname(__file__), "dxf_template.txt"), 'r') as fIn:
        sDXFCode = fIn.read()

        with open(_sPath_Out, 'w') as fOut:
            fOut.write(sDXFCode.format(
                insunits=get_INSUNITS_value(),
                entities="\n".join(_list_DXF_entity_lines)))
            fOut.write('\n')

            _log('"{}" was created/overwritten.'.format(_sPath_Out))


def run(context):
    try:
        _log("\nStart of script.\n", "V"*40, )
        main()
    except:
        _onFail()
    finally:
        _log("^"*40, "\nEnd of script.")
