import CadKit;
import cadkit.Shape;

/** One pinned STEP per process, so malformed or very large models can be timed out by the caller. */
class SupplierCadCheck {
    static function vector(v:CadKit.Vec3):Array<Float> return [v.get_x(),v.get_y(),v.get_z()];
    static function main() {
        var path = Sys.getEnv("MATERIA_CAD_FILE"), resultPath = Sys.getEnv("MATERIA_CAD_RESULT");
        if (path == null || resultPath == null) { Sys.println("CAD validation worker ready"); return; }
        var shape = Shape.importStep(path);
        try {
            var bounds = shape.bounds(), low = vector(bounds.get_min()), high = vector(bounds.get_max());
            var solidCount = shape.subshapeCount(CadKit.ShapeKind.Solid);
            var faceCount = shape.subshapeCount(CadKit.ShapeKind.Face);
            var valid = CadKit.shapeValidChecked(shape.borrowHandle()) != 0;
            var invalidSolids = 0;
            for (i in 0...solidCount) {
                var solid = shape.subshape(CadKit.ShapeKind.Solid,i);
                if (CadKit.shapeValidChecked(solid.borrowHandle()) == 0) invalidSolids++;
                solid.close();
            }
            var cylinders:Array<Dynamic> = [], planes:Array<Dynamic> = [];
            // Small models get full analytic-surface evidence. Large assemblies retain bounds/topology.
            if (faceCount <= 2000) for (i in 0...faceCount) {
                var face = shape.subshape(CadKit.ShapeKind.Face,i);
                if (face.surfaceKind() == CadKit.SurfaceKind.Cylinder) {
                    var axis = face.faceAxis();
                    cylinders.push({face:i,radius:axis.get_radius(),origin:vector(axis.get_origin()),
                        direction:vector(axis.get_direction())});
                } else if (face.surfaceKind() == CadKit.SurfaceKind.Plane) {
                    planes.push({face:i,centre:vector(face.center()),normal:vector(face.faceNormal()),area:face.faceArea()});
                }
                face.close();
            }
            var volume = shape.volume();
            var usable = valid && solidCount > 0 && invalidSolids == 0 && volume > 0 && Math.isFinite(volume);
            for (i in 0...3) if (!Math.isFinite(low[i]) || !Math.isFinite(high[i]) || high[i] <= low[i]) usable = false;
            var result:Dynamic = {status:usable ? "valid_solid_geometry" : "geometry_review_required",
                lengthUnit:"mm",low:low,high:high,dimensions:[for(i in 0...3) high[i]-low[i]],
                solids:solidCount,invalidSolids:invalidSolids,faces:faceCount,
                shells:shape.subshapeCount(CadKit.ShapeKind.Shell),valid:valid,volumeMm3:volume,
                cylinders:cylinders,planes:planes,analyticSurfacesComplete:faceCount<=2000,
                mountingFrameStatus:"unverified",massPropertiesStatus:"material_and_density_unverified"};
            sys.io.File.saveContent(resultPath,haxe.Json.stringify(result,null,"  ") + "\n");
        } catch (error:Dynamic) {shape.close();throw error;}
        shape.close();
    }
}
