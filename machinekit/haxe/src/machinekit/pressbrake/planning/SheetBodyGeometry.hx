package machinekit.pressbrake.planning;

import cadkit.PhysicalProperties;
import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.sheetmetal.OutlineBoolean;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetEvaluation;
import cadkit.sheetmetal.SheetSolid;
import cadkit.sheetmetal.Wall;
import cadkit.sheetmetal.WallOutline;

/** Fixed material patches for rigid simulation bodies; geometry and mass come from the developed stock. */
class SheetBodyGeometry {
	public final id:String;
	public final wall:Int;
	public final bend:Int;
	public final properties:PhysicalProperties;
	public final error:Float;
	final center:Vector;
	final allowance:Float;
	final maximumAngle:Float;
	final solids:Array<ConvexPrism>;
	function new(id:String, wall:Int, bend:Int, outline:WallOutline, thickness:Float,
			allowance:Float, error:Float, curveError:Float, maximumAngle:Float) {
		this.id = id; this.wall = wall; this.bend = bend; this.allowance = allowance;
		this.maximumAngle = maximumAngle;
		var shape = SheetSolid.wallSolid(new Wall(wall, outline), thickness);
		try { properties = shape.massProperties(); shape.close(); }
		catch (failure:Dynamic) { shape.close(); throw failure; }
		if (Math.abs(properties.volume - outline.area() * thickness) > Math.max(1e-8, properties.volume * 1e-9))
			throw new SheetDiagnosis("press.body-volume", "Exact stock volume disagrees with developed area and thickness", [id]);
		center = properties.centerOfMass;
		var profile = SheetPrismProfile.of(id, outline, thickness, curveError);
		this.error = error + profile.curveError;
		solids = [];
		for (piece in profile.pieces()) solids.push(piece.placed(Location.translation(center.scale(-1))));
	}
	public function pieces():Array<ConvexPrism> return solids.copy();

	/** Body axes follow the patch centroid tangent; local collision geometry and inertia stay fixed. */
	public function frame(evaluation:SheetEvaluation):Location {
		if (bend == 0) return evaluation.wall(wall).compose(Location.translation(center));
		var zone = evaluation.bend(bend), fraction = center.x / allowance;
		if (Math.abs(zone.angle) > maximumAngle)
			throw new SheetDiagnosis("press.body-angle", "Actual angle exceeds the rigid patch approximation budget", [id]);
		var angle = zone.angle * fraction;
		return new Location(new Plane(zone.point(fraction, center.y, -center.z),
			zone.frame.direction(new Vector(Math.cos(angle), 0, Math.sin(angle))),
			zone.frame.direction(new Vector(-Math.sin(angle), 0, Math.cos(angle)))));
	}

	/** Tangent-patch error includes curvature, thickness mapping and authored boundary chords. */
	public static function of(sheet:Sheet, maximumAngle:Float = Math.PI,
			maximumError:Float = 0.05):Array<SheetBodyGeometry> {
		if (!Math.isFinite(maximumAngle) || maximumAngle <= 0 || maximumAngle > Math.PI
			|| !Math.isFinite(maximumError) || maximumError <= 0)
			throw new SheetDiagnosis("press.body-resolution", "Rigid patches need an angle in (0, pi] and positive error in mm");
		var thickness = sheet.thickness(), result:Array<SheetBodyGeometry> = [];
		for (wall in sheet.walls()) result.push(new SheetBodyGeometry("wall:" + wall.id, wall.id, 0,
			wall.outline, thickness, 0, 0, maximumError, maximumAngle));
		var developed = sheet.evaluate(sheet.flat());
		for (bend in sheet.bends()) {
			var length = developed.bend(bend.id).allowance;
			var count = 1, error = patchError(length, thickness, maximumAngle, count);
			while (error > maximumError / 2) {
				count *= 2;
				if (count > 1000000) throw new SheetDiagnosis("press.body-resolution", "Rigid bend exceeds the one-million-patch budget", ["bend:" + bend.id]);
				error = patchError(length, thickness, maximumAngle, count);
			}
			var regions = bend.materialRegions(length), width = bend.edgeA.span.length();
			var area = 0.0, expected = 0.0;
			for (region in regions) expected += region.area();
			for (i in 0...count) {
				var left = length * i / count, right = length * (i + 1) / count;
				var clip = WallOutline.polygon([new Point2(left, 0), new Point2(right, 0),
					new Point2(right, width), new Point2(left, width)]);
				for (region in regions) for (patch in OutlineBoolean.intersection(region, clip, "body-strip:" + bend.id + ":" + i)) {
					area += patch.area();
					result.push(new SheetBodyGeometry("bend:" + bend.id + ":patch:" + result.length,
						0, bend.id, patch, thickness, length, error, maximumError / 2, maximumAngle));
				}
			}
			if (Math.abs(area - expected) > Math.max(1e-8, expected * 1e-9))
				throw new SheetDiagnosis("press.body-area", "Rigid patches must conserve all developed bend material", ["bend:" + bend.id]);
		}
		return result;
	}
	static function patchError(length:Float, thickness:Float, angle:Float, count:Int):Float
		return (length * angle + thickness * angle * angle) / (4 * count * count) + thickness * angle / (2 * count);
}
