import CadKit;
import cadkit.Shape;
import cadkit.modeling.Vector;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.BendRule;
import cadkit.sheetmetal.KFactorTable;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetMaterial;
import cadkit.sheetmetal.SheetSolid;
import cadkit.sheetmetal.SheetState;
import cadkit.sheetmetal.WallOutline;

/** Native shape validity, graph-derived volumes and permanent names (SM2). */
class SheetSolidSmoke {
	static var checks:Int = 0;
	static function check(value:Bool, label:String):Void { checks++; if (!value) throw label; }
	static function near(actual:Float, expected:Float, label:String, tolerance:Float = 1e-6):Void {
		check(Math.isFinite(actual) && Math.abs(actual - expected) <= tolerance, label + ": expected " + expected + ", got " + actual);
	}
	public static function run():Int {
		var parts = [SheetDemoParts.p1(), SheetDemoParts.p2(), SheetDemoParts.p3(), SheetDemoParts.p4(), SheetDemoParts.p5()];
		for (sheet in parts) {
			var flat = SheetSolid.build(sheet, sheet.flat());
			var folded = SheetSolid.build(sheet, sheet.folded());
			check(CadKit.shapeValidChecked(flat.borrowHandle()) != 0, "valid flat sheet");
			check(CadKit.shapeValidChecked(folded.borrowHandle()) != 0, "valid folded sheet");
			near(flat.subshapeCount(CadKit.ShapeKind.Solid), 1, "flat pieces fuse into one solid");
			near(folded.subshapeCount(CadKit.ShapeKind.Solid), 1, "folded pieces fuse into one solid");
			var walls = 0.0;
			for (w in sheet.walls()) walls += w.outline.area() * sheet.thickness();
			var flatVolume = walls;
			var foldedVolume = walls;
			for (b in sheet.bends()) {
				var length = sheet.wall(b.wallA).outline.segment(b.edgeA.loop, b.edgeA.segment).length();
				flatVolume += sheet.bendAllowance(b) * length * sheet.thickness();
				foldedVolume += Math.abs(b.angle) * length * sheet.thickness() * (b.radius + sheet.thickness() / 2);
			}
			near(flat.volume(), flatVolume, "flat volume", flatVolume * 1e-6);
			near(folded.volume(), foldedVolume, "folded volume", foldedVolume * 1e-6);
			check(Math.abs(flat.volume() - folded.volume()) > 1, "K != .5 changes flat versus folded volume");
			names(flat); names(folded);
			axes(sheet, folded, sheet.folded());
			var original = sortedNames(folded);
			check(sortedNames(flat) == original, "flat and folded faces have the same graph identities");
			var b = sheet.bends()[0];
			for (parameters in [[b.angle * 0.8, b.radius], [b.angle, b.radius * 1.3]]) {
				var edit = sheet.updateBend(b.id, parameters[0], parameters[1]);
				var changed = SheetSolid.build(sheet, sheet.folded());
				check(sortedNames(changed) == original, "names survive angle or radius edit");
				changed.close();
			}
			var restored = sheet.updateBend(b.id, b.angle, b.radius);
			var state = new SheetState();
			for (bend in sheet.bends()) state.set(bend.id, bend.angle / 2);
			var partial = SheetSolid.build(sheet, state);
			check(CadKit.shapeValidChecked(partial.borrowHandle()) != 0, "valid partial sheet");
			names(partial);
			axes(sheet, partial, state);
			partial.close(); folded.close(); flat.close();
		}
		var sheet = SheetDemoParts.p1();
		var initial = SheetSolid.build(sheet, sheet.folded());
		var edited = sheet.updateWall(2, WallOutline.rectangle(45, 100));
		var changed = SheetSolid.build(sheet, sheet.folded());
		check(sortedNames(initial) == sortedNames(changed), "names survive straight flange length edit");
		initial.close(); changed.close();
		curvesAndNeutralMidplane();
		return checks;
	}
	static function curvesAndNeutralMidplane():Void {
		var sheet = new Sheet(SheetDemoParts.materialA());
		var outline = new WallOutline([Segment.arc(new Point2(0, 0), 10, 0, 2 * Math.PI)],
			[[Segment.arc(new Point2(2, 1), 2, 0, -2 * Math.PI)]]);
		var wall = sheet.addWall(outline);
		var curved = SheetSolid.build(sheet, sheet.flat());
		near(curved.volume(), 96 * Math.PI * 1.5, "circular outline and hole volume");
		names(curved);
		curved.close();
		var semicircle = new WallOutline([Segment.arc(new Point2(0, 0), 10, 0, Math.PI),
			Segment.line(new Point2(-10, 0), new Point2(10, 0))]);
		var replaced = sheet.updateWall(wall.id, semicircle);
		var half = SheetSolid.build(sheet, sheet.flat());
		near(half.volume(), 50 * Math.PI * 1.5, "partial arc outline volume");
		names(half);
		half.close();
		var centered = new Sheet(new SheetMaterial("neutral-midplane", 1.5, BendRule.KFactor(new KFactorTable([0], [0.5]))));
		var base = centered.addWall(WallOutline.rectangle(30, 20));
		var flange = SheetDemoParts.flange(centered, base, 1, 15, Math.PI / 2);
		var flat = SheetSolid.build(centered, centered.flat());
		var folded = SheetSolid.build(centered, centered.folded());
		near(flat.volume(), folded.volume(), "K=.5 preserves flat and folded volume");
		flat.close(); folded.close();
	}
	static function sortedNames(shape:Shape):String {
		var names = shape.elementNames(CadKit.ShapeKind.Face).copy();
		names.sort(function(a:String, b:String) { return Reflect.compare(a, b); });
		return names.join("\n");
	}
	static function names(shape:Shape):Void {
		var names = shape.elementNames(CadKit.ShapeKind.Face);
		var seen:Map<String, Bool> = new Map();
		for (name in names) {
			check(name.length > 0, "every face has a name");
			check(!seen.exists(name), "face names are unique: " + name);
			check(name.indexOf("#") < 0 && name.indexOf("@") < 0 && name.indexOf("~") < 0, "face names are strong: " + name);
			seen.set(name, true);
		}
	}
	static function axes(sheet:Sheet, shape:Shape, state:SheetState):Void {
		var evaluation = sheet.evaluate(state);
		var cylinders = 0;
		for (i in 0...shape.subshapeCount(CadKit.ShapeKind.Face)) {
			var face = shape.subshape(CadKit.ShapeKind.Face, i);
			if (face.surfaceKind() == CadKit.SurfaceKind.Cylinder) {
				cylinders++;
				var axis = face.faceAxis();
				var origin = Vector.fromNative(axis.get_origin());
				var direction = Vector.fromNative(axis.get_direction());
				var matches = 0;
				for (b in sheet.bends()) {
					var zone = evaluation.bend(b.id);
					var radius:Float = cast zone.innerRadius;
					var distance = origin.subtract(zone.axis()).cross(zone.frame.plane.yDirection).length();
					if (distance < 1e-6 && Math.abs(Math.abs(direction.dot(zone.frame.plane.yDirection)) - 1) < 1e-6
						&& (Math.abs(axis.get_radius() - radius) < 1e-6 || Math.abs(axis.get_radius() - radius - sheet.thickness()) < 1e-6)) matches++;
				}
				check(matches == 1, "cylindrical face axis and radius match one evaluated bend");
			} else check(face.surfaceKind() == CadKit.SurfaceKind.Plane, "other faces are planar");
			face.close();
		}
		near(cylinders, 2 * sheet.bends().length, "two cylindrical skins per bend");
	}
}
