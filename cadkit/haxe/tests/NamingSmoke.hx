import CadKit;
import cadkit.Shape;

/** Topological names seen from Haxe (plans/TOPOLOGICAL_NAMING.md, TN1): the core's names cross the ABI intact. */
class NamingSmoke {
	public static function run():Void {
		check(Shape.namingScheme() == 1, "naming scheme 1");
		var box = Shape.box(10, 20, 30);
		var faces = box.elementNames(CadKit.ShapeKind.Face);
		check(faces.length == 6 && faces.indexOf("box.+z") >= 0, "box faces are named by role: " + faces.join(","));
		check(box.elementNames(CadKit.ShapeKind.Edge).indexOf("E(box.+x|box.+z)") >= 0, "box edges are named by their faces");
		var stamped = box.stamped("f3", []);
		check(stamped.elementNames(CadKit.ShapeKind.Face).indexOf("f3:box.+z") >= 0, "stamping tags new names");
		var moved = stamped.translate(cadkit.Geometry.vec3(1, 2, 3));
		var again = moved.stamped("f4", [stamped]);
		check(again.elementNames(CadKit.ShapeKind.Face).join(",") == stamped.elementNames(CadKit.ShapeKind.Face).join(","),
			"names carried through a move are not stamped again");
		var line = Shape.fromOwnedHandle(CadKit.lineChecked(cadkit.Geometry.vec3(0, 0, 0), cadkit.Geometry.vec3(1, 0, 0)));
		var named = line.withElementNames(CadKit.ShapeKind.Edge, ["rectangle edge"]);
		check(named.elementName(CadKit.ShapeKind.Edge, 0) == "rectangle%20edge", "seeded ids are escaped");
		// Face descriptors (what an editor without the B-rep matches mates on) carry the names.
		var descriptors = cadkit.parametric.GeometricConnectors.describeFaces(stamped);
		check(descriptors.indexOf('"name":"f3:box.+z"') >= 0, "face descriptors carry names");
		for (shape in [named, line, again, moved, stamped, box])
			shape.close();
	}

	static function check(condition:Bool, message:String):Void {
		if (!condition)
			throw "NamingSmoke: " + message;
	}
}
