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
		checkDeleted();
		checkScheme();
		for (shape in [named, line, again, moved, stamped, box])
			shape.close();
	}

	/** A reference whose name was made by a feature the document no longer has is `Deleted`, not merely lost. */
	static function checkDeleted():Void {
		var document = new cadkit.parametric.Document();
		var box = document.add(new cadkit.parametric.features.BoxFeature(10, 20, 30));
		var fillet = document.add(new cadkit.parametric.features.FilletFeature(box, 1));
		document.recompute();
		var gone = cadkit.parametric.TopologyFingerprint.fromData(CadKit.ShapeKind.Face, CadKit.SurfaceKind.Plane, CadKit.CurveKind.Unknown,
			1e6, 1e6, 1e6, 0, 0, 1, 1, true, "f99:box.+z");
		var reference = cadkit.parametric.TopologyReference.fromFingerprint(fillet, CadKit.ShapeKind.Face, gone, box);
		check(reference.remap() == cadkit.parametric.ReferenceState.Deleted, "a reference made by a missing feature is Deleted");
		var lost = cadkit.parametric.TopologyFingerprint.fromData(CadKit.ShapeKind.Face, CadKit.SurfaceKind.Plane, CadKit.CurveKind.Unknown,
			1e6, 1e6, 1e6, 0, 0, 1, 1, true, "f1:box.+w");
		var other = cadkit.parametric.TopologyReference.fromFingerprint(fillet, CadKit.ShapeKind.Face, lost, box);
		check(other.remap() == cadkit.parametric.ReferenceState.Unresolved, "a lost element of an existing feature is Unresolved");
		document.close();
	}

	/** A stored name from other naming rules is dropped on load (TN-D13); one from today's rules is kept. */
	static function checkScheme():Void {
		var record:Dynamic = {surface: "plane", curve: "unknown", x: 0, y: 0, z: 0, dx: 0, dy: 0, dz: 1, measure: 1,
			name: "f1:box.+z", naming: Shape.namingScheme()};
		check(cadkit.parametric.DocumentCodec.decodeFingerprint(record, CadKit.ShapeKind.Face).name == "f1:box.+z", "current names load");
		Reflect.setField(record, "naming", Shape.namingScheme() + 1);
		check(cadkit.parametric.DocumentCodec.decodeFingerprint(record, CadKit.ShapeKind.Face).name == null, "other rules' names are dropped");
		Reflect.deleteField(record, "naming");
		check(cadkit.parametric.DocumentCodec.decodeFingerprint(record, CadKit.ShapeKind.Face).name == null, "unversioned names are dropped");
	}

	static function check(condition:Bool, message:String):Void {
		if (!condition)
			throw "NamingSmoke: " + message;
	}
}
