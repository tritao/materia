import cadkit.modeling.Axis;
import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Placement;
import cadkit.parametric.PlacementFrames;
import materia.assembly.AssemblyFrames;

/** Frame conversions must retain rotations on every matrix-to-quaternion branch. */
class PlacementFramesSmoke {
	public static function run():Void {
		var identity = PlacementFrames.toAssemblyFrame(Placement.identity());
		if (identity.x != 0 || identity.y != 0 || identity.z != 0 || identity.qx != 0 ||
			identity.qy != 0 || identity.qz != 0 || identity.qw != 1)
			throw "Identity placement changed during conversion";
		check(Axis.X(), Math.PI);
		check(Axis.Y(), Math.PI);
		check(Axis.Z(), Math.PI);
		check(new Axis(new Vector(), new Vector(1, 2, 3)), 5 * Math.PI / 6);
	}

	static function check(axis:Axis, angle:Float):Void {
		var rotated = Location.rotation(axis, angle).plane;
		var original = new Placement(new Plane(new Vector(7, -8, 9), rotated.xDirection, rotated.normal));
		var frame = PlacementFrames.toAssemblyFrame(original);
		var matrix = AssemblyFrames.toRotationMatrix(frame);
		if (matrix[0] + matrix[4] + matrix[8] >= 0)
			throw "Expected a negative matrix trace";
		var restored = PlacementFrames.fromAssemblyFrame(frame);
		near(restored.location.plane.origin, original.location.plane.origin);
		near(restored.location.plane.xDirection, original.location.plane.xDirection);
		near(restored.location.plane.yDirection, original.location.plane.yDirection);
		near(restored.location.plane.normal, original.location.plane.normal);
		var again = PlacementFrames.toAssemblyFrame(restored);
		var dot = frame.qx * again.qx + frame.qy * again.qy + frame.qz * again.qz + frame.qw * again.qw;
		if (Math.abs(Math.abs(dot) - 1) > 1e-9) throw "Quaternion changed on round trip";
	}

	static function near(actual:Vector, expected:Vector):Void {
		if (Math.abs(actual.x - expected.x) > 1e-9 || Math.abs(actual.y - expected.y) > 1e-9 ||
			Math.abs(actual.z - expected.z) > 1e-9)
			throw "Placement frame changed on round trip";
	}
}
