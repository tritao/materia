package cadkit.parametric;

import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Conversions between CadKit placements and portable assembly frames. */
class PlacementFrames {
	public static function toAssemblyFrame(placement:Placement):AssemblyFrame {
		if (placement == null) throw "CadKit placement is null";
		var plane = placement.location.plane;
		var x = plane.xDirection, y = plane.yDirection, z = plane.normal;
		return AssemblyFrames.fromRotationMatrix(plane.origin.x, plane.origin.y, plane.origin.z,
			[x.x, y.x, z.x, x.y, y.y, z.y, x.z, y.z, z.z]);
	}

	public static function fromAssemblyFrame(frame:AssemblyFrame):Placement {
		var matrix = AssemblyFrames.toRotationMatrix(frame);
		return new Placement(new Plane(new Vector(frame.x, frame.y, frame.z),
			new Vector(matrix[0], matrix[3], matrix[6]), new Vector(matrix[2], matrix[5], matrix[8])));
	}
}
