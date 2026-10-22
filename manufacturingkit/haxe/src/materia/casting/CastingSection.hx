package materia.casting;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.casting.CastingProfile.CastingSegment;

typedef CastingSectionContour = {
	var contour:Array<CastingSegment>;
	/** Even depths bound material; odd depths bound holes. All contours are CCW. */
	var depth:Int;
}

/** Exact filled section in a declared plane; shared by drawing and CAM bridges. */
typedef CastingSection = {
	var plane:AssemblyFrame;
	var contours:Array<CastingSectionContour>;
}
