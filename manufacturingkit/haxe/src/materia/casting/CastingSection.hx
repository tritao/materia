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

/** Whole-solid prism proven by the geometry backend, including holes/islands. */
typedef CastingPrismSection = {
	var pose:AssemblyFrame;
	var depth:Float;
	var contours:Array<CastingSectionContour>;
}
