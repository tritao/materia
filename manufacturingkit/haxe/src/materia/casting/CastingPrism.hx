package materia.casting;

import materia.assembly.AssemblyRecord.AssemblyFrame;

typedef CastingPoint2 = {var x:Float; var y:Float;}

/** Convex planar envelope / exact line-profile cavity piece. Local +Z extrusion. */
typedef CastingPrism = {var id:String; var profile:Array<CastingPoint2>; var height:Float; var pose:AssemblyFrame;}
