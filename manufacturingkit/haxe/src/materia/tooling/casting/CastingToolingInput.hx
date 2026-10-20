package materia.tooling.casting;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.casting.CastingPrism;
import materia.tooling.ToolingEnums.ToolingLengthUnit;
import materia.casting.InsertSpec.InsertRole;
import materia.casting.InsertSpec.InsertFinish;

import materia.casting.CastingFeature;
import materia.casting.InsertHole;

/** Steel box envelope with explicit drilling patterns; dimensions are in mm.
 * The matching casting recess is the full box, including the bore footprint.
 * Local +Z identifies the mounting-face normal; the face is at local Z = height.
 */
typedef CastingToolingInsert = {
	var id:String;
	var width:Float;
	var depth:Float;
	var height:Float;
	var holes:Array<InsertHole>;
	var material:materia.casting.InsertMaterial.InsertMaterialSpec;
	var pose:AssemblyFrame;
	var role:InsertRole;

	/** Original source label, preserved without using it as a closed vocabulary. */
	var label:String;

	var positionTolerance:Float;
	var finish:InsertFinish;
}

/** A detached, machine-independent design snapshot. Right-handed source coordinates.
 * No draft, core strategy, anchorage qualification or finishing allowance is inferred.
 */
typedef CastingToolingInput = {
	var id:String;
	var revision:Int;
	var lengthUnit:ToolingLengthUnit;
	var envelope:Array<CastingPrism>;
	var features:Array<CastingFeature>;
	var inserts:Array<CastingToolingInsert>;
}
