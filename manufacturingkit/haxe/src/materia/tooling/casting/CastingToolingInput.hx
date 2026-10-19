package materia.tooling.casting;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.tooling.ToolingPrism;
import materia.tooling.ToolingEnums.ToolingLengthUnit;
import materia.tooling.ToolingEnums.InsertRole;
import materia.tooling.ToolingEnums.InsertFinish;

enum abstract CastingFeatureKind(String) to String {
	var Void = "void";
	var InsertRecess = "insert-recess";
}

/** Unclipped design cut, centered in X/Y with its bottom at local Z = 0.
 * Extensions outside the envelope are intentional and must be clipped by planning.
 */
typedef CastingToolingFeature = {
	var id:String;
	var kind:CastingFeatureKind;
	var insertId:Null<String>;
	var width:Float;
	var depth:Float;
	var height:Float;
	var pose:AssemblyFrame;
}

/** Steel box envelope, optionally bored along local +Z; dimensions are in mm.
 * The matching casting recess is the full box, including the bore footprint.
 * Local +Z identifies the mounting-face normal; the face is at local Z = height.
 */
typedef CastingToolingInsert = {
	var id:String;
	var width:Float;
	var depth:Float;
	var height:Float;
	var boreDiameter:Float;
	var pose:AssemblyFrame;
	var role:InsertRole;
	/** Original source label, preserved without using it as a closed vocabulary. */
	var sourceRole:String;
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
	var envelope:Array<ToolingPrism>;
	var features:Array<CastingToolingFeature>;
	var inserts:Array<CastingToolingInsert>;
}
