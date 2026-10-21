package materia.tooling.casting;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.casting.CastingPrism;
import materia.tooling.ToolingEnums.ToolingLengthUnit;
import materia.casting.InsertSpec.InsertRole;
import materia.casting.InsertSpec.InsertFinish;

import materia.casting.CastingFeature;
import materia.casting.InsertHole;

/** Neutral insert exterior and source drilling patterns, in mm.
 * Local +Z identifies the mounting-face normal, at the body's height.
 */
typedef CastingToolingInsert = {
	var id:String;
	var body:materia.casting.InsertSpec.InsertBody;
	var holes:Array<InsertHole>;
	var origin:materia.casting.InsertOrigin;
	var anchorage:materia.casting.InsertAnchorage;
	var interfaces:Array<materia.casting.InterfaceRequirement>;
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
	var datumScheme:Null<materia.casting.FoundationDatumScheme>;
	var interfaceContext:Null<materia.casting.CastingInterfaceContext>;
	var interfaces:Array<materia.casting.InterfaceRequirement>;
	var envelope:Array<CastingPrism>;
	var features:Array<CastingFeature>;
	var inserts:Array<CastingToolingInsert>;
}
