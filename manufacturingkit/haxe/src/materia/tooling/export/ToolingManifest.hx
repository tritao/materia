package materia.tooling.export;

import materia.tooling.ToolingEnums.ToolingLengthUnit;
import materia.tooling.ToolingValidation;
import materia.tooling.export.ToolingFabrication.ReplacementGroup;

typedef ToolingArtifact = {var path:String; var bytes:Int; var sha256:String;}

/** Portable neutral artifact inventory; file IO and hashing belong to its producer. */
typedef ToolingManifest = {
	var format:String;
	var version:Int;
	var phase:Int;
	var lengthUnit:ToolingLengthUnit;
	var sourceFingerprint:String;
	var fabricationFingerprint:String;
	var packageFingerprint:String;
	var files:Array<ToolingArtifact>;
	var fabrication:FabricationReadiness;
	var validation:Array<ToolingValidation>;
	var replacementGroups:Array<ReplacementGroup>;
}
