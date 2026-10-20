package materia.tooling.export;

enum abstract ResinSolvent(String) to String {
	var Unknown = "unknown";
	var StyreneBearing = "styrene-bearing";
	var StyreneFree = "styrene-free";
}

/** Declared load/material inputs. Evidence is tied to this package's input fingerprint. */
typedef PlateSupport = {
	var panelId:String;
	var spanXmm:Float;
	var spanYmm:Float;
	var youngPa:Float;
	var poisson:Float;
	var allowableStressPa:Float;
	var maximumDeflectionMm:Float;
	var evidence:Array<String>;
}

typedef BraceSupport = {
	var id:String;
	var panelIds:Array<String>;
	var spanMm:Float;
	var tributaryWidthMm:Float;
	var widthMm:Float;
	var depthMm:Float;
	var youngPa:Float;
	var allowableStressPa:Float;
	var maximumDeflectionMm:Float;
	var evidence:Array<String>;
}

typedef FastenerCapacity = {
	var id:String;
	var loadIds:Array<String>;
	var hardwareIds:Array<String>;
	var allowableN:Float;
	var evidence:Array<String>;
}

typedef CoreRestraint = {
	var coreId:String;
	var allowableN:Float;
	var bearingAreaMm2:Float;
	var allowableBearingPa:Float;
	var evidence:Array<String>;
}

typedef RigidToolingRestraint = {
	var memberId:String;
	var upliftCapacityN:Float;
	var lateralXCapacityN:Float;
	var lateralYCapacityN:Float;
	var evidence:Array<String>;
}

typedef FabricationSpec = {
	var revision:Int;
	var sheetMaterialId:String;
	var solvent:ResinSolvent;
	var adhesiveContainsEPS:Null<Bool>;

	var resinReference:String;

	var compatibilityEvidence:Array<String>;
	var gravityMps2:Float;
	var loadFactor:Float;

	/** Additional peak pressure above hydrostatic, not a guessed vibration multiplier. */
	var compactionPressurePa:Null<Float>;

	var compactionEvidence:Array<String>;
	var plates:Array<PlateSupport>;
	var braces:Array<BraceSupport>;
	var fasteners:Array<FastenerCapacity>;
	var coreRestraints:Array<CoreRestraint>;
	var rigidRestraints:Array<RigidToolingRestraint>;
	var blankMarginMm:Float;
	var blankKerfMm:Float;
	var couponClearancesMm:Array<Float>;
}
