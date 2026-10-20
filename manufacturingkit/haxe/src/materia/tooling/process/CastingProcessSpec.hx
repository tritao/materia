package materia.tooling.process;

enum abstract CureKineticsKind(String) to String {
	var FirstOrderArrhenius = "first-order-arrhenius";
}

typedef CureKinetics = {
	var kind:CureKineticsKind;
	var preExponentialPerSecond:Float;
	var activationEnergyJPerMol:Float;
	var initialConversion:Float;
}

enum abstract ThermalBoundaryKind(String) to String {
	var Insulated = "insulated";
	var Convection = "convection";
	var FixedTemperature = "fixed-temperature";
}

/** Convection coefficient includes any declared panel/covering thermal resistance.
 * No board, EPS or steel conductivity is inferred from its material name.
 */
typedef ThermalBoundary = {
	var kind:ThermalBoundaryKind;
	var temperatureC:Null<Float>;
	var heatTransferWPerM2K:Null<Float>;
	var evidence:Array<String>;
}

typedef CureSection = {
	var id:String;
	var sourceIds:Array<String>;

	/** Exact extracted spans, empty for an unbound authored solver example. */
	var spanIds:Array<String>;

	var geometryFingerprint:Null<String>;

	var thicknessMm:Float;
	var left:ThermalBoundary;
	var right:ThermalBoundary;
	var evidence:Array<String>;
}

/** All physical properties may be unknown. Reaction enthalpy is per kg RESIN;
 * specific heat/conductivity refer to the whole aggregate/resin mix.
 */
typedef ResinCureSpec = {
	var reference:String;
	var resinMassFraction:Null<Float>;
	var heatOfReactionJPerKgResin:Null<Float>;
	var mixSpecificHeatJPerKgK:Null<Float>;
	var mixConductivityWPerMK:Null<Float>;
	var resinMaximumTemperatureC:Null<Float>;
	var kinetics:Null<CureKinetics>;
	var evidence:Array<String>;
}

typedef CureSolverSettings = {
	var cells:Int;
	var timeStepSeconds:Float;
	var durationSeconds:Float;
	var convergenceToleranceC:Float;
	var maximumIterations:Int;
}

/** Recipe-bound screening assumptions, not qualified process capacities.
 * Null limits leave the corresponding rule pending. Evidence records provenance.
 */
typedef CastabilityLimits = {
	var maximumAggregateSizeMm:Null<Float>;
	var minimumPassageAggregateRatio:Null<Float>;
	var maximumSectionThicknessMm:Null<Float>;
	var evidence:Array<String>;
}

/** Versioned analysis inputs, bound to an exact recipe. Kept separate from the
 * existing source/mold and fabrication formats; never silently upgrades them.
 */
typedef CastingProcessSpec = {
	var id:String;
	var revision:Int;
	var recipeFingerprint:String;
	var initialTemperatureC:Null<Float>;
	var ambientTemperatureC:Null<Float>;
	var temperatureMarginC:Null<Float>;
	var pourRateMm3PerSecond:Null<Float>;
	var maximumAirEscapeWidthMm:Null<Float>;
	var castability:CastabilityLimits;
	var cure:ResinCureSpec;
	var sections:Array<CureSection>;
	var solver:CureSolverSettings;
}
