import materia.tooling.export.FabricationSpec;
import camkit.CamToolingPanels.PanelCamSettings;
import toolpathkit.path.Point3;
import toolpathkit.setup.TravelEnvelope;
import toolpathkit.tool.Tool;
import materia.tooling.panel.PanelPlan;

/** Study values, not measured shop qualification. Missing capacities remain explicit. */
class MillFabricationSetup {
	public static function spec():FabricationSpec
		return {
			revision: 1,
			sheetMaterialId: "tooling-coated-board",
			solvent: Unknown,
			adhesiveContainsEPS: null,
			resinReference: "pending binder",
			compatibilityEvidence: [],
			gravityMps2: 9.80665,
			loadFactor: 1.5,
			compactionPressurePa: null,
			compactionEvidence: [],
			plates: [],
			braces: [],
			fasteners: [],
			coreRestraints: [],
			rigidRestraints: [],
			blankMarginMm: 12,
			blankKerfMm: 3.2,
			couponClearancesMm: [0.1, 0.2, 0.3]
		};

	/** Unmeasured supplier/process properties remain null, never nominal resin data. */
	public static function process(recipe:materia.tooling.ToolingRecipe):materia.tooling.process.CastingProcessSpec
		return {
			id: "mill-base-process-review",
			revision: 1,
			recipeFingerprint: materia.tooling.ToolingCodec.recipeFingerprint(recipe),
			initialTemperatureC: null,
			ambientTemperatureC: null,
			temperatureMarginC: null,
			pourRateMm3PerSecond: null,
			maximumAirEscapeWidthMm: null,
			faceContacts: [],
			openFaceContact: null,
			castability: {
				maximumAggregateSizeMm: null,
				minimumPassageAggregateRatio: null,
				maximumSectionThicknessMm: null,
				evidence: []
			},
			cure: {
				reference: recipe.resinSpec,
				resinMassFraction: null,
				heatOfReactionJPerKgResin: null,
				mixSpecificHeatJPerKgK: null,
				mixConductivityWPerMK: null,
				resinMaximumTemperatureC: null,
				gelMaximumTemperatureC: null,
				kinetics: null,
				evidence: []
			},
			sections: [],
			solver: {
				cells: 41,
				timeStepSeconds: 30,
				durationSeconds: 86400,
				maximumExtensionSeconds: 86400,
				convergenceToleranceC: 0.000001,
				maximumIterations: 100,
				remainingHeatToleranceC: 0.01
			}
		};

	public static function cam(panels:Array<PanelPlan>):PanelCamSettings {
		if (panels.length == 0)
			throw "Panel machining requires panels";
		var diameters:Array<Float> = [];
		for (p in panels)
			for (op in p.operations)
				if (diameters.indexOf(op.toolDiameter) < 0)
					diameters.push(op.toolDiameter);
		diameters.sort((a, b) -> Reflect.compare(a, b));
		return {
			chordTolerance: 0.00002,
			arcTolerance: 0.00002,
			blendTolerance: 0.00002,
			cutDirection: Climb,
			spindleSpinUpSeconds: 2,
			diameterMatchTolerance: 0.00005,
			toolMeasurementReference: null,
			singleToolPrograms: true,
			spoilboardPlane: -0.018,
			breakthroughAllowance: 0.001,
			vacuumHold: null,
			onionSkinDepth: 0.0005,
			approachClearance: 0.0005,
			drillPeckRatio: 3,
			drillPeckDepth: 0.003,
			travelSpace: MachineCoordinates,
			workOriginReference: null,
			nominalDiameters: [
				for (i in 0...diameters.length)
					{
						number: i + 1,
						diameter: diameters[i] * 0.001
					}
			],
			safeZ: 0.012,
			spindleRpm: 12000,
			feed: 0.01,
			plungeFeed: 0.002,
			stepDown: 0.003,
			stepOverFraction: 0.4,
			blankMargin: 0.012,
			tools: [
				for (i in 0...diameters.length) new Tool(i + 1, 0, diameters[i] * 0.001, null, 0.025, 0.03, diameters[i] * 0.001)
			],
			fixtures: [],
			travel: new TravelEnvelope(new Point3(-0.1, -0.1, -0.1), new Point3(3, 3, 0.3)),
			workOrigin: new Point3(0, 0, 0),
			workholdingReference: null
		};
	}
}
