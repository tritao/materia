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
			curePeakC: null,
			cureEvidence: [],
			compatibilityEvidence: [],
			gravityMps2: 9.80665,
			loadFactor: 1.5,
			compactionPressurePa: null,
			compactionEvidence: [],
			plates: [],
			braces: [],
			fasteners: [],
			coreRestraints: [],
			blankMarginMm: 12,
			blankKerfMm: 3.2,
			couponClearancesMm: [0.1, 0.2, 0.3]
		};

	public static function cam(panels:Array<PanelPlan>):PanelCamSettings {
		var diameters:Array<Float> = [];
		for (p in panels)
			for (op in p.operations)
				if (diameters.indexOf(op.toolDiameter) < 0)
					diameters.push(op.toolDiameter);
		diameters.sort((a, b) -> Reflect.compare(a, b));
		return {
			chordTolerance: 0.00002,
			safeZ: 0.012,
			spindleRpm: 12000,
			feed: 0.01,
			plungeFeed: 0.002,
			stepDown: 0.003,
			stepOverFraction: 0.4,
			blankMargin: 0.012,
			tools: [for (i in 0...diameters.length) new Tool(i + 1, 0, diameters[i] * 0.001)],
			fixtures: [],
			travel: new TravelEnvelope(new Point3(-0.1, -0.1, -0.1), new Point3(3, 3, 0.3)),
			workOrigin: new Point3(0, 0, 0),
			workholdingReference: null
		};
	}
}
