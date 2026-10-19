import machinekit.component.ComponentValues;
import machinekit.structural.MineralFoundation;
import machinekit.structural.CastInsert;
import machinekit.structural.CastingFeatures;
import machinekit.tooling.FoundationToolingAdapter;
import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingDocument;
import materia.tooling.ToolingCodec;
import materia.tooling.panel.PanelJoints;

/** Independent casting study; dimensions come from this foundation instance via the adapter. */
class SinglePocketSetup {
	public static function foundation(bedCore:Bool = false):MineralFoundation
		return new MineralFoundation(new ComponentValues().setNumber("width", 240)
			.setNumber("depth", 180)
			.setNumber("height", 100)
			.setToken("voids", CastingFeatures.encode([CastingFeatures.box("pocket", 100, 80, 61, 0, 0, bedCore ? 40 : -1)])),
			[
				{
					id: "seat",
					part: new CastInsert(new ComponentValues().setNumber("width", 40)
						.setNumber("depth", 35)
						.setNumber("height", 15)
						.setNumber("boreDiameter", 5)),
					pose: F.translation(80, 0, 85)
				}
			]);

	public static function document(thickness:Float = 18, quarterTurn:Bool = false, bedCore:Bool = false):ToolingDocument {
		var input = FoundationToolingAdapter.snapshot(foundation(bedCore), bedCore ? "single-pocket-bed-study" : "single-pocket-study", 1),
			stock = PanelJoints.stock(thickness);
		stock.materialSpec = "coated particleboard";
		return {
			input: input,
			setup: {
				id: quarterTurn ? "inverted-quarter-turn" : "inverted",
				revision: 1,
				sourceFingerprint: ToolingCodec.inputFingerprint(input),
				fillHeightMm: 100,
				freeboardMm: 0,
				sourceToMold: F.fromRotationMatrix(0, 0, 100, quarterTurn ? [0, -1, 0, -1, 0, 0, 0, 0, -1] : [1, 0, 0, 0, -1, 0, 0, 0, -1]),
				features: [
					{
						featureId: "pocket",
						strategy: SacrificialEPS,
						withdrawal: null
					}
				]
			},
			recipe: {
				id: "box-mold-study",
				revision: 1,
				panel: stock,
				core: {
					materialSpec: "EPS grade pending",
					width: 1000,
					depth: 500,
					height: 100,
					densityKgM3: null,
					compressionStrengthPa: null,
					compressionStrainFraction: null,
					maximumTemperatureC: null,
					evidence: []
				},
				jointLiningAllowance: 0.0,
				jointMachiningAllowance: 0.05,
				coreWireAxis: materia.tooling.ToolingRecipe.CoreWireAxis.Y,
				hardware: {
					insertBoreDiameter: 5,
					insertClearanceDiameter: 7,
					insertScrewDiameter: 6,
					insertThreadSpec: "M6",
					insertEngagement: 8,
					plugDepth: 2,
					plugMaterial: "steel",
					sealMargin: 3,
					sealMaterial: "pending compatible sealing compound",
					minimumBridgeWidth: 80,
					locatingPinDiameter: 6,
					supportScrewDiameter: 6,
					supportClearanceDiameter: 7,
					secondaryPinTravel: 2,
					padDepth: 54,
					labelOffset: 8
				},
				jointClearance: 0.2,
				linerThickness: 0.1,
				routerDiameter: 6,
				minimumLigament: 2,
				resinSpec: "pending resin selection",
				releaseSpec: "pending nonbonding seal and covering",
				adhesiveSpec: "pending EPS compatible adhesive",
				processEvidence: []
			}
		};
	}
}
