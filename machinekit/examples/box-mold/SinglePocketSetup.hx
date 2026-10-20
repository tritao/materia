import machinekit.component.ComponentValues;
import machinekit.structural.MineralFoundation;
import machinekit.structural.CastInsert;
import materia.casting.CastingPatterns;
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
			.setToken("features", CastingPatterns.encode([CastingPatterns.box("pocket",materia.casting.CastingFeature.CastingFeatureKind.Lightening,[CastingPatterns.face(bedCore?"body.z-max":"body.z-min")], 100, 80, 61, 0, 0, bedCore ? 40 : -1)])),
			[
				{
					id: "seat",
					openings: [CastingPatterns.face("body.z-max")],
					part: new CastInsert(new ComponentValues().setToken("role", "guide-seat")
						.setNumber("width", 40)
						.setNumber("depth", 35)
						.setNumber("height", 15)
						.setToken("holes",machinekit.structural.InsertHoles.tappedBlind(machinekit.structural.InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(6).mating()),15))),
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
						priority: 0,
						withdrawal: null
					}
				]
			},
			recipe: {
				featureRules: materia.tooling.casting.CastingStrategies.standardRules(),
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
				adhesiveLineThickness: 0.2,
				mixDensityKgM3: 2400,
				steelDensityKgM3: 7850,
				handlingLoadFactor: 1.5,
				hardware: {
					plugDepth: 5,
					plugMaterial: "steel",
					sealMargin: 3,
					sealMaterial: "pending compatible sealing compound",
					minimumBridgeWidth: 80,
					locatingPinDiameter: 6,
					supportScrewDiameter: 6,
					supportClearanceDiameter: 7,
					secondaryPinTravel: 2,
					padDepth: 54,
					labelOffset: 8,
					datumThickness: 8,
					datumPostWidth: 20,
					formerFlangeThickness: 12,
					formerFlangeMargin: 10,
					fixturePinDiameter: 3,
					fixturePinEmbed: 3,
					fixturePinSpacing: 8,
					fixtureSlotToolDiameter: 3
				},
				jointClearance: 0.2,
				linerThickness: 0.1,
				routerDiameter: 6,
				minimumLigament: 2,
				resinSpec: "pending resin selection",
				releaseSpec: "pending nonbonding seal and covering",
				adhesiveSpec: "pending EPS compatible adhesive",
				processEvidence: [],
				designReview: {
					minimumCornerRadiusMm: null,
					minimumInsertCoverMm: null,
					minimumInsertEdgeMm: null,
					minimumFixingAccessMm: null,
					minimumLiftingPoints: null,
					removalClearanceMm: null
				}
			}
		};
	}
}
