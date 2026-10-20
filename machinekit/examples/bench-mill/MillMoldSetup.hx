import machinekit.milling.MillFrame;
import machinekit.milling.MillXYStageLayout;
import machinekit.tooling.FoundationToolingAdapter;
import materia.tooling.casting.CastingToolingInput;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.ToolingDocument;
import materia.tooling.ToolingCodec;
import materia.tooling.panel.PanelJoints;
import materia.assembly.AssemblyFrames as F;

/** Tooling uses the complete frame instantiated by the mechanical preview. */
class MillMoldSetup {
	public static function frame():MillFrame {
		var reference = new BenchMill(false, false),
			layout = MillFramePreview.layout(reference);
		var z = MillFramePreview.zLayout(reference, layout),
			xy = new MillXYStageLayout(layout, z.spindleY, BenchMill.RAIL);
		return new MillFrame(layout, z, xy);
	}

	public static function input(frame:MillFrame):CastingToolingInput
		return FoundationToolingAdapter.snapshot(frame.base, "bench-mill-base", 1);

	public static function document(frame:MillFrame, thickness:Float = 18):ToolingDocument {
		var input = input(frame),
			height = frame.layout.railPlane,
			stock = PanelJoints.stock(thickness);
		stock.materialSpec = "coated particleboard";
		return {
			input: input,
			setup: {
				id: "base-inverted",
				revision: 1,
				sourceFingerprint: ToolingCodec.inputFingerprint(input),
				fillHeightMm: height,
				freeboardMm: 0,
				sourceToMold: F.fromRotationMatrix(0, 0, height, [1, 0, 0, 0, -1, 0, 0, 0, -1]),
				features: [for (id in ["front-y-bearing-seat", "rear-y-drive-seat"])
					if (input.features.filter(feature -> feature.id == id).length == 1) {
					featureId: id, strategy: materia.tooling.casting.CastingSetup.CoreStrategy.ReusableFormer,
					withdrawal: {x: 0, y: 0, z: -1}, priority: 10
				}]
			},
			recipe: {
				featureRules: materia.tooling.casting.CastingStrategies.standardRules(),
				id: "mill-base-panel-eps-review",
				revision: 1,
				panel: stock,
				core: {
					materialSpec: "EPS supplier grade pending",
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
					minimumBridgeWidth: 40,
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
				resinSpec: "pending binder",
				releaseSpec: "pending release and seam system",
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

	public static function options(frame:MillFrame):CastingMoldOptions {
		var input = input(frame);
		return {
			groutedJoints: [
				{
					id: "column-base",
					insertIds: [for (i in input.inserts) if (i.role == "column-mount") i.id],
					gapMm: null,
					jackingReference: null,
					boltSleeveReference: null,
					cureBeforeTorqueEvidence: []
				}
			],
			precision: [
				for (i in input.inserts)
					{
						insertId: i.id,
						route: materia.tooling.casting.CastingMoldPlanner.sourceRoute(i.finish),
						allowance: null,
						masterReference: null,
						masterFlatnessMm: null,
						releaseSpec: null,
						holeProtectionReference: null,
						evidence: []
					}
			],
			compaction: {
				method: Undeclared,
				bedInterfaceReference: null,
				frequencyHz: null,
				amplitudeMm: null,
				durationSeconds: null,
				evidence: []
			},
			anchorage: [
				for (i in input.inserts)
					{
						insertId: i.id,
						kind: Pending,
						geometry: [],
						pulloutN: null,
						shearN: null,
						evidence: []
					}
			],
			serviceOrientation: materia.assembly.AssemblyFrames.identity(),
			overlapRule: ReservedThenPriority,
			liftingEvidence: []
		};
	}
}
