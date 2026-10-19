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
				features: [
					for (f in input.features)
						if (f.kind == "void") {
							featureId: f.id,
							strategy: StringTools.startsWith(f.id, "underside-") ? SacrificialEPS : ReusableFormer,
							withdrawal: StringTools.startsWith(f.id, "underside-") ? null : StringTools.startsWith(f.id, "foot-access-") ? {
								x: f.pose.x < 0 ? -1.0 : 1.0,
								y: 0.0,
								z: 0.0
							} : StringTools.startsWith(f.id, "foot-stem-") ? {
								x: 0.0,
								y: 0.0,
								z: 1.0
								} : {x: 0.0, y: 0.0, z: -1.0}
						}
				]
			},
			recipe: {
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
				resinSpec: "pending binder",
				releaseSpec: "pending release and seam system",
				adhesiveSpec: "pending EPS compatible adhesive",
				processEvidence: []
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
						route: i.role == "guide-seat"
						|| i.role == "bearing-seat" ? Replicated : i.role == "column-mount" ? AsCast : Machined,
						allowance: null,
						masterReference: null,
						masterFlatnessMm: null,
						releaseSpec: null,
						damGeometry: [],
						holeProtectionReference: null,
						pourGroup: null,
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
			requiresFlip: true,
			liftingEvidence: []
		};
	}
}
