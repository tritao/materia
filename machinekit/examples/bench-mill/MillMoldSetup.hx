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
	public static function frame():BenchMillFrame {
		var reference = new BenchMill(false, false),
			layout = MillFramePreview.layout(reference);
		var z = MillFramePreview.zLayout(reference, layout),
			xy = new MillXYStageLayout(layout, z.spindleY, BenchMill.RAIL);
		return BenchMillFrame.create(layout, z, xy);
	}

	public static function input(frame:BenchMillFrame):CastingToolingInput {
		var context=machinekit.tooling.FoundationAssemblyContext.snapshot(frame,"base/casting",
			[for (insert in frame.base.insertLocations()) {id:insert.id,member:"base/"+insert.id}]);
		return FoundationToolingAdapter.snapshot(frame.base,"bench-mill-base",1,context);
	}

	public static function document(frame:BenchMillFrame, thickness:Float = CastingShopRecipe.BOARD_THICKNESS_MM):ToolingDocument {
		var input = input(frame), height = frame.layout.railPlane;
		var recipe = CastingShopRecipe.tooling("mill-base-panel-eps-review", thickness);
		return {
			input: input,
			setup: {
				id: "base-inverted",
				revision: 1,
				sourceFingerprint: ToolingCodec.inputFingerprint(input),
				fillHeightMm: height,
				freeboardMm: 0,
				sourceToMold: F.fromRotationMatrix(0, 0, height, [1, 0, 0, 0, -1, 0, 0, 0, -1]),
				features: []
			},
			recipe: recipe
		};
	}

	public static function options(frame:BenchMillFrame):CastingMoldOptions {
		var input = input(frame);
		return {
			replicationJigs: [],
			foundationJointProcesses: [],
			precision: [
				for (i in input.inserts)
					{
						insertId: i.id,
						jigReference: null,
						masterReference: null,
						masterFlatnessMm: null,
						releaseSpec: null,
						holeProtectionReference: null,
						evidence: []
					}
			],
			compaction: CastingShopRecipe.compaction(),

			serviceOrientation: materia.assembly.AssemblyFrames.identity(),
			overlapRule: ReservedThenPriority,
			liftingEvidence: []
		};
	}
}
