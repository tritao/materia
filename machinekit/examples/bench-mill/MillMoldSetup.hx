import machinekit.milling.MillXYStageLayout;
import machinekit.tooling.FoundationToolingAdapter;
import materia.tooling.casting.CastingToolingInput;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.ToolingDocument;
import materia.tooling.ToolingCodec;
import materia.tooling.panel.PanelJoints;
import materia.assembly.AssemblyFrames as F;

enum abstract MillMoldCasting(String) {
  var Base = "base";
  var Column = "column";
}

/** Tooling uses the complete frame instantiated by the mechanical preview. */
class MillMoldSetup {

  public static function input(frame:BenchMillFrame,
    casting:MillMoldCasting = MillMoldCasting.Base):CastingToolingInput {
    if (casting == MillMoldCasting.Column) {
      var context = machinekit.tooling.FoundationAssemblyContext.snapshot(frame,
        "column/casting", [for (insert in frame.column.insertLocations()) {
        id:insert.id, member:"column/" + insert.id
      }
      ]);
      return FoundationToolingAdapter.snapshot(frame.column, "bench-mill-column", 1, context);
    }
    var context = machinekit.tooling.FoundationAssemblyContext.snapshot(frame,
      "base/casting", [for (insert in frame.base.insertLocations()) {
      id:insert.id, member:"base/" + insert.id
    }
    ]);
    return FoundationToolingAdapter.snapshot(frame.base, "bench-mill-base", 1, context);
  }

  public static function document(
    frame:BenchMillFrame,
    thickness:Float = CastingShopRecipe.BOARD_THICKNESS_MM,
    casting:MillMoldCasting = MillMoldCasting.Base
  ):ToolingDocument {
    if (casting == MillMoldCasting.Column) {
      var document = document(frame, thickness, MillMoldCasting.Base), source = input(frame, MillMoldCasting.Column);
      var depth = frame.layout.columnDepth;
      document.input = source;
      document.setup = {
        id: "column-front-on-bed",
        revision: 1,
        sourceFingerprint: ToolingCodec.inputFingerprint(source),
        fillHeightMm: depth,
        freeboardMm: 0,
        sourceToMold: F.fromRotationMatrix(
          0,
          0,
          depth / 2,
          [
            0,
            0,
            1,
            1,
            0,
            0,
            0,
            1,
            0
          ]
        )
      };
      document.recipe.id = "mill-column-panel-eps-review";
      document.recipe.coreWireAxis = materia.tooling.ToolingRecipe.CoreWireAxis.Z;
      for (rule in document.recipe.featureRules) if (rule.kind == materia.casting.CastingFeature.CastingFeatureKind
        .ServiceOpening) rule.preferredAxis = materia.tooling.casting.CastingStrategies.WithdrawalAxis.Y;
      return document;
    }
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
        sourceToMold: F.fromRotationMatrix(
          0,
          0,
          height,
          [
            1,
            0,
            0,
            0,
            -1,
            0,
            0,
            0,
            -1
          ]
        )
      },
      recipe: recipe
    };
  }

  public static function options(frame:BenchMillFrame,
    casting:MillMoldCasting = MillMoldCasting.Base):CastingMoldOptions {
    var input = input(frame, casting);
    return {replicationJigs: [], foundationJointProcesses: [], precision: [for (i in input.inserts) {
      insertId:i.id, jigReference:null,
        masterReference:null, masterFlatnessMm:null, releaseSpec:null, holeProtectionReference:null, evidence:[]
    }
    ], compaction:CastingShopRecipe.compaction(), serviceOrientation:materia.assembly.AssemblyFrames.identity(),
      overlapRule:ReservedThenPriority, liftingEvidence:[]};
  }
}
