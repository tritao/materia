import cadkit.modeling.Scope;
import cadkit.modeling.Part;
import materia.assembly.AssemblyFrames as F;
import materia.casting.Box3;
import materia.casting.BoxAlgebra;
import materia.tooling.ToolingCodec;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.process.CastingProcessAnalysis;
import machinekit.tooling.CastingMoldCad;
import machinekit.tooling.CastingWallReview;
import machinekit.tooling.ToolingProcessExport;

/** Source-only decision report. No source mutation, kit export or CAM run. */
class CastingAggregateReview {
  static function main():Void {
    var args = Sys.args(), out = args.length == 0 ? "machinekit/examples/casting-shop/build/aggregate-review" : args[0];
    machinekit.tooling.ToolingFiles.directory(out);
    var only = args.length > 1 ? args[1] : null;
    if (only != null && ["base",
      "column", "plinth", "hobber-column"].indexOf(only) < 0) throw "Unknown casting selector: " + only;
    var frame = MillProject.smallFrame();
    var entries = [
      {id: "base", document: MillMoldSetup.document(frame), options: MillMoldSetup.options(frame)},
      {
        id: "column",
        document: MillMoldSetup.document(
          frame,
          CastingShopRecipe.BOARD_THICKNESS_MM,
          MillMoldSetup.MillMoldCasting.Column
        ),
        options: MillMoldSetup.options(
          frame,
          MillMoldSetup.MillMoldCasting.Column
        )
      }
    ];
    if (only == null || only == "plinth") {
      var source = new machinekit.hobbing.MineralCastBed(), document = HobberMoldSetup.document(source);
      entries.push({id: "plinth", document: document, options: HobberMoldSetup.options(document.input)});
    }
    if (only == null || only == "hobber-column") {
      var source = new machinekit.hobbing.MineralCastColumn(), document = HobberMoldSetup.document(
        source,
        HobberMoldSetup.HobberMoldCasting.Column
      );
      entries.push({id: "hobber-column", document: document, options: HobberMoldSetup.options(document.input)});
    }
    var summary:Array<Dynamic> = [], allRows:Array<Array<String>> = [], allWitnesses:Array<Array<String>> = [];
    for (entry in entries) if (only == null || entry.id == only) {
      Sys.println("START aggregate source " + entry.id);
      var plan = CastingMoldCad.plan(entry.document, entry.options), spec = CastingShopRecipe.process(
        plan.document.recipe,
        "aggregate-source-review"
      );
      var fabrication = CastingShopRecipe.fabrication();
      var process = CastingProcessAnalysis.analyze(
        plan,
        spec,
        fabrication.gravityMps2,
        fabrication,
        machinekit.tooling.CastingFillAnalysis.fromPlan,
        CastingWallReview.fromPlan
      );
      var report = process.aggregate, dir = out + "/" + entry.id;
      if (report == null) throw "Missing classified aggregate report";
      machinekit.tooling.ToolingFiles.directory(dir);
      var rows = ToolingProcessExport.writeAggregate(plan, report, dir);
      if (allRows.length == 0) allRows.push(rows.groups[0]);
      for (index in 1...rows.groups.length) allRows.push(rows.groups[index]);
      if (allWitnesses.length == 0) allWitnesses.push(rows.witnesses[0]);
      for (index in 1...rows.witnesses.length) allWitnesses.push(rows.witnesses[index]);
      summary.push({id: entry.id,
        sourceFingerprint: report.sourceFingerprint, failingPairs: report.groups.filter(group -> group.status
        == materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Fail).length,
        classes: [for (kind in materia.tooling.process.AggregateGate.kinds()) {
        kind:kind, pairs:report.groups.filter(group -> group.kind == kind).length,
          failingPairs:report.groups.filter(group -> group.kind == kind
          && group.status == materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Fail).length
      }
      ], validation:report.validation});
      if (entry.id == "base") aft(plan, frame, dir);
      Sys.println("DONE aggregate source " + entry.id + ": " + report.failures.length + " failing locations");
    }
    sys.io.File.saveContent(out + "/summary.json", ToolingCodec.canonical(summary) + "\n");
    sys.io.File.saveContent(out + "/aggregate-groups.csv", materia.tooling.export.ToolingFabrication.csv(allRows));
    sys.io.File.saveContent(
      out + "/aggregate-witnesses.csv",
      materia.tooling.export.ToolingFabrication.csv(allWitnesses)
    );
  }

  static function aft(plan:CastingMoldPlan, frame:BenchMillFrame, dir:String):Void {
    var scope = new Scope();
    try {
      var layout = frame.layout;
      var region:Box3 = {
        x: -layout.baseWidth / 2,
        y: layout.columnFront - layout.baseCentreY,
        z: 0,
        width: layout.baseWidth,
        depth: layout.rear - layout.columnFront,
        height: layout.baseHeight
      };
      var molded = scope.own(CastingMoldCad.source(plan)), inverse = F.inverse(plan.document.setup.sourceToMold);
      var source = scope.own(machinekit.assembly.PosedParts.moved(molded, inverse));
      var clip = scope.own(machinekit.tooling.ToolingBlock.placedBox(region)),
        actual = scope.own(source.intersect(clip));
      var cuts:Array<Box3> = [];
      for (obstacle in CastingWallReview.sourceObstacles(plan, scope)) {
        var b = obstacle.box;
        cuts.push(materia.tooling.casting.CastingMoldPlanner.box(
          b.width,
          b.depth,
          b.height,
          F.compose(
            inverse,
            F.translation(
              b.x + b.width / 2,
              b.y + b.depth / 2,
              b.z
            )
          )
        ));
      }
      // Obstacle bounds only propose a conservative inner certificate. A native
      // Boolean proves it is contained in the actual aft region before credit.
      var cells = BoxAlgebra.coalesce(BoxAlgebra.subtract([region], cuts)), best:Null<Box3> = null, lower = 0.0;
      for (cell in cells) {
        var diameter = Math.min(cell.width, Math.min(cell.depth, cell.height));
        if (diameter > lower) {
          lower = diameter;
          best = cell;
        }
      }
      var difference = 0.0;
      if (best != null) {
        var inner = scope.own(machinekit.tooling.ToolingBlock.placedBox(best)),
          outside = scope.own(inner.subtract(actual));
        difference = outside.volume();
        if (difference > 1e-5) throw "Aft section inner certificate is outside the actual casting";
      }
      var bounds = actual.shape.bounds(), low = bounds.get_min(), high = bounds.get_max();
      var upper = Math.min(high.get_x() - low.get_x(),
        Math.min(high.get_y() - low.get_y(), high.get_z() - low.get_z()));
      var spec = CastingShopRecipe.process(plan.document.recipe, "aft-section-review");
      var decision = CastingProcessAnalysis.sectionLimit(lower, upper, spec.compound.thickness);
      var check = materia.tooling.process.FillAnalysis.check(
        "process.compound.aft-section",
        [plan.document.input.id],
        decision.status,
        "Certified aft inscribed-diameter interval " + lower + ".." + upper + " mm. " + decision.message,
        []
      );
      var record = {
        sourceFingerprint: materia.tooling.casting.CastingMoldPlanner.fingerprint(plan),
        sourceRegion: region,
        actualVolumeMm3: actual.volume(),
        diameterLowerMm: lower,
        diameterUpperMm: upper,
        innerBox: best,
        innerBoxOutsideVolumeMm3: difference,
        limit: materia.tooling.process.CastingProcessSpec.ProcessSpecs.thicknessRecord(spec.compound.thickness),
        validation: check,
        method: "Contained box supplies an inscribed-sphere lower bound; exact outer extents supply its upper bound. No non-box orthogonal/junction result is claimed."
      };
      sys.io.File.saveContent(dir + "/mill-base-aft-section.json", ToolingCodec.canonical(record) + "\n");
      scope.close();
    } catch (error:Dynamic) {
      scope.close();
      throw error;
    }
  }
}
