/** Mill entry point; the kit exporter and workshop declarations are shared. */
class MillFabricationPreview {
  public static function main():Void {
    var frame = MillProject.smallFrame();
    write(
      MillMoldSetup.document(frame),
      MillMoldSetup.options(frame),
      frame.base,
      "base",
      "mill-base-process-review",
      (plan, review) -> MillMoldChecks.run(frame, plan, review)
    );
  }

  public static function write(
    document:materia.tooling.ToolingDocument,
    options:materia.tooling.casting.CastingMoldPlan.CastingMoldOptions,
    foundation:machinekit.structural.MineralFoundation,
    stem:String,
    processId:String,
    ?extraChecks:materia.tooling.casting.CastingMoldPlan -> machinekit.tooling.CastingMoldCad.CastingMoldReview -> Void
  ):Void {
    var cwd = Sys.getCwd();
    var root = sys.FileSystem.exists(cwd + "/machinekit") ? cwd + "/machinekit/examples/bench-mill" : cwd;
    CastingShopExport.write(document, options, foundation, stem, processId, root, extraChecks);
  }
}
