import machinekit.motion.LinearRailBlock;
import machinekit.milling.MillXYStageLayout;
import machinekit.assembly.AssemblyPreview;
import materia.project.SceneArtifact;

/** Explicit supplier-CAD review build; the ordinary mill remains self contained. */
class HybridMillPreview {
  static function main():Void {
    var root = Sys.getCwd();
    while (!sys.FileSystem.exists(root + "/machinekit/examples/linear-rail-reference")) {
      var parent = haxe.io.Path.directory(root);
      if (parent == root || parent == "") throw "Repository not found";
      root = parent;
    }
    var output = root + "/machinekit/examples/linear-rail-reference/build/";
    var block = LinearRailBlock.metric(BenchMill.RAIL, LinearRailBlock.SUPPLIER);
    var reference = new BenchMill(false, false), layout = MillFramePreview.layout(reference);
    var z = MillFramePreview.zLayout(reference, layout, block);
    var xy = new MillXYStageLayout(layout, z.spindleY, BenchMill.RAIL, block);
    var frame = BenchMillFrame.create(layout, z, xy);
    var assembly = MillFramePreview.concept(reference, frame);
    BallScrewDetailChecks.check(assembly, xy, z);
    var count = 0;
    for (row in assembly.components()) if (Std.isOfType(row.component, LinearRailBlock)) {
      var installed:LinearRailBlock = cast row.component;
      if (installed.drawing != LinearRailBlock.SUPPLIER || installed.spec.designation != BenchMill.RAIL)
        throw 'Carriage ${row.id} must use supplier CAD for ${BenchMill.RAIL}, got ${installed.drawing}/${installed.spec.designation}';
      count++;
    }
    if (count != 12) throw 'Expected twelve supplier carriages, got $count';
    sys.io.File.saveBytes(
      output + "mill-hybrid.mtrg",
      SceneArtifact.encode(AssemblyPreview.scene(assembly, "mill-hybrid-rail-reference"))
    );
    Sys.println('Exported hybrid mill with $count supplier carriages: ' + output + "mill-hybrid.mtrg");
  }
}
