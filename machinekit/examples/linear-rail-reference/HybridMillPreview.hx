import machinekit.motion.HiwinHgh15ca;
import machinekit.milling.MillFrame;
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
        var block = new HiwinHgh15ca(output + "supplier/HGH15CA.step");
        try {
            var reference = new BenchMill(false, false), layout = MillFramePreview.layout(reference);
            var z = MillFramePreview.zLayout(reference, layout, block);
            var xy = new MillXYStageLayout(layout, z.spindleY, BenchMill.RAIL, block);
            var frame = new MillFrame(layout, z, xy);
            var assembly = MillFramePreview.concept(reference, frame);
            var count = assembly.components().filter(row -> row.component == block).length;
            if (count != 12) throw 'Expected twelve supplier carriages, got $count';
            sys.io.File.saveBytes(output + "mill-hybrid.mtrg",
                SceneArtifact.encode(AssemblyPreview.scene(assembly, "mill-hybrid-rail-reference")));
            Sys.println('Exported hybrid mill with $count supplier carriages: ' + output + "mill-hybrid.mtrg");
            block.close();
        } catch (error:Dynamic) { block.close(); throw error; }
    }
}
