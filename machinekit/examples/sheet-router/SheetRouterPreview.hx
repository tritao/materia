import haxe.io.Bytes;
import machinekit.assembly.AssemblyPreview;
import materia.project.SceneArtifact;

class SheetRouterPreview {
  public static function router():Bytes return SceneArtifact.encode(AssemblyPreview.scene(
    new SheetRouter(),
    "sheet-router",
    256
  ));
  public static function longRouter():Bytes return SceneArtifact.encode(AssemblyPreview.scene(
    new SheetRouter(new SheetRouterSpec(3150)),
    "sheet-router-3150",
    256
  ));

  /** Writes the standard router's scene to `build/sheet-router.mtrg` (the project uses `router` and `longRouter` directly). */
  public static function main():Void {
    var root = Sys.getCwd();
    while (!sys.FileSystem.exists(root + "/machinekit/examples/sheet-router/SheetRouter.hx")) {
      var parent = haxe.io.Path.directory(root);
      if (parent == root || parent.length == 0) throw "Cannot locate the sheet-router example";
      root = parent;
    }
    var output = root + "/machinekit/examples/sheet-router/build";
    sys.FileSystem.createDirectory(output);
    sys.io.File.saveBytes(output + "/sheet-router.mtrg", router());
    Sys.println("Sheet router scene written to " + output + "/sheet-router.mtrg");
  }
}
