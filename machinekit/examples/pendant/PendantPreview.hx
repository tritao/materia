import machinekit.assembly.MachineAssembly;
import machinekit.pendant.PendantScene;
import machinekit.pendant.ReferencePendant;
import materia.project.SceneArtifact;

/** The reference jog pendant: face plate, shell, board envelope and the placed controls. */
class PendantPreview {
  public static function assembly():MachineAssembly return ReferencePendant.assembly();

  /**
   * The scene's plate, shell and board come from their recorded steps, which the source map lists. `savedDocument` is
   * the editor's saved recipe: the case's dimensions, editable on the plate.
   */
  public static function preview(?savedDocument:String):haxe.io.Bytes return SceneArtifact.encode(PendantScene.build(savedDocument));

  public static function main():Void {
    var bytes = preview();
    var root = Sys.getCwd();
    while (!sys.FileSystem.exists(root + "/machinekit/examples/pendant/PendantPreview.hx")) {
      var parent = haxe.io.Path.directory(root);
      if (parent == root || parent.length == 0) throw "Cannot locate pendant example repository";
      root = parent;
    }
    var output = root + "/machinekit/examples/pendant/build";
    sys.FileSystem.createDirectory(output);
    sys.io.File.saveBytes(output + "/pendant.scene", bytes);
    Sys.println("Pendant scene written");
  }
}
