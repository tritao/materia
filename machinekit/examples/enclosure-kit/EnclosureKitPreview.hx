import machinekit.assembly.MachineAssembly;
import machinekit.assembly.AssemblyPreview;
import machinekit.enclosure.EnclosureFrame;
import machinekit.enclosure.EnclosurePanel;
import machinekit.enclosure.SlidingDoor;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact;

/** Kit geometry example; no machine envelope or containment qualification is implied. */
class EnclosureKitPreview {
  public static function assembly():MachineAssembly {
    var result = new MachineAssembly();
    result.include("frame", new EnclosureFrame(1000, 600, 800));
    var root = AssemblyFrames.translation(-520, -320, -40);
    function panel(id:String, width:Float, pose:AssemblyFrame):Void {
      var part = new EnclosurePanel(width, 800);
      result.addComponent(id, part, pose);
      result.addMemberConnector("frame/post00", id, AssemblyFrames.compose(AssemblyFrames.inverse(root),
        AssemblyFrames.compose(pose, part.connector("mount").frame)));
      result.addMate(id+"-mount", "fixed", "frame/post00", id, id, "mount");
    }
    panel("back", 1000, AssemblyFrames.translation(0, 320, 0));
    panel("left", 600, AssemblyFrames.fromRotationMatrix(-520, 0, 0, [0,-1,0, 1,0,0, 0,0,1]));
    panel("right", 600, AssemblyFrames.fromRotationMatrix(520, 0, 0, [0,-1,0, 1,0,0, 0,0,1]));
    panel("frontLeft", 250, AssemblyFrames.translation(-375, -320, 0));
    panel("frontRight", 250, AssemblyFrames.translation(375, -320, 0));
    var pose = AssemblyFrames.translation(0, -350, 0);
    result.include("door", new SlidingDoor(500, 800), pose);
    result.addMemberConnector("frame/post00", "door", AssemblyFrames.compose(AssemblyFrames.inverse(root), pose));
    result.addMate("door-mount", "fixed", "frame/post00", "door", "door/rail", "mount");
    return result;
  }
  public static function preview():haxe.io.Bytes return SceneArtifact.encode(AssemblyPreview.scene(assembly(), "enclosure-kit"));
  public static function main():Void {
    var bytes = preview();
    var root = Sys.getCwd();
    while (!sys.FileSystem.exists(root + "/machinekit/examples/enclosure-kit/EnclosureKitPreview.hx")) {
      var parent = haxe.io.Path.directory(root);
      if (parent == root || parent.length == 0) throw "Cannot locate enclosure example repository";
      root = parent;
    }
    var output = root + "/machinekit/examples/enclosure-kit/build";
    sys.FileSystem.createDirectory(output);
    sys.io.File.saveBytes(output + "/enclosure.scene", bytes);
    Sys.println("Enclosure kit scene written");
  }
}
