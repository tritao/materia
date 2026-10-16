import materia.project.SceneArtifact;
import materia.assembly.AssemblyFrames;
import machinekit.assembly.AssemblyPreview;
import machinekit.robotics.DualArmGripperTool;
import machinekit.robotics.RobotFlange;
import robotkit.collision.CollisionClearance;
import machinekit.assembly.MachineAssembly;
import machinekit.pneumatic.AirSupply;

/** Current tending fixtures/tool only; never imports or regenerates the generic arm certificate. */
class FixtureExport {
  static function digest(bytes:haxe.io.Bytes):String {
    var hash = haxe.crypto.Sha256.make(bytes), result = new StringBuf();
    for (i in 0...hash.length) result.add(StringTools.hex(hash.get(i),2).toLowerCase());
    return result.toString();
  }
  public static function main():Void {
    var directory = ReferenceAssetLocation.DIRECTORY+"/cell-registration";
    sys.FileSystem.createDirectory(directory);
    var mill = new EnclosedBenchMill();
    var scene = AssemblyPreview.scene(mill,"tending-fixture-reference");
    var state = new cadkit.modeling.AssemblyState(scene.assemblyDefinition);
    for (i in 0...3) state.setJoint("mill/"+mill.mill.specs[i].id,mill.loadPosition[i]);
    state.setJoint("door",mill.doorStroke);
    state.setJoint("mill/vise/jaw",0);
    scene.assemblyState = state.record();
    var bytes = SceneArtifact.encode(scene);
    sys.io.File.saveBytes(directory+"/mill.mtrg",bytes);
    var tool = new DualArmGripperTool(32,BenchMill.STOCK_WIDTH,60,16,false,
      BenchMill.STOCK_DEPTH-4*CollisionClearance.MARGIN*1000,6,10);
    var effector = tool.build(new RobotFlange(50));
    effector.mount("plate","robot");
    var assembly = new MachineAssembly();
    assembly.include("tool",effector);
    assembly.addComponent("airSupply",new AirSupply(),AssemblyFrames.translation(1200,-500,0));
    assembly.connectPorts("tool-air","airSupply","air","tool","compressedAir");
    var toolScene = AssemblyPreview.scene(assembly,"tending-dual-tool-template");
    var toolBytes = SceneArtifact.encode(toolScene);
    sys.io.File.saveBytes(directory+"/tool.mtrg",toolBytes);
    var toolState = new cadkit.modeling.AssemblyState(toolScene.assemblyDefinition);
    var mount = toolState.worldConnector("tool/plate","robot");
    // MachineKit mates use +Y normal; imported URDF flanges use +Z normal.
    var mountZ = AssemblyFrames.compose(mount,AssemblyFrames.fromRotationMatrix(0,0,0,[1,0,0,0,0,1,0,-1,0]));
    var normalY = AssemblyFrames.transformVector(mount,0,1,0),normalZ = AssemblyFrames.transformVector(mountZ,0,0,1);
    if (Math.abs(normalY.x-normalZ.x)>1e-10 || Math.abs(normalY.y-normalZ.y)>1e-10 || Math.abs(normalY.z-normalZ.z)>1e-10)
      throw "Tool mount normal conversion differs";
    var savedMill = SceneArtifact.decode(bytes),savedTool = SceneArtifact.decode(toolBytes);
    var millCheck = new cadkit.modeling.AssemblyState(savedMill.assemblyDefinition,savedMill.assemblyState);
    var toolCheck = new cadkit.modeling.AssemblyState(savedTool.assemblyDefinition,savedTool.assemblyState);
    for (name in ["mill/vise/body","mill/stock"]) {
      var expected = state.worldPose(name),actual = millCheck.worldPose(name);
      if (Math.abs(expected.x-actual.x)+Math.abs(expected.y-actual.y)+Math.abs(expected.z-actual.z)>1e-8)
        throw "Saved fixture position differs";
    }
    for (name in ["tool/plate","tool/gripperA/body","tool/gripperB/body"]) {
      var expected = toolState.worldPose(name),actual = toolCheck.worldPose(name);
      if (Math.abs(expected.x-actual.x)+Math.abs(expected.y-actual.y)+Math.abs(expected.z-actual.z)>1e-8)
        throw "Saved tool position differs";
    }
    var frames = {millLoad:state.worldConnector("mill/vise/body","load"),stock:state.worldPose("mill/stock"),
      toolMount:mount,toolMountZ:mountZ,
      toolA:toolState.worldConnector("tool/gripperA/body","tcp"),toolB:toolState.worldConnector("tool/gripperB/body","tcp")};
    sys.io.File.saveContent(directory+"/fixtures.json",haxe.Json.stringify({status:"current_fixture_geometry_not_registered_arm",
      lengthUnit:"mm",millArtifactSha256:digest(bytes),toolArtifactSha256:digest(toolBytes),frames:frames,
      loadPosition:mill.loadPosition,doorStroke:mill.doorStroke,
      toolMountBasis:"Existing tending dual gripper with generic 50-4-M6 plate; ZeroErr adapter fit is unverified",
      stockDimensionsMm:[BenchMill.STOCK_WIDTH,BenchMill.STOCK_DEPTH,BenchMill.STOCK_HEIGHT],
      exportBytecodeSha256:digest(sys.io.File.getBytes(Sys.programPath())),
      checks:["Saved fixture/tool frame positions","MachineKit +Y to URDF +Z tool mount normal"],
      scope:"Current CAD mill at loading axes, door open, jaw open; loaded reference stock retained. No arm, tray layout, mission or reach certificate reused."},null,"  "));
    Sys.println("Exported current tending mill and physical dual-tool template to "+directory);
  }
}
