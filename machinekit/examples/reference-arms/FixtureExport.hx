import materia.project.SceneArtifact;
import materia.assembly.AssemblyFrames;
import machinekit.assembly.AssemblyPreview;
import machinekit.robotics.DualArmGripperTool;
import machinekit.robotics.RobotFlange;
import robotkit.collision.CollisionClearance;
import machinekit.assembly.MachineAssembly;
import machinekit.pneumatic.AirSupply;
import CncRouter.RouterPlate;
import cadkit.parametric.GeometricConnectors;
import cadbridge.AssemblyPhysicalPartView;
import robotkit.spatial.Vec3;
import robotkit.manipulation.Grasp;

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
    var configPath = ReferenceAssetLocation.DIRECTORY+"/../tending-registration-config.json";
    var config:Dynamic = haxe.Json.parse(sys.io.File.getContent(configPath));
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
    var toolStates:Array<Dynamic> = [];
    for (holding in ["empty","A","B","AB"]) {
      var gripState = new cadkit.modeling.AssemblyState(toolScene.assemblyDefinition);
      for (channel in ["A","B"]) if (holding.indexOf(channel)>=0)
        gripState.setJoint('tool/gripper$channel/jaw-left-slide',(tool.spec.maximumPartWidth()-BenchMill.STOCK_WIDTH)/2);
      toolScene.assemblyState = gripState.record();
      var packed = SceneArtifact.encode(toolScene);
      var name = 'tool-$holding.mtrg';
      sys.io.File.saveBytes(directory+"/"+name,packed);
      toolStates.push({holding:holding,file:name,sha256:digest(packed)});
    }
    toolScene.assemblyState = null;
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
    var stock = new RouterPlate(BenchMill.STOCK_WIDTH,BenchMill.STOCK_DEPTH,BenchMill.STOCK_HEIGHT,"aluminium 6061","Bearing block blank");
    var stockShape = stock.geometry();
    var faces = GeometricConnectors.planarFaces(GeometricConnectors.describeFaces(stockShape.shape));
    stockShape.close();
    var physical = AssemblyPhysicalPartView.fromSceneArtifact(scene);
    var hull:Array<Float> = [for (part in physical.parts) if (part.id=="mill/stock") part.collisionHull][0];
    var grasp = Grasp.find([for (face in faces) {origin:new Vec3(face.origin.x,face.origin.y,face.origin.z),normal:new Vec3(face.normal.x,face.normal.y,face.normal.z)}],
      [for (i in 0...Std.int(hull.length/3)) new Vec3(hull[i*3],hull[i*3+1],hull[i*3+2])],
      new Vec3(0,0,1),tool.spec.minimumPartWidth(),tool.spec.expectedPartWidth,tool.spec.maximumPartWidth(),tool.spec.fingerLength,new Vec3(1,0,0));
    var graspLocal = {x:grasp.contact.x,y:grasp.contact.y,z:grasp.contact.z,
      qx:grasp.toolRotation.x,qy:grasp.toolRotation.y,qz:grasp.toolRotation.z,qw:grasp.toolRotation.w};
    var trays = new MachineAssembly(), seats:Array<Dynamic> = [];
    var pitch:Array<Float> = config.trayPitchMm, centre:Array<Float> = config.trayTableCenterMm;
    var top:Float = config.trayTableTopMm, offset:Float = config.trayBankOffsetMm;
    var rows:Int = config.trayRows,columns:Int = config.trayColumns;
    var bank = new PartTray(rows,columns,BenchMill.STOCK_WIDTH,BenchMill.STOCK_DEPTH,pitch[0],pitch[1]);
    trays.include("table",new TendingCellTable(bank.width,bank.depth,top,offset),AssemblyFrames.translation(centre[0],centre[1],centre[2]));
    for (index in 0...2) {
      var name = index==0 ? "infeed" : "outfeed";
      var placement = AssemblyFrames.translation(centre[0]+(index==0 ? -offset : offset),centre[1],top);
      trays.include(name,new PartTray(rows,columns,BenchMill.STOCK_WIDTH,BenchMill.STOCK_DEPTH,pitch[0],pitch[1]),placement);
      for (row in 0...rows) for (column in 0...columns) seats.push({name:'$name-$row-$column',
        frame:AssemblyFrames.compose(placement,bank.seat(row,column))});
    }
    var trayBytes = SceneArtifact.encode(AssemblyPreview.scene(trays,"candidate-tending-trays"));
    sys.io.File.saveBytes(directory+"/trays.mtrg",trayBytes);
    var production = TendingCyclePlan.bearingBlocks();
    var transfers = [for (request in production.transferRequests()) {station:request.station,tool:request.tool,fromTool:request.fromTool,
      fromPose:request.fromPose,toPose:request.toPose,door:Std.string(request.door),role:Std.string(request.role)}];
    var mass = effector.massPropertiesAtMount();
    var centreZ = AssemblyFrames.transformPoint(AssemblyFrames.fromRotationMatrix(0,0,0,[1,0,0,0,0,-1,0,1,0]),
      mass.centreOfMass.x,mass.centreOfMass.y,mass.centreOfMass.z);
    sys.io.File.saveContent(directory+"/fixtures.json",haxe.Json.stringify({status:"current_fixture_geometry_not_registered_arm",
      lengthUnit:"mm",millArtifactSha256:digest(bytes),toolArtifactSha256:digest(toolBytes),frames:frames,
      loadPosition:mill.loadPosition,doorStroke:mill.doorStroke,
      toolMountBasis:"Existing tending dual gripper with generic 50-4-M6 plate; ZeroErr adapter fit is unverified",
      stockDimensionsMm:[BenchMill.STOCK_WIDTH,BenchMill.STOCK_DEPTH,BenchMill.STOCK_HEIGHT],
      exportBytecodeSha256:digest(sys.io.File.getBytes(Sys.programPath())),
      configSha256:haxe.crypto.Sha256.encode(sys.io.File.getContent(configPath)),
      toolStates:toolStates,trayArtifactSha256:digest(trayBytes),traySeats:seats,graspLocal:graspLocal,transfers:transfers,
      toolMassEstimate:{massKg:mass.mass,centerOfMassAtMountZMm:[centreZ.x,centreZ.y,centreZ.z],
        unaccounted:mass.unaccounted,unaccountedInertia:mass.unaccountedInertia,
        scope:"Existing tool component declared/CAD mass estimates; external air supply excluded; not measured delivered hardware"},
      checks:["Saved fixture/tool frame positions","MachineKit +Y to URDF +Z tool mount normal"],
      scope:"Current CAD mill loading state and tool; proposed native tray layout; open and held jaw states; source production transfers. No arm mission or reach certificate reused."},null,"  "));
    Sys.println("Exported current tending mill and physical dual-tool template to "+directory);
  }
}
