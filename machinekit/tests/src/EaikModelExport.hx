import machinekit.assembly.MachineAssembly;
import machinekit.assembly.AssemblyPreview;
import machinekit.robotics.CobotArm;
import cadbridge.AssemblyPhysicalPartView;
import cadbridge.AssemblySimulationBridge;
import robotkit.manipulation.Manipulator;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.model.JointType;

/** Export compiler-derived geometry and independent FK to the native spike. */
class EaikModelExport {
  static var lines:Array<String> = [];
  static function numbers(values:Array<Float>):Void lines.push(values.join(" "));
  static function pose(value:Transform3):Void numbers([value.translation.x,value.translation.y,value.translation.z,
    value.rotation.x,value.rotation.y,value.rotation.z,value.rotation.w]);
  static function fixture(name:String, authored:MachineAssembly, spherical:Bool):Void {
    var scene=materia.project.SceneArtifact.decode(materia.project.SceneArtifact.encode(AssemblyPreview.scene(authored,name)));
    var converted=AssemblySimulationBridge.toRobotModel(cast scene.assemblyDefinition,AssemblyPhysicalPartView.fromSceneArtifact(scene));
    var model=converted.model;
    var flanges=[for(frame in model.frames)if(frame.name=="toolFlange robot flange")frame];
    if(flanges.length!=1)throw "Expected one compiled flange";
    var arm=new Manipulator(model,model.links[0].id,flanges[0].id);
    var origins:Array<Vec3> = [],axes:Array<Vec3> = [];
    var current=Transform3.identity();
    for(joint in arm.pathJoints()){
      var frame=current.compose(Transform3.fromArrays(joint.parentFramePosition,joint.parentFrameRotation));
      switch(joint.type){
        case Revolute,Continuous:
          origins.push(frame.translation);axes.push(frame.transformVector(Vec3.fromArray(joint.axis)).normalized());
        case Fixed:
        case _:throw "EAIK fixture must be 6R";
      }
      current=frame.compose(Transform3.fromArrays(joint.childFramePosition,joint.childFrameRotation).inverse());
    }
    if(axes.length!=6)throw "Expected six compiled axes";
    var zero=arm.tcpPose([for(_ in 0...6)0.0]);
    lines.push(name);
    for(axis in axes)numbers(axis.toArray());
    numbers(origins[0].toArray());
    for(i in 1...6)numbers(origins[i].sub(origins[i-1]).toArray());
    numbers(zero.translation.sub(origins[5]).toArray());
    numbers(zero.rotation.toRotationMatrix());
    numbers([for(i in 0...6)arm.group.limitsOf(i).lower]);
    numbers([for(i in 0...6)arm.group.limitsOf(i).upper]);
    lines.push("0"); // Production fixtures no longer embed a retired solver descriptor.
    lines.push("1005");
    for(sample in 0...1000){
      var q=[for(joint in 0...6){var bounds=arm.group.limitsOf(joint);
        bounds.lower+(0.5+0.45*Math.sin((sample+1)*(joint+1)*1.61803398875))*(bounds.upper-bounds.lower);}];
      numbers(q);pose(arm.tcpPose(q));
    }
    for(wrist in [0.0,1e-12,-1e-12,1e-7,Math.PI]){
      var q=[0.2,-0.3,0.4,0.5,wrist,0.7];numbers(q);pose(arm.tcpPose(q));
    }
    Sys.println('Exported compiled EAIK fixture $name');
  }
  public static function main():Void {
    lines.push("7");
    fixture("RobotArm700",new RobotArm(false,null,machinekit.robotics.IndustrialArmClass.Reach700),true);
    fixture("RobotArm900",new RobotArm(false,null,machinekit.robotics.IndustrialArmClass.Reach900),true);
    fixture("RobotArm1300",new RobotArm(false,null,machinekit.robotics.IndustrialArmClass.Reach1300),true);
    fixture("Cobot500",new CobotArm(machinekit.robotics.CobotClass.Reach500),false);
    fixture("Cobot850",new CobotArm(machinekit.robotics.CobotClass.Reach850),false);
    fixture("Cobot900",new CobotArm(machinekit.robotics.CobotClass.Reach900),false);
    fixture("Cobot1300",new CobotArm(machinekit.robotics.CobotClass.Reach1300),false);
    var output=Sys.getEnv("EAIK_MODEL_FIXTURES");
    if(output==null)throw "Set EAIK_MODEL_FIXTURES to the native spike input file";
    sys.io.File.saveContent(output,lines.join("\n")+"\n");
  }
}
