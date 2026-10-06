import machinekit.robotics.IndustrialArmClass;
import machinekit.assembly.AssemblyPreview;
import cadbridge.AssemblyPhysicalPartView;
import cadbridge.AssemblySimulationBridge;
import robotkit.manipulation.Manipulator;
import robotkit.manipulation.ArmClearance;
import robotkit.model.ActuatorDrive.ServoDrive;

/** Physical acceptance uses the same compiled geometry and masses as the app. */
class RobotArmPhysicalTests {
  static function check(value:Bool,message:String):Void {if(!value)throw message;}
  static function run(cls:IndustrialArmClass):Void {
    var authored=new RobotArm(false,null,cls);
    authored.check().throwIfErrors();
    var mass=authored.massProperties();
    check(mass.mass>0 && mass.inertia!=null && mass.unaccounted.length==0 && mass.unaccountedInertia.length==0,
      "All industrial parts and BOM extras need mass and inertia");
    var bom=authored.billOfMaterials(),expected=new Map<String,Int>();
    for(entry in authored.components()){
      var item=entry.component.bom;
      expected.set(item.partNumber,(expected.exists(item.partNumber)?expected.get(item.partNumber):0)+item.quantity);
    }
    for(number in expected.keys())check(bom.quantity(number)==expected.get(number),"BOM must retain every authored component quantity");
    var scene=materia.project.SceneArtifact.decode(materia.project.SceneArtifact.encode(AssemblyPreview.scene(authored,"industrial-physical")));
    var converted=AssemblySimulationBridge.toRobotModel(cast scene.assemblyDefinition,AssemblyPhysicalPartView.fromSceneArtifact(scene));
    var model=converted.model,flange=[for(frame in model.frames)if(frame.name=="toolFlange robot flange")frame][0];
    var arm=new Manipulator(model,model.links[0].id,flange.id);
    var home=[for(_ in 0...6)0.0];
    var clearance=new ArmClearance(arm,[for(hull in converted.linkHulls){name:hull.part,link:model.links[hull.link].id,
      vertices:hull.vertices,tool:StringTools.startsWith(hull.part,"tool/")}],home,processkit.WeldPathPlanner.AIR_MARGIN);
    var continuous:Array<Float> = [];
    for(index in 0...6){
      var id=arm.jointIds()[index];
      var motors:Array<robotkit.model.Actuator> = [];
      for(motor in model.actuators)switch motor.transmission {
        case SimpleTransmission(joint,_,_):if(joint==id)motors.push(motor);
        default:
      }
      check(motors.length==1,"Every industrial joint must have one servo");
      var motor=motors[0];
      check(Std.isOfType(motor.drive,ServoDrive),"Industrial drives must retain servo ratings");
      var drive:ServoDrive=cast motor.drive;
      var ratio=switch motor.transmission {case SimpleTransmission(_,value,_):Math.abs(value);default:throw "Expected simple industrial transmission";};
      check(Math.abs(ratio-authored.specs[index].gearbox.ratio)<1e-9,"Compiled gearbox ratio must match BOM");
      var bound=model.coupledLimits(id);
      check(bound.velocity!=null && bound.velocity>0 && bound.effort!=null && bound.effort>0,"Compiled industrial drive limits must be positive");
      check(Math.abs(cast(bound.velocity,Float)-drive.maxSpeedValue/ratio)<1e-8,"Joint speed must derive from the servo and gearbox");
      check(Math.abs(cast(bound.effort,Float)-drive.peakTorqueValue*ratio*motor.efficiency)<1e-8,"Joint peak torque must derive from servo, gearbox and efficiency");
      var bounds=arm.group.limitsOf(index);
      check(Math.abs(bounds.lower+authored.specs[index].initial-authored.specs[index].lower)<1e-10 &&
        Math.abs(bounds.upper+authored.specs[index].initial-authored.specs[index].upper)<1e-10,"Compiled limits retain absolute mechanical travel");
      continuous.push(drive.ratedTorque*ratio*motor.efficiency);
    }
    var links=[for(link in model.links)link.id];
    function potential(q:Array<Float>):Float {
      var poses=arm.linkPoses(q,links),energy=0.0;
      for(index in 0...model.links.length){var link=model.links[index];
        var centre=poses[index].transformPoint(robotkit.spatial.Vec3.fromArray(link.centerOfMass));
        energy+=link.mass*9.81*centre.z;
      }
      return energy;
    }
    var poses:Array<{name:String,q:Array<Float>}> = [{name:"ready",q:home}];
    var zero=[for(index in 0...6)-authored.specs[index].initial];poses.push({name:"industrial-zero",q:zero});
    for(index in 0...6)for(side in 0...2){var q=home.copy(),bounds=arm.group.limitsOf(index);
      q[index]=side==0?bounds.lower:bounds.upper;poses.push({name:'j${index+1}-${side==0?"lower":"upper"}',q:q});}
    // Verify travel to each stop as well as its endpoint; joint combinations
    // still use the planner's clearance checks for their actual route.
    for(endpoint in poses.copy())if(StringTools.startsWith(endpoint.name,"j")){
      var angle=0.0;for(value in endpoint.q)angle=Math.max(angle,Math.abs(value));
      var steps=Std.int(Math.ceil(angle/0.05));
      for(sample in 1...steps)poses.push({name:'${endpoint.name}-travel-$sample',
        q:[for(value in endpoint.q)value*sample/steps]});
    }
    var cadModel=new cadkit.modeling.AssemblyModel("mm");authored.addTo(cadModel,"");
    var state=new cadkit.modeling.AssemblyState(cadModel.definition("physical"));
    function placed(id:String):cadkit.modeling.Part {
      var component=[for(entry in authored.components())if(entry.id==id)entry.component][0];
      var frame=state.worldPose(id),x=materia.assembly.AssemblyFrames.transformVector(frame,1,0,0),
        z=materia.assembly.AssemblyFrames.transformVector(frame,0,0,1);
      var source=component.geometry();
      try {var result=source.placed(new cadkit.modeling.Location(new cadkit.modeling.Plane(
        new cadkit.modeling.Vector(frame.x,frame.y,frame.z),new cadkit.modeling.Vector(x.x,x.y,x.z),new cadkit.modeling.Vector(z.x,z.y,z.z))));
        source.close();return result;
      }catch(error:Dynamic){source.close();throw error;}
    }
    var worst=[for(_ in 0...6)0.0],violations:Array<String> = [];
    for(pose in poses){
      var failure=clearance.violation(pose.q);
      if(failure!=null){
        for(index in 0...6)state.setJoint(authored.specs[index].id,pose.q[index]+authored.specs[index].initial);
        state.forwardKinematics();
        var a=placed(failure.a),b=placed(failure.b),overlap=0.0;
        try {var common=a.intersect(b);overlap=common.shape.volume();common.close();a.close();b.close();}
        catch(error:Dynamic){a.close();b.close();throw error;}
        violations.push('${pose.name}: ${failure.a}/${failure.b}, ${failure.distance*1000} mm, exact overlap=$overlap mm3');
      }
      for(index in 0...6){var low=pose.q.copy(),high=pose.q.copy();low[index]-=1e-5;high[index]+=1e-5;
        worst[index]=Math.max(worst[index],Math.abs((potential(high)-potential(low))/2e-5));}
    }
    for(index in 0...6)check(Math.isFinite(worst[index]) && worst[index]<continuous[index],
      'Industrial j${index+1} cannot hold sampled static gravity continuously');
    check(violations.length==0,'Industrial ${authored.reference.designation} limit interference: ${violations.join("; ")}');
    Sys.println('physical ${authored.reference.designation}: samples=${poses.length}, mass=${mass.mass}, BOM=${bom.lines().length}, continuous=$continuous, gravity=$worst, collisions=$violations');
  }
  public static function main():Void {
    for(cls in [Reach700,Reach900,Reach1300])run(cls);
    Sys.println("Industrial RobotArm all-size physical acceptance passed");
  }
}
