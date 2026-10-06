import machinekit.robotics.IndustrialArmClass;
import machinekit.assembly.AssemblyPreview;
import cadbridge.AssemblyPhysicalPartView;
import cadbridge.AssemblySimulationBridge;
import robotkit.manipulation.Manipulator;
import motionkit.robot.EaikKinematics;
import motionkit.robot.ManipulatorKinematics;
import motionkit.kinematics.SixAxisConfiguration;
import motionkit.path.PoseMath;

/** Authored mechanisms and production limits; no supplied OPW parameters. */
class RobotArmAnalyticTests {
  static function check(value:Bool,message:String):Void {if(!value)throw message;}
  static function runAssembly(authored:machinekit.assembly.MachineAssembly,label:String):Void {
    authored.check().throwIfErrors();
    // Round-trip through the saved scene codec, which resolves materials and
    // fills mesh inertia exactly as a generated project does before bridging.
    var scene=materia.project.SceneArtifact.decode(materia.project.SceneArtifact.encode(
      AssemblyPreview.scene(authored,"industrial-analytic")));
    if(scene.assemblyDefinition==null)throw "Authored arm has no definition";
    var converted=AssemblySimulationBridge.toRobotModel(cast scene.assemblyDefinition,AssemblyPhysicalPartView.fromSceneArtifact(scene));
    var model=converted.model;
    var flanges=[for(frame in model.frames)if(frame.name=="toolFlange robot flange")frame];
    check(flanges.length==1,"Authored arm must export exactly one tool flange");
    var arm=new Manipulator(model,model.links[0].id,flanges[0].id);
    check(arm.group.count()==6,"Industrial arm must compile to six joints");
    var analytic=new EaikKinematics(arm),numeric=new ManipulatorKinematics(arm);
    var eaik=new motionkit.robot.EaikChain(arm);
    var eaikLabels=new motionkit.robot.EaikConfiguration(eaik);
    var seen=[for(_ in 0...8)false],complete=0,maximum=0,pinsChecked=false;
    var labels=["front/up/no-flip","front/down/no-flip","back/up/no-flip","back/down/no-flip",
      "front/up/flip","front/down/flip","back/up/flip","back/down/flip"];
    for(sample in 0...1000){
      var q=[for(joint in 0...6){
        var bounds=arm.group.limitsOf(joint);
        var fraction=0.5+0.45*Math.sin((sample+1)*(joint+1)*1.61803398875);
        bounds.lower+fraction*(bounds.upper-bounds.lower);
      }];
      var target=numeric.forward(q);
      check(PoseMath.distance(analytic.forward(q),target)<1e-6 && PoseMath.angle(analytic.forward(q),target)<1e-6,
        'Model-derived FK disagrees for $label, sample $sample');
      var slots=[for(_ in 0...8)false],original=false;
      var branches=analytic.branches(target,q);
      var exact=MotionKitNative.mk_eaik_inverse(eaik.solver,eaik.localTarget(target,q),32);
      check(exact.status==TrajectoryCoreConstants.MK_OK && exact.out_count>0,"Production EAIK compiled-chain inverse");
      var eaikOriginal=false;
      for(i in 0...exact.out_count){
        var raw=exact.out_solutions[i],joints=[for(j in 0...6)raw.get_joints(j)];
        var complete=q.copy();for(j in 0...6)complete[eaik.armIndices[j]]=joints[j];
        var actual=numeric.forward(complete);
        check(PoseMath.distance(actual,target)<1e-9 && PoseMath.angle(actual,target)<1e-9,"Production EAIK independent compiled FK");
        var same=true;for(j in 0...6)if(Math.abs(Math.atan2(Math.sin(joints[j]-q[j]),Math.cos(joints[j]-q[j])))>1e-9)same=false;
        eaikOriginal=eaikOriginal || same;
      }
      check(eaikOriginal,"Production EAIK contains original branch");
      for(branch in branches){
        check(branch.configuration!=null,"Every industrial solution needs a label");
        var configuration:SixAxisConfiguration=cast branch.configuration;
        check(configuration.shoulder+"/"+configuration.elbow+"/"+configuration.wrist==labels[branch.branch],"Industrial branch label convention");
        check(configuration.accepts(SixAxisConfiguration.of("EAIK",branch.branch,branch.q)),"Industrial physical turn labels match joint coordinates");
        check(eaikLabels.branch(branch.q)==branch.branch,"EAIK native and Haxe geometric labels agree");
        var actual=numeric.forward(branch.q);
        check(PoseMath.distance(actual,target)<1e-6 && PoseMath.angle(actual,target)<1e-6,"Every returned branch passes independent model FK");
        var same=true;
        for(joint in 0...6){
          var bounds=arm.group.limitsOf(joint);
          check(branch.q[joint]>=bounds.lower-1e-10 && branch.q[joint]<=bounds.upper+1e-10,"Every solution respects authored limits");
          if(Math.abs(branch.q[joint]-q[joint])>1e-5)same=false;
        }
        original=original || same;slots[branch.branch]=true;seen[branch.branch]=true;
      }
      check(original,'Industrial inverse lost authored joints at sample $sample');
      var count=[for(slot in slots)if(slot)slot].length;
      maximum=Std.int(Math.max(maximum,count));
      if(count==8){
        complete++;
        if(!pinsChecked){
          for(slot in 0...8){
            var source=[for(branch in branches)if(branch.branch==slot)branch][0];
            var pin:SixAxisConfiguration=cast source.configuration;
            var request=new motionkit.kinematics.PathRequest([0.0,1.0],[target,target],source.q,
              new motionkit.kinematics.IkTolerance(1e-6,1e-6),[for(_ in 0...6)1.0],[for(_ in 0...6)1.0]);
            var problem=new motionkit.robot.CandidateProblem(arm,request,
              new motionkit.robot.CandidateProblem.CandidateSamplingOptions(1,1,1,true,null,null,pin));
            check(problem.family=="EAIK","Authored industrial ladder must use the analytic backend");
            for(layer in problem.samples){
              check(layer.candidates.length>0,'Authored configuration pin lost slot $slot');
              for(candidate in layer.candidates)check(pin.accepts(candidate.configuration),"Authored pinned candidates preserve labels and physical turns");
            }
            var selected=motionkit.robot.StructuredLadder.search(problem);
            check(selected.diagnostic==null,"Every authored configuration pin selects a complete route");
            var refiner=new motionkit.robot.AnalyticPathRefiner(arm,problem,selected);
            check(pin.accepts(refiner.sample(0.5,target,motionkit.path.OrientationPolicy.Fixed,source.q).configuration),
              "Every authored configuration pin survives native sampling, selection and refinement");
          }
          pinsChecked=true;
        }
      }
    }
    Sys.println('6R $label: 1000 round trips, slots=${seen}, max=$maximum, eight-branch targets=$complete');
    check([for(slot in seen)if(slot)slot].length==8,"All eight industrial branches must be reachable within authored limits");
    check(complete>0,"An authored target must expose all eight labelled solutions within real limits");
  }
  public static function main():Void {
    for(cls in [Reach700,Reach900,Reach1300])runAssembly(new RobotArm(false,null,cls),Std.string(cls));
    for(cls in [machinekit.robotics.CobotClass.Reach500,machinekit.robotics.CobotClass.Reach850,
        machinekit.robotics.CobotClass.Reach900,machinekit.robotics.CobotClass.Reach1300])
      runAssembly(new machinekit.robotics.CobotArm(cls),Std.string(cls));
    Sys.println("Production EAIK all-size RobotArm/Cobot acceptance passed");
  }
}
