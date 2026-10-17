import cadkit.modeling.AssemblyModel;
import materia.assembly.AssemblyFrames as F;
import machinekit.assembly.PosedParts;
import machinekit.hobbing.*;
import stockkit.hobbing.HobbingAxes;
import gearkit.GearSpec;

class FlexsplineHobberChecks {
  static var assertions=0;
  static function check(ok:Bool,label:String):Void{assertions++;if(!ok)throw label;}
  static function near(a:Float,b:Float,e:Float,label:String):Void check(Math.abs(a-b)<=e,'$label: $a vs $b');
  static function modelForRuntime(machine:FlexsplineHobber):robotkit.model.RobotModel {
    var model=new AssemblyModel("mm");machine.addTo(model,"");
    var physical:cadbridge.AssemblySimulationBridge.AssemblyPhysicalData={metresPerUnit:0.001,
      parts:[for(part in machine.components()){id:part.id,materialId:"steel",volume:1000.0,centerOfMass:[0.0,0,0],
        inertia:[100000.0,0,0,0,100000.0,0,0,0,100000.0],density:7850.0}]};
    return cadbridge.AssemblySimulationBridge.toRobotModel(model.definition("hobber"),physical).model;
  }
  public static function run():Void {
    var machine=new FlexsplineHobber();machine.validate();
    var runtimeModel=modelForRuntime(machine);
    for(joint in [FlexsplineHobber.B,FlexsplineHobber.C]){
      var drive=runtimeModel.actuators.filter(a->switch a.transmission{case SimpleTransmission(id,_,_):id==joint;})[0];
      check(runtimeModel.encoderFor(drive)!=null&&drive.positionLoopRate==4000,"hobber gear drive has explicitly bound installed encoder and loop rate");
    }
    for(joint in [FlexsplineHobber.A,FlexsplineHobber.C])
      check(runtimeModel.coupledLimits(joint).requireAcceleration()>0,"auxiliary rotary positioning acceleration is stated");
    var compiled=robotkit.runtime.RobotRuntimeCompiler.compile(runtimeModel,new robotkit.profile.RobotProfile());
    var tilt=[for(i in 0...runtimeModel.joints.length)if(runtimeModel.joints[i].id==FlexsplineHobber.A)i][0];
    check(compiled.joints[tilt].servoStiffness>0,"uncoupled braked swivel gets physical hold gains");
    var cam=FlexsplineHobberCam.job(machine);
    check(cam.passes.length==1&&cam.tableRpm>0,"HB8 CAM fits installed travel and drive limits");
    var ring=new stockkit.hobbing.RingCompliance(machine.spec.rootRadius()-0.3,0.6,machine.spec.faceWidth,210000,1000,"explicit test fixture stiffness");
    var sync=new stockkit.hobbing.HobbingErrorSource("sync","runtime observations","test zero measured following error",(t,a)->a);
    var errors=FlexsplineHobberErrors.sources(machine,t->20,ring,sync);
    check(errors.length==13,"HB11 installed parts expose independently selectable error sources");
    var mandrelError=errors.filter(e->e.id=="mandrel")[0];
    near(mandrelError.apply(0,new HobbingAxes(46,0,4,0,0,0)).pose().get_translation(0),46-machine.mandrel.eccentricity,1e-12,"part eccentricity controls physical work offset");
    check(errors.filter(e->e.id=="wall")[0].basis.indexOf("explicit test fixture stiffness")>=0,"wall support assumption stays in budget provenance");
    var forceStock=stockkit.RingStock.annulus(new stockkit.RingGrid(29,30,0,1,1,1));
    var forces=new stockkit.hobbing.HobbingForces(forceStock,30,20*Math.PI/180,1);
    try {
      forceStock.cutRegion([new stockkit.RingCut(0,0,1,0,0.001,0,0)]);
      forces.afterPass(new stockkit.hobbing.GashPass(0,0,0,0.5,1),new HobbingAxes(46,0,0,0,0,0),new HobbingAxes(46,0,0,0,0.625,0),16);
      check(FlexsplineHobberLoads.fits(machine,cam,forces),"HB10 installed ratings accept modest measured chip load with reserve");
      var options=FlexsplineHobberLoads.planOptions(forces);
      check(options.resistingEfforts.get("X")==forces.maxRadial,"radial load reaches PlanCheck");
      check(options.resistingEfforts.get(FlexsplineHobber.C)==forces.maxTableTorqueBound,"table reaction reaches PlanCheck");
      forceStock.cutRegion([new stockkit.RingCut(0,0,1,0.001,1,0,1)]);
      forces.afterPass(new stockkit.hobbing.GashPass(0,1,1,1.5,2),new HobbingAxes(46,0,0,0,0,0),new HobbingAxes(46,0,0,0,0.625,0),16);
      check(!FlexsplineHobberLoads.fits(machine,cam,forces),"HB10 installed ratings reject heavy measured chip load");
    }catch(e:Dynamic){forces.dispose();forceStock.dispose();throw e;}forces.dispose();forceStock.dispose();
    var model=new AssemblyModel("mm");machine.addTo(model,"");var state=model.initialState();
    var parts=new PosedParts();
    try {
      for(x in [machine.travel[0].lower,100.0])for(y in [-10.0,10])for(z in [-10.0,60])for(a in [-Math.PI/12,Math.PI/12]){
        var b=0.73,c=1.13;state.setJoint("X",x);state.setJoint("Y",y);state.setJoint("Z",z);
        state.setJoint(FlexsplineHobber.A,a);state.setJoint(FlexsplineHobber.B,b);state.setJoint(FlexsplineHobber.C,c);state.forwardKinematics();
        var actual=machine.hobToWork(state),expected=new HobbingAxes(x,y,z,a,b,c).pose();
        near(actual.x,expected.get_translation(0),1e-8,"mechanical hob/work frame X");
        near(actual.y,expected.get_translation(1),1e-8,"mechanical hob/work frame Y");
        near(actual.z,expected.get_translation(2),1e-8,"mechanical hob/work frame Z");
        for(j in 0...3){var v=F.transformVector(actual,j==0?1:0,j==1?1:0,j==2?1:0);
          near(v.x,expected.get_rotation(j),1e-8,"mechanical rotation X");near(v.y,expected.get_rotation(3+j),1e-8,"mechanical rotation Y");near(v.z,expected.get_rotation(6+j),1e-8,"mechanical rotation Z");}
        var cutter=parts.posed(machine.hob,state.worldPose("hob"));
        try {
          for(id in ["mandrel","table/platen"]){var p=machine.components().filter(m->m.id==id)[0].component;
            var fixture=parts.posed(p,state.worldPose(id));
            try {
              // Native bounds prove separating planes: radial for the cylindrical mandrel,
              // axial for the table. This is a conservative clearance lower bound.
              var toolBox=cutter.shape.bounds(),fixtureBox=fixture.shape.bounds();
              var gap=id=="mandrel"?toolBox.get_min().get_x()-fixtureBox.get_max().get_x():toolBox.get_min().get_z()-fixtureBox.get_max().get_z();
              check(gap>0.05,'hob clearance from $id at travel corner: $gap mm');
            }catch(e:Dynamic){fixture.close();throw e;}fixture.close();}
        }catch(e:Dynamic){cutter.close();throw e;}cutter.close();
      }
      var joint=model.definition("hobber").joints;
      check(joint.filter(j->j.id==FlexsplineHobber.C).length==1,"independent table C joint");
      var definition=model.definition("hobber");
      check(definition.couplings.filter(k->k.source==FlexsplineHobber.B||k.target==FlexsplineHobber.C).length==0,"B/C have no mechanical gear coupling");
      for(axis in ["X","Y","Z"]){var j=joint.filter(k->k.id==axis)[0];check(j.limits.lower!=null&&j.limits.upper!=null,"bounded linear travel");}
      check(!FoundationIntegrity.checkMounts(machine,machine.precisionMounts).hasErrors(),"foundation precision mounts all land on inserts");
      for(axis in ["X","Y","Z"]){
        var coupling=definition.couplings.filter(k->k.id==axis+"lead")[0];
        near(Math.abs(coupling.ratio),2*Math.PI/5,1e-12,"ball screw derives rad/mm coupling");
        var drive=machine.actuatorFor(axis+"drive");check(drive.maxRate>0&&drive.maxEffort>0,"installed servo supplies finite drive limits");
      }
      var actuators=definition.actuators;if(actuators==null)throw "missing drives";
      check(actuators.length==6,"six powered drives exported");
      var bDrive=actuators.filter(a->a.joint==FlexsplineHobber.B)[0];check(bDrive.processVelocity!=null,"B is a process velocity spindle");
      var aDrive=actuators.filter(a->a.joint==FlexsplineHobber.A)[0];
      check(aDrive.brake!=null,"A has a spring-applied static clamp with release feedback");
      check(machine.billOfMaterials().lines().length>30,"machine BOM contains mechanical and drive hardware");
      check(machine.encode().length>0,"complete mechanical description serializes");
    }catch(e:Dynamic){parts.close();throw e;}
    parts.close();
    for(s in [new GearSpec(1,40,10,20*Math.PI/180),new GearSpec(0.5,100,15,20*Math.PI/180)]){
      var other=new FlexsplineHobber(s),depth=s.pitchRadius()+other.hob.design.pitchDiameter/2+s.profileShift*s.module;
      check(depth>=other.travel[0].lower&&depth<=other.travel[0].upper,"small/comparison blank reaches full depth with matching fixture");
    }
    Sys.println('FlexsplineHobberChecks: $assertions assertions passed');
  }
  public static function main():Void {
    if(Sys.args().length>0&&(Sys.args()[0]=="scene"||Sys.args()[0]=="cutting")){
      var cutting=Sys.args()[0]=="cutting";
      var data=materia.project.SceneArtifact.decode(cutting?FlexsplineHobberPreview.cutting():FlexsplineHobberPreview.hobber());
      if(cutting){var moving=data.machineMotion;if(moving==null||moving.hobbing==null)throw "cutting job missing from artifact";
        check(moving.hobbing.teeth==40&&moving.hobbing.gashes==4,"coarse cutting job round-trips typed wire metadata");}
      var definition=data.assemblyDefinition;if(definition==null)throw "preview missing definition";
      var actuators=definition.actuators;if(actuators==null)throw "missing preview drives";
      check(actuators.length==6&&data.parts.length>30,"project preview exports complete mechanical scene and drives");
      Sys.println('FlexsplineHobber scene: $assertions assertions passed, ${data.parts.length} definitions');
    }else run();
  }
}
