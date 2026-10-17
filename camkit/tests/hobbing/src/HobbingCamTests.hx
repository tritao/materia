import camkit.*;
import gearkit.*;
import toolpathkit.path.*;
import toolpathkit.path.Point3;
import toolpathkit.motion.*;
import motionkit.trajectory.Trajectory;
import motionkit.program.MotionOp;

class HobbingCamTests {
  static var assertions=0;
  static function check(ok:Bool,label:String):Void{assertions++;if(!ok)throw label;}
  static function near(a:Float,b:Float,tol:Float,label:String):Void check(Math.abs(a-b)<=tol,'$label: $a vs $b');
  static function rejects(f:Void->Void,text:String):Void{
    var error="";try{f();}catch(e:Dynamic){error=Std.string(e);}check(error.indexOf(text)>=0,'expected $text, got $error');
  }
  static function make(spec:GearSpec):HobDesign return new HobDesign(ConjugateRack.of(new ToothProfile(spec)),1,32,12);
  public static function main():Void {
    var spec=new GearSpec(0.3,200,8,20*Math.PI/180),hob=make(spec);
    var job=new HobbingJob(spec,hob,40);
    near(job.rpm,300000/(Math.PI*32),1e-9,"cutting speed sets B rpm");
    near(job.tableRpm,job.rpm/200,1e-12,"starts/teeth sets work rpm");
    near(job.axialFeed,0.5*job.tableRpm/60,1e-12,"axial feed per work turn");
    check(job.passes.length==1,"default full-depth cut");
    var pass=job.passes[0],radius=spec.tipRadius(),R=hob.outsideDiameter/2,D=pass.centreDistance,a=-hob.leadAngle;
    near(D,spec.pitchRadius()+hob.pitchDiameter/2+spec.profileShift*spec.module,1e-12,"finished centre distance");
    var observedZ=0.0,observedS=0.0;
    // Independent continuous-cylinder boundary samples: recover actual Y, axial S and global Z.
    for(i in 0...10001){
      var w=Math.max(-R,D-radius)+(Math.min(R,D+radius)-Math.max(-R,D-radius))*i/10000;
      var boundY=Math.sqrt(Math.max(0,radius*radius-(D-w)*(D-w))),boundV=Math.sqrt(Math.max(0,R*R-w*w));
      for(y in [-boundY,boundY])for(v in [-boundV,boundV]){
        var S=(-y-v*Math.sin(a))/Math.cos(a),z=S*Math.sin(a)-v*Math.cos(a);
        check(Math.abs(z)<=pass.engagement.overrun+1e-8,"entry/exit cover cylinder engagement");
        check(Math.abs(S)<=pass.engagement.axialHalfSpan+1e-8,"finite hob covers active axial span");
        observedZ=Math.max(observedZ,Math.abs(z));observedS=Math.max(observedS,Math.abs(S));
      }
    }
    near(observedZ,pass.engagement.overrun,0.001,"overrun bound is tight, not an arbitrary allowance");
    near(observedS,pass.engagement.axialHalfSpan,0.001,"active hob length bound is tight");
    check(pass.zStart+pass.engagement.overrun<0&&pass.zEnd-pass.engagement.overrun>spec.faceWidth,"clear at both ends");
    near(pass.seconds,pass.revolutions/job.tableRpm*60,1e-10,"nominal cycle from work revolutions");
    var plannedSeconds=0.0;
    var planned=Trajectory.generateStateToState([pass.zStart/1000],[0],[0],[pass.zEnd/1000],
      [job.axialFeed/1000],[0.2],[5]);
    try{plannedSeconds=planned.durationSeconds();check(Math.abs(plannedSeconds/pass.seconds-1)<0.01,"MotionKit planned cutting cycle agrees within one percent");}
    catch(e:Dynamic){planned.dispose();throw e;}planned.dispose();
    var s=new HobbingSettings();s.climb=false;
    var reverse=new HobbingJob(spec,hob,40,s);near(reverse.passes[0].zStart,pass.zEnd,1e-12,"conventional feed reverses Z");
    near(reverse.cuttingSeconds,job.cuttingSeconds,1e-12,"cut direction preserves work revolutions");
    var multi=new HobbingSettings();multi.depths=[job.fullDepth/2,job.fullDepth];multi.shifts=[-5,5];multi.crown=0.02;
    var crowned=new HobbingJob(spec,hob,40,multi);
    check(crowned.passes.length==2,"rough and finish passes");
    check(crowned.passes[0].centreDistance>crowned.passes[1].centreDistance,"infeed increases to finish");
    near(crowned.passes[1].tablePhase-crowned.passes[0].tablePhase,-10*Math.cos(hob.leadAngle)/spec.pitchRadius(),1e-12,"shift retains tooth phase");
    near(crowned.crownAt(4),0.02,1e-12,"crown middle is thicker");near(crowned.crownAt(0),0,1e-12,"crown vanishes at ends");
    multi.crown=0.03;near(crowned.crownAt(4),0.02,1e-12,"job snapshots mutable settings");
    var faster=new HobbingJob(new GearSpec(0.3,100,8,20*Math.PI/180),hob,40);
    near(faster.tableRpm,2*job.tableRpm,1e-12,"tooth count changes program gearing without rebuilding a machine");
    var doubleHob=new HobDesign(ConjugateRack.of(new ToothProfile(spec)),2,32,12);
    var doubleStart=new HobbingJob(spec,doubleHob,40);
    near(doubleStart.tableRpm,2*job.tableRpm,1e-12,"multi-start hob sets work speed");
    var shiftedSpec=new GearSpec(0.3,200,8,20*Math.PI/180,false,0.2);
    near(new HobbingJob(shiftedSpec,hob,40).passes[0].centreDistance,D+0.06,1e-12,"profile shift changes finished radial setting");
    rejects(()->{new HobbingJob(new GearSpec(0.3,200,8,20*Math.PI/180,true),hob,40);},"external spur");
    rejects(()->{new HobbingJob(new GearSpec(0.5,200,8,20*Math.PI/180),hob,40);},"matching normal");
    multi.shifts=[0,20];rejects(()->{new HobbingJob(spec,hob,40,multi);},"Hob too short");
    rejects(()->{new HobbingJob(spec,hob,10);},"Hob too short");
    var bad=new HobbingSettings();bad.depths=[job.fullDepth/2];rejects(()->{new HobbingJob(spec,hob,40,bad);},"full tooth depth");
    var limits=new HobbingLimits(new Point3(30,-10,-10),new Point3(100,10,60),-Math.PI/12,Math.PI/12,8000,120);
    limits.check(job);check(true,"reference job fits HB6 travel and spindle limits");
    rejects(()->{new HobbingLimits(new Point3(30,-10,0),new Point3(100,10,60),-1,1,8000,120).check(job);},"Z position");
    rejects(()->{new HobbingLimits(new Point3(0,-20,-20),new Point3(100,20,60),-1,1,2000,120).check(job);},"Hob spindle overspeed");
    var binding=new MachineBinding("hobber-axes","X","Y","Z",0.02,new ToolpathChannels("","hob"));
    binding.spindleJointId="spindle/B";binding.gearJointIds=["X","Y","Z","swivel/A","spindle/B","table/C"];
    binding.gearFollowers=["table/C"];
    for(id in ["spindle/B","table/C"])binding.gearEncoders.set(id,{latencyNs:1000000,quantum:2*Math.PI/8388608,loopHz:4000});
    var lowered=new ToolpathLowering(binding).lower(job.program.ops),program=lowered.program;
    if(program==null)throw "missing lowered hobbing program";
    var feedFound=false,infeedFound=false,tiltFound=false,gearFound=false,stopFound=false;
    for(op in program.ops)switch op{
      case FollowPath(path,_,feed,_):
        if(Math.abs(feed-0.0005)<1e-12)infeedFound=true;
        if(Math.abs(feed-job.axialFeed/1000)<1e-10){feedFound=true;near(path.length(),(pass.zEnd-pass.zStart)/1000,1e-10,"feed path covers derived overrun");}
      case MoveJ(JointPosition(3,position),_,_):tiltFound=true;near(position,-hob.leadAngle,1e-12,"swivel lead tilt in radians");
      case SetOutput("motion.gear/5",Process("gear.engage",_)):gearFound=true;
      case WaitInput("hob.stopped",_,_):stopFound=true;
      case _:
    }
    check(feedFound&&infeedFound&&tiltFound&&gearFound&&stopFound,"CAM lowers to motion, gearbox and measured spindle barriers");
    // Compile the actual lowered program, including absolute A/C positions from nonzero starts.
    var m=new robotkit.model.RobotModel("hobbing logical axes"),base=m.addLink(new robotkit.model.Link("base"));
    for(id in binding.gearJointIds){
      var link=m.addLink(new robotkit.model.Link(id+" link"));
      var joint=m.addJoint(new robotkit.model.Joint(id,id=="X"||id=="Y"||id=="Z"?robotkit.model.JointType.Prismatic:robotkit.model.JointType.Continuous,base,link));
      joint.limits=new robotkit.model.JointLimits(-1,1,1,10,2);
    }
    var axes=[for(id in ["X","Y","Z"])new motionkit.axis.MotionAxisBlueprint(id,[id],-1,1,1,2,0)];
    var blueprint=new motionkit.robot.MotionSystemBlueprint(m,new robotkit.runtime.RobotRuntimeBlueprint(1,6,7),axes);
    var solver=new motionkit.robot.AxisKinematics(blueprint,"X","Y","Z");
    var validation=new motionkit.trajectory.ValidationLimits(6,haxe.Int64.ofInt(1),haxe.Int64.ofInt(0));
    for(j in 0...6){validation.position(j,-1,1);validation.velocity(j,1);validation.acceleration(j,2);validation.jerk(j,10);}
    var compiler=new motionkit.robot.ProgramCompiler(solver,validation,"hobber-axes",[for(_ in 0...6)1.0],
      [for(_ in 0...6)2.0],[for(_ in 0...6)10.0],motionkit.robot.StartTolerances.uniform(6,0.00001,0.01,0.01));
    var compiled=compiler.compile(program,[0.06,0.0,0.0,0.2,0.0,0.7],haxe.Int64.ofInt(1));
    var cuttingPlans=0;
    try{
      for(block in compiled.blocks)for(i in 0...block.plans.length){
        var plan=block.plans[i];
        switch program.ops[block.opIndices[i]]{
          case FollowPath(_,_,feed,_) if(Math.abs(feed-job.axialFeed/1000)<1e-10):
            cuttingPlans++;var q=plan.evaluate(plan.durationSeconds).positions;
            near(q[3],-hob.leadAngle,1e-9,"planned cut retains absolute tilt");
            near(q[5],pass.tablePhase,1e-9,"planned cut retains additive table phase");
            check(Math.abs(plan.durationSeconds/pass.seconds-1)<0.01,"complete lowered cut plans to nominal work-revolution time");
          case _:
        }
      }
      check(cuttingPlans==1,"one compiled cutting stroke");
    }catch(e:Dynamic){compiled.dispose();throw e;}compiled.dispose();
    var crownError=0.0;
    for(op in crowned.program.ops)switch op{
      case Move(Cut,Line(a,b),_,_,_) if(Math.abs(a.z-b.z)>1e-15):
        var z=(a.z+b.z)*500;
        var baseX=a.x*1000-crowned.crownAt(a.z*1000);
        crownError=Math.max(crownError,Math.abs((a.x+b.x)*500-baseX-crowned.crownAt(z)));
      case _:
    }
    check(crownError<=multi.crownChordTolerance/2+1e-9,"crown chords leave half tolerance for blending");
    var wrong=new MachineBinding("work","X","Y","Z",0.02,new ToolpathChannels(""));
    rejects(()->{new ToolpathLowering(wrong).lower([FeedPerRevolution("C","Z",0.001,Provenance.cam(0)),
      Move(Cut,Line(new Point3(0,0,0),new Point3(0,0,0.01)),0.001,0,Provenance.cam(0))]);},"active spindle");
    Sys.println('Hobbing CAM: $assertions assertions passed; overrun ${pass.engagement.overrun} mm per end, ${pass.revolutions} work turns, ${pass.seconds} s nominal, $plannedSeconds s S-curve');
  }
}
