import TendingElectricEdgeExecutor.TendingElectricEdgeDriver;
import TendingElectricEdgeExecutor.TendingElectricEdgeFeedback;
import robotkit.runtime.*;
import robotkit.core.JointTarget;
import haxe.Int64;
import runtime.memory.Arena;
import runtime.memory.RawPtr;
import runtime.memory.NativeSpan;

/** Actual SimKit joint observations and contacts. The owning runner advances the shared world once per tick. */
class TendingElectricSimDriver implements TendingElectricEdgeDriver {
  final harness:SimulationHarness;
  final runtime:RobotRuntime;
  final scene:TendingElectricSimScene;
  final contactNames:Map<Int,String>;
  final grips:Array<SimulatedElectricGripper>;
  var from:Array<Float> = [];
  var target:Array<Float> = [];
  var elapsed=0.0;
  var duration=0.0;
  var admitted=false;
  public var minimumMotionSeconds=0.0;
  var previous:Array<Float> = [];
  var travel=0.0;
  final planMemory=new Arena(4096);
  final payloads:Map<String,{object:nativekit.sim.SimObject,grip:SimulatedElectricGripper,mass:robotkit.tool.MassProperties}> = [];
  public var retentionFault(default,null):Dynamic = null;
  public function carrying(side:String):Bool return payloads.exists(side);
  public function carriedObjects():Array<nativekit.sim.SimObject> return [for(payload in payloads)payload.object];
  public function carriedGrasps():Array<{side:String,object:nativekit.sim.SimObject}> {
    var result:Array<{side:String,object:nativekit.sim.SimObject}> = [];
    for(side in payloads.keys()) {
      var payload:{object:nativekit.sim.SimObject,grip:SimulatedElectricGripper,mass:robotkit.tool.MassProperties}=cast payloads.get(side);
      result.push({side:side,object:payload.object});
    }
    return result;
  }
  public function carryingObject(object:nativekit.sim.SimObject):Bool {
    for(payload in payloads)if(payload.object.handle.rawValue()==object.handle.rawValue())return true;
    return false;
  }
  public function expectPayload(side:String,object:nativekit.sim.SimObject,grip:SimulatedElectricGripper,mass:robotkit.tool.MassProperties):Void {
    if(mass==null || mass.massKg<=0)throw "Carried stock needs its resolved CAD mass properties";
    payloads.set(side,{object:object,grip:grip,mass:mass});
    admitted=true; // Confirmed grasp observations establish a live physics frame, including seeded contact tests.
  }
  public function releasedPayload(side:String):Void payloads.remove(side);
  public function checkRetention(?releasing:String):Void {
    var contacts=harness.simulation.robotContacts(runtime);
    for(side in payloads.keys())if(side!=releasing) {
      var payload:{object:nativekit.sim.SimObject,grip:SimulatedElectricGripper,mass:robotkit.tool.MassProperties}=cast payloads.get(side);
      if(payload.grip.grippedObjects(contacts).indexOf(payload.object.handle.rawValue())<0) {
        retentionFault={side:side,object:contactNames.get(payload.object.handle.rawValue()),
          time:harness.simulation.simulationTime(),openingM:payload.grip.opening(),
          pose:harness.simulation.objectPose(payload.object),contacts:[for(contact in contacts)
            if(contact.otherKind==RobotContactOtherKind.Object && contact.otherObject==payload.object.handle.rawValue())
            {link:contact.linkIndex,jaw:payload.grip.jawContact(contact.linkIndex),active:contact.active,
              wrenchValid:contact.wrenchValid,forceN:contact.force.norm(),distanceMm:contact.distance*1000}]};
        stopAndHold();throw "Lost bilateral grasp on gripper "+side+": "+contactNames.get(payload.object.handle.rawValue());
      }
    }
  }
  var settledSamples=0;
  var sequence=100;
  public var activeStock:Null<nativekit.sim.SimObject> = null;
  public var bodyNames:Map<Int,String> = [];
  var observeTargetSupport=false;
  public function configure(edge:Dynamic):Void {
    // A released stock's support contact remains after the robot withdraws.
    // Separation supervision observes the robot and its carried payloads.
    observeTargetSupport=edge.kind=="guardedJointEdge" && edge.separating!=true;
  }
  public var stopped(default,null)=false;
  public function new(harness:SimulationHarness,runtime:RobotRuntime,scene:TendingElectricSimScene,names:Map<Int,String>,grips:Array<SimulatedElectricGripper>) {
    this.harness=harness;this.runtime=runtime;this.scene=scene;contactNames=names;this.grips=grips;
  }
  public function joints():Array<Float> return [for(j in scene.armJoints)harness.simulation.jointSample(0,j).get_position()];
  function tip():Array<Float> {
    var p=harness.simulation.linkPose(0,scene.flangeLink);
    var f=materia.assembly.AssemblyFrames.compose({x:p.position[0],y:p.position[1],z:p.position[2],qx:p.rotation[0],qy:p.rotation[1],qz:p.rotation[2],qw:p.rotation[3]},scene.flangeOffset);
    return [f.x,f.y,f.z];
  }
  public function start(target:Array<Float>):Void {
    if(stopped)throw "Stopped simulation driver";
    admitted=true;
    settledSamples=0;this.target=target.copy();from=joints();elapsed=0;duration=0.02;travel=0;previous=tip();
    // One C2 quintic scalar advances every arm axis along this exact validated joint edge.
    for(j in 0...6) {
      var drive=scene.blueprint.joints[scene.armJoints[j]],delta=Math.abs(target[j]-from[j]);
      var rate=Math.min(0.5,drive.requireRate());
      var acceleration=drive.maxAcceleration==null ? 0.5 : Math.min(0.5,drive.maxAcceleration);
      if(!(rate>0) || !(acceleration>0))throw "Electric trajectory needs positive model limits";
      duration=Math.max(duration,Math.max(1.875*delta/rate,Math.sqrt(5.773503*delta/acceleration)));
    }
    duration=Math.max(Math.max(0.1,minimumMotionSeconds),duration);
    var nanos=Int64.add(Int64.fromFloat(duration*1e9),Int64.ofInt(1));duration=Int64.toFloat(nanos)*1e-9;
    var measured=[for(j in 0...scene.blueprint.jointCount)harness.simulation.jointSample(0,j).get_position()];
    // Arm-only source coordinates: process jaws retain independent ownership and their physical coupling.
    var starts:RawPtr<Int64> = planMemory.alloc(1),durations:RawPtr<Int64> = planMemory.alloc(1);
    var degrees:RawPtr<Int> = planMemory.alloc(1),coefficients:RawPtr<Float> = planMemory.alloc(36);
    starts.store(Int64.ofInt(0));durations.store(nanos);degrees.store(5);
    for(j in 0...6) {
      var delta=target[j]-from[j],row=[from[j],0,0,10*delta/Math.pow(duration,3),-15*delta/Math.pow(duration,4),6*delta/Math.pow(duration,5)];
      for(power in 0...6)coefficients.offset(j*6+power).store(row[power]);
    }
    var couplings=[for(c in scene.blueprint.couplings)new robotkit.core.CoupledJoint(c.follower,c.leader,c.ratio,c.offset)];
    var arrays=new robotkit.execution.SegmentArrays(new NativeSpan<Int64>(starts,1),new NativeSpan<Int64>(durations,1),
      new NativeSpan<Int>(degrees,1),new NativeSpan<Float>(coefficients,36),scene.armJoints,measured,couplings);
    // Metadata for independently controlled process axes must obey the ideal mechanical coupling.
    // This projects only the uncommanded reference metadata, never the measured physics state.
    var planStart=[for(j in 0...measured.length)arrays.coefficient(0,j,0)];
    var zeros=[for(_ in measured)0.0];
    runtime.submitPlan(robotkit.execution.ExecutionPlanSubmission.ofArrays(Int64.ofInt(sequence),Int64.ofInt(scene.blueprint.revision),
      Int64.ofInt(scene.blueprint.calibrationRevision),RobotKitRuntimeConstants.RK_PLAN_CAPABILITY_TRAJECTORY_QUEUE,
      planStart,zeros,zeros,arrays,[for(_ in measured)0.00001],[for(_ in measured)0.0001],
      [for(_ in measured)0.01],true,[],true),sequence++);
  }
  public function dispose():Void planMemory.dispose();
  public function advance(dt:Float):Void {
    if(stopped || !admitted)return;
    elapsed+=dt;
    var measured=[for(j in 0...scene.blueprint.jointCount)harness.simulation.jointSample(0,j).get_position()];
    var loads:Array<{link:Int,point:Array<Float>,mass:Float}> = [];
    for(payload in payloads) {
      var p=harness.simulation.objectPose(payload.object);
      var c=payload.mass.centerOfMass;
      var point=materia.assembly.AssemblyFrames.transformPoint({x:p.x,y:p.y,z:p.z,qx:p.qx,qy:p.qy,qz:p.qz,qw:p.qw},c.x,c.y,c.z);
      loads.push({link:scene.flangeLink,point:[point.x,point.y,point.z],mass:payload.mass.massKg});
    }
    var gravity=scene.gravity.torques(measured,loads);
    for(index in scene.armJoints)harness.simulation.setServoFeedforward(0,index,gravity[index]);
  }
  public function feedback():TendingElectricEdgeFeedback {
    checkRetention();
    var q=joints(),arrived=target.length==6 && elapsed>=duration;
    if(target.length==6)for(j in 0...6)if(Math.abs(q[j]-target[j])>0.000005 || Math.abs(harness.simulation.jointSample(0,scene.armJoints[j]).get_velocity())>0.00005)arrived=false;
    if(arrived)settledSamples++;else settledSamples=0;
    arrived=settledSamples>=5;
    var at=tip(),squared=0.0;
    if(previous.length!=3)previous=at.copy();
    for(a in 0...3)squared+=(at[a]-previous[a])*(at[a]-previous[a]);
    travel+=Math.sqrt(squared)*1000;previous=at;
    var raw=harness.simulation.robotContacts(runtime),held:Array<Int> = [];
    for(grip in grips)for(id in grip.grippedObjects(raw))if(held.indexOf(id)<0)held.push(id);
    var contacts:Array<String> = [],penetration=0.0,force=0.0;
    for(contact in raw)if(contact.active) {
      var intendedGrip=false;
      if(contact.otherKind==RobotContactOtherKind.Object)for(grip in grips)
        if(grip.jawContact(contact.linkIndex) && grip.grippedObjects(raw).indexOf(contact.otherObject)>=0)intendedGrip=true;
      if(intendedGrip)continue;
      var id=contact.otherKind==RobotContactOtherKind.Object ? contactNames.get(contact.otherObject) :
        'robot-link-${contact.otherRobot}-${contact.otherLink}';
      if(id==null)id="unidentified physics contact";
      if(contacts.indexOf(id)<0)contacts.push(id);
      penetration=Math.max(penetration,-contact.distance*1000);
      if(contact.wrenchValid)force=Math.max(force,contact.force.norm());
    }
    // A free stock/fixture contact is outside robotContacts; use the immutable session snapshot.
    var monitored:Array<nativekit.sim.SimObject> = [];
    if(activeStock!=null)monitored.push(cast activeStock);
    for(payload in payloads)if(monitored.indexOf(payload.object)<0)monitored.push(payload.object);
    var worldContacts=harness.space.session.contacts();
    for(object in monitored) {
      var body=harness.simulation.objectBody(object).rawValue();
      if(observeTargetSupport || held.indexOf(object.handle.rawValue())>=0)for(contact in worldContacts)if(contact.active && (contact.bodyA==body || contact.bodyB==body)) {
        var other=contact.bodyA==body ? contact.bodyB : contact.bodyA;
        var name=bodyNames.get(other);
        // Jaw/workpiece contacts establish the grasp; only named environment bodies constrain this phase.
        if(name==null)continue;
        if(contacts.indexOf(name)<0)contacts.push(name);
        penetration=Math.max(penetration,-contact.distance*1000);
        if(contact.wrenchValid) {
          var norm=0.0;for(component in contact.force)norm+=component*component;
          force=Math.max(force,Math.sqrt(norm));
        }
      }
    }
    return {joints:q,arrived:arrived,contacts:contacts,travelMm:travel,penetrationMm:penetration,forceN:force,
      speedMmPerSecond:Math.sqrt(squared)*1000/0.01};
  }
  public function stopAndHold():Void {
    if(stopped)return;stopped=true;runtime.submitStop(sequence++,false);for(grip in grips)grip.stopAndHold();
  }
  /** Controlled contact stop discards the unused search tail; production remains admitted. */
  public function holdAtContact():Void {
    if(stopped)throw "Cannot resume a latched driver";
    runtime.submitAbort(sequence++);target=[];
  }
}
