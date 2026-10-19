/** Live feedback for one exact-stop electric joint edge. Distances are millimetres. */
typedef TendingElectricEdgeFeedback = {
  var joints:Array<Float>;
  var arrived:Bool;
  var contacts:Array<String>;
  var travelMm:Float;
  var penetrationMm:Float;
  @:optional var forceN:Float;
  @:optional var speedMmPerSecond:Float;
}

/** Host supplies measured feedback and owns physical motion; it must retain the gripper on stop. */
interface TendingElectricEdgeDriver {
  public function joints():Array<Float>;
  public function start(target:Array<Float>):Void;
  public function feedback():TendingElectricEdgeFeedback;
  public function stopAndHold():Void;
}

/** Tick-driven route executor. No synthesized contact feedback and no automatic recovery. */
class TendingElectricEdgeExecutor {
  final edges:Array<Dynamic>;
  final driver:TendingElectricEdgeDriver;
  final jointTolerance:Float;
  final beforeEdge:Null<Dynamic->Void>;
  var index=0;
  var started=false;
  var elapsed=0.0;
  var guardedTravel=0.0;
  var guard:Null<TendingGuardedAction> = null;
  public var stopped(default,null)=false;
  public var complete(default,null)=false;
  public var failure(default,null):Null<String> = null;
  public function edgeIndex():Int return index;
  public function new(edges:Array<Dynamic>,driver:TendingElectricEdgeDriver,jointTolerance:Float=1e-8,?beforeEdge:Dynamic->Void) {
    if(edges.length==0 || driver==null)throw "Electric executor needs edges and feedback driver";
    if(!Math.isFinite(jointTolerance) || jointTolerance<=0)throw "Invalid joint observation tolerance";
    this.edges=edges;this.driver=driver;this.jointTolerance=jointTolerance;this.beforeEdge=beforeEdge;
  }
  function stop(reason:String):Void {
    stopped=true;failure=reason;driver.stopAndHold();
  }
  public function tick(dt:Float):Void {
    if(stopped || complete)return;
    if(!Math.isFinite(dt) || dt<=0){stop("Invalid runtime timestep");return;}
    try {
      var edge:Dynamic=edges[index];
      if(!started) {
        if(!TendingElectricRuntimeProfile.same(driver.joints(),edge.from,jointTolerance)){stop("Route entry differs from measured joints");return;}
        if(edge.kind!="jointEdge" && edge.kind!="guardedJointEdge"){stop("Unknown electric edge kind");return;}
        if(edge.kind=="guardedJointEdge" && guard==null) {
          var policy:Dynamic=edge.contactPolicy;
          var bounds:Dynamic=edge.separating==true ? policy.withdrawal : policy.approach;
          guardedTravel=0;
          var contactRequired=true;
          if(Reflect.hasField(policy.approach,"requireContactAtArrival"))contactRequired=policy.approach.requireContactAtArrival==true;
          guard=new TendingGuardedAction(cast policy.expectedContacts,bounds.maxTravelMm,policy.timeoutSeconds,
            policy.forceLimitN,policy.speedLimitMmPerSecond,contactRequired);
        }
        if(beforeEdge!=null)beforeEdge(edge);
        driver.start(cast edge.to);started=true;elapsed=0;
      }
      elapsed+=dt;
      var observed=driver.feedback();
      if(elapsed>90){stop("Electric edge timeout");return;}
      if(observed==null || !TendingElectricRuntimeProfile.same(observed.joints,observed.joints) || observed.contacts==null ||
        !Math.isFinite(observed.penetrationMm) || observed.penetrationMm<0 || !Math.isFinite(observed.travelMm) || observed.travelMm<0) {
        stop("Invalid runtime feedback");return;
      }
      if(guard!=null) {
        var active:TendingGuardedAction=cast guard;
        var lastGuard=index+1==edges.length || edges[index+1].kind!="guardedJointEdge";
        active.observe(dt,guardedTravel+observed.travelMm,observed.contacts,observed.penetrationMm,observed.arrived && lastGuard,
          edge.separating==true,observed.forceN,observed.speedMmPerSecond);
        if(active.stopped){stop(Std.string(active.failure)+"; contacts="+observed.contacts.join(", ")+"; penetrationMm="+observed.penetrationMm);return;}
      } else if(observed.contacts.length>0 || observed.penetrationMm>0) {stop("Unexpected free-space contact: "+observed.contacts.join(", ")+"; penetrationMm="+observed.penetrationMm);return;}
      if(observed.arrived) {
        if(!TendingElectricRuntimeProfile.same(observed.joints,edge.to,jointTolerance)){stop("Arrival joint mismatch");return;}
        if(guard!=null) {
          var active:TendingGuardedAction=cast guard;
          var lastGuard=index+1==edges.length || edges[index+1].kind!="guardedJointEdge";
          if(lastGuard && !active.complete){stop("Guard did not complete");return;}
          guardedTravel+=observed.travelMm;
          if(lastGuard)guard=null;
        }
        index++;started=false;
        if(index==edges.length)complete=true;
      }
    } catch(error:Dynamic)stop("Runtime feedback/driver failure: "+Std.string(error));
  }
}
