typedef TendingContactLoad = {
  var axialN:Float;
  var resultantN:Float;
  var maximumContactN:Float;
  var summedMagnitudesN:Float;
  var vector:Array<Float>;
}

/** Bounded, one-way contact search. A contact requests a hold, never a retry. */
class TendingContactSearchAction {
  public var holding(default,null)=false;
  public var complete(default,null)=false;
  public var stopped(default,null)=false;
  public var failure(default,null):Null<String> = null;
  final expected:Array<String>;
  final travelLimit:Float;
  final timeout:Float;
  final forceLimit:Float;
  final threshold:Float;
  final speedLimit:Float;
  final totalSpeedLimit:Float;
  final stableRequired:Int;
  final restGapLimit:Null<Float>;
  var elapsed=0.0;
  var holdElapsed=0.0;
  var stable=0;
  public function new(policy:Dynamic) {
    expected=Reflect.hasField(policy,"expectedContacts") ? cast policy.expectedContacts : [policy.contact];
    travelLimit=policy.commandTravelMm;timeout=policy.timeoutSeconds;
    forceLimit=policy.forceLimitN;threshold=policy.contactThresholdN;speedLimit=policy.speedLimitMmPerSecond;
    totalSpeedLimit=policy.totalPointSpeedLimitMmPerSecond;
    stableRequired=policy.stableSamples;
    restGapLimit=Reflect.hasField(policy,"restGapLimitMm") ? (cast policy.restGapLimitMm:Float) : null;
    if(restGapLimit!=null && (!Math.isFinite(restGapLimit) || restGapLimit<=0 || restGapLimit>0.01 || expected.length!=1))
      throw "Touch-and-rest locating needs one stop and at most 0.01 mm observed gap";
    if(expected==null || expected.length==0 || stableRequired<1 || policy.numericalPenetrationLimitMm!=0.001)
      throw "Invalid locating identity or penetration policy";
    for(i in 0...expected.length)if(expected[i]==null || expected[i].length==0 || expected.indexOf(expected[i])!=i)throw "Invalid contact search identity";
    for(v in [travelLimit,timeout,forceLimit,threshold,speedLimit,totalSpeedLimit])if(!Math.isFinite(v) || v<=0)throw "Invalid locating bound";
    if(totalSpeedLimit<speedLimit)throw "Total point speed cannot be below the closing point-speed limit";
    if(threshold>=forceLimit)throw "Locating threshold must be below its force limit";
    {
      var command:Float=policy.commandSpeedMmPerSecond;
      if(!Math.isFinite(command) || command<=0 || command>speedLimit*0.25)
        throw "Locating command needs explicit margin below measured point-speed limit";
    }
  }
  public function observe(dt:Float,travelMm:Float,contacts:Array<String>,forceN:Float,penetrationMm:Float,
      speedMmPerSecond:Float,atRest:Bool,?totalPointSpeedMmPerSecond:Float,?observedGapMm:Float):Void {
    if(stopped || complete)return;
    var totalSpeed:Float=totalPointSpeedMmPerSecond==null ? speedMmPerSecond : (cast totalPointSpeedMmPerSecond:Float);
    elapsed+=dt;if(holding)holdElapsed+=dt;
    if(!Math.isFinite(dt) || dt<=0 || contacts==null) {stop("invalid locating feedback");return;}
    for(v in [travelMm,forceN,penetrationMm,speedMmPerSecond,totalSpeed])if(!Math.isFinite(v) || v<0) {stop("invalid locating feedback");return;}
    var loaded=false;
    if(forceN>=threshold)for(name in contacts)if(expected.indexOf(name)>=0)loaded=true;
    if(elapsed>timeout || holdElapsed>2)stop("locating timeout");
    else if(travelMm>travelLimit+0.000001)stop("locating travel exceeded");
    else if(penetrationMm>0.001)stop("locating penetration exceeded");
    else if(forceN>forceLimit)stop("locating force exceeded");
    else if(!holding && !loaded && speedMmPerSecond>speedLimit)stop("locating closing speed exceeded");
    else if(totalSpeed>totalSpeedLimit)stop("locating total point speed exceeded");
    else {
      for(name in contacts)if(expected.indexOf(name)<0)stop("unexpected locating contact: "+name);
      if(stopped)return;
      if(loaded)holding=true;
      var nearStop=false;
      if(holding && restGapLimit!=null) {
        if(observedGapMm==null || !Math.isFinite(observedGapMm) || observedGapMm< -0.001) {stop("invalid observed stop gap");return;}
        nearStop=observedGapMm<=restGapLimit;
      }
      if(holding && (restGapLimit!=null ? nearStop : loaded) && atRest)stable++;else stable=0;
      if(stable>=stableRequired)complete=true;
    }
  }
  public function finishSearch():Void {if(!complete && !holding && !stopped)stop("expected contact not found within search corridor");}
  function stop(reason:String):Void {stopped=true;failure=reason;}
  /** Forces are world-axis forces on bodyA; body ordering must not change the load. */
  public static function contactLoad(contacts:Array<Dynamic>,body:Int,axis:Array<Float>):TendingContactLoad {
    if(axis.length!=3)throw "Contact load needs three axis coordinates";
    var norm=0.0;for(v in axis){if(!Math.isFinite(v))throw "Invalid contact axis";norm+=v*v;}
    if(Math.abs(norm-1)>0.000001)throw "Contact load axis must be unit length";
    var vector=[0.0,0,0],axial=0.0,maximum=0.0,summed=0.0;
    for(contact in contacts) {
      if((contact.bodyA==body)==(contact.bodyB==body))throw "Contact load must belong to exactly one selected body";
      var force:Array<Float> = cast contact.force;
      if(force==null || force.length!=3)throw "Contact load requires a measured force vector";
      var sign=contact.bodyA==body ? 1.0 : -1.0,squared=0.0,projection=0.0;
      for(i in 0...3){var v=force[i];if(!Math.isFinite(v))throw "Contact force must be finite";vector[i]+=sign*v;squared+=v*v;projection+=v*axis[i];}
      axial+=Math.abs(projection);var magnitude=Math.sqrt(squared);maximum=Math.max(maximum,magnitude);summed+=magnitude;
    }
    var squared=0.0;for(v in vector)squared+=v*v;
    return {axialN:axial,resultantN:Math.sqrt(squared),maximumContactN:maximum,summedMagnitudesN:summed,vector:vector};
  }
  /** Upper bound on closing velocity of any hull point, including rotation. */
  public static function closingPointSpeed(normalVelocity:Float,angularSpeed:Float,radius:Float):Float
    return Math.max(0,-normalVelocity)+radius*angularSpeed;
  public static function checks():Int {
    var p={contact:"vise/end-stop",commandTravelMm:0.65,timeoutSeconds:180.0,forceLimitN:5.0,
      contactThresholdN:0.05,commandSpeedMmPerSecond:0.0025,speedLimitMmPerSecond:0.01,totalPointSpeedLimitMmPerSecond:0.05,stableSamples:10,numericalPenetrationLimitMm:0.001};
    var good=new TendingContactSearchAction(p);
    good.observe(0.01,0.2,[],0,0,0.005,false);
    if(good.holding || good.complete)throw "Free travel claimed locating contact";
    good.observe(0.01,0.5,[p.contact],0.2,0.0001,0.005,false);
    if(!good.holding || good.complete)throw "First contact must hold before completion";
    for(_ in 0...10)good.observe(0.01,0.5,[p.contact],0.2,0.0001,0,true);
    if(!good.complete || good.stopped)throw "Stable end-stop locating failed";
    for(mode in 0...8) {
      var bad=new TendingContactSearchAction(p);
      bad.observe(mode==0 ? 181 : 0.01,mode==1 ? 0.651 : 0.5,mode==2 ? ["wrong fixture"] : mode==5 ? [] : [p.contact],
        mode==3 ? 5.1 : mode==5 ? 0 : 0.2,mode==4 ? 0.0011 : 0,mode==5 ? 0.011 : 0,false);
      if(mode==6){bad.observe(2.1,0.5,[],0,0,0,true);}
      if(mode==7){bad=new TendingContactSearchAction(p);bad.finishSearch();}
      if(!bad.stopped || bad.complete)throw "Locating fault did not latch: "+mode;
      bad.observe(0.01,0.5,[p.contact],0.2,0,0,true);
      if(!bad.stopped || bad.complete)throw "Locating fault retried itself";
    }
    if(closingPointSpeed(0.000156,0,0.04)!=0 || closingPointSpeed(-0.000156,0,0.04)!=0.000156 ||
      Math.abs(closingPointSpeed(0.000156,0.01,0.04)-0.0004)>1e-12)throw "Directional point-speed bound lost recoil or rotation";
    var recoil=new TendingContactSearchAction(p);recoil.observe(0.01,0.5,[],0,0,0,false,0.051);
    if(!recoil.stopped)throw "Outward recoil bypassed its total-speed limit";
    var settling=new TendingContactSearchAction(p);
    settling.observe(0.01,0.5,[p.contact],0.2,0.0001,0.005,false,0.005);
    settling.observe(0.01,0.5,[p.contact],0.2,0.0001,0.02,false,0.03);
    if(settling.stopped || settling.complete || !settling.holding)throw "Bounded contact settling must hold without releasing";
    settling.observe(0.01,0.5,[p.contact],0.2,0.0001,0.02,false,0.051);
    if(!settling.stopped)throw "Contact hold bypassed its total point-speed ceiling";
    var touch=new TendingContactSearchAction(p);
    touch.observe(0.01,0.5,[p.contact],0.2,0.0001,0.02,false,0.03);
    if(touch.stopped || !touch.holding || touch.complete)throw "First loaded contact must request a bounded hold";
    var support:Dynamic=haxe.Json.parse(haxe.Json.stringify(p));support.expectedContacts=["parallel/0","parallel/1"];
    var multiple=new TendingContactSearchAction(support);
    for(_ in 0...10)multiple.observe(0.01,0.5,["parallel/1"],0.2,0.0001,0,true);
    if(!multiple.complete || multiple.stopped)throw "Named support subset did not establish a stable contact search";
    var wrong=new TendingContactSearchAction(support);wrong.observe(0.01,0.5,["wrong support"],0.2,0,0,false);
    if(!wrong.stopped)throw "Support search accepted an unrelated contact";
    var forces:Array<Dynamic> = [{bodyA:7,bodyB:8,force:[1.7,0.0,2.35]},{bodyA:8,bodyB:7,force:[1.55,0.0,-1.6]}];
    var load=contactLoad(forces,7,[0.0,0,1]);
    if(Math.abs(load.axialN-3.95)>1e-12 || load.resultantN>=5 || load.summedMagnitudesN<=5 || load.maximumContactN>=5)
      throw "Opposing friction must cancel only in the resultant, not axial/contact loads";
    var swapped:Array<Dynamic> = [{bodyA:8,bodyB:7,force:[-1.7,0.0,-2.35]},{bodyA:7,bodyB:8,force:[-1.55,0.0,1.6]}];
    var other=contactLoad(swapped,7,[0.0,0,1]);
    if(Math.abs(load.resultantN-other.resultantN)>1e-12)throw "Body ordering changed measured resultant";
    var rejected=false;try contactLoad(forces,99,[0.0,0,1]) catch(_:Dynamic)rejected=true;
    if(!rejected)throw "Unrelated-body force was accepted";
    var datum:Dynamic=haxe.Json.parse(haxe.Json.stringify(p));datum.restGapLimitMm=0.01;
    var recoiled=new TendingContactSearchAction(datum);
    recoiled.observe(0.01,0.5,[p.contact],0.2,0.0001,0.005,false,0.005,-0.0001);
    for(_ in 0...10)recoiled.observe(0.01,0.5,[],0,0,0,true,0,0.002);
    if(!recoiled.complete || recoiled.stopped)throw "Measured touch followed by bounded stationary recoil did not locate";
    var untouched=new TendingContactSearchAction(datum);
    for(_ in 0...10)untouched.observe(0.01,0.5,[],0,0,0,true,0,0.002);
    if(untouched.holding || untouched.complete)throw "Near-stop geometry fabricated a touch";
    for(gap in [0.011,0.02]) {
      var distant=new TendingContactSearchAction(datum);
      distant.observe(0.01,0.5,[p.contact],0.2,0,0,false,0,0);
      for(_ in 0...10)distant.observe(0.01,0.5,[],0,0,0,true,0,gap);
      if(distant.complete)throw "Touch history admitted excessive recoil";
      distant.observe(2.1,0.5,[],0,0,0,true,0,gap);
      if(!distant.stopped)throw "Distant contact hold failed to latch a stop";
    }
    var missingGap=new TendingContactSearchAction(datum);
    missingGap.observe(0.01,0.5,[p.contact],0.2,0,0,true);
    if(!missingGap.stopped)throw "Touch-and-rest locating admitted missing gap observation";
    return 25;
  }
}
