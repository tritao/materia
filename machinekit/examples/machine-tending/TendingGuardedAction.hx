/** Contact supervisor over an already approved path, with latched stop/hold on failure. */
class TendingGuardedAction {
  public final expected:Array<String>;
  public final maxTravelMm:Float;
  public final timeoutSeconds:Float;
  public final forceLimitN:Null<Float>;
  public final speedLimitMmPerSecond:Null<Float>;
  public final requireContactAtArrival:Bool;
  public var stopped(default,null):Bool=false;
  public var complete(default,null):Bool=false;
  public var failure(default,null):Null<String> =null;
  var elapsed:Float=0;
  public function new(expected:Array<String>,maxTravelMm:Float,timeoutSeconds:Float,
      ?forceLimitN:Float,?speedLimitMmPerSecond:Float,requireContactAtArrival:Bool=true) {
    if(expected.length==0 || !Math.isFinite(maxTravelMm) || maxTravelMm<=0 ||
      !Math.isFinite(timeoutSeconds) || timeoutSeconds<=0)throw "Guarded action needs contact identities and finite bounds";
    for(v in [forceLimitN,speedLimitMmPerSecond])if(v!=null && (!Math.isFinite(v) || v<=0))throw "Qualified force/speed limits must be positive";
    this.expected=expected.copy();this.maxTravelMm=maxTravelMm;this.timeoutSeconds=timeoutSeconds;
    this.forceLimitN=forceLimitN;this.speedLimitMmPerSecond=speedLimitMmPerSecond;
    this.requireContactAtArrival=requireContactAtArrival;
  }
  public function hasForceAndSpeedLimits():Bool return forceLimitN!=null && speedLimitMmPerSecond!=null;
  public function observe(dt:Float,travelMm:Float,contacts:Array<String>,penetrationMm:Float,
      arrived:Bool,separating:Bool,?forceN:Float,?speedMmPerSecond:Float):Void {
    if(stopped || complete)return;
    elapsed+=dt;
    if(!Math.isFinite(dt) || dt<=0 || !Math.isFinite(travelMm) || travelMm<0 || !Math.isFinite(penetrationMm) || penetrationMm<0)stop("invalid feedback");
    else if(elapsed>timeoutSeconds)stop("timeout");
    else if(travelMm>maxTravelMm+1e-6)stop("approved travel exceeded");
    else if(penetrationMm>0.001)stop("penetration exceeds numerical contact tolerance");
    else if(forceLimitN!=null && (forceN==null || !Math.isFinite(forceN) || Math.abs(forceN)>forceLimitN))stop("force limit or feedback failure");
    else if(speedLimitMmPerSecond!=null && (speedMmPerSecond==null || !Math.isFinite(speedMmPerSecond) || Math.abs(speedMmPerSecond)>speedLimitMmPerSecond))stop("speed limit or feedback failure");
    else {
      for(contact in contacts)if(expected.indexOf(contact)<0)stop("unexpected contact: "+contact);
      if(!stopped && arrived) {
        if(separating ? contacts.length==0 : !requireContactAtArrival || contacts.length>0)complete=true;
        else stop(separating ? "contact did not separate" : "expected contact missing");
      }
    }
  }
  function stop(reason:String):Void {stopped=true;failure=reason;}
  public function result():Dynamic return {complete:complete,stopped:stopped,failure:failure,
    stopResponse:"stop robot motion, retain payload/gripper state, inhibit further production; no automatic retry or blind retreat",
    hardwareReady:false,hasForceAndSpeedLimits:hasForceAndSpeedLimits(),maxTravelMm:maxTravelMm,timeoutSeconds:timeoutSeconds,
    numericalPenetrationLimitMm:0.001,requireContactAtArrival:requireContactAtArrival,forceLimitN:forceLimitN,speedLimitMmPerSecond:speedLimitMmPerSecond};
  public static function checks():Int {
    var good=new TendingGuardedAction(["stock/seat"],10,5);good.observe(0.01,10,["stock/seat"],0,true,false);
    if(!good.complete || good.hasForceAndSpeedLimits())throw "Model contact completion or hardware qualification is wrong";
    var separated=new TendingGuardedAction(["stock/seat"],10,5);separated.observe(0.01,10,[],0,true,true);
    if(!separated.complete)throw "Guarded separation did not complete";
    for(mode in 0...10) {
      // Force/speed values are fault-test fixtures, never a hardware configuration.
      var guard=new TendingGuardedAction(["stock/seat"],10,5,mode>=5 ? 10 : null,mode==6 || mode==8 ? 2 : null);
      guard.observe(mode==0 ? 6 : 0.01,mode==1 ? 11 : 10,mode==2 ? ["robot/fixture"] : mode==3 ? [] : ["stock/seat"],
        mode==4 ? 0.002 : mode==9 ? -0.001 : 0,true,false,mode==7 ? null : mode==5 ? 11 : 0,mode==8 ? null : mode==6 ? 3 : 0);
      if(!guard.stopped || guard.complete)throw "Guarded fault did not latch";
      guard.observe(0.01,0,["stock/seat"],0,true,false,0,0);if(!guard.stopped)throw "Guarded failure retried itself";
    }
    var stuck=new TendingGuardedAction(["stock/seat"],10,5);stuck.observe(0.01,10,["stock/seat"],0,true,true);
    if(!stuck.stopped)throw "Guarded separation accepted persistent contact";
    var carried=new TendingGuardedAction(["stock/seat"],10,5,null,null,false);
    carried.observe(0.01,10,[],0,true,false);
    if(!carried.complete)throw "A fully gripped placement requires support force before release";
    for(contact in ["wrong fixture","stock/seat"]) {
      var placement=new TendingGuardedAction(["stock/seat"],10,5,null,null,false);
      placement.observe(0.01,10,[contact],contact=="stock/seat" ? 0.002 : 0,true,false);
      if(!placement.stopped)throw "Optional pre-release contact bypassed collision supervision";
    }
    return 16;
  }
}
