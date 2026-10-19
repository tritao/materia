package processkit.guard;

enum DoorPhase { Idle; Secured; Stopping; Unlocking; Opening; Open; Closing; Locking; Fault; }
enum DoorCommand { Hold; OpenDoor; CloseDoor; }
enum GuardFault { None; SafetyChain; InvalidFeedback; ConflictingRequests; GuardLost; SetupOverspeed; StandstillLost; Timeout; }

/** One sampled field/controller image. Standstill and lock feedback are observations, not commands. */
class GuardInput {
  public var openRequest = false;
  public var closeRequest = false;
  public var cutRequest = false;
  public var cutActive = false;
  public var reset = false;
  public var closed = false;
  public var open = false;
  public var locked = false;
  public var spindleStandstill = false;
  public var axesStandstill = false;
  public var axisSpeed = 0.0; // maximum absolute axis speed, metres/second
  public var safetyReady = false; // E-stop chain healthy
  public var boundaryClear = false;
  public var setupMode = false;
  public var holdToRun = false;
  public function new() {}
}

class GuardOutput {
  public final phase:DoorPhase;
  public final fault:GuardFault;
  public final door:DoorCommand;
  public final lock:Bool;
  public final allowCut:Bool;
  public final allowSetupMotion:Bool;
  /** Stop hazardous spindle/process-axis motion; door actuation has its own command. */
  public final stop:Bool;
  public final setupSpeedLimit:Float;
  public function new(phase, fault, door, lock, allowCut, allowSetupMotion, setupSpeedLimit) {
    this.phase=phase;this.fault=fault;this.door=door;this.lock=lock;
    this.allowCut=allowCut;this.allowSetupMotion=allowSetupMotion;
    this.stop=!allowCut&&!allowSetupMotion;this.setupSpeedLimit=setupSpeedLimit;
  }
}

/** Simulation/control contract, not a certified safety controller. Faults latch until an explicit safe reset. */
class EnclosureGuard {
  public final setupSpeedLimit:Float;
  public final timeoutSeconds:Float;
  public var phase(default,null):DoorPhase = Idle;
  public var fault(default,null):GuardFault = None;
  var elapsed = 0.0;
  var previousOpen = false;
  var previousClose = false;
  var previousReset = false;
  var previousCut = false;
  var cutting = false;
  var setupWasEnabled = false;
  var setupStopping = false;
  var setupStopElapsed = 0.0;

  public function new(setupSpeedLimit:Float, timeoutSeconds:Float) {
    if (!Math.isFinite(setupSpeedLimit)||setupSpeedLimit<=0||!Math.isFinite(timeoutSeconds)||timeoutSeconds<=0)
      throw "Guard needs a positive setup speed limit and feedback timeout";
    this.setupSpeedLimit=setupSpeedLimit;this.timeoutSeconds=timeoutSeconds;
  }

  public function update(i:GuardInput, dt:Float):GuardOutput {
    if(i==null||!Math.isFinite(dt)||dt<=0)throw "Guard needs an input image and positive controller time";
    var opening=i.openRequest&&!previousOpen,closing=i.closeRequest&&!previousClose;
    var reset=i.reset&&!previousReset,cut=i.cutRequest&&!previousCut;
    previousOpen=i.openRequest;previousClose=i.closeRequest;previousReset=i.reset;previousCut=i.cutRequest;
    var valid=Math.isFinite(i.axisSpeed)&&i.axisSpeed>=0&&!(i.open&&i.closed)&&(!i.locked||i.closed);
    var stationary=i.axesStandstill&&i.spindleStandstill;
    if(!valid)trip(InvalidFeedback);
    if(!i.safetyReady||!i.boundaryClear)trip(SafetyChain);
    if(i.openRequest&&i.closeRequest)trip(ConflictingRequests);
    if(i.cutActive&&(!i.closed||!i.locked||i.setupMode))trip(GuardLost);
    if(i.setupMode&&i.axisSpeed>setupSpeedLimit)trip(SetupOverspeed);
    if(fault!=None&&reset&&valid&&i.safetyReady&&i.boundaryClear&&stationary&&i.closed&&i.locked&&
        !i.cutActive&&!i.cutRequest&&!i.openRequest&&!i.closeRequest&&!i.holdToRun) {
      fault=None;enter(Secured);
    }
    if(fault==None) {
      if(phase==Idle) {
        if(i.closed&&i.locked)enter(Secured);
        else if(i.open&&!i.locked)enter(Open);
      }
      if(phase==Secured&&(!i.closed||!i.locked))trip(GuardLost);
      var busy=phase==Stopping||phase==Unlocking||phase==Opening||phase==Closing||phase==Locking;
      if(busy&&(opening||closing)) {
        if(opening&&!wantOpen||closing&&wantOpen)trip(ConflictingRequests);
        opening=false;closing=false; // Repeated requests cannot extend a feedback timeout.
      }
      if(opening&&fault==None) {cutting=false;enter(Stopping);}
      if(closing&&fault==None) {cutting=false;enter(Stopping);}
      // Keep the chosen direction until its feedback confirms completion.
      if(opening)wantOpen=true;
      if(closing)wantOpen=false;
      if(phase==Stopping&&stationary&&!i.cutActive)enter(wantOpen?Unlocking:Closing);
      if(phase==Unlocking&&!i.locked)enter(Opening);
      if((phase==Opening||phase==Closing&&!i.closed)&&i.locked)trip(InvalidFeedback);
      if(phase==Opening&&i.open)enter(Open);
      if(phase==Closing&&i.closed)enter(Locking);
      if(phase==Locking&&i.closed&&i.locked)enter(Secured);
      var accessible=phase==Unlocking||phase==Opening||phase==Open||phase==Closing;
      var setupAllowed=i.setupMode&&i.holdToRun&&phase==Open&&i.spindleStandstill&&!i.cutActive;
      if(setupWasEnabled&&!setupAllowed&&!i.axesStandstill) {setupStopping=true;setupStopElapsed=0;}
      if(i.axesStandstill) {setupStopping=false;setupStopElapsed=0;}
      if(setupStopping) {setupStopElapsed+=dt;if(setupStopElapsed>=timeoutSeconds)trip(Timeout);}
      if(accessible&&(!i.spindleStandstill||(!i.axesStandstill&&!setupAllowed&&!setupStopping)))trip(StandstillLost);
      if(phase==Locking&&!i.closed)trip(GuardLost);
      if(fault==None&&(phase==Stopping||phase==Unlocking||phase==Opening||phase==Closing||phase==Locking)) {
        elapsed+=dt;if(elapsed>=timeoutSeconds)trip(Timeout);
      }
      if(!i.cutRequest||i.setupMode)cutting=false;
      if(cut&&phase==Secured&&!i.setupMode&&!i.openRequest&&!i.closeRequest&&!i.holdToRun)cutting=true;
    }
    var healthy=fault==None&&i.safetyReady&&i.boundaryClear;
    var allowCut=healthy&&cutting&&i.cutRequest&&phase==Secured&&i.closed&&i.locked&&!i.setupMode;
    var setup=healthy&&i.setupMode&&i.holdToRun&&phase==Open&&i.spindleStandstill&&!i.cutActive&&i.axisSpeed<=setupSpeedLimit&&!setupStopping;
    setupWasEnabled=setup;
    var door:DoorCommand=!healthy?Hold:phase==Opening?OpenDoor:phase==Closing?CloseDoor:Hold;
    var lock=!(healthy&&(phase==Unlocking||phase==Opening||phase==Open||phase==Closing));
    return new GuardOutput(phase,fault,door,lock,allowCut,setup,setupSpeedLimit);
  }
  var wantOpen = true;
  function enter(next:DoorPhase):Void {if(phase!=next){phase=next;elapsed=0;}}
  function trip(reason:GuardFault):Void {if(fault==None)fault=reason;cutting=false;enter(Fault);}
}
