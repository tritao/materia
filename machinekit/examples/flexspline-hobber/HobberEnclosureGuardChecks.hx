import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import processkit.guard.EnclosureGuard.GuardInput;
import processkit.guard.EnclosureGuard.GuardOutput;
import processkit.guard.EnclosureGuard.GuardFault;
import machinekit.enclosure.SlidingDoor;

/** Exercises the fitted door's real assembly coordinate, sensor metadata, and motion permission consumer. */
class HobberEnclosureGuardChecks {
  static var checks=0;
  static function require(ok:Bool,why:String):Void {checks++;if(!ok)throw why;}
  public static function run(assembly:machinekit.assembly.MachineAssembly,stroke:Float):Dynamic {
    var model=new AssemblyModel("mm");assembly.addTo(model,"");var state=model.initialState();
    var cutEnabled=false,stop=true,locked=true,lockDelay=0.0;
    var controller=new HobberEnclosureControl(state,stroke,o->{cutEnabled=o.allowCut;stop=o.stop;});
    var input=new GuardInput();input.safetyReady=true;input.boundaryClear=true;
    input.spindleStandstill=true;input.axesStandstill=true;input.locked=true;
    var dt=0.01;
    function tick():GuardOutput {
      input.locked=locked;
      var output=controller.update(input,dt);
      // Independent simulated lock feedback, delayed after its command; not inferred from door position alone.
      if(output.lock!=locked) {
        lockDelay+=dt;
        if(lockDelay>=0.05) {locked=output.lock&&input.closed;lockDelay=0;}
      } else lockDelay=0;
      return output;
    }
    var output=tick();require(input.closed&&!input.open&&!cutEnabled,"Initial door is closed and idle");
    input.cutRequest=true;output=tick();require(cutEnabled&&!stop,"Safety consumer receives cut permission");
    input.cutActive=true;input.spindleStandstill=false;input.openRequest=true;output=tick();
    require(stop&&!cutEnabled&&output.lock&&state.joint(HobberEnclosureControl.JOINT)==0,"Spinning spindle keeps door stationary and locked");
    input.openRequest=false;
    for(_ in 0...25) {tick();require(state.joint(HobberEnclosureControl.JOINT)==0,"No timed standstill assumption");}
    input.spindleStandstill=true;input.axesStandstill=false;input.cutActive=false;
    for(_ in 0...10)tick();require(locked,"Moving axes still retain the lock");
    input.axesStandstill=true;
    var elapsed=0.0,previous=state.joint(HobberEnclosureControl.JOINT);
    while(!input.open&&elapsed<10) {
      output=tick();var now=state.joint(HobberEnclosureControl.JOINT);
      require(now>=previous&&now-previous<=SlidingDoor.SPEED_MM_PER_SECOND*dt+1e-8,"Door respects actuator rate");
      require(!cutEnabled,"Cutting stays inhibited throughout opening");
      previous=now;elapsed+=dt;
    }
    require(input.open&&!input.closed&&Math.abs(state.joint(HobberEnclosureControl.JOINT)-stroke)<1e-8,"Actual joint reaches full open travel");
    require(elapsed>=stroke/SlidingDoor.SPEED_MM_PER_SECOND,"Open feedback cannot arrive before physical travel");
    input.setupMode=true;input.holdToRun=true;input.axisSpeed=0.02;input.axesStandstill=false;
    output=tick();require(output.allowSetupMotion&&!cutEnabled,"Door-open setup is reduced-speed and hold-to-run only");
    input.axesStandstill=true;input.axisSpeed=0;input.holdToRun=false;input.setupMode=false;
    input.cutRequest=false;input.closeRequest=true;tick();input.closeRequest=false;elapsed=0;
    while((!input.closed||!locked)&&elapsed<10){tick();elapsed+=dt;require(!cutEnabled,"Closing cannot enable a stale cut request");}
    output=tick();require(input.closed&&locked&&output.phase==Secured,"Closed and lock feedback confirm the secured state");
    input.cutRequest=true;tick();require(cutEnabled,"Fresh cut request works after reclosure");
    input.safetyReady=false;output=tick();require(output.fault==SafetyChain&&stop&&!cutEnabled,"E-stop output immediately removes cut permission");
    input.safetyReady=true;input.cutRequest=false;input.reset=true;output=tick();
    require(output.fault==None&&!cutEnabled,"Safe reset does not restart the cut");
    var definition=model.definition("hobber-guard-checks");
    var switches=[for(sensor in definition.sensors)if(sensor.id=="enclosure/door/closed"||sensor.id=="enclosure/door/open")sensor];
    require(switches.length==2,"Exported assembly contains both door end switches");
    for(sensor in switches)require(sensor.joint==HobberEnclosureControl.JOINT&&sensor.kind=="joint_switch","Door feedback binds to the actual sliding joint");
    var roundtrip=machinekit.assembly.MachineAssembly.decode(assembly.encode()),reopened=new AssemblyModel("mm");roundtrip.addTo(reopened,"");
    require([for(s in reopened.definition("reopened").sensors)if(s.id=="enclosure/door/closed"||s.id=="enclosure/door/open")s].length==2,"Door switches survive assembly save/reopen");
    Sys.println('Hobber door guard: $checks joint travel and interlock checks passed');
    return {checks:checks,strokeMm:stroke,doorSpeedMmPerSecond:SlidingDoor.SPEED_MM_PER_SECOND,setupSpeedLimitMetresPerSecond:controller.guard.setupSpeedLimit,
      standstillConfirmedUnlock:true,closedAndLockedCutGate:true,emergencyStop:true,feedbackSource:"assembly joint switches; independent simulated lock and standstill feedback",
      scope:"Kinematic controller simulation; not a hardware safety qualification or a desktop cutting-runtime binding"};
  }
  public static function main():Void {
    var fitted=HobberEnclosurePreview.assembled(),report=run(fitted.assembly,fitted.shell.layout.stroke);
    var output=Sys.getCwd()+"/machinekit/examples/flexspline-hobber/build/enclosure";
    sys.FileSystem.createDirectory(output);sys.io.File.saveContent(output+"/guard-checks.json",haxe.Json.stringify(report,null,"  "));
  }
}
