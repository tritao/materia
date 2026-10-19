package tests;

import app.EnclosureGuardStatus;
import app.EnclosureGuardStatus.EnclosureGuardObservation;
import app.EnclosureGuardStatus.EnclosureRuntimeState;

class EnclosureGuardStatusTests {
  static var checks=0;
  static function check(ok:Bool,why:String):Void {checks++;if(!ok)throw why;}
  static function observation():EnclosureGuardObservation return {
    phase:"Idle",message:"Guard feedback",position:0,closed:false,open:false,locked:false,
    spindleStandstill:false,axesStandstill:false,canStart:false,allowCut:false,fault:false,interrupted:false,
    feedbackSampled:false,initialized:false,doorMoving:false
  };
  public static function run():Void {
    var o=observation(),paused=new EnclosureGuardStatus(o,false);
    check(paused.state==Paused&&paused.canResume,"Unsampled prepared controller is explicitly paused");
    check(!paused.canOpen&&!paused.canClose&&!paused.canStartCut&&!paused.canReset,"Paused controller offers no live guard actions");
    check(paused.doorMessage.indexOf("not been sampled")>=0&&paused.motionMessage.indexOf("not been sampled")>=0,"Unsampled status cannot claim moving doors or stopped axes");
    var initial=paused.withRunning(true);
    check(initial.state==Initializing&&!initial.canResume&&!initial.canStartCut,"Starting before feedback explicitly initializes");
    check(paused.state==Paused,"Transport publication does not mutate an older snapshot");
    o.phase="Open";o.open=true;o.position=0.21;o.feedbackSampled=true;
    check(new EnclosureGuardStatus(o,true).state==Initializing,"End-switch feedback alone is not startup readiness");
    o.initialized=true;o.axesStandstill=true;o.spindleStandstill=true;
    var opened=new EnclosureGuardStatus(o,true);
    check(opened.state==Ready&&opened.canClose&&!opened.canOpen&&!opened.canStartCut,"Ready open guard permits closing but never cutting");
    o.phase="Secured";o.open=false;o.closed=true;o.locked=true;o.canStart=true;
    var secured=new EnclosureGuardStatus(o,true);
    check(secured.canOpen&&secured.canStartCut&&!secured.canClose,"Ready secured guard publishes permitted actions");
    check(opened.withRunning(false).open&&!opened.withRunning(false).closed,"Published observation is insulated from caller mutation");
    var stopped=secured.withRunning(false);
    check(stopped.state==Paused&&stopped.initialized&&stopped.feedbackSampled&&!stopped.canStartCut,"Pause retains measured feedback while disabling live actions");
    check(stopped.withRunning(true).state==Ready,"Resume preserves completed initialization");
    o.phase="Fault";o.fault=true;o.message="Guard stopped: feedback lost";
    var fault=new EnclosureGuardStatus(o,false);
    check(fault.state==Faulted&&fault.message==o.message&&fault.canResume&&fault.canResetSimulation,"Fault is visible even while playback is paused");
    check(!fault.canOpen&&!fault.canClose&&!fault.canStartCut&&!fault.canReset,"Paused fault cannot grant guard actions");
    check(fault.withRunning(true).canReset,"Running secured standstill fault can request reset");
    o.axesStandstill=false;
    check(!new EnclosureGuardStatus(o,true).canReset,"Reset permission requires measured standstill");
    Sys.println('Enclosure status: $checks checks passed');
  }
}
