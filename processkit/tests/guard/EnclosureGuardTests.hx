import processkit.guard.EnclosureGuard;
import processkit.guard.EnclosureGuard.GuardInput;
import processkit.guard.EnclosureGuard.GuardFault;
import processkit.guard.EnclosureGuard.DoorPhase;
import processkit.guard.EnclosureGuard.DoorCommand;

class EnclosureGuardTests {
  static var checks=0;
  static function require(ok:Bool,why:String):Void {checks++;if(!ok)throw why;}
  static function ready():GuardInput {
    var i=new GuardInput();i.safetyReady=true;i.boundaryClear=true;i.closed=true;i.locked=true;
    i.spindleStandstill=true;i.axesStandstill=true;return i;
  }
  public static function main():Void {
    var g=new EnclosureGuard(0.025,1),i=ready(),o=g.update(i,0.01);
    require(!o.allowCut&&o.stop,"Startup does not start a cut");
    i.cutRequest=true;o=g.update(i,0.01);require(o.allowCut,"Closed and locked permits a requested cut");
    i.openRequest=true;i.cutActive=true;i.spindleStandstill=false;o=g.update(i,0.01);
    require(o.phase==Stopping&&o.lock&&o.stop&&!o.allowCut&&o.door==Hold,"Opening while cutting requests stop and retains lock");
    i.openRequest=false;i.cutActive=false;i.spindleStandstill=true;i.axesStandstill=false;o=g.update(i,0.01);
    require(o.lock&&o.phase==Stopping,"Spindle standstill alone cannot unlock");
    i.spindleStandstill=false;i.axesStandstill=true;o=g.update(i,0.01);
    require(o.lock&&o.phase==Stopping,"Axis standstill alone cannot unlock");
    i.spindleStandstill=true;o=g.update(i,0.01);
    require(!o.lock&&o.door==Hold&&o.phase==Unlocking,"Both standstill confirmations permit unlock command only");
    o=g.update(i,0.01);require(o.door==Hold,"Lock command is not lock feedback");
    i.locked=false;o=g.update(i,0.01);require(o.door==OpenDoor&&!o.allowCut,"Confirmed unlock permits opening");
    i.closed=false;o=g.update(i,0.01);require(o.phase==Opening&&!o.allowCut,"Intermediate door travel cannot enable cutting");
    i.open=true;o=g.update(i,0.01);require(o.phase==Open&&o.door==Hold,"Open feedback stops actuator");
    i.setupMode=true;i.holdToRun=true;o=g.update(i,0.01);
    require(o.allowSetupMotion&&!o.allowCut&&o.setupSpeedLimit==0.025,"Setup allows held reduced-speed axes, never cutting");
    i.axesStandstill=false;i.axisSpeed=0.02;o=g.update(i,0.01);require(o.allowSetupMotion,"Measured setup speed below policy limit is allowed");
    i.holdToRun=false;o=g.update(i,0.01);require(o.stop&&!o.allowSetupMotion,"Releasing hold-to-run stops moving axes");
    require(o.fault==None,"A normal hold release requests controlled stopping without a fault");
    i.axesStandstill=true;i.axisSpeed=0;o=g.update(i,0.01);require(o.stop&&!o.allowSetupMotion,"Confirmed setup stop remains idle");

    g=new EnclosureGuard(0.025,1);i=ready();g.update(i,0.01);
    i.openRequest=true;g.update(i,0.01);i.openRequest=false;i.locked=false;g.update(i,0.01);
    i.closed=false;i.open=true;g.update(i,0.01);i.closeRequest=true;o=g.update(i,0.01);
    require(o.door==CloseDoor&&!o.allowCut,"Close request drives actual door closure");
    i.closeRequest=false;i.open=false;i.closed=true;o=g.update(i,0.01);
    require(o.lock&&o.phase==Locking&&!o.allowCut,"Closed switch alone cannot permit cutting");
    i.locked=true;o=g.update(i,0.01);require(o.phase==Secured&&!o.allowCut,"Lock feedback does not start a cut");
    i.cutRequest=true;o=g.update(i,0.01);require(o.allowCut,"Fresh request after closure can cut");
    i.locked=false;o=g.update(i,0.01);require(o.fault==GuardLost&&o.stop&&!o.allowCut,"Lost lock trips active permission");
    i.locked=true;i.cutRequest=false;i.reset=true;o=g.update(i,0.01);require(o.fault==None&&!o.allowCut,"Safe reset requires a fresh start");
    i.cutRequest=true;o=g.update(i,0.01);require(o.allowCut,"Explicit restart after reset works");
    i.safetyReady=false;o=g.update(i,0.01);require(o.fault==SafetyChain&&o.stop&&!o.allowCut,"E-stop opens the permission chain");
    i.safetyReady=true;i.reset=false;g.update(i,0.01);i.reset=true;o=g.update(i,0.01);
    require(o.fault!=None,"Reset with held cut request is refused");

    for(kind in 0...5) {
      g=new EnclosureGuard(0.025,0.1);i=ready();g.update(i,0.01);
      switch kind {
        case 0: i.open=true;
        case 1: i.openRequest=true;i.closeRequest=true;
        case 2: i.boundaryClear=false;
        case 3: i.setupMode=true;i.axisSpeed=0.026;
        case 4: i.openRequest=true;i.spindleStandstill=false;
      }
      o=g.update(i,0.11);
      require(o.fault!=None&&o.stop&&!o.allowCut&&o.door==Hold,'Fault case $kind holds the door and removes motion permission');
    }
    // A cut request issued while open must not become a delayed automatic start.
    g=new EnclosureGuard(0.025,1);i=ready();i.closed=false;i.open=true;i.locked=false;i.cutRequest=true;
    require(!g.update(i,0.01).allowCut,"Open guard denies cut request");
    i.closeRequest=true;g.update(i,0.01);i.open=false;i.closed=true;i.closeRequest=false;g.update(i,0.01);
    i.locked=true;require(!g.update(i,0.01).allowCut,"A held request cannot restart after closing");
    // Holding start across a mode change must not restart an automatic cut.
    g=new EnclosureGuard(0.025,1);i=ready();g.update(i,0.01);i.cutRequest=true;
    require(g.update(i,0.01).allowCut,"Automatic cut starts from a fresh request");
    i.setupMode=true;require(!g.update(i,0.01).allowCut,"Setup mode removes automatic cutting permission");
    i.setupMode=false;require(!g.update(i,0.01).allowCut,"Leaving setup does not resume a held cut request");
    // A released enable must stop the axes; a missing stop confirmation latches a fault.
    g=new EnclosureGuard(0.025,0.1);i=ready();i.closed=false;i.open=true;i.locked=false;
    i.setupMode=true;i.holdToRun=true;g.update(i,0.01);
    i.axesStandstill=false;i.axisSpeed=0.02;g.update(i,0.01);i.holdToRun=false;
    require(g.update(i,0.01).fault==None,"A setup stop has time to decelerate");
    o=g.update(i,0.11);require(o.fault==Timeout&&o.stop,"Missing setup standstill confirmation faults");
    // Opening must stop if standstill feedback is lost, and missing lock feedback must time out.
    for(axis in [false,true]) {
      g=new EnclosureGuard(0.025,1);i=ready();g.update(i,0.01);
      i.openRequest=true;g.update(i,0.01);i.openRequest=false;i.locked=false;g.update(i,0.01);
      if(axis)i.axesStandstill=false;else i.spindleStandstill=false;
      o=g.update(i,0.01);require(o.fault==StandstillLost&&o.door==Hold&&o.stop,"Lost standstill stops door travel");
      i.reset=true;o=g.update(i,0.01);require(o.fault!=None,"Reset while moving is refused");
    }
    g=new EnclosureGuard(0.025,0.1);i=ready();g.update(i,0.01);
    i.openRequest=true;g.update(i,0.01);i.openRequest=false;o=g.update(i,0.11);
    require(o.fault==Timeout&&o.lock&&o.door==Hold,"Missing unlock confirmation times out without moving the leaf");
    g=new EnclosureGuard(0.025,0.1);i=ready();g.update(i,0.01);
    for(step in 0...12) {i.openRequest=step%2==0;o=g.update(i,0.01);}
    require(o.fault==Timeout,"Repeated open pulses cannot postpone missing-lock-feedback timeout");
    Sys.println('Enclosure guard: $checks checks passed');
  }
}
