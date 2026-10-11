package processkit.skill;

import robotkit.skill.*;
import robotkit.core.RobotSnapshot;
import processkit.feeding.ScrewFeederDevice;
import processkit.screwdriving.ScrewdriverOutputs;

/** Arm motion stays idle while metering, air and physical receipt run; tightening remains disabled. */
class ReceiveBlownScrew implements Skill {
  final outputs:ScrewdriverOutputs;
  final slot:Int;
  final wait:WaitForScrewFeeder;
  public function new(device:ScrewFeederDevice, timeout:Float, outputs:ScrewdriverOutputs, slot:Int) {
    if (outputs == null || slot <= 0) throw "Blow receipt needs its qualified controller output and program slot";
    this.outputs = outputs; this.slot = slot;
    wait = new WaitForScrewFeeder(device,timeout,"receiving nosepiece");
  }
  public function start():Void {outputs.start(false); outputs.reverse(false); outputs.selectProgram(slot); outputs.vacuum(true); wait.start();}
  public function update(snapshot:RobotSnapshot, durationSeconds:Float):SkillStatus return wait.update(snapshot,durationSeconds);
  public function cancel():Void {wait.cancel(); outputs.start(false); outputs.reverse(false);}
  public function status():SkillStatus return wait.status();
  public function result():Null<SkillResult> return wait.result();
}
