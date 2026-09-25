package robotkit.skill;

import robotkit.navigation.Navigator;
import robotkit.perception.DockingTarget;
import robotkit.perception.PerceptionSnapshot;
import robotkit.power.Power;
import robotkit.world.RobotSnapshot;

private enum ChargeStage {
  Docking;
  WaitingForCharge;
}

/** Docks at a detected charging target and waits for the requested battery charge. */
class Charge implements Skill {
  public final dock:Dock;
  public final power:Power;
  public final targetChargeFraction:Float;
  final lifecycle:SkillLifecycle = new SkillLifecycle();
  var stage:ChargeStage = Docking;

  public function new(navigator:Navigator, target:DockingTarget, power:Power,
      targetChargeFraction:Float,
      observePerception:RobotSnapshot -> PerceptionSnapshot,
      ?minimumConfidence:Float = 0.5) {
    if (navigator == null || target == null || power == null ||
        observePerception == null ||
        !Math.isFinite(targetChargeFraction) || targetChargeFraction <= 0.0 ||
        targetChargeFraction > 1.0)
      throw "Charge requires navigation, docking target, power, and a valid charge fraction";
    dock = new Dock(navigator, target, observePerception, minimumConfidence);
    this.power = power;
    this.targetChargeFraction = targetChargeFraction;
  }

  public function start():Void {
    lifecycle.begin();
    stage = Docking;
    dock.start();
    switch dock.status() {
      case Failed(message): lifecycle.fail(message);
      case _: // The docking subskill advances on update.
    }
  }

  public function update(snapshot:RobotSnapshot, durationSeconds:Float):SkillStatus {
    if (!lifecycle.isRunning()) return lifecycle.status();
    if (switch stage { case Docking: true; case _: false; }) {
      switch dock.update(snapshot, durationSeconds) {
        case Succeeded: stage = WaitingForCharge;
        case Cancelled: lifecycle.cancel();
        case Failed(message): lifecycle.fail(message);
        case Running:
        case Idle: lifecycle.fail("docking subskill returned to idle");
      }
    }
    if (stage == WaitingForCharge && lifecycle.isRunning()) {
      var battery = power.batteryState();
      if (battery != null && battery.chargeFraction >= targetChargeFraction)
        lifecycle.succeed("target battery charge reached");
    }
    return lifecycle.status();
  }

  public function cancel():Void {
    if (!lifecycle.isRunning()) return;
    dock.cancel();
    lifecycle.cancel();
  }

  public function status():SkillStatus return lifecycle.status();
  public function result():Null<SkillResult> return lifecycle.result();
}
