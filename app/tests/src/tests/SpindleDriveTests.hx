package tests;

import motionkit.robot.CouplingProjection;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.RobotRuntimeJointCouplingBlueprint;
import robotkit.runtime.RobotRuntimeVelocityDriveBlueprint;
import toolpathkit.motion.SpindleDriveBinding;

/** Nominal command conversion is independent of physical belt stretch and measured spindle feedback. */
class SpindleDriveTests {
  static var checks = 0;
  static function check(ok:Bool, message:String):Void {
    checks++;
    if (!ok) throw message;
  }
  static function near(actual:Float, expected:Float, label:String):Void
    check(Math.abs(actual - expected) < 1e-9 * Math.max(1, Math.abs(expected)), label + ': $actual != $expected');
  static function reject(action:Void -> Void, label:String):Void {
    var rejected = false;
    try action() catch (_:Dynamic) rejected = true;
    check(rejected, label);
  }
  static function blueprint(joint:Int, scale:Float):RobotRuntimeBlueprint {
    var result = new RobotRuntimeBlueprint(0, 4, 1);
    result.velocityDrives.push(new RobotRuntimeVelocityDriveBlueprint(joint, "spindle.speed", "spindle.direction", scale, 10, 1000));
    return result;
  }
  public static function main():Int {
    run();
    return 0;
  }
  public static function run():Void {
    checks = 0;
    var rpm = 2 * Math.PI / 60;
    var direct = SpindleDriveBinding.resolve(blueprint(0, rpm), 0);
    check(direct.name == "spindle", "Drive channel name is preserved");
    near(direct.speed(600), 600, "Direct RPM drive");
    near(direct.direction(-1), -1, "Direct reverse direction");

    // The hobber declares motor = ratio * B, including a position offset that cannot affect velocity.
    var belt = blueprint(1, rpm);
    belt.couplings.push(new RobotRuntimeJointCouplingBlueprint(0, 1, -3, 0.7));
    var spindle = SpindleDriveBinding.resolve(belt, 0);
    near(spindle.speed(600), 1800, "Motor follower is resolved back to the spindle through a reversed 3:1 belt");
    near(spindle.direction(1), -1, "Belt reverses commanded direction");
    near(spindle.direction(0), 0, "Off stays off");
    belt.couplings.push(new RobotRuntimeJointCouplingBlueprint(1, 2, 2, -1.2));
    belt.velocityDrives.splice(0, 1);
    belt.velocityDrives.push(new RobotRuntimeVelocityDriveBlueprint(2, "spindle.speed", "spindle.direction", 1, 10, 1000));
    spindle = SpindleDriveBinding.resolve(belt, 0);
    near(spindle.speed(600), 60 * 2 * Math.PI, "Chained reduction respects a radian/second speed channel");
    near(spindle.direction(-1), 1, "Reverse spindle command through a reversing chain");

    var forward = blueprint(0, rpm);
    forward.couplings.push(new RobotRuntimeJointCouplingBlueprint(0, 1, 0.25, 0));
    near(SpindleDriveBinding.resolve(forward, 1).speed(600), 2400, "Motor leader drives spindle follower");
    reject(() -> {SpindleDriveBinding.resolve(forward, 3);}, "Unrelated drive cannot own a spindle");
    forward.velocityDrives.push(new RobotRuntimeVelocityDriveBlueprint(1, "other.speed", "other.direction", rpm, 10, 1000));
    reject(() -> {SpindleDriveBinding.resolve(forward, 1);}, "Two drives on the same mechanical coordinate are ambiguous");
    var empty = new RobotRuntimeBlueprint(0, 4, 1);
    reject(() -> {SpindleDriveBinding.resolve(empty, 0);}, "Spindle requires a declared drive");
    var mixed = blueprint(2, rpm);
    mixed.couplings.push(new RobotRuntimeJointCouplingBlueprint(0, 2, 1, 0));
    mixed.couplings.push(new RobotRuntimeJointCouplingBlueprint(1, 2, 1, 0));
    reject(() -> {SpindleDriveBinding.resolve(mixed, 0);}, "A differential cannot identify spindle speed from one motor");
    var cycle = blueprint(0, rpm);
    cycle.couplings.push(new RobotRuntimeJointCouplingBlueprint(0, 1, 1, 0));
    cycle.couplings.push(new RobotRuntimeJointCouplingBlueprint(1, 0, 1, 0));
    reject(() -> {SpindleDriveBinding.resolve(cycle, 0);}, "Cyclic mechanical relations are rejected");
    var named = new RobotRuntimeBlueprint(0, 1, 1);
    named.velocityDrives.push(new RobotRuntimeVelocityDriveBlueprint(0, "spindle.speed", "other.direction", rpm, 10, 1000));
    reject(() -> {SpindleDriveBinding.resolve(named, 0);}, "Speed and direction keep the shared-name rule");
    Sys.println('Spindle drive: $checks checks passed');
  }
}
