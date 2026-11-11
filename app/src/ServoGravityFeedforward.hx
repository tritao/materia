package app;

import robotkit.kinematics.GravityTorques;
import robotkit.model.RobotModel;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.RobotRuntimeIdentity;
import robotkit.runtime.Simulation;
import robotkit.simulation.SimulatedRobot;

/**
 * Gravity compensation for one simulated robot's servo drives, as a drive's model-based torque feedforward: before each
 * tick, at most every `PERIOD` of simulated time, the holding effort of every servo-driven joint at the measured pose
 * (coupled followers' loads reflected onto their drives) is passed to the simulation. A vertical axis then holds its
 * target instead of sagging by load over servo stiffness. Efforts beyond a drive's rating are clamped to it.
 */
class ServoGravityFeedforward implements SessionMember {
  /** How often the efforts are recomputed (s of simulated time): they change only as the pose does. */
  static inline final PERIOD = 0.002;

  final simulation:Simulation;
  final robotIndex:Int;
  final robot:SimulatedRobot;
  final gravity:GravityTorques;
  final jointCount:Int;
  final driven:Array<Int>;
  final ratings:Array<Float>;
  var sinceUpdate = 1e300;
  var lastTime = -1.0;

  /** Null when the robot has no servo-driven joint or moves on a floating base (its gravity frame is not fixed). */
  public static function of(simulation:Simulation, robotIndex:Int, robot:SimulatedRobot, model:RobotModel,
      blueprint:RobotRuntimeBlueprint):Null<ServoGravityFeedforward> {
    var identity:Null<RobotRuntimeIdentity> = blueprint.identity;
    if (identity == null || model.floatingBase) return null;
    var driven:Array<Int> = [];
    for (actuator in model.actuators) if (actuator.drive != null && actuator.drive.servo() != null) switch actuator.transmission {
      case SimpleTransmission(id, _, _):
        var index = identity.jointIndex(id);
        if (index >= 0 && blueprint.joints[index].servoStiffness > 0 && driven.indexOf(index) < 0) driven.push(index);
      case _:
    }
    return driven.length == 0 ? null : new ServoGravityFeedforward(simulation, robotIndex, robot, model, blueprint, identity, driven);
  }

  function new(simulation:Simulation, robotIndex:Int, robot:SimulatedRobot, model:RobotModel, blueprint:RobotRuntimeBlueprint,
      identity:RobotRuntimeIdentity, driven:Array<Int>) {
    this.simulation = simulation;
    this.robotIndex = robotIndex;
    this.robot = robot;
    gravity = new GravityTorques(model, identity);
    jointCount = blueprint.joints.length;
    this.driven = driven;
    ratings = [for (index in driven) {
      var rating = blueprint.joints[index].maxEffort;
      rating == null || !(rating > 0) ? 1e300 : rating;
    }];
  }

  public function feed():Void {
    var now = simulation.simulationTime();
    sinceUpdate += lastTime < 0 ? PERIOD : now - lastTime;
    lastTime = now;
    if (sinceUpdate < PERIOD) return;
    sinceUpdate = 0;
    var positions = robot.snapshot().positions;
    var q = [for (j in 0...jointCount) positions.get(j)];
    var efforts = gravity.actuated(q, driven);
    for (k in 0...driven.length) simulation.setServoFeedforward(robotIndex, driven[k], Math.max(-ratings[k], Math.min(ratings[k], efforts[driven[k]])));
  }

  public function beforeReset():Void {}

  public function reset():Void {
    sinceUpdate = 1e300;
    lastTime = -1;
  }

  public function present():Void {}
}
