package motionkit.robot;

import haxe.Int64;
import motionkit.trajectory.ExecutionPlan;
import motionkit.trajectory.PlanDiagnostic;
import robotkit.model.ActuatorDrive.ServoDrive;
import robotkit.model.ActuatorDrive.StepperDrive;
import robotkit.model.DriveLoads;
import robotkit.model.DriveLoads.AxisLoad;
import robotkit.model.RobotModel;
import robotkit.model.SteadyLoads;

/**
 * What a plan check assumes about the machine and how much it asks of the drives. The friction
 * and force defaults are assumptions, not datasheet values: a hobby-class CNC's linear rails drag
 * a few newtons, and a cutting force depends on the tool, the material and the feed, so a machine
 * that wants either states it.
 */
class PlanCheckOptions {
  /** Share of a drive's torque-speed curve a motor may use: 1 allows the curve itself. */
  public var margin:Float = 1.0;
  /** Gravity and friction on the axes, which planning with `RobotModel.coupledLimits(id, steady)` also honours. */
  public var steady:SteadyLoads = new SteadyLoads();
  /** A cutting force allowance in N, added against the motion of an axis on feed moves; 0 means none. */
  public var cuttingForce:Float = 0.0;
  /** Moves programmed at no more than this feed (m/s) count as cutting; 0 means no move does. */
  public var cuttingFeedLimit:Float = 0.0;
  /** Worst deviation of the tool from its commanded path the machine tolerates, in metres. */
  public var tolerance:Float = 1e-4;
  /** Longest gap between samples along a plan, in seconds. */
  public var sampleStep:Float = 0.004;
  /** True to reject a plan the check flags (the compiler throws), false to report it. */
  public var rejects:Bool = false;

  public function new() {}

  public function copy():PlanCheckOptions {
    var result = new PlanCheckOptions();
    result.margin = margin;
    result.steady = steady.copy();
    result.cuttingForce = cuttingForce;
    result.cuttingFeedLimit = cuttingFeedLimit;
    result.tolerance = tolerance;
    result.sampleStep = sampleStep;
    result.rejects = rejects;
    return result;
  }
}

/**
 * Checks a plan against the drives of the machine it is for, once, when the compiler has made it:
 * the torque each motor would need along the plan against what its drive can give (a stepper's
 * pull-out curve, a servo's peak and rated torque), and how far the axes' drives stretch or lag
 * under the plan's forces. It reads the plan's polynomial segments, so the same check serves a
 * simulation and a device.
 *
 * The model's actuators, couplings, masses and inertias supply everything: see `DriveLoads`. Joint
 * `jointIds[i]` is plan joint `i`; axes the plan does not move are not checked.
 */
class PlanCheck {
  public final options:PlanCheckOptions;
  final loads:Array<AxisLoad> = [];
  final planJoint:Array<Int> = [];
  final model:RobotModel;
  final jointIds:Array<String>;
  /** The way each axis last moved, to see a reversal between plans. */
  final lastDirection:Array<Float>;

  public function new(model:RobotModel, jointIds:Array<String>, ?options:PlanCheckOptions) {
    if (model == null || jointIds == null) throw "A plan check needs a model and its joint ids";
    this.model = model;
    this.jointIds = jointIds.copy();
    this.options = options == null ? new PlanCheckOptions() : options.copy();
    for (load in DriveLoads.of(model, this.options.steady)) {
      var index = jointIds.indexOf(load.axis);
      if (index < 0) continue;
      loads.push(load);
      planJoint.push(index);
    }
    lastDirection = [for (_ in loads) 0.0];
  }

  /** A check for another planning thread: the same machine, with no memory of earlier plans. */
  public function fork():PlanCheck return new PlanCheck(model, jointIds, options);

  /** Whether any axis of the plan has a motor to check. */
  public function checks():Bool return loads.length > 0;

  /** The checked axes' loads, for reports. */
  public function axisLoads():Array<AxisLoad> return loads.copy();

  /** Checks `plan`, made from op `opIndex` at programmed speed `feed` (m/s, 0 when not a path move). */
  public function check(plan:ExecutionPlan, opIndex:Int, feed:Float):PlanCheckResult {
    var starts = plan.segmentStarts(), durations = plan.segmentDurations();
    var degrees = plan.segmentDegrees(), coefficients = plan.segmentCoefficients();
    var count = starts.length();
    var stride = ExecutionPlan.COEFFICIENT_STRIDE, jointCount = plan.jointCount;
    var cutting = options.cuttingFeedLimit > 0.0 && feed > 0.0 && feed <= options.cuttingFeedLimit;
    // Per motor: worst ratio and where, samples over, and the sum of squares for a servo's RMS.
    var motorCount = 0;
    for (load in loads) motorCount += load.motors.length;
    var worstRatio = [for (_ in 0...motorCount) 0.0];
    var worstTorque = [for (_ in 0...motorCount) 0.0];
    var worstAvailable = [for (_ in 0...motorCount) 0.0];
    var worstTime = [for (_ in 0...motorCount) 0.0];
    var over = [for (_ in 0...motorCount) 0];
    var squares = [for (_ in 0...motorCount) 0.0];
    var worstDeviation = [for (_ in loads) 0.0];
    var worstDeviationTime = [for (_ in loads) 0.0];
    var firstDirection = [for (_ in loads) 0.0];
    var lastSeen = [for (_ in loads) 0.0];
    var reversed = [for (_ in loads) false];
    // Lost motion per axis: while every motor of an axis is over its curve and all of them are steppers, the
    // rotors have lost sync and the axis does not advance, so it falls behind by what the plan commands.
    var lostTimes:Array<Array<Float>> = [for (_ in loads) []];
    var lostValues:Array<Array<Float>> = [for (_ in loads) []];
    var lostCumulative = [for (_ in loads) 0.0];
    var wasLosing = [for (_ in loads) false];
    var duration = 0.0;
    var c = [for (_ in 0...stride) 0.0];
    for (segment in 0...count) {
      var length = Int64.toFloat(durations.get(segment)) * 1e-9;
      var begin = Int64.toFloat(starts.get(segment)) * 1e-9;
      var degree = degrees.get(segment);
      var steps = Std.int(Math.max(1.0, Math.ceil(length / options.sampleStep)));
      var step = length / steps;
      duration += length;
      var motorIndex = 0;
      for (axis in 0...loads.length) {
        var load = loads[axis];
        var base = (segment * jointCount + planJoint[axis]) * stride;
        for (power in 0...degree + 1) c[power] = coefficients.get(base + power);
        var resisting = load.friction +
          (cutting && load.sliding ? options.cuttingForce : 0.0);
        for (sample in 0...steps + 1) {
          var tau = step * sample;
          var velocity = 0.0, acceleration = 0.0;
          for (power in 1...degree + 1) {
            velocity += power * c[power] * Math.pow(tau, power - 1);
            if (power >= 2) acceleration += power * (power - 1) * c[power] * Math.pow(tau, power - 2);
          }
          // End points belong to two segments: weight them half so the RMS counts each moment once.
          var weight = (sample == 0 || sample == steps) ? 0.5 * step : step;
          var direction = velocity > 1e-9 ? 1.0 : velocity < -1e-9 ? -1.0 : 0.0;
          if (direction != 0.0) {
            if (firstDirection[axis] == 0.0) firstDirection[axis] = direction;
            if (lastSeen[axis] != 0.0 && lastSeen[axis] != direction) reversed[axis] = true;
            lastSeen[axis] = direction;
          }
          var losing = load.motors.length > 0;
          for (index in 0...load.motors.length) {
            var motor = load.motors[index];
            var slot = motorIndex + index;
            var torque = load.motorTorque(motor, velocity, acceleration, resisting);
            var available = options.margin * motor.actuator.torqueCurve().torqueAt(motor.ratio * velocity);
            var ratio = Math.abs(torque) > 1e-12 ? (available > 0.0 ? Math.abs(torque) / available : 1e9) : 0.0;
            squares[slot] += torque * torque * weight;
            if (ratio > worstRatio[slot]) {
              worstRatio[slot] = ratio;
              worstTorque[slot] = Math.abs(torque);
              worstAvailable[slot] = available;
              worstTime[slot] = begin + tau;
            }
            if (ratio > 1.0 + 1e-6) over[slot]++;
            else losing = false;
            if (!Std.isOfType(motor.actuator.drive, StepperDrive)) losing = false;
          }
          if (losing) {
            if (!wasLosing[axis]) {
              lostTimes[axis].push(begin + tau);
              lostValues[axis].push(lostCumulative[axis]);
            }
            lostCumulative[axis] += velocity * weight;
            lostTimes[axis].push(begin + tau);
            lostValues[axis].push(lostCumulative[axis]);
          } else if (wasLosing[axis]) {
            lostTimes[axis].push(begin + tau);
            lostValues[axis].push(lostCumulative[axis]);
          }
          wasLosing[axis] = losing;
          if (load.stiffness > 0.0) {
            var deviation = Math.abs(load.mass * acceleration + (direction >= 0.0 ? 1.0 : -1.0) * resisting) / load.stiffness;
            if (deviation > worstDeviation[axis]) {
              worstDeviation[axis] = deviation;
              worstDeviationTime[axis] = begin + tau;
            }
          }
        }
        motorIndex += load.motors.length;
      }
    }
    var diagnostics:Array<PlanDiagnostic> = [];
    var slips:Array<PlanSlip> = [];
    var overallRatio = 0.0, overallMotor = "", overallDeviation = 0.0, overallAxis = "";
    var motorIndex = 0;
    for (axis in 0...loads.length) {
      var load = loads[axis];
      for (index in 0...load.motors.length) {
        var slot = motorIndex + index;
        var actuator = load.motors[index].actuator;
        var ratio = Math.min(worstRatio[slot], 1e9);
        if (ratio > overallRatio) {
          overallRatio = ratio;
          overallMotor = actuator.id;
        }
        if (over[slot] > 0) {
          var servo = Std.isOfType(actuator.drive, ServoDrive);
          diagnostics.push(new PlanDiagnostic(servo ? PlanDiagnosticKind.ServoPeakTorque : PlanDiagnosticKind.StepperStall,
            opIndex, actuator.id, load.axis, worstTime[slot], worstTorque[slot], worstAvailable[slot], over[slot]));
        }
        if (Std.isOfType(actuator.drive, ServoDrive) && duration > 0.0) {
          var drive:ServoDrive = cast actuator.drive;
          var rms = Math.sqrt(squares[slot] / duration);
          var rated = options.margin * drive.ratedTorque;
          if (rms > rated * (1.0 + 1e-6))
            diagnostics.push(new PlanDiagnostic(PlanDiagnosticKind.ServoRatedTorque, opIndex, actuator.id, load.axis,
              0.0, rms, rated, 1));
        }
      }
      motorIndex += load.motors.length;
      if (lostTimes[axis].length > 0 && lostCumulative[axis] != 0.0) {
        var first = load.motors[0];
        var drive:StepperDrive = cast first.actuator.drive;
        var steps = Math.abs(lostCumulative[axis] * first.ratio) / (2.0 * Math.PI / drive.fullStepsPerRevolution);
        slips.push(new PlanSlip(load.axis, [for (motor in load.motors) motor.actuator.id], lostTimes[axis], lostValues[axis], steps));
      }
      if (load.stiffness > 0.0) {
        // A reversal at the start of this plan, against how the axis last moved, or inside it, loses the nut's backlash.
        var startsReversed = lastDirection[axis] != 0.0 && firstDirection[axis] != 0.0 && lastDirection[axis] != firstDirection[axis];
        var deviation = worstDeviation[axis] + ((reversed[axis] || startsReversed) ? load.backlash : 0.0);
        if (deviation > overallDeviation) {
          overallDeviation = deviation;
          overallAxis = load.axis;
        }
        if (deviation > options.tolerance)
          diagnostics.push(new PlanDiagnostic(PlanDiagnosticKind.Accuracy, opIndex, load.axis, load.axis,
            worstDeviationTime[axis], deviation, options.tolerance, 1));
      } else if (load.backlash > 0.0) {
        var startsReversed = lastDirection[axis] != 0.0 && firstDirection[axis] != 0.0 && lastDirection[axis] != firstDirection[axis];
        var deviation = (reversed[axis] || startsReversed) ? load.backlash : 0.0;
        if (deviation > overallDeviation) {
          overallDeviation = deviation;
          overallAxis = load.axis;
        }
        if (deviation > options.tolerance)
          diagnostics.push(new PlanDiagnostic(PlanDiagnosticKind.Accuracy, opIndex, load.axis, load.axis, 0.0, deviation,
            options.tolerance, 1));
      }
      if (lastSeen[axis] != 0.0) lastDirection[axis] = lastSeen[axis];
    }
    return new PlanCheckResult(diagnostics, overallRatio, overallMotor, overallDeviation, overallAxis, slips);
  }
}
