package motionkit.robot;

import haxe.Int64;
import motionkit.trajectory.ExecutionPlan;
import motionkit.trajectory.PlanDiagnostic;
import robotkit.model.ActuatorDrive.ServoDrive;
import robotkit.model.ActuatorDrive.StepperDrive;
import robotkit.model.Actuator;
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
  final slotActuators:Array<Actuator> = [];
  /** The first axis each actuator serves, which names it in diagnostics. */
  final slotAxes:Array<String> = [];
  /** Per axis, the actuator slot of each of its motors. */
  final motorSlots:Array<Array<Int>> = [];
  final model:RobotModel;
  final jointIds:Array<String>;
  /** The way each axis last moved, to see a reversal between plans. */
  final lastDirection:Array<Float>;

  public function new(model:RobotModel, jointIds:Array<String>, ?options:PlanCheckOptions) {
    if (model == null || jointIds == null) throw "A plan check needs a model and its joint ids";
    this.model = model;
    this.jointIds = jointIds.copy();
    this.options = options == null ? new PlanCheckOptions() : options.copy();
    for (load in DriveLoads.of(model, this.options.steady, jointIds)) {
      var index = jointIds.indexOf(load.axis);
      if (index < 0) continue;
      loads.push(load);
      planJoint.push(index);
    }
    lastDirection = [for (_ in loads) 0.0];
    // One slot per actuator, in order of first use: a motor that serves several axes has one.
    for (load in loads) {
      var slots:Array<Int> = [];
      for (motor in load.motors) {
        var slot = slotActuators.indexOf(motor.actuator);
        if (slot < 0) {
          slot = slotActuators.length;
          slotActuators.push(motor.actuator);
          slotAxes.push(load.axis);
        }
        slots.push(slot);
      }
      motorSlots.push(slots);
    }
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
    // Per motor (an actuator, however many axes it serves): worst ratio and where, samples over, and
    // the sum of squares for a servo's RMS.
    var motorCount = slotActuators.length;
    var worstRatio = [for (_ in 0...motorCount) 0.0];
    var worstTorque = [for (_ in 0...motorCount) 0.0];
    var worstAvailable = [for (_ in 0...motorCount) 0.0];
    var worstTime = [for (_ in 0...motorCount) 0.0];
    var over = [for (_ in 0...motorCount) 0];
    var squares = [for (_ in 0...motorCount) 0.0];
    var torques = [for (_ in 0...motorCount) 0.0];
    var speeds = [for (_ in 0...motorCount) 0.0];
    var drags = [for (_ in 0...motorCount) 0.0];
    var slotOver = [for (_ in 0...motorCount) false];
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
    var c = [for (axis in 0...loads.length) [for (_ in 0...stride) 0.0]];
    var velocities = [for (_ in loads) 0.0];
    var accelerations = [for (_ in loads) 0.0];
    for (segment in 0...count) {
      var length = Int64.toFloat(durations.get(segment)) * 1e-9;
      var begin = Int64.toFloat(starts.get(segment)) * 1e-9;
      var degree = degrees.get(segment);
      var steps = Std.int(Math.max(1.0, Math.ceil(length / options.sampleStep)));
      var step = length / steps;
      duration += length;
      for (axis in 0...loads.length) {
        var base = (segment * jointCount + planJoint[axis]) * stride;
        for (power in 0...degree + 1) c[axis][power] = coefficients.get(base + power);
      }
      for (sample in 0...steps + 1) {
        var tau = step * sample;
        // End points belong to two segments: weight them half so the RMS counts each moment once.
        var weight = (sample == 0 || sample == steps) ? 0.5 * step : step;
        for (slot in 0...motorCount) { torques[slot] = 0.0; speeds[slot] = 0.0; drags[slot] = 0.0; }
        for (axis in 0...loads.length) {
          var load = loads[axis], coefficient = c[axis];
          var resisting = load.friction + (cutting && load.sliding ? options.cuttingForce : 0.0);
          var velocity = 0.0, acceleration = 0.0;
          for (power in 1...degree + 1) {
            velocity += power * coefficient[power] * Math.pow(tau, power - 1);
            if (power >= 2) acceleration += power * (power - 1) * coefficient[power] * Math.pow(tau, power - 2);
          }
          velocities[axis] = velocity;
          accelerations[axis] = acceleration;
          var direction = velocity > 1e-9 ? 1.0 : velocity < -1e-9 ? -1.0 : 0.0;
          if (direction != 0.0) {
            if (firstDirection[axis] == 0.0) firstDirection[axis] = direction;
            if (lastSeen[axis] != 0.0 && lastSeen[axis] != direction) reversed[axis] = true;
            lastSeen[axis] = direction;
          }
          // A motor on several axes (CoreXY) takes the sum of what each axis asks of it.
          for (index in 0...load.motors.length) {
            var motor = load.motors[index], slot = motorSlots[axis][index];
            torques[slot] += load.motorTorqueWithoutDrag(motor, velocity, acceleration, resisting);
            speeds[slot] += motor.ratio * velocity;
            if (Math.abs(motor.ratio * velocity) > 1e-12) drags[slot] += motor.drag;
          }
        }
        for (slot in 0...motorCount) {
          var actuator = slotActuators[slot];
          var speed = speeds[slot];
          var torque = torques[slot] + (speed > 1e-12 ? drags[slot] : speed < -1e-12 ? -drags[slot] : 0.0);
          var available = options.margin * actuator.torqueCurve().torqueAt(speed);
          var ratio = Math.abs(torque) > 1e-12 ? (available > 0.0 ? Math.abs(torque) / available : 1e9) : 0.0;
          squares[slot] += torque * torque * weight;
          slotOver[slot] = ratio > 1.0 + 1e-6;
          if (ratio > worstRatio[slot]) {
            worstRatio[slot] = ratio;
            worstTorque[slot] = Math.abs(torque);
            worstAvailable[slot] = available;
            worstTime[slot] = begin + tau;
          }
          if (slotOver[slot]) over[slot]++;
        }
        var forces = [for (axis in 0...loads.length) loads[axis].force(velocities[axis], accelerations[axis],
          loads[axis].friction + (cutting && loads[axis].sliding ? options.cuttingForce : 0.0))];
        var elastic = loads.length > 0 && loads[0].elastic != null ? loads[0].elastic.deflections(forces) : [];
        for (axis in 0...loads.length) {
          var load = loads[axis];
          var velocity = velocities[axis], acceleration = accelerations[axis];
          var resisting = load.friction + (cutting && load.sliding ? options.cuttingForce : 0.0);
          var direction = velocity > 1e-9 ? 1.0 : velocity < -1e-9 ? -1.0 : 0.0;
          var losing = load.motors.length > 0;
          for (index in 0...load.motors.length) {
            var slot = motorSlots[axis][index];
            if (!slotOver[slot] || !Std.isOfType(slotActuators[slot].drive, StepperDrive)) losing = false;
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
            var deviation = Math.abs(elastic[axis]);
            if (deviation > worstDeviation[axis]) {
              worstDeviation[axis] = deviation;
              worstDeviationTime[axis] = begin + tau;
            }
          }
        }
      }
    }
    var diagnostics:Array<PlanDiagnostic> = [];
    var slips:Array<PlanSlip> = [];
    var overallRatio = 0.0, overallMotor = "", overallDeviation = 0.0, overallAxis = "";
    for (slot in 0...motorCount) {
      var actuator = slotActuators[slot];
      var ratio = Math.min(worstRatio[slot], 1e9);
      if (ratio > overallRatio) {
        overallRatio = ratio;
        overallMotor = actuator.id;
      }
      if (over[slot] > 0) {
        var servo = Std.isOfType(actuator.drive, ServoDrive);
        diagnostics.push(new PlanDiagnostic(servo ? PlanDiagnosticKind.ServoPeakTorque : PlanDiagnosticKind.StepperStall,
          opIndex, actuator.id, slotAxes[slot], worstTime[slot], worstTorque[slot], worstAvailable[slot], over[slot]));
      }
      if (Std.isOfType(actuator.drive, ServoDrive) && duration > 0.0) {
        var drive:ServoDrive = cast actuator.drive;
        var rms = Math.sqrt(squares[slot] / duration);
        var rated = options.margin * drive.ratedTorque;
        if (rms > rated * (1.0 + 1e-6))
          diagnostics.push(new PlanDiagnostic(PlanDiagnosticKind.ServoRatedTorque, opIndex, actuator.id, slotAxes[slot],
            0.0, rms, rated, 1));
      }
    }
    for (axis in 0...loads.length) {
      var load = loads[axis];
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
    for (diagnostic in diagnostics) {
      var labels:Array<String> = [];
      for (load in loads) {
        if (diagnostic.kind == PlanDiagnosticKind.Accuracy) {
          if (load.axis == diagnostic.axis) for (label in robotkit.model.EngineeringAssumptions.labels(load.assumptions, PlanDiagnostic.quantities(diagnostic.kind)))
            if (labels.indexOf(label) < 0) labels.push(label);
        } else for (motor in load.motors) if (motor.actuator.id == diagnostic.subject)
          for (label in robotkit.model.EngineeringAssumptions.labels(motor.assumptions, PlanDiagnostic.quantities(diagnostic.kind))) if (labels.indexOf(label) < 0) labels.push(label);
      }
      labels.sort(Reflect.compare);
      diagnostic.assumed = labels;
    }
    var result = new PlanCheckResult(diagnostics, overallRatio, overallMotor, overallDeviation, overallAxis, slips);
    for (load in loads) {
      var limiter = model.coupledLimits(load.axis, options.steady).velocityLimiter;
      if (limiter != "") result.speedLimits.push('${load.axis} limited by $limiter');
    }
    return result;
  }
}
