package motionkit.robot;

import haxe.Int64;
import motionkit.trajectory.ExecutionPlan;
import motionkit.trajectory.PlanDiagnostic;
import motionkit.robot.TrajectoryStream.StreamSegments;
import motionkit.robot.TrajectoryStream.ArrayStreamSegments;
import motionkit.trajectory.Trajectory;
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
  /** Declared signed acceleration (rad/s²) of a geared process spindle during this plan.
      Programs must ramp to steady speed before engaging, or supply the actual envelope. */
  public var processAccelerations:Map<String, Float> = new Map();
  public var margin:Float = 1.0;
  /** Explicit nonnegative resisting joint loads (N for sliding, Nm for rotary),
      conservatively present throughout this check, including rapid moves.
      Process-only spindles absent from the plan need a separate power/rating check. */
  public var resistingEfforts:Map<String, Float> = new Map();
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
    for(id => effort in resistingEfforts)result.resistingEfforts.set(id,effort);
    for (id => value in processAccelerations) result.processAccelerations.set(id, value);
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
 * Checks polynomial motion against the drives of the machine it is for:
 * the torque each motor would need along the plan against what its drive can give (a stepper's
 * pull-out curve, a servo's peak and rated torque), and how far the axes' drives stretch or lag
 * under the plan's forces. It reads the plan's polynomial segments, so the same check serves a
 * simulation and a device.
 *
 * Rack drives use the same load Jacobian as screws and belts; assembled mesh, reducer and hinge
 * allowances reach the accuracy check through the coupling compliance and backlash.
 * The model's actuators, couplings, masses and inertias supply everything: see `DriveLoads`. Joint
 * `jointIds[i]` is plan joint `i`; axes the plan does not move are not checked.
 */
class PlanCheck {
  /** Process-only spindle rating check. Call for every cutting operation before admitting its plan.
      Ratings must be available mechanical power/torque at the commanded rpm, not input kVA. */
  public static function spindle(operation:String,powerW:Float,torqueNm:Float,availableW:Float,availableNm:Float):Void {
    for(v in [powerW,torqueNm,availableW,availableNm])
      if(!Math.isFinite(v)||v<0)throw operation+": invalid spindle rating/load";
    if(powerW>availableW||torqueNm>availableNm)
      throw operation+": spindle power/torque exceeded ("+powerW+" W, "+torqueNm+" Nm)";
  }

  public final options:PlanCheckOptions;
  final gears:GearCheckState;
  final loads:Array<AxisLoad> = [];
  final planJoint:Array<Int> = [];
  final modelJoint:Array<Int> = [];
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
    gears = new GearCheckState(model);
    this.jointIds = jointIds.copy();
    this.options = options == null ? new PlanCheckOptions() : options.copy();
    for(id => effort in this.options.resistingEfforts) {
      if(jointIds.indexOf(id)<0||!Math.isFinite(effort)||effort<0)throw 'Invalid resisting effort for planned joint $id';
    }
    for (load in DriveLoads.of(model, this.options.steady, jointIds)) {
      var index = jointIds.indexOf(load.axis);
      if (index < 0) continue;
      loads.push(load);
      planJoint.push(index);
      modelJoint.push([for (joint in model.joints) joint.id].indexOf(load.axis));
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

  function resistingEffort(id:String):Float {
    var effort=options.resistingEfforts.get(id);return effort==null?0:effort;
  }

  /** A check for another planning thread: the same machine, with no memory of earlier plans. */
  public function fork():PlanCheck return new PlanCheck(model, jointIds, options);

  /** Whether any axis of the plan has a motor to check. */
  public function checks():Bool return loads.length > 0;

  /** The checked axes' loads, for reports. */
  public function axisLoads():Array<AxisLoad> return loads.copy();

  /** Checks `plan`, made from op `opIndex` at programmed speed `feed` (m/s, 0 when not a path move). */
  public function check(plan:ExecutionPlan, opIndex:Int, feed:Float):PlanCheckResult {
    return checkSegments(new CheckedPlanSegments(plan), opIndex, feed, plan.events);
  }

  /** Direct moves and live servo chunks use the same drive and accuracy checks as programs. */
  public function checkTrajectory(trajectory:Trajectory, ?opIndex:Int = -1, ?feed:Float = 0.0):PlanCheckResult
    return checkSegments(new ArrayStreamSegments(trajectory.segments()), opIndex, feed);

  public function checkSegments(segments:StreamSegments, ?opIndex:Int = -1, ?feed:Float = 0.0, ?events:Array<motionkit.event.TimedEvent>):PlanCheckResult {
    if (segments.jointCount() != jointIds.length) throw "Plan check joint count mismatch";
    var ordered = events == null ? [] : events.copy();
    // Stable order is essential for atomic begin/term/engage groups at one timestamp.
    for (i in 1...ordered.length) if (haxe.Int64.compare(ordered[i - 1].timeNs, ordered[i].timeNs) > 0)
      throw "Plan check needs ordered process events";
    var eventIndex = 0;
    var gearSpeedWorst:Map<String, PlanDiagnostic> = new Map();
    var count = segments.count();
    var stride = ExecutionPlan.COEFFICIENT_STRIDE;
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
      var length = Int64.toFloat(segments.durationNs(segment)) * 1e-9;
      var begin = Int64.toFloat(segments.startNs(segment)) * 1e-9;
      var degree = segments.degree(segment);
      var steps = Std.int(Math.max(1.0, Math.ceil(length / options.sampleStep)));
      var step = length / steps;
      duration += length;
      for (axis in 0...loads.length) {
        for (power in 0...degree + 1)
          c[axis][power] = segments.coefficient(segment, planJoint[axis], power);
      }
      var times = [for (i in 0...steps + 1) step * i];
      // Check both sides of every gear/process event, including short intervals between grid samples.
      for (event in ordered) {
        var time = Int64.toFloat(event.timeNs) * 1e-9 - begin;
        if (time >= 0 && time <= length) {
          times.push(time);
          if (time > 0) times.push(Math.max(0, time - 1e-10));
        }
      }
      times.sort(Reflect.compare);
      var samples:Array<Float> = [];
      for (time in times) if (samples.length == 0 || time > samples[samples.length - 1]) samples.push(time);
      for (sample in 0...samples.length) {
        var tau = samples[sample];
        var weight = ((sample > 0 ? tau - samples[sample - 1] : 0) +
          (sample + 1 < samples.length ? samples[sample + 1] - tau : 0)) * 0.5;
        while (eventIndex < ordered.length && Int64.toFloat(ordered[eventIndex].timeNs) * 1e-9 <= begin + tau + 1e-15) {
          var event = ordered[eventIndex++]; gears.observe(event.channel, event.value);
        }
        // Physical velocity/acceleration include the measured-master contribution. The polynomial
        // remains the additive part; this does not turn a programmable relation into a model coupling.
        var gearV:Map<Int, Float> = new Map(), gearA:Map<Int, Float> = new Map();
        var masterCache:Map<Int, {velocity:Float, acceleration:Float}> = new Map();
        function masterMotion(joint:Int):{velocity:Float, acceleration:Float} {
          var cached = masterCache.get(joint);
          if (cached != null) return cached;
          var v = 0.0, a = 0.0;
          var process = gears.processVelocity(joint);
          if (process != null) {
            var id = model.joints[joint].id, alpha = options.processAccelerations.get(id);
            if (alpha == null || !Math.isFinite(alpha)) throw 'Electronic gear master $id needs a declared process acceleration';
            v = process; a = alpha;
          } else {
            var master = jointIds.indexOf(model.joints[joint].id);
            if (master < 0) throw 'Electronic gear master ${model.joints[joint].id} is absent from the plan';
            for (power in 1...degree + 1) {
              var coefficient = segments.coefficient(segment, master, power);
              v += power * coefficient * Math.pow(tau, power - 1);
              if (power >= 2) a += power * (power - 1) * coefficient * Math.pow(tau, power - 2);
            }
          }
          var terms = gears.active.get(joint);
          if (terms != null) for (term in terms) {
            var motion = masterMotion(term.master);
            v += term.ratio * motion.velocity; a += term.ratio * motion.acceleration;
          }
          var result = {velocity:v, acceleration:a}; masterCache.set(joint, result); return result;
        }
        for (follower => terms in gears.active) {
          var v = 0.0, a = 0.0;
          for (term in terms) {
            var motion = masterMotion(term.master);
            v += term.ratio * motion.velocity; a += term.ratio * motion.acceleration;
          }
          if (jointIds.indexOf(model.joints[follower].id) < 0) throw 'Electronic gear follower ${model.joints[follower].id} is absent from the additive plan';
          gearV.set(follower, v); gearA.set(follower, a);
        }
        for (slot in 0...motorCount) { torques[slot] = 0.0; speeds[slot] = 0.0; drags[slot] = 0.0; }
        for (axis in 0...loads.length) {
          var load = loads[axis], coefficient = c[axis];
          var resisting = load.friction + resistingEffort(load.axis) + (cutting && load.sliding ? options.cuttingForce : 0.0);
          var velocity = 0.0, acceleration = 0.0;
          for (power in 1...degree + 1) {
            velocity += power * coefficient[power] * Math.pow(tau, power - 1);
            if (power >= 2) acceleration += power * (power - 1) * coefficient[power] * Math.pow(tau, power - 2);
          }
          var joint = modelJoint[axis];
          var extraV = gearV.get(joint), extraA = gearA.get(joint);
          if (extraV != null && extraA != null) {
            velocity += extraV; acceleration += extraA;
            var limit = model.coupledLimits(load.axis, options.steady).requireVelocity();
            if (Math.abs(velocity) > limit * (1 + 1e-6)) {
              var previous = gearSpeedWorst.get(load.axis);
              if (previous == null || Math.abs(velocity) > previous.value)
                gearSpeedWorst.set(load.axis, new PlanDiagnostic(PlanDiagnosticKind.GearSpeed, opIndex,
                  load.axis, load.axis, begin + tau, Math.abs(velocity), limit, 1));
            }
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
          loads[axis].friction + resistingEffort(loads[axis].axis) + (cutting && loads[axis].sliding ? options.cuttingForce : 0.0))];
        var elastic = loads.length > 0 && loads[0].elastic != null ? loads[0].elastic.deflections(forces) : [];
        for (axis in 0...loads.length) {
          var load = loads[axis];
          var velocity = velocities[axis], acceleration = accelerations[axis];
          var resisting = load.friction + resistingEffort(load.axis) + (cutting && load.sliding ? options.cuttingForce : 0.0);
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
    for (diagnostic in gearSpeedWorst) diagnostics.push(diagnostic);
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

/** Read the native program arrays without copying their coefficients. */
private class CheckedPlanSegments implements StreamSegments {
  final plan:ExecutionPlan;
  final starts:runtime.memory.NativeSpan<Int64>;
  final durations:runtime.memory.NativeSpan<Int64>;
  final degrees:runtime.memory.NativeSpan<Int>;
  final coefficients:runtime.memory.NativeSpan<Float>;

  public function new(plan:ExecutionPlan) {
    this.plan = plan;
    starts = plan.segmentStarts();
    durations = plan.segmentDurations();
    degrees = plan.segmentDegrees();
    coefficients = plan.segmentCoefficients();
  }

  public function count():Int return starts.length();
  public function jointCount():Int return plan.jointCount;
  public function startNs(index:Int):Int64 return starts.get(index);
  public function durationNs(index:Int):Int64 return durations.get(index);
  public function degree(index:Int):Int return degrees.get(index);
  public function coefficient(index:Int, joint:Int, power:Int):Float
    return coefficients.get((index * plan.jointCount + joint) * ExecutionPlan.COEFFICIENT_STRIDE + power);
}
