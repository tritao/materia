package motionkit.robot;

import motionkit.axis.MotionAxisBlueprint;

import robotkit.model.RobotModel;
import robotkit.model.Transmission;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.RobotRuntimeCompiler;

/** Compiled RobotKit model plus the semantic logical-axis view. */
class MotionSystemBlueprint {
  public final model:RobotModel;
  public final runtime:RobotRuntimeBlueprint;
  public final axes:Array<MotionAxisBlueprint>;
  public final fixedTimestepSeconds:Float;
  /** Smooth replacement lead in owner periods; defaults to two. */
  public var replacementMarginOwnerPeriods:Int = 2;
  /** Owner period used for replacement lead; defaults to fixedTimestepSeconds. */
  public var replacementOwnerPeriodSeconds:Float;

  public function new(model:RobotModel, runtime:RobotRuntimeBlueprint,
      axes:Array<MotionAxisBlueprint>, ?fixedTimestepSeconds:Float = 0.01) {
    if (model == null || runtime == null) throw "Motion system needs a robot model and runtime blueprint";
    if (axes == null || axes.length == 0) throw "Motion system needs at least one logical axis";
    if (!Math.isFinite(fixedTimestepSeconds) || fixedTimestepSeconds <= 0.0)
      throw "Motion system timestep must be finite and positive";
    var ids = new Map<String, Bool>();
    for (axis in axes) {
      if (axis == null) throw "Motion system cannot contain a null axis";
      if (ids.exists(axis.id)) throw 'Motion system contains duplicate axis "${axis.id}"';
      ids.set(axis.id, true);
    }
    this.model = model;
    this.runtime = runtime;
    this.axes = axes.copy();
    this.fixedTimestepSeconds = fixedTimestepSeconds;
    this.replacementOwnerPeriodSeconds = fixedTimestepSeconds;
  }

  public static function fromRobotModel(model:RobotModel, axes:Array<MotionAxisBlueprint>,
      ?revision:Int = 1, ?fixedTimestepSeconds:Float = 0.01):MotionSystemBlueprint {
    if (model == null || axes == null) throw "Motion system needs a model and axes";
    var mapped:Array<MotionAxisBlueprint> = [];
    for (axis in axes) {
      if (axis == null || axis.hasExplicitMapping) {
        mapped.push(axis);
        continue;
      }
      var ratios:Array<Float> = [];
      var offsets:Array<Float> = [];
      var transmitted:Array<Bool> = [];
      var found = 0;
      for (jointId in axis.jointIds) {
        var selected:Null<robotkit.model.Actuator> = null;
        // The first actuator is the deterministic reference if a joint has
        // two drives. Device-side skew monitoring is specified separately.
        for (actuator in model.actuators) {
          if (actuator == null || actuator.transmission == null)
            throw "Motion system model contains an actuator without a transmission";
          switch actuator.transmission {
          case SimpleTransmission(targetId, _, _) if (targetId == jointId):
            if (selected == null) selected = actuator;
          case _:
          }
        }
        if (selected == null) {
          ratios.push(1.0);
          offsets.push(0.0);
          transmitted.push(false);
        } else switch selected.transmission {
          case SimpleTransmission(_, ratio, offset):
            ratios.push(ratio);
            offsets.push(offset);
            transmitted.push(true);
            found++;
        }
      }
      var scales = [for (_ in axis.jointIds) 1.0];
      var jointOffsets = [for (_ in axis.jointIds) 0.0];
      var resolved = [for (index in 0...axis.jointIds.length) index == 0];
      if (found > 0 && transmitted[0]) {
        var referenceRatio = ratios[0], referenceOffset = offsets[0];
        // Equating actuator coordinates gives q_i = o_i + (r_0/r_i)*(q_0-o_0).
        for (index in 0...ratios.length) if (transmitted[index]) {
          scales[index] = referenceRatio / ratios[index];
          jointOffsets[index] = offsets[index] - scales[index] * referenceOffset;
          resolved[index] = true;
        }
      }
      var coupled = false;
      for (_ in 0...axis.jointIds.length) for (coupling in model.couplings) {
          var leader = axis.jointIds.indexOf(coupling.leader);
          var follower = axis.jointIds.indexOf(coupling.follower);
          if (leader < 0 || follower < 0) continue;
          // A joint that sums several leaders (a CoreXY motor) moves with no one axis: it maps to none.
          var terms = 0;
          for (other in model.couplings) if (other.follower == coupling.follower) terms++;
          if (terms > 1) continue;
          coupled = true;
          if (resolved[leader]) {
            scales[follower] = coupling.ratio * scales[leader];
            jointOffsets[follower] = coupling.ratio * jointOffsets[leader] + coupling.offset;
            resolved[follower] = true;
          } else if (resolved[follower]) {
            scales[leader] = scales[follower] / coupling.ratio;
            jointOffsets[leader] = (jointOffsets[follower] - coupling.offset) / coupling.ratio;
            resolved[leader] = true;
          }
      }
      if (found == 0 && !coupled) {
        mapped.push(axis);
        continue;
      }
      for (index in 0...resolved.length) if (!resolved[index])
        throw 'Motion axis "${axis.id}" has an unmapped joint "${axis.jointIds[index]}"';
      mapped.push(new MotionAxisBlueprint(axis.id, axis.jointIds,
        axis.lowerLimit, axis.upperLimit, axis.maxVelocity,
        axis.maxAcceleration, axis.homePosition, scales, jointOffsets));
    }
    return new MotionSystemBlueprint(model, RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile(), revision),
      mapped, fixedTimestepSeconds);
  }
}
