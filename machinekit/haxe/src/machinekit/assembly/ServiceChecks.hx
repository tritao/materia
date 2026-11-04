package machinekit.assembly;

import cadkit.modeling.AssemblyState;
import cadkit.modeling.Part;
import machinekit.assembly.PosedParts.PartBox;
import machinekit.design.DesignRequirement;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/**
 * The service checks `AssemblyIntegrity` runs on the assembly's declared requirements:
 *
 * - `integrity.service.<id>` (`ServiceRemoval`): the removed members, taken along their steps with the prerequisites
 *   already off, clear every member that stays. The screws that hold the removed and the prerequisite members come off with
 *   them. The path is sampled every `STEP` mm (and at its end), with the running gap against any member that is farther than
 *   the gap when the removal starts; members already in contact at the start (a flange on its seat) only may not interpenetrate.
 *   A sampled sweep: a sliver thinner than `STEP` can be missed.
 * - `integrity.drain.<id>` (`GuardedChannel`): a channel a cover protects states how it drains: the floor falls at least
 *   `MIN_FALL` and the bore the design names exists in the member that carries it, in material.
 */
class ServiceChecks {
  /** Distance between the sampled poses of a removal, mm. */
  public static inline var STEP:Float = 2.0;
  /** Least fall of a channel floor toward its drain, mm per mm. */
  public static inline var MIN_FALL:Float = 0.01;

  public static function run(assembly:MachineAssembly, flat:FlatAssembly, state:AssemblyState, posed:PosedParts,
      diagnostics:Diagnostics, overlapLimit:Float, runningGap:Null<Float>, rest:Map<String, Float>):Void {
    for (requirement in flat.designRequirements) switch requirement {
      case ServiceRemoval(id, members, steps, prerequisites, pose):
        try removal(flat, state, posed, diagnostics, overlapLimit, runningGap, rest, id, members, steps, prerequisites, pose) catch (error:Dynamic)
          diagnostics.error("integrity.service." + id, id, 'Service removal "$id" could not be checked: ' + Std.string(error));
      case GuardedChannel(id, channel, covers, drain):
        try channelDrain(flat, state, posed, diagnostics, id, channel, covers, drain) catch (error:Dynamic)
          diagnostics.error("integrity.drain." + id, id, 'Guarded channel "$id" could not be checked: ' + Std.string(error));
      case _:
    }
  }

  static function removal(flat:FlatAssembly, state:AssemblyState, posed:PosedParts, diagnostics:Diagnostics, overlapLimit:Float,
      runningGap:Null<Float>, rest:Map<String, Float>, id:String, members:Array<String>, steps:Array<ServiceStep>,
      prerequisites:Array<String>, pose:Array<{joint:String, value:Float}>):Void {
    var code = "integrity.service." + id;
    if (members.length == 0 || steps.length == 0) throw "a removal needs members and at least one step";
    var known = new Map<String, Bool>();
    for (member in flat.members) known.set(member.id, true);
    for (name in members.concat(prerequisites)) if (!known.exists(name)) throw 'unknown member "$name"';
    for (id in rest.keys()) state.setJoint(id, rest.get(id));
    for (j in pose) state.setJoint(j.joint, j.value);
    state.forwardKinematics();
    // Members that are off: the prerequisites and every screw that holds a removed or prerequisite member.
    var away = new Map<String, Bool>();
    for (name in prerequisites) away.set(name, true);
    var moving = new Map<String, Bool>();
    for (name in members) moving.set(name, true);
    for (joint in flat.definition.joints) if (joint.role == AssemblyJointRole.Tree && joint.type == AssemblyJointType.Fixed) {
      var parentScrew = Std.isOfType(flat.member(joint.parent), machinekit.standard.ScrewFastener);
      var childScrew = Std.isOfType(flat.member(joint.child), machinekit.standard.ScrewFastener);
      if (childScrew && (away.exists(joint.parent) || moving.exists(joint.parent))) away.set(joint.child, true);
      if (parentScrew && (away.exists(joint.child) || moving.exists(joint.child))) away.set(joint.parent, true);
    }
    for (name in members) away.remove(name);
    // Screws that hold a removed member to a member that stays are unscrewed: they were marked above via the removed member.
    var movers:Array<{id:String, part:Part, box:PartBox, base:AssemblyFrame}> = [];
    var obstacles:Array<{id:String, part:Part, box:PartBox}> = [];
    var owned:Array<Part> = [];
    try {
      for (member in flat.members) {
        if (away.exists(member.id)) continue;
        var world = state.worldPose(member.id);
        if (moving.exists(member.id)) {
          var part = posed.posed(member.component, world);
          owned.push(part);
          movers.push({id: member.id, part: part, box: PosedParts.boxOf(part), base: world});
        } else {
          var part = posed.posed(member.component, world);
          owned.push(part);
          obstacles.push({id: member.id, part: part, box: PosedParts.boxOf(part)});
        }
      }
      // Contacts at the start: those pairs are not held to the running gap.
      var touching = new Map<String, Bool>();
      var gap = runningGap == null ? 0.0 : runningGap;
      if (gap > 0) for (mover in movers) for (other in obstacles) {
        if (PosedParts.apart(mover.box, other.box, gap)) continue;
        if (mover.part.distance(other.part) < gap) touching.set(mover.id + "|" + other.id, true);
      }
      var blocked = new Map<String, Bool>();
      var travelled = 0.0, offset = [0.0, 0.0, 0.0];
      for (step in steps) {
        var length = Math.sqrt(step.direction[0] * step.direction[0] + step.direction[1] * step.direction[1] + step.direction[2] * step.direction[2]);
        if (!(length > 1e-9) || !(step.distance > 0)) throw "a step needs a direction and a positive distance";
        var unit = [for (c in step.direction) c / length];
        var count = Math.ceil(step.distance / STEP);
        var origin = offset.copy();
        for (k in 1...count + 1) {
          var d = Math.min(step.distance, k * STEP);
          var now = [origin[0] + unit[0] * d, origin[1] + unit[1] * d, origin[2] + unit[2] * d];
          for (mover in movers) {
            var frame:AssemblyFrame = {
              x: mover.base.x + now[0], y: mover.base.y + now[1], z: mover.base.z + now[2],
              qx: mover.base.qx, qy: mover.base.qy, qz: mover.base.qz, qw: mover.base.qw
            };
            var shifted = posed.posed(flat.member(mover.id), frame);
            var box = PosedParts.boxOf(shifted);
            for (other in obstacles) {
              var key = mover.id + "|" + other.id;
              if (blocked.exists(key)) continue;
              var margin = touching.exists(key) ? 0.0 : gap;
              if (PosedParts.apart(box, other.box, Math.max(margin, 0.0))) continue;
              var shared = PosedParts.commonVolume(shifted, box, other.part, other.box);
              var close = shared > overlapLimit;
              var measured = 0.0;
              if (!close && margin > 0) {
                measured = shifted.distance(other.part);
                close = measured < margin;
              }
              if (!close) continue;
              blocked.set(key, true);
              diagnostics.error(code, other.id,
                'Removing ${mover.id} (' + members.join(", ") + ') is blocked by ${other.id}: '
                + (shared > overlapLimit ? 'they share ${round(shared)} mm3' : 'only ${round(measured)} mm apart (running gap ${round(margin)} mm)')
                + ' after ${round(travelled + d)} mm of the path, at ${round(now[0])}, ${round(now[1])}, ${round(now[2])} from the installed position'
                + (prerequisites.length > 0 ? '; prerequisites off: ' + prerequisites.join(", ") : ""));
            }
            shifted.close();
          }
        }
        offset = [origin[0] + unit[0] * step.distance, origin[1] + unit[1] * step.distance, origin[2] + unit[2] * step.distance];
        travelled += step.distance;
      }
    } catch (error:Dynamic) {
      for (part in owned) part.close();
      for (id in rest.keys()) state.setJoint(id, rest.get(id));
      throw error;
    }
    for (part in owned) part.close();
    for (id in rest.keys()) state.setJoint(id, rest.get(id));
    state.forwardKinematics();
  }

  static function channelDrain(flat:FlatAssembly, state:AssemblyState, posed:PosedParts, diagnostics:Diagnostics, id:String,
      channel:String, covers:Array<String>, drain:Array<ChannelDrain>):Void {
    var code = "integrity.drain." + id;
    var known = new Map<String, Bool>();
    for (member in flat.members) known.set(member.id, true);
    if (!known.exists(channel)) throw 'unknown channel member "$channel"';
    for (cover in covers) if (!known.exists(cover)) throw 'unknown cover member "$cover"';
    if (drain.length == 0) {
      diagnostics.error(code, channel, 'Channel $channel is guarded by ${covers.join(", ")} but declares no drain path to a chip/coolant drain');
      return;
    }
    for (path in drain) {
      var dx = path.low[0] - path.high[0], dy = path.low[1] - path.high[1], run = Math.sqrt(dx * dx + dy * dy), fall = path.high[2] - path.low[2];
      if (!(fall > 0) || (run > 1e-9 && fall / run < MIN_FALL - 1e-12)) diagnostics.error(
        code, channel,
        'The floor of $channel falls ${round(fall)} mm over ${round(run)} mm (${round(run > 0 ? 100 * fall / run : 0)} %): at least ${round(100 * MIN_FALL)} % is needed to carry chips and coolant to the drain'
      );
      if (!known.exists(path.drain)) throw 'unknown drain member "${path.drain}"';
      if (!(path.diameter > 0) || !(path.outletZ < path.low[2])) throw "the drain needs a diameter and an outlet below the floor";
      var drainPart = posed.posed(flat.member(path.drain), state.worldPose(path.drain));
      var box = PosedParts.boxOf(drainPart);
      var radius = path.diameter / 2;
      var bore = Part.cylinderSpan(radius - 0.1, path.outletZ, path.low[2] - 0.2, path.low[0], path.low[1]);
      var wall = Part.cylinderSpan(radius + 4, path.outletZ + 1, path.low[2] - 3, path.low[0], path.low[1]);
      var clear = PosedParts.commonVolume(bore, PosedParts.boxOf(bore), drainPart, box);
      var around = PosedParts.commonVolume(wall, PosedParts.boxOf(wall), drainPart, box);
      bore.close();
      wall.close();
      drainPart.close();
      if (clear > overlapLimitForBore()) diagnostics.error(
        code, path.drain, 'The drain bore (diameter ${round(path.diameter)} mm from z ${round(path.low[2])} down to ${round(path.outletZ)}) is blocked: ${round(clear)} mm3 of ${path.drain} stands in it'
      );
      else if (!(around > 1.0)) diagnostics.error(
        code, path.drain, 'No drain bore exists at (${round(path.low[0])}, ${round(path.low[1])}): ${path.drain} has no material around that axis, so there is no hole through it'
      );
    }
  }

  static inline function overlapLimitForBore():Float return 1.0;

  static function round(value:Float):Float return Math.round(value * 1000) / 1000;
}
