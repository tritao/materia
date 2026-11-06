import machinekit.assembly.AxisBuilder;
import machinekit.structural.*;
import machinekit.standard.*;
import materia.assembly.AssemblyFrames as F;
import materia.casting.InsertHole;

/** A thread exception is a checked coaxial mating thread, never an occurrence
 * name exemption. Head/plain shank/tip constraints remain part of the check. */
class RackMountInterfaces {
  static final checked:Map<String, Bool> = [];
  /** These exact bounded/deforming interfaces are verified by RadialPinionTests;
  * they are not generic clearance exemptions for an entire moving body. */
  public static function boundedMotionFit(machine:AxisBuilder, a:String, b:String):Null<String> {
    function pair(x:String, y:String):Bool return a == x && b == y || a == y && b == x;
    for (suffix in ["X", "YLeft", "YRight"]) {
      var p = "pinion" + suffix;
      if (pair(p + "Preload", p + "UpperStop") || pair(p + "Preload",
        p + "LowerStop")) return "monotone hard-stop contact; native stroke bounds";
      if (pair(p + "Preload", p + "PivotThrustFront") || pair(p + "Preload",
        p + "PivotThrustRear")) return "coaxial planar thrust fit; native bounds and selected axial endplay";
      if (pair(p + "PivotBush", p + "PivotThrustFront") || pair(p + "PivotBush", p + "PivotThrustRear")) {
        var bush:machinekit.standard.Bushing = cast machine.component(p + "PivotBush");
        var thrustId = a == p + "PivotBush" ? b : a;
        var thrust:machinekit.standard.Bushing = cast machine.component(thrustId);
        var relative = F.compose(F.inverse(machine.zeroPose(p + "PivotBush")), machine.zeroPose(thrustId));
        var axis = F.transformVector(relative, 0, 0, 1);
        if (Math.abs(relative.x) + Math.abs(relative.y) + Math.abs(axis.x) + Math.abs(axis.y) + Math.abs(axis.z - 1)
          > 1e-6) throw "Thrust shim is not coaxial with its pivot bush";
        var gap = relative.z < 0 ? -relative.z - thrust.length : relative.z - bush.length;
        if (gap < 0.005 || gap > 0.02) throw "Pivot bush/thrust axial clearance outside selected fit";
        return "rotation-invariant coaxial annular faces; positive selected axial clearance";
      }
      if (pair(p + "PivotBush",
        p + "PivotPin")) return "coaxial circular journal; native stroke bounds and positive diametral clearance";
      if (pair(p + "SpringCup",
        p + "SpringPack")) return "deforming pack; monotone loaded-height bounds including cup tilt";
      if (pair(p + "SpringCup",
        p + "SpringAdjuster")) return "guide/bore fit; analytic whole-stroke eccentricity bound plus native bounds";
    }
    if (!sameFitAssembly(machine, a, b)) return null;
    var reducerId:String, shaftId:String;
    if (Std.isOfType(machine.component(a), machinekit.motion.PlanetaryReducer) && Std.isOfType(
      machine.component(b),
      machinekit.motion.SteppedShaft
    )) {
      reducerId = a;
      shaftId = b;
    } else if (Std.isOfType(machine.component(b), machinekit.motion.PlanetaryReducer) && Std.isOfType(
      machine.component(a),
      machinekit.motion.SteppedShaft
    )) {
      reducerId = b;
      shaftId = a;
    } else return null;
    var reducer:machinekit.motion.PlanetaryReducer = cast machine.component(reducerId),
      shaft:machinekit.motion.SteppedShaft = cast machine.component(shaftId);
    var relative = F.compose(F.inverse(machine.zeroPose(reducerId)),
      machine.zeroPose(shaftId)), axis = F.transformVector(
        relative,
        0,
        0,
        1
      );
    if (Math.abs(relative.x) + Math.abs(relative.y) + Math.abs(axis.x) + Math.abs(axis.y) > 1e-6) return null;
    var z = relative.z, overlap = false;
    for (section in shaft.sections) {
      var end = z + axis.z * section.length;
      // The output journal may begin exactly at the housing's output plane.
      // A face-seated, bore-sized shaft is also an intentional circular fit.
      if (Math.min(Math.max(z, end), reducer.n("length")) >= Math.max(Math.min(z, end), 0) - 1e-6) {
        overlap = true;
        if (section.diameter
          > reducer.n("bore") + 1e-6) throw "Rotary shaft shoulder does not fit the declared reducer bore";
      }
      z = end;
    }
    return overlap ? "coaxial circular shaft/reducer seating; all seated or overlapping diameters checked" : null;
  }
  /** These groups select their own fit tolerances; they never exempt solid
  * intersections. Independent groups retain the 1.5 mm installation envelope. */
  public static function sameFitAssembly(machine:AxisBuilder, a:String, b:String):Bool {
    var section = ~/^(rackMount(?:X|YLeft|YRight)Section[0-9]+)/;
    var first:Null<String> = null, second:Null<String> = null;
    if (section.match(a)) first = section.matched(1);
    if (section.match(b)) second = section.matched(1);
    if (first != null && first == second) return true;
    for (suffix in ["X", "YLeft", "YRight"]) {
      var prefix = "pinion" + suffix;
      var inA = StringTools.startsWith(a, prefix) || a == "motor" + suffix;
      var inB = StringTools.startsWith(b, prefix) || b == "motor" + suffix;
      if (inA && inB) return true;
      if ((a == "rack" + suffix && StringTools.startsWith(b,
        "rackMount" + suffix + "Section")) || (b == "rack" + suffix && StringTools.startsWith(
          a,
          "rackMount" + suffix + "Section"
        ))) return true;
    }
    return false;
  }
  public static function clearanceBore(machine:AxisBuilder, hostId:String, screwId:String):Bool {
    var host = machine.component(hostId), screw = machine.component(screwId), holes:Array<InsertHole>;
    if (!Std.isOfType(screw, SocketHeadCapScrew)) return false;
    var bolt:SocketHeadCapScrew = cast screw;
    if (Std.isOfType(host, MountDrilledPlate)) holes = cast(host, MountDrilledPlate).holes();
    else if (Std.isOfType(host, RectTubeMember)) holes = cast(host, RectTubeMember).drilling;
    else if (Std.isOfType(host, LinearMountReplication)) holes = InsertHoles.records(host.values().token("holes"));
    else return false;
    var relative = F.compose(F.inverse(machine.zeroPose(hostId)), machine.zeroPose(screwId));
    for (h in holes) if (h.thread == null && h.diameter >= bolt.diameter + 0.2) {
      var p = F.compose(F.inverse(h.frame), relative), axis = F.transformVector(p, 0, 0, 1);
      if (Math.abs(p.x) + Math.abs(p.y) > 1e-6
        || Math.abs(axis.x) + Math.abs(axis.y) > 1e-6 || Math.abs(axis.z + 1) > 1e-6) continue;
      if (Math.min(p.z + bolt.length, h.depth) > Math.max(p.z, 0) + 1e-6) return true;
    }
    return false;
  }
  public static function validateThreadSolid(machine:AxisBuilder,
    cache:machinekit.assembly.PosedParts, fit:Dynamic):Void {
    var host = machine.component(fit.host), screw = machine.component(fit.screw);
    var holes = Std.isOfType(host, MountDrilledPlate) ? cast(host, MountDrilledPlate).holes() : Std.isOfType(
      host,
      LinearMountNutCassette
    ) ? cast(
      host,
      LinearMountNutCassette
    ).serviceThreads() : InsertHoles.records(host.values().token("holes"));
    var matches = [for (h in holes) if (h.id == fit.hole) h];
    if (matches.length != 1) throw "Thread identity lost";
    var h = matches[0], relative = F.compose(F.inverse(machine.zeroPose(fit.host)), machine.zeroPose(fit.screw));
    var key = host.geometryKey(Preview)
      + "|" + screw.geometryKey(Preview) + "|" + haxe.Json.stringify(relative) + "|" + h.id;
    if (checked.exists(key)) return;
    // Remove only the declared thread's major-diameter envelope. Any interference
    // left over is a head, plain-shank, blind-tip or unrelated-stock clash.
    var thread = h.thread;
    if (thread == null) throw "Thread exception lost its thread";
    var raw = cadkit.modeling.Part.cylinder(thread.majorDiameter / 2 + 0.0001, h.depth + 0.0002);
    var tool = machinekit.assembly.PosedParts.moved(raw, F.compose(h.frame, F.translation(0, 0, -0.0001)));
    raw.close();
    var relieved = cache.local(host).subtract(tool);
    tool.close();
    var bolt = machinekit.assembly.PosedParts.moved(cache.local(screw), relative), common = relieved.intersect(bolt);
    var volume = common.volume();
    common.close();
    bolt.close();
    relieved.close();
    if (volume > 0.001) throw 'Interference outside declared thread: ${fit.host} / ${fit.screw}: $volume mm3';
    checked.set(key, true);
  }
  public static function threadFit(machine:AxisBuilder, hostId:String, screwId:String):Null<Dynamic> {
    var host = machine.component(hostId), screw = machine.component(screwId), holes:Array<InsertHole>;
    if (Std.isOfType(host, MountDrilledPlate)) holes = cast(host, MountDrilledPlate).holes();
    else if (Std.isOfType(host, LinearMountNutCassette)) holes = cast(host, LinearMountNutCassette).serviceThreads();
    else if (Std.isOfType(host, LinearMountSteel)) holes = InsertHoles.records(host.values().token("holes"));
    else return null;
    var diameter:Float, pitch:Float, length:Float, threadLength:Float;
    if (Std.isOfType(screw, SocketHeadCapScrew)) {
      var s:SocketHeadCapScrew = cast screw;
      diameter = s.diameter;
      pitch = s.pitch;
      length = s.length;
      threadLength = s.threadLength;
    } else if (Std.isOfType(screw, SocketSetScrew)) {
      var s:SocketSetScrew = cast screw;
      diameter = s.thread.majorDiameter;
      pitch = s.thread.pitch;
      length = s.length;
      threadLength = length;
    } else return null;
    var relative = F.compose(F.inverse(machine.zeroPose(hostId)), machine.zeroPose(screwId));
    for (h in holes) {
      var thread = h.thread;
      if (thread == null
        || Math.abs(thread.majorDiameter - diameter) >= 1e-9 || Math.abs(thread.pitch - pitch) >= 1e-9) continue;
      var p = F.compose(F.inverse(h.frame), relative), axis = F.transformVector(p, 0, 0, 1);
      if (Math.abs(p.x) + Math.abs(p.y) > 1e-6
        || Math.abs(axis.x) + Math.abs(axis.y) > 1e-6 || Math.abs(axis.z + 1) > 1e-6) continue;
      var counterbore = h.counterbore;
      var tip = p.z + length,
        plainEnd = tip - threadLength, threadStart = counterbore == null ? 0.0 : counterbore.depth;
      var fullEnd:Float = cast h.threadDepth, rules = materia.casting.InsertHole.InsertHoleCodec.threadLimits(h);
      var engagement = Math.min(tip, fullEnd) - Math.max(plainEnd, Math.max(0, threadStart));
      if (engagement + 1e-6 < rules.minEngagement
        || plainEnd > threadStart + 1e-6) throw 'Insufficient thread engagement / plain shank: $hostId / $screwId';
      if (h.extent == Blind && tip
        > fullEnd - rules.minTipGap + 1e-6) throw 'Insufficient free usable thread beyond tip: $hostId / $screwId';
      return {
        host: hostId,
        screw: screwId,
        hole: h.id,
        diameterMm: diameter,
        pitchMm: pitch,
        engagementMm: engagement,
        minEngagementMm: rules.minEngagement,
        fullThreadTipGapMm: fullEnd - tip,
        drillTipGapMm: h.depth - tip,
        policy: h.limits.policyId
      };
    }
    return null;
  }
}
