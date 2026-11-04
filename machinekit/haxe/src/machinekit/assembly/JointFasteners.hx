package machinekit.assembly;

import machinekit.component.BoltedFace;
import machinekit.component.BoltedFace.BoltHole;
import machinekit.component.BoltedFace.BoltRecess;
import machinekit.component.BoltedFace.MachinedCut;
import machinekit.component.BoltedFace.ThreadMaterial;
import machinekit.assembly.Diagnostics.DiagnosticSeverity;
import machinekit.component.BomItem;
import machinekit.component.MachineComponent;
import machinekit.standard.MetricScrewSpec;
import machinekit.standard.PropertyClass;
import machinekit.standard.SocketHeadCapScrew;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

typedef FastenerOptions = {
  /** The fastener class; 12.9, as the catalogue screws are drawn, when absent. */
  @:optional var propertyClass:PropertyClass;
  /** How far (mm) a hole may stand from its partner, and a face from the other's plane. */
  @:optional var tolerance:Float;
};

/** One bolted joint: two faces that meet, the fastener derived for them and the host drilling it needs. */
typedef FastenedJoint = {
  var id:String;
  var clampedMember:String;
  var clampedFace:String;
  var receivingMember:String;
  var receivingFace:String;
  /** The fastener standard and designation of one fastener, and how many the joint takes. */
  var standard:String;
  var size:String;
  var length:Float;
  var propertyClass:PropertyClass;
  var count:Int;
  /** Clamped material under the head, thread engagement and the least engagement the receiving material needs, mm. */
  var grip:Float;
  var engagement:Float;
  var minEngagement:Float;
  /** The holes in the receiving face frame, and whether the joint made them (the host is drilled to suit). */
  var hostHoles:Array<BoltHole>;
  var drilledToSuit:Bool;
  /** The tap-drill diameter and the depths of thread and of drilling the host needs for them, mm. */
  var tapDrill:Float;
  var threadDepth:Float;
  var drillDepth:Float;
  var item:BomItem;
}

/** What `JointFasteners.derive` found: the joints, every finding, and the purchase lines the joints add up to. */
class FastenerDerivation {
  public final joints:Array<FastenedJoint> = [];
  public final diagnostics = new Diagnostics();
  /**
	 * The holes the joints need cut into their parts, by member id, in each part's frame: the tap-drill holes of a host
	 * drilled to suit and the clearance holes of a clamped face whose holes the part does not draw. They are derived once,
	 * here, from the solved joint; `MachineAssembly.deriveFasteners` hands them to the parts (`MachineComponent.machine`).
	 */
  public final machining = new Map<String, Array<MachinedCut>>();

  public function new() {
  }

  /** Throws every error found, one per line. */
  public function require():Void diagnostics.throwIfErrors();

  public function codes():Array<String> return [for (item in diagnostics.items) if (item.severity == DiagnosticSeverity.Error) item.code];

  /** The purchase lines: joints that use the same fastener are one line, counted once per hole. */
  public function lines():Array<{item:BomItem, quantity:Int}> {
    var out:Array<{item:BomItem, quantity:Int}> = [];
    var at = new Map<String, Int>();
    for (joint in joints) {
      var key = joint.item.partNumber;
      if (at.exists(key)) out[at.get(key)].quantity += joint.count;
      else {
        at.set(key, out.length);
        out.push({item: joint.item, quantity: joint.count});
      }
    }
    return out;
  }

  public function count():Int {
    var total = 0;
    for (joint in joints) total += joint.count;
    return total;
  }
}

private typedef Placed = {member:String, face:BoltedFace, world:AssemblyFrame};

/**
 * Derives the screws of the bolted joints of an assembly from the faces its parts declare (`MachineComponent.fasteningFaces`),
 * so no assembly lists fasteners by hand. A clamped face must meet exactly one receiving face: the planes coincide with
 * opposed normals, and the members are held together by fixed mates. The two are then validated against each other:
 *
 * - every hole of the clamped face lands on a tapped hole of the receiving face, or inside its footprint (with an edge
 *   distance) where the host is drilled to suit; a receiving hole is used once;
 * - the fastener size agrees where both sides name one, the clearance hole passes it, and its head fits the recess;
 * - the length is the shortest ISO 4762 length whose engagement in the receiving material (`BoltedFace.engagementFactor`
 *   nominal diameters) is met beyond the grip, and the thread, with a tip gap in a blind hole, takes that engagement.
 *
 * A host drilled to suit gets its holes, tap drill and depths in the result. Findings use the codes `fasteners.*`.
 */
class JointFasteners {
  public static inline var TOLERANCE:Float = 0.05;
  /** Centre-to-edge distance of a derived hole in a footprint, in nominal diameters. */
  public static inline var EDGE_DIAMETERS:Float = 1.5;
  /** Centre-to-centre distance of holes derived in one footprint, in nominal diameters. */
  public static inline var SPACING_DIAMETERS:Float = 2.5;
  /** Back wall left behind a blind drilling, mm. */
  public static inline var BACK_WALL:Float = 1.0;

  public static function derive(assembly:MachineAssembly, ?options:FastenerOptions):FastenerDerivation {
    var result = new FastenerDerivation();
    var tolerance = options == null || options.tolerance == null ? TOLERANCE : options.tolerance;
    var propertyClass = options == null || options.propertyClass == null ? PropertyClass.Steel129 : options.propertyClass;
    var poses = assembly.solvedPoses();
    var group = rigidGroups(assembly);
    var clamped:Array<Placed> = [], receiving:Array<Placed> = [];
    for (entry in assembly.components()) {
      var pose = poses.get(entry.id);
      if (pose == null) continue;
      var seen = new Map<String, Bool>();
      for (face in entry.component.fasteningFaces()) {
        if (seen.exists(face.id)) {
          result.diagnostics.error("fasteners.face", entry.id, 'Member ${entry.id} declares face "${face.id}" twice');
          continue;
        }
        seen.set(face.id, true);
        var placed = {member: entry.id, face: face, world: AssemblyFrames.compose(pose, face.frame)};
        (face.isClamped() ? clamped : receiving).push(placed);
      }
    }
    var members = new Map<String, MachineComponent>();
    for (entry in assembly.components()) members.set(entry.id, entry.component);
    var used = new Map<String, String>();
    var derived = new Map<String, Array<{x:Float, y:Float, joint:String}>>();
    for (a in clamped) {
      var subject = a.member + "#" + a.face.id;
      var candidates = [for (b in receiving) if (b.member != a.member && meets(a, b, tolerance)) b];
      if (candidates.length == 0) {
        result.diagnostics.error(
          "fasteners.unmatched",
          subject,
          '$subject has clearance holes but no receiving face where it lies: its fasteners have nothing to thread into'
        );
        continue;
      }
      if (candidates.length > 1) {
        result.diagnostics.error(
          "fasteners.ambiguous",
          subject,
          '$subject meets several receiving faces: ${[for (b in candidates) b.member + "#" + b.face.id].join(", ")}'
        );
        continue;
      }
      var b = candidates[0];
      var joint = joinFaces(result, a, b, tolerance, propertyClass, group, members, used, derived);
      if (joint != null) result.joints.push(joint);
    }
    for (key in derived.keys()) spacing(result, key, derived.get(key), result.joints);
    return result;
  }

  /** Whether `a` lies on `b`'s plane with opposed normals and at least one of its holes over `b`'s footprint. */
  static function meets(a:Placed, b:Placed, tolerance:Float):Bool {
    var local = AssemblyFrames.compose(AssemblyFrames.inverse(b.world), a.world);
    var normal = AssemblyFrames.transformVector(local, 0, 0, 1);
    if (normal.z > -0.9999 || Math.abs(local.z) > tolerance) return false;
    // The holes' extent overlaps the receiving extent: a face that merely lies on the plane elsewhere is another face.
    var lowX = Math.POSITIVE_INFINITY, highX = Math.NEGATIVE_INFINITY, lowY = Math.POSITIVE_INFINITY, highY = Math.NEGATIVE_INFINITY;
    for (hole in a.face.holes) {
      var at = AssemblyFrames.transformPoint(local, hole.x, hole.y, 0);
      lowX = Math.min(lowX, at.x);
      highX = Math.max(highX, at.x);
      lowY = Math.min(lowY, at.y);
      highY = Math.max(highY, at.y);
    }
    var tapped = b.face.holes;
    var bLowX = -b.face.width / 2, bHighX = b.face.width / 2, bLowY = -b.face.height / 2, bHighY = b.face.height / 2;
    if (tapped != null) {
      bLowX = Math.POSITIVE_INFINITY;
      bHighX = Math.NEGATIVE_INFINITY;
      bLowY = Math.POSITIVE_INFINITY;
      bHighY = Math.NEGATIVE_INFINITY;
      for (hole in tapped) {
        bLowX = Math.min(bLowX, hole.x);
        bHighX = Math.max(bHighX, hole.x);
        bLowY = Math.min(bLowY, hole.y);
        bHighY = Math.max(bHighY, hole.y);
      }
    }
    return lowX <= bHighX + tolerance && highX >= bLowX - tolerance && lowY <= bHighY + tolerance && highY >= bLowY - tolerance;
  }

  /** Whether (x, y) in `face` is on one of its holes, or inside its footprint inset by `edge`. */
  static function over(face:BoltedFace, x:Float, y:Float, tolerance:Float, edge:Float):Bool {
    if (face.holes != null) {
      for (hole in face.holes) if (Math.abs(hole.x - x) <= tolerance && Math.abs(hole.y - y) <= tolerance) return true;
      return false;
    }
    return Math.abs(x) <= face.width / 2 - edge + 1e-9 && Math.abs(y) <= face.height / 2 - edge + 1e-9;
  }

  static function joinFaces(
    result:FastenerDerivation,
    a:Placed,
    b:Placed,
    tolerance:Float,
    propertyClass:PropertyClass,
    group:Map<String, Int>,
    members:Map<String, MachineComponent>,
    used:Map<String, String>,
    derived:Map<String, Array<{x:Float, y:Float, joint:String}>>
  ):Null<FastenedJoint> {
    var diagnostics = result.diagnostics;
    var id = a.member + "#" + a.face.id + " -> " + b.member + "#" + b.face.id;
    var before = diagnostics.items.length;
    if (group.get(a.member) != group.get(b.member)) diagnostics.error(
      "fasteners.not-fixed",
      id,
      '$id: the members are not held together by fixed mates, so a bolted joint between them contradicts the model'
    );
    var clampedSize:Null<String> = null, clearance = 0.0, grip = 0.0, recess:Null<BoltRecess> = null;
    switch a.face.role {
      case Clamped(size, c, g, r):
        clampedSize = size;
        clearance = c;
        grip = g;
        recess = r;
      case _:
    }
    var receivingSize:Null<String> = null, depth = 0.0, thickness = 0.0, through = false, material:ThreadMaterial = ThreadMaterial.Steel;
    switch b.face.role {
      case Receiving(size, d, t, th, m):
        receivingSize = size;
        depth = d;
        thickness = t;
        through = th;
        material = m;
      case _:
    }
    if (clampedSize != null && receivingSize != null && clampedSize != receivingSize) diagnostics.error(
      "fasteners.thread",
      id,
      '$id: the clearance holes are for $clampedSize but the thread is $receivingSize'
    );
    var size = clampedSize ?? receivingSize;
    if (size == null) {
      diagnostics.error("fasteners.size", id, '$id: neither face names the fastener size');
      return null;
    }
    var catalog = SocketHeadCapScrew.catalog();
    if (!catalog.exists(size)) {
      diagnostics.error("fasteners.size", id, '$id: no catalogue screw of size $size');
      return null;
    }
    var spec = catalog.get(size);
    // Pattern correspondence, in the receiving face frame.
    var local = AssemblyFrames.compose(AssemblyFrames.inverse(b.world), a.world);
    var points:Array<BoltHole> = [];
    var matched = new Map<Int, Int>();
    var tappedHoles = b.face.holes;
    var drilled = tappedHoles == null;
    var index = 0;
    for (hole in a.face.holes) {
      var at = AssemblyFrames.transformPoint(local, hole.x, hole.y, 0);
      points.push({x: at.x, y: at.y});
      if (tappedHoles == null) {
        if (!over(b.face, at.x, at.y, tolerance, EDGE_DIAMETERS * spec.diameter)) diagnostics.error(
          "fasteners.edge",
          id,
          '$id: hole ${index + 1} at (${round(at.x)}, ${round(at.y)}) is within ${EDGE_DIAMETERS} diameters of the edge of the '
          + 'receiving footprint ${round(b.face.width)} x ${round(b.face.height)} or outside it'
        );
      } else {
        var found = -1;
        for (i in 0...tappedHoles.length) {
          var candidate = tappedHoles[i];
          if (Math.abs(candidate.x - at.x) <= tolerance && Math.abs(candidate.y - at.y) <= tolerance) found = i;
        }
        if (found < 0) diagnostics.error(
          "fasteners.pattern",
          id,
          '$id: hole ${index + 1} at (${round(at.x)}, ${round(at.y)}) has no tapped hole under it; the patterns differ'
        );
        else if (matched.exists(found)) diagnostics.error(
          "fasteners.pattern",
          id,
          '$id: holes ${matched.get(found) + 1} and ${index + 1} fall on the same tapped hole'
        );
        else matched.set(found, index);
        if (found >= 0) {
          var key = b.member + "#" + b.face.id + "#" + found;
          if (used.exists(key)) diagnostics.error(
            "fasteners.duplicate",
            id,
            '$id: tapped hole ${found + 1} of ${b.member}#${b.face.id} already holds a fastener of ${used.get(key)}'
          );
          else used.set(key, id);
        }
      }
      index++;
    }
    // The fastener.
    if (clearance + 1e-9 < spec.clearanceFine) diagnostics.error(
      "fasteners.clearance",
      id,
      '$id: ${round(clearance)} mm clearance holes do not pass an $size screw (${round(spec.clearanceFine)} mm needed)'
    );
    if (recess != null && (spec.headDiameter > recess.diameter + 1e-9 || spec.headHeight > recess.depth + 1e-9)) diagnostics.error(
      "fasteners.head-clearance",
      id,
      '$id: an $size head (${round(spec.headDiameter)} dia x ${round(spec.headHeight)}) does not fit its '
      + '${round(recess.diameter)} dia x ${round(recess.depth)} recess'
    );
    var minEngagement = BoltedFace.engagementFactor(material) * spec.diameter;
    var tipGap = through ? 0.0 : spec.pitch;
    var reach = depth - tipGap;
    if (minEngagement > reach + 1e-9) diagnostics.error(
      "fasteners.engagement",
      id,
      '$id: ${material} needs ${round(minEngagement)} mm of $size thread engagement but the receiving thread allows '
      + '${round(reach)} mm (depth ${round(depth)}${through ? "" : ", less a tip gap of " + round(tipGap)})'
    );
    var length = SocketHeadCapScrew.standardLengthAtLeast(grip + minEngagement);
    if (length == null) {
      diagnostics.error("fasteners.length", id, '$id: no standard length reaches grip ${round(grip)} + engagement ${round(minEngagement)}');
      return null;
    }
    var engagement = length - grip;
    if (engagement > reach + 1e-9 && minEngagement <= reach + 1e-9) diagnostics.error(
      "fasteners.length",
      id,
      '$id: the shortest standard length ${round(length)} gives ${round(engagement)} mm of engagement; the thread takes '
      + '${round(reach)} mm. No standard length fits grip ${round(grip)}'
    );
    if (length - spec.threadLength > grip + 1e-9) diagnostics.error(
      "fasteners.shank",
      id,
      '$id: the unthreaded shank of an $size x ${round(length)} screw (${round(length - spec.threadLength)} mm) is longer than the grip '
      + '${round(grip)} mm, so it enters the thread'
    );
    var threadDepth = Math.min(depth, Math.ceil((engagement + tipGap) * 2) / 2);
    var drillDepth = through ? thickness : threadDepth + 2 * spec.pitch + 0.3 * spec.diameter;
    var wall = thickness;
    var wallAt = b.face.thicknessAt;
    if (wallAt != null) for (point in points) wall = Math.min(wall, wallAt(point.x, point.y));
    if (drilled && !through && drillDepth + BACK_WALL > wall + 1e-9) diagnostics.error(
      "fasteners.back-wall",
      id,
      '$id: a ${round(drillDepth)} mm tapped hole leaves less than ${BACK_WALL} mm of back wall in ${round(wall)} mm of material'
    );
    if (hasErrorSince(diagnostics, before)) return null;
    var screw = SocketHeadCapScrew.metric(size, length, propertyClass);
    var host = b.member + "#" + b.face.id;
    if (drilled) {
      if (!derived.exists(host)) derived.set(host, []);
      for (point in points) derived.get(host).push({x: point.x, y: point.y, joint: id});
    }
    // The holes the parts are cut with, in their own frames.
    if (drilled) {
      if (!members.get(b.member).machinable()) diagnostics.error(
        "fasteners.machining",
        id,
        '$id: ${b.member} is drilled to suit the joint but its geometry does not cut derived holes'
      );
      else for (point in points) cutsOf(result, b.member).push({
        frame: b.face.frame,
        x: point.x,
        y: point.y,
        diameter: spec.tapDrill,
        from: -1.0,
        to: through ? thickness + 1 : drillDepth
      });
    }
    if (!a.face.drawn) {
      if (!members.get(a.member).machinable()) diagnostics.error(
        "fasteners.machining",
        id,
        '$id: ${a.member} has clearance holes its geometry does not draw and does not cut derived holes'
      );
      else for (hole in a.face.holes) {
        var back = grip + (recess == null ? 0.0 : recess.depth);
        cutsOf(result, a.member).push({frame: a.face.frame, x: hole.x, y: hole.y, diameter: clearance, from: -1.0, to: back + 1});
        if (recess != null) cutsOf(result, a.member).push({
          frame: a.face.frame,
          x: hole.x,
          y: hole.y,
          diameter: recess.diameter,
          from: grip,
          to: back + 1
        });
      }
    }
    if (hasErrorSince(diagnostics, before)) return null;
    return {
      id: id,
      clampedMember: a.member,
      clampedFace: a.face.id,
      receivingMember: b.member,
      receivingFace: b.face.id,
      standard: "ISO 4762",
      size: size,
      length: length,
      propertyClass: propertyClass,
      count: points.length,
      grip: grip,
      engagement: engagement,
      minEngagement: minEngagement,
      hostHoles: points,
      drilledToSuit: drilled,
      tapDrill: spec.tapDrill,
      threadDepth: threadDepth,
      drillDepth: drillDepth,
      item: screw.bom
    };
  }

  static function cutsOf(result:FastenerDerivation, member:String):Array<MachinedCut> {
    var cuts = result.machining.get(member);
    if (cuts == null) {
      cuts = [];
      result.machining.set(member, cuts);
    }
    return cuts;
  }

  static function hasErrorSince(diagnostics:Diagnostics, from:Int):Bool {
    for (i in from...diagnostics.items.length) if (diagnostics.items[i].severity == DiagnosticSeverity.Error) return true;
    return false;
  }

  /**
	 * Holes derived in one footprint by different joints must not coincide (one hole counted twice) nor crowd each other.
	 */
  static function spacing(result:FastenerDerivation, host:String, holes:Array<{x:Float, y:Float, joint:String}>,
      joints:Array<FastenedJoint>):Void {
    var diameter = 0.0;
    for (joint in joints) if (joint.drilledToSuit && joint.receivingMember + "#" + joint.receivingFace == host)
      diameter = Math.max(diameter, Std.parseFloat(joint.size.substr(1)));
    for (i in 0...holes.length) for (j in i + 1...holes.length) {
      var dx = holes[i].x - holes[j].x, dy = holes[i].y - holes[j].y, distance = Math.sqrt(dx * dx + dy * dy);
      if (distance < 1e-3) result.diagnostics.error(
        "fasteners.duplicate",
        host,
        'Two fasteners (${holes[i].joint}; ${holes[j].joint}) are derived for the same hole of $host'
      );
      else if (distance < SPACING_DIAMETERS * diameter) result.diagnostics.error(
        "fasteners.spacing",
        host,
        'Derived holes of $host are only ${round(distance)} mm apart (${holes[i].joint}; ${holes[j].joint})'
      );
    }
  }

  /** The rigid body each member belongs to, joined by its fixed tree mates. */
  static function rigidGroups(assembly:MachineAssembly):Map<String, Int> {
    var flat = assembly.derived();
    var parent = new Map<String, String>();
    function find(id:String):String {
      var p = parent.get(id);
      if (p == null || p == id) return id;
      var root = find(p);
      parent.set(id, root);
      return root;
    }
    for (member in flat.members) parent.set(member.id, member.id);
    for (joint in flat.definition.joints) if (joint.role == AssemblyJointRole.Tree && joint.type == AssemblyJointType.Fixed)
      parent.set(find(joint.child), find(joint.parent));
    var roots = new Map<String, Int>();
    var out = new Map<String, Int>();
    for (member in flat.members) {
      var root = find(member.id);
      if (!roots.exists(root)) roots.set(root, [for (_ in roots) 0].length);
      out.set(member.id, roots.get(root));
    }
    return out;
  }

  static function round(value:Float):Float return Math.round(value * 100) / 100;
}
