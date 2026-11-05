package machinekit.assembly;

import machinekit.component.BoltedFace;
import machinekit.component.BoltedFace.BoltHole;
import machinekit.component.BoltedFace.BoltRecess;
import machinekit.component.BoltedFace.MachinedCut;
import machinekit.component.BoltedFace.ThreadMaterial;
import machinekit.component.BoltedFace.PinnedFace;
import machinekit.standard.DowelPin;
import machinekit.assembly.Diagnostics.DiagnosticSeverity;
import machinekit.component.BomItem;
import machinekit.component.MachineComponent;
import machinekit.standard.FlatWasher;
import machinekit.standard.HexNut;
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
  /** Length (mm) of the driver access cylinder along the axis beyond a head or nut; `JointFasteners.ACCESS_LENGTH` when absent. */
  @:optional var accessLength:Float;
};

/**
 * A cylinder along a fastener's axis, per hole, in the receiving face frame (+Z into the receiving part, the clamped part
 * at negative z): `from` to `to` mm, `diameter` wide. `AssemblyIntegrity` checks that it is clear of the members: of every
 * member, or for an `access` volume only of those held rigidly with the clamped part (a moving member can be moved away).
 * `code` names the finding (`fasteners.head-clearance`, `fasteners.hole`, `fasteners.access`).
 */
typedef FastenerVolume = {name:String, code:String, diameter:Float, from:Float, to:Float, access:Bool};

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
  /** Clamped material, washer under the head and (for a bolt) the plate and nut washer: the length the thread must clear, mm. */
  var stack:Float;
  /** A bolt: the receiving face is a plate, and a washer and nut hold it. */
  var nutted:Bool;
  /** Washers and nuts the joint adds, one line each, counted over all its holes. */
  var extras:Array<{item:BomItem, count:Int}>;
  var volumes:Array<FastenerVolume>;
}

/** One dowelled joint: two pinned faces that meet with the same holes, and the pins derived for them. */
typedef DowelJoint = {
  var id:String;
  var memberA:String;
  var faceA:String;
  var memberB:String;
  var faceB:String;
  var standard:String;
  var size:String;
  var diameter:Float;
  var length:Float;
  var fit:String;
  var count:Int;
  /** How deep each part's reamed hole is and how far the pin stands in each, mm. */
  var depthA:Float;
  var depthB:Float;
  var engagementA:Float;
  var engagementB:Float;
  /** The holes in the frame of member B's face. */
  var hostHoles:Array<BoltHole>;
  var item:BomItem;
  /** The pin in its holes, in member B's face frame (A lies at negative z). */
  var volumes:Array<FastenerVolume>;
}

/** What `JointFasteners.derive` found: the joints, every finding, and the purchase lines the joints add up to. */
class FastenerDerivation {
  public final joints:Array<FastenedJoint> = [];
  /** The dowelled joints (`PinnedFace`), beside the bolted ones. */
  public final dowels:Array<DowelJoint> = [];
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
      for (line in [{item: joint.item, count: joint.count}].concat(joint.extras)) {
        var key = line.item.partNumber;
        if (at.exists(key)) out[at.get(key)].quantity += line.count;
        else {
          at.set(key, out.length);
          out.push({item: line.item, quantity: line.count});
        }
      }
    }
    for (joint in dowels) {
      var key = joint.item.partNumber;
      if (at.exists(key)) out[at.get(key)].quantity += joint.count;
      else {
        at.set(key, out.length);
        out.push({item: joint.item, quantity: joint.count});
      }
    }
    return out;
  }

  /** The dowel pins over all dowelled joints. */
  public function dowelCount():Int {
    var total = 0;
    for (joint in dowels) total += joint.count;
    return total;
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
  /** The driver's way in along the axis beyond a head or nut: a hex key's short arm, mm. */
  public static inline var ACCESS_LENGTH:Float = 25;
  /** Nuts stand this many pitches beyond the plate's washer: the screw shows at least this far past the nut. */
  public static inline var NUT_PROTRUSION_PITCHES:Float = 2;

  public static function derive(assembly:MachineAssembly, ?options:FastenerOptions):FastenerDerivation {
    var result = new FastenerDerivation();
    var tolerance = options == null || options.tolerance == null ? TOLERANCE : options.tolerance;
    var propertyClass = options == null || options.propertyClass == null ? PropertyClass.Steel129 : options.propertyClass;
    var access = options == null || options.accessLength == null ? ACCESS_LENGTH : options.accessLength;
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
    var matchedAlternatives = new Map<String, Bool>();
    var unmatched:Array<{placed:Placed, subject:String}> = [];
    var used = new Map<String, String>();
    var derived = new Map<String, Array<{x:Float, y:Float, joint:String}>>();
    for (a in clamped) {
      var subject = a.member + "#" + a.face.id;
      var candidates = [for (b in receiving) if (b.member != a.member && meets(a, b, tolerance)) b];
      if (candidates.length == 0) {
        unmatched.push({placed: a, subject: subject});
        continue;
      }
      if (a.face.alternative != null) matchedAlternatives.set(a.member + "#" + a.face.alternative, true);
      if (a.face.holes == null) {
        // Drilled to suit: it takes the pattern of each receiving face under it (a seat for several rail blocks).
        for (b in candidates) {
          if (b.face.holes == null) {
            result.diagnostics.error(
              "fasteners.suit",
              subject,
              '$subject is drilled to suit and so is ${b.member}#${b.face.id}: neither knows the hole pattern'
            );
            continue;
          }
          var seat = AssemblyFrames.compose(AssemblyFrames.inverse(a.world), b.world);
          var mapped:Array<BoltHole> = [for (hole in b.face.holes) {
            var at = AssemblyFrames.transformPoint(seat, hole.x, hole.y, 0);
            {x: at.x, y: at.y};
          }];
          var paired = {member: a.member, face: a.face.paired(mapped), world: a.world};
          var joint = joinFaces(result, paired, b, tolerance, propertyClass, access, group, members, used, derived);
          if (joint != null) result.joints.push(joint);
        }
        continue;
      }
      if (candidates.length > 1) {
        // Several receiving faces in the plane (one per thread size of a cast insert): the one whose tapped holes the pattern lands on.
        var covering = [for (b in candidates) if (covers(a, b, tolerance)) b];
        if (covering.length == 1) candidates = covering;
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
      var joint = joinFaces(result, a, b, tolerance, propertyClass, access, group, members, used, derived);
      if (joint != null) result.joints.push(joint);
    }
    // A face is unmatched unless it is one of a set of alternatives and another of them is bolted on.
    for (entry in unmatched) {
      var alternative = entry.placed.face.alternative;
      if (alternative != null && matchedAlternatives.exists(entry.placed.member + "#" + alternative)) continue;
      result.diagnostics.error(
        "fasteners.unmatched",
        entry.subject,
        '${entry.subject} has clearance holes but no receiving face where it lies: its fasteners have nothing to thread into'
      );
    }
    for (key in derived.keys()) spacing(result, key, derived.get(key), result.joints);
    derivePins(result, assembly, poses, group, members, tolerance);
    return result;
  }

  /**
	 * The dowelled joints: pinned faces that meet (opposed normals on one plane) pair when their holes coincide. A face with some
	 * holes in common with another but not all, or of another diameter, is an error; a face with no partner is one too.
	 */
  static function derivePins(result:FastenerDerivation, assembly:MachineAssembly, poses:Map<String, AssemblyFrame>, group:Map<String, Int>,
      members:Map<String, MachineComponent>, tolerance:Float):Void {
    var diagnostics = result.diagnostics;
    var pins:Array<{member:String, face:PinnedFace, world:AssemblyFrame}> = [];
    for (entry in assembly.components()) {
      var pose = poses.get(entry.id);
      if (pose == null) continue;
      var seen = new Map<String, Bool>();
      for (face in entry.component.pinnedFaces()) {
        if (seen.exists(face.id)) {
          diagnostics.error("fasteners.face", entry.id, 'Member ${entry.id} declares pinned face "${face.id}" twice');
          continue;
        }
        seen.set(face.id, true);
        pins.push({member: entry.id, face: face, world: AssemblyFrames.compose(pose, face.frame)});
      }
    }
    var paired = [for (_ in pins) false];
    for (i in 0...pins.length) for (j in i + 1...pins.length) {
      var a = pins[i], b = pins[j];
      if (a.member == b.member) continue;
      var local = AssemblyFrames.compose(AssemblyFrames.inverse(b.world), a.world);
      var normal = AssemblyFrames.transformVector(local, 0, 0, 1);
      if (normal.z > -0.9999 || Math.abs(local.z) > tolerance) continue;
      var at = [for (hole in a.face.holes) AssemblyFrames.transformPoint(local, hole.x, hole.y, 0)];
      function onB(x:Float, y:Float):Bool {
        for (hole in b.face.holes) if (Math.abs(hole.x - x) <= tolerance && Math.abs(hole.y - y) <= tolerance) return true;
        return false;
      }
      var shared = 0;
      for (p in at) if (onB(p.x, p.y)) shared++;
      if (shared == 0) continue;
      var id = a.member + "#" + a.face.id + " ~ " + b.member + "#" + b.face.id;
      if (paired[i] || paired[j]) {
        diagnostics.error("fasteners.pin-duplicate", id, '$id: a pinned face is already paired with another');
        continue;
      }
      if (shared != at.length || at.length != b.face.holes.length) {
        diagnostics.error("fasteners.pin-pattern", id, '$id: the dowel holes differ (${at.length} and ${b.face.holes.length} holes, $shared in common)');
        continue;
      }
      if (Math.abs(a.face.diameter - b.face.diameter) > 1e-9) {
        diagnostics.error("fasteners.pin-diameter", id, '$id: the dowel holes are ${round(a.face.diameter)} and ${round(b.face.diameter)} mm');
        continue;
      }
      paired[i] = true;
      paired[j] = true;
      if (group.get(a.member) != group.get(b.member)) diagnostics.error(
        "fasteners.not-fixed",
        id,
        '$id: the members are not held together by fixed mates, so a dowelled joint between them contradicts the model'
      );
      var spec:Null<machinekit.standard.DowelPinSpec> = null;
      for (designation in DowelPin.catalog().designations()) {
        var candidate = DowelPin.catalog().get(designation);
        if (Math.abs(candidate.diameter - a.face.diameter) < 1e-9) spec = candidate;
      }
      if (spec == null) {
        diagnostics.error("fasteners.size", id, '$id: no ISO 8734 dowel pin of ${round(a.face.diameter)} mm');
        continue;
      }
      var minEngagement = PinnedFace.ENGAGEMENT_DIAMETERS * spec.diameter;
      var usableA = a.face.usable(), usableB = b.face.usable();
      if (usableA + 1e-9 < minEngagement || usableB + 1e-9 < minEngagement) {
        diagnostics.error(
          "fasteners.dowel-depth",
          id,
          '$id: a ${round(spec.diameter)} mm dowel needs ${round(minEngagement)} mm in each part but the holes allow ${round(usableA)} and ${round(usableB)} mm'
        );
        continue;
      }
      var length = DowelPin.standardLengthAtLeast(2 * minEngagement);
      if (length == null || length > usableA + usableB + 1e-9) {
        diagnostics.error(
          "fasteners.dowel-depth",
          id,
          '$id: no standard dowel length fits holes of ${round(usableA)} and ${round(usableB)} mm (at least ${round(2 * minEngagement)} mm needed)'
        );
        continue;
      }
      var engagementA = Math.min(usableA, Math.max(length / 2, length - usableB)), engagementB = length - engagementA;
      var pin = DowelPin.metric(spec.size, length);
      var cuts = [{p: a, member: a.member}, {p: b, member: b.member}];
      for (side in cuts) if (!side.p.face.drawn) {
        if (!members.get(side.member).machinable()) {
          diagnostics.error("fasteners.machining", id, '$id: ${side.member} has dowel holes its geometry does not draw and does not cut derived holes');
          continue;
        }
        var holes = side.p == a ? a.face.holes : b.face.holes;
        for (hole in holes) cutsOf(result, side.member).push({
          frame: side.p.face.frame,
          x: hole.x,
          y: hole.y,
          diameter: spec.diameter,
          from: -1.0,
          to: side.p.face.through ? side.p.face.depth + 1 : side.p.face.depth,
          kind: "dowel",
          thread: null,
          fit: side.p.face.fit,
          through: side.p.face.through
        });
      }
      var margin = 0.01;
      result.dowels.push({
        id: id,
        memberA: a.member,
        faceA: a.face.id,
        memberB: b.member,
        faceB: b.face.id,
        standard: "ISO 8734",
        size: spec.size,
        diameter: spec.diameter,
        length: length,
        fit: b.face.fit,
        count: at.length,
        depthA: a.face.depth,
        depthB: b.face.depth,
        engagementA: engagementA,
        engagementB: engagementB,
        hostHoles: [for (hole in b.face.holes) {x: hole.x, y: hole.y}],
        item: pin.bom,
        volumes: [{
          name: "dowel pin",
          code: "fasteners.hole",
          diameter: HOLE_SHARE * spec.diameter,
          from: -engagementA + margin,
          to: engagementB - margin,
          access: false
        }]
      });
    }
    for (i in 0...pins.length) if (!paired[i]) diagnostics.error(
      "fasteners.pin-unmatched",
      pins[i].member + "#" + pins[i].face.id,
      '${pins[i].member}#${pins[i].face.id} has dowel holes but no pinned face meets it with the same holes'
    );
  }

  /** Whether `a` lies on `b`'s plane with opposed normals and at least one of its holes over `b`'s footprint. */
  static function meets(a:Placed, b:Placed, tolerance:Float):Bool {
    var local = AssemblyFrames.compose(AssemblyFrames.inverse(b.world), a.world);
    var normal = AssemblyFrames.transformVector(local, 0, 0, 1);
    if (normal.z > -0.9999 || Math.abs(local.z) > tolerance) return false;
    if (a.face.holes == null) {
      // A footprint takes the receiving face whose holes are centred inside it: a face that is merely beside it is another.
      var tappedHoles = b.face.holes;
      if (tappedHoles == null) return true;
      var cx = 0.0, cy = 0.0;
      for (hole in tappedHoles) {
        cx += hole.x / tappedHoles.length;
        cy += hole.y / tappedHoles.length;
      }
      var at = AssemblyFrames.transformPoint(AssemblyFrames.inverse(local), cx, cy, 0);
      return Math.abs(at.x) <= a.face.width / 2 + tolerance && Math.abs(at.y) <= a.face.height / 2 + tolerance;
    }
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

  /** Whether every hole of `a` lands on a tapped hole of `b` (a receiving face that lists its holes). */
  static function covers(a:Placed, b:Placed, tolerance:Float):Bool {
    var tapped = b.face.holes, holes = a.face.holes;
    if (tapped == null || holes == null) return false;
    var local = AssemblyFrames.compose(AssemblyFrames.inverse(b.world), a.world);
    for (hole in holes) {
      var at = AssemblyFrames.transformPoint(local, hole.x, hole.y, 0), found = false;
      for (t in tapped) if (Math.abs(t.x - at.x) <= tolerance && Math.abs(t.y - at.y) <= tolerance) found = true;
      if (!found) return false;
    }
    return true;
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
    accessLength:Float,
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
    var nutted = false, plateClearance = 0.0;
    switch b.face.role {
      case Through(size, c, t):
        receivingSize = size;
        thickness = t;
        nutted = true;
        plateClearance = c;
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
    // A clamped face drilled to suit: its holes keep an edge distance from its footprint.
    if (a.face.width > 0) {
      var edge = EDGE_DIAMETERS * spec.diameter;
      var index = 0;
      for (hole in a.face.holes) {
        index++;
        if (Math.abs(hole.x) > a.face.width / 2 - edge + 1e-9 || Math.abs(hole.y) > a.face.height / 2 - edge + 1e-9) diagnostics.error(
          "fasteners.edge",
          id,
          '$id: hole $index at (${round(hole.x)}, ${round(hole.y)}) is within ${EDGE_DIAMETERS} diameters of the edge of the clamped '
          + 'footprint ${round(a.face.width)} x ${round(a.face.height)} or outside it'
        );
      }
    }
    // The fastener.
    if (clearance + 1e-9 < spec.clearanceFine) diagnostics.error(
      "fasteners.clearance",
      id,
      '$id: ${round(clearance)} mm clearance holes do not pass an $size screw (${round(spec.clearanceFine)} mm needed)'
    );
    if (nutted && plateClearance + 1e-9 < spec.clearanceFine) diagnostics.error(
      "fasteners.clearance",
      id,
      '$id: ${round(plateClearance)} mm holes in the plate do not pass an $size screw (${round(spec.clearanceFine)} mm needed)'
    );
    // Washers: under the head where the clamped face asks for one, and always under the nut of a bolt.
    var headWasher = a.face.washer ? washerOf(diagnostics, id, size) : null;
    var nutWasher = nutted ? washerOf(diagnostics, id, size) : null;
    var nut = nutted ? nutOf(diagnostics, id, size) : null;
    var lift = headWasher == null ? 0.0 : headWasher.thickness;
    var headWasherOuter = headWasher == null ? 0.0 : headWasher.outerDiameter;
    var headWidth = Math.max(spec.headDiameter, headWasherOuter);
    if (recess != null && (headWidth > recess.diameter + 1e-9 || spec.headHeight + lift > recess.depth + 1e-9)) diagnostics.error(
      "fasteners.head-clearance",
      id,
      '$id: an $size head${headWasher == null ? "" : " on its washer"} (${round(headWidth)} dia x ${round(spec.headHeight + lift)}) does not fit its '
      + '${round(recess.diameter)} dia x ${round(recess.depth)} recess'
    );
    if (hasErrorSince(diagnostics, before)) return null;
    var washerT = nutWasher == null ? 0.0 : nutWasher.thickness, washerOuter = nutWasher == null ? 0.0 : nutWasher.outerDiameter;
    var nutHeight = nut == null ? 0.0 : nut.height, nutAcross = nut == null ? 0.0 : nut.acrossFlats;
    var minEngagement:Float, length:Null<Float>, engagement:Float, stack = grip + lift, tipGap = 0.0, reach = 0.0;
    if (nutted) {
      stack += thickness + washerT;
      minEngagement = nutHeight + NUT_PROTRUSION_PITCHES * spec.pitch;
      length = SocketHeadCapScrew.standardLengthAtLeast(stack + minEngagement);
      if (length == null) {
        diagnostics.error("fasteners.length", id, '$id: no standard length reaches stack ${round(stack)} + nut ${round(minEngagement)}');
        return null;
      }
      engagement = length - stack;
    } else {
      minEngagement = BoltedFace.engagementFactor(b.face.rated ? ThreadMaterial.Steel : material) * spec.diameter;
      if (b.face.rated && BoltedFace.engagementFactor(material) > 1.0) diagnostics.warning(
        "fasteners.rated-thread",
        id,
        '$id: the maker rates a ${round(depth)} mm $size thread in ${material}; the material rule asks for '
        + '${round(BoltedFace.engagementFactor(material) * spec.diameter)} mm of engagement, the screw is chosen for ${round(minEngagement)}'
      );
      tipGap = through ? 0.0 : spec.pitch;
      reach = depth - tipGap;
      if (minEngagement > reach + 1e-9) diagnostics.error(
        "fasteners.engagement",
        id,
        '$id: ${material} needs ${round(minEngagement)} mm of $size thread engagement but the receiving thread allows '
        + '${round(reach)} mm (depth ${round(depth)}${through ? "" : ", less a tip gap of " + round(tipGap)})'
      );
      length = SocketHeadCapScrew.standardLengthAtLeast(stack + minEngagement);
      if (length == null) {
        diagnostics.error("fasteners.length", id, '$id: no standard length reaches grip ${round(stack)} + engagement ${round(minEngagement)}');
        return null;
      }
      engagement = length - stack;
      if (engagement > reach + 1e-9 && minEngagement <= reach + 1e-9) diagnostics.error(
        "fasteners.length",
        id,
        '$id: the shortest standard length ${round(length)} gives ${round(engagement)} mm of engagement; the thread takes '
        + '${round(reach)} mm. No standard length fits grip ${round(stack)}'
      );
    }
    if (length - spec.threadLength > stack + 1e-9) diagnostics.error(
      "fasteners.shank",
      id,
      '$id: the unthreaded shank of an $size x ${round(length)} screw (${round(length - spec.threadLength)} mm) is longer than the grip '
      + '${round(stack)} mm, so it enters the ${nutted ? "nut" : "thread"}'
    );
    var threadDepth = nutted ? 0.0 : Math.min(depth, Math.ceil((engagement + tipGap) * 2) / 2);
    var drillDepth = nutted ? 0.0 : through ? thickness : threadDepth + 2 * spec.pitch + 0.3 * spec.diameter;
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
        to: through ? thickness + 1 : drillDepth,
        kind: "tapped",
        thread: size
      });
    }
    if (nutted && !b.face.drawn) {
      if (!members.get(b.member).machinable()) diagnostics.error(
        "fasteners.machining",
        id,
        '$id: ${b.member} has clearance holes its geometry does not draw and does not cut derived holes'
      );
      else for (point in points) cutsOf(result, b.member).push({
        frame: b.face.frame,
        x: point.x,
        y: point.y,
        diameter: plateClearance,
        from: -1.0,
        to: thickness + 1,
        kind: "clearance",
        thread: size
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
        cutsOf(result, a.member).push({frame: a.face.frame, x: hole.x, y: hole.y, diameter: clearance, from: -1.0, to: back + 1, kind: "clearance", thread: size});
        if (recess != null) cutsOf(result, a.member).push({
          frame: a.face.frame,
          x: hole.x,
          y: hole.y,
          diameter: recess.diameter,
          from: grip,
          to: back + 1,
          kind: "counterbore",
          thread: size
        });
      }
    }
    if (hasErrorSince(diagnostics, before)) return null;
    // The fastener's envelope, per hole, in the receiving frame; the clamped part lies at negative z. Each end is drawn a
    // hair short of the faces it touches, so touching is not overlap.
    var volumes:Array<FastenerVolume> = [];
    var margin = 0.01;
    // The shank's hole through the clamped part, and the thread's (or the plate's) hole in the receiving part.
    volumes.push({name: "clamped hole", code: "fasteners.hole", diameter: HOLE_SHARE * spec.diameter, from: -grip + margin, to: -margin, access: false});
    volumes.push({
      name: nutted ? "plate hole" : "threaded hole",
      code: "fasteners.hole",
      diameter: HOLE_SHARE * (nutted ? spec.diameter : spec.tapDrill),
      from: margin,
      to: (nutted ? thickness : threadDepth) - margin,
      access: false
    });
    var seat = -grip;
    if (headWasher != null) {
      volumes.push({name: "head washer", code: "fasteners.head-clearance", diameter: headWasherOuter, from: seat - lift + margin, to: seat - margin, access: false});
      seat -= lift;
    }
    volumes.push({name: "head", code: "fasteners.head-clearance", diameter: spec.headDiameter, from: seat - spec.headHeight + margin, to: seat - margin, access: false});
    // A recess is the driver's way in: it must be clear to its mouth, and then the driver's own length beyond it.
    var pocket = recess == null ? 0.0 : Math.max(0.0, recess.depth - spec.headHeight - lift);
    volumes.push({
      name: "driver access",
      code: "fasteners.access",
      diameter: spec.headDiameter,
      from: seat - spec.headHeight - accessLength - pocket,
      to: seat - spec.headHeight - margin,
      access: true
    });
    var extras:Array<{item:BomItem, count:Int}> = [];
    if (headWasher != null) extras.push({item: FlatWasher.metric(size).bom, count: points.length});
    if (nutted) {
      var far = thickness + margin;
      volumes.push({name: "nut washer", code: "fasteners.head-clearance", diameter: washerOuter, from: far, to: thickness + washerT - margin, access: false});
      var corners = nutAcross / Math.cos(Math.PI / 6);
      volumes.push({name: "nut", code: "fasteners.head-clearance", diameter: corners, from: thickness + washerT + margin, to: thickness + washerT + nutHeight - margin, access: false});
      volumes.push({
        name: "wrench access",
        code: "fasteners.access",
        diameter: corners,
        from: thickness + washerT + nutHeight + margin,
        to: thickness + washerT + nutHeight + accessLength,
        access: true
      });
      extras.push({item: FlatWasher.metric(size).bom, count: points.length});
      extras.push({item: HexNut.metric(size).bom, count: points.length});
    }
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
      item: screw.bom,
      stack: stack,
      nutted: nutted,
      extras: extras,
      volumes: volumes
    };
  }

  /** The share of a hole's diameter the clearance volumes take, so a modelled hole's polygons and rounding do not catch. */
  public static inline var HOLE_SHARE:Float = 0.9;

  static function washerOf(diagnostics:Diagnostics, id:String, size:String):Null<machinekit.standard.FlatWasherSpec> {
    var catalog = FlatWasher.catalog();
    if (!catalog.exists(size)) {
      diagnostics.error("fasteners.washer", id, '$id: no ISO 7089 washer of size $size');
      return null;
    }
    return catalog.get(size);
  }

  static function nutOf(diagnostics:Diagnostics, id:String, size:String):Null<machinekit.standard.HexNutSpec> {
    var catalog = HexNut.catalog();
    if (!catalog.exists(size)) {
      diagnostics.error("fasteners.nut", id, '$id: no ISO 4032 nut of size $size');
      return null;
    }
    return catalog.get(size);
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
      else if (holes[i].joint != holes[j].joint && distance < SPACING_DIAMETERS * diameter) result.diagnostics.error(
        "fasteners.spacing",
        host,
        'Derived holes of $host are only ${round(distance)} mm apart (${holes[i].joint}; ${holes[j].joint})'
      );
    }
  }

  /** The rigid body each member belongs to, joined by its fixed tree mates. */
  public static function rigidGroups(assembly:MachineAssembly):Map<String, Int> {
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
