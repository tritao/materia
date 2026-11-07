package machinekit.assembly;

import cadkit.modeling.Part;
import machinekit.component.Solids;

import machinekit.component.BoltedFace;
import machinekit.component.BoltedFace.BoltHole;
import machinekit.component.BoltedFace.BoltRecess;
import machinekit.component.BoltedFace.MachinedCut;
import machinekit.component.BoltedFace.ThreadMaterial;
import machinekit.component.BoltedFace.PinnedFace;
import machinekit.standard.DowelPin;
import machinekit.assembly.Diagnostics.DiagnosticSeverity;
import machinekit.assembly.JointSlip;
import machinekit.assembly.JointSlip.JointLoad;
import machinekit.assembly.JointSlip.JointSlipResult;
import machinekit.assembly.JointSlip.SlipPin;
import machinekit.assembly.JointSlip.SlipScrew;
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
  /** Derive only these clamped members, with all assembly hosts visible. Absent means every joint. */
  @:optional var clampedMembers:Array<String>;
  /** The fastener class; 12.9, as the catalogue screws are drawn, when absent. */
  @:optional var propertyClass:PropertyClass;
  /** How far (mm) a hole may stand from its partner, and a face from the other's plane. */
  @:optional var tolerance:Float;
  /** Length (mm) of the driver access cylinder along the axis beyond a head or nut; `JointFasteners.ACCESS_LENGTH` when absent. */
  @:optional var accessLength:Float;
  /** The loads the machine declares its joints carry; each is checked for slip (`JointSlip`, findings `fasteners.slip`). */
  @:optional var loads:Array<JointLoad>;
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
  /** The same holes in world coordinates, mm. */
  var worldHoles:Array<{x:Float, y:Float, z:Float}>;
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
  /** The holes in the frame of member B's face, and in world coordinates. */
  var hostHoles:Array<BoltHole>;
  var worldHoles:Array<{x:Float, y:Float, z:Float}>;
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
  /** The slip check of every declared load (`FastenerOptions.loads`), in the order declared. */
  public final slips:Array<JointSlipResult> = [];
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
  /** Lead's 2026-11-07 made-steel thread rule: centre >=d and wall beyond major >=d/2. */
  public static inline var TAPPED_EDGE_DIAMETERS:Float = 1.0;

  /** Supplier-designed holes are outside the made-part edge rule, by procurement kind. */
  public static function requiresEdgeCheck(part:MachineComponent):Bool return switch part.bom.procurement {
    case Buy(_, _, _): false;
    case _: true;
  };

  public static function receivingEdgeRadius(face:BoltedFace, diameter:Float):Float return switch face.role {
    case Receiving(_, _, _, _, Steel): TAPPED_EDGE_DIAMETERS * diameter;
    case _: EDGE_DIAMETERS * diameter;
  };
  /** Centre-to-centre distance of holes derived in one footprint, in nominal diameters. */
  public static inline var SPACING_DIAMETERS:Float = 2.5;
  /** Back wall left behind a blind drilling, mm. */
  public static inline var BACK_WALL:Float = 1.0;
  /** The driver's way in along the axis beyond a head or nut: a hex key's short arm, mm. */
  public static inline var ACCESS_LENGTH:Float = 25;
  /** Nuts stand this many pitches beyond the plate's washer: the screw shows at least this far past the nut. */
  public static inline var NUT_PROTRUSION_PITCHES:Float = 2;

  /** Actual-solid edge support for every hole of a derived joint. The caller
   * supplies both finished, posed members after joint machining. No supplier
   * hole-pattern exceptions are inferred here. Bought members are excluded by
   * procurement kind; their made mating members are still checked. */
  public static function checkEdgeSupport(joint:FastenedJoint, receivingFace:BoltedFace,
    receivingFrame:AssemblyFrame, clampedPart:MachineComponent, receivingPart:MachineComponent,
    parts:Map<String, Part>, diagnostics:Diagnostics):Void {
    // Edge distance is a property of the actual member, not the declared rectangular
    // fastening footprint. Inspect each hole on both sides of every derived joint.
    var screwSpec = machinekit.standard.SocketHeadCapScrew.catalog().get(joint.size);
    var clampedClearance = screwSpec.clearanceMedium;
    var receivingClearance = joint.nutted ? screwSpec.clearanceMedium : joint.tapDrill;
    var recessDepth = 0.0;
    var counterboreDiameter = 0.0;
    var bearingDiameter = screwSpec.headDiameter;
    var clampedSlot:Null<machinekit.component.BoltedFace.BoltSlot> = null;
    for (face in clampedPart.fasteningFaces()) if (face.id == joint.clampedFace) clampedSlot = face.slot;
    for (clampedFace in clampedPart.fasteningFaces()) if (clampedFace.id == joint.clampedFace)
      switch clampedFace.role {
        case Clamped(_, clearance, _, recess):
          clampedClearance = clearance;
          if (recess != null) {
            bearingDiameter = Math.max(bearingDiameter, recess.diameter);
            recessDepth = recess.depth;
            counterboreDiameter = recess.diameter;
          }
        case _:
      }
    if (joint.nutted) switch receivingFace.role {
      case Through(_, clearance, _): receivingClearance = clearance;
      case _:
    }
    for (v in joint.volumes) if (v.name == "head" || v.name == "head washer")
      bearingDiameter = Math.max(bearingDiameter, v.diameter);
    var receiverRadius = receivingEdgeRadius(receivingFace, screwSpec.diameter);
    var receiverFrom = 0.01;
    var receiverTo = 0.2;
    var receiverBearingRadius = 0.0;
    // A through plate is loaded under its far-side nut/washer, as a clamped
    // counterbore is loaded at its floor. A front pilot recess is not that seat.
    switch receivingFace.role {
      case Through(_, _, thickness):
        receiverFrom = thickness - Math.min(0.2, thickness / 2);
        receiverTo = thickness - 0.01;
        for (v in joint.volumes) if (v.name == "nut" || v.name == "nut washer") {
          receiverBearingRadius = Math.max(receiverBearingRadius, v.diameter / 2 + HEAD_EDGE_WALL);
          receiverRadius = Math.max(receiverRadius, receiverBearingRadius);
        }
      case _:
    }
    var supportSections = [
      {section: "head bearing", member: joint.clampedMember, inner: clampedClearance / 2 + 0.01,
        outer: requiredEdgeRadius(screwSpec.diameter, bearingDiameter),
        from: -joint.grip + 0.01, to: -joint.grip + Math.min(0.2, joint.grip / 2)},
      {section: "receiving face", member: joint.receivingMember, inner: receivingClearance / 2 + 0.01,
        outer: receiverRadius, from: receiverFrom, to: receiverTo}
    ];
    if (recessDepth > 0) supportSections.push({section: "counterbore mouth", member: joint.clampedMember,
      inner: counterboreDiameter / 2 + 0.01,
      outer: requiredEdgeRadius(screwSpec.diameter, counterboreDiameter),
      from: -joint.grip - recessDepth + 0.01,
      to: -joint.grip - recessDepth + Math.min(0.2, recessDepth / 2)});
    for (side in supportSections) {
      if (!requiresEdgeCheck(side.member == joint.clampedMember ? clampedPart : receivingPart)) continue;
      var body = parts.get(side.member);
      if (body == null) throw 'Missing fastener edge-check member ${side.member}';
      for (point in joint.hostHoles) {
        var at = AssemblyFrames.compose(receivingFrame, AssemblyFrames.translation(point.x, point.y, 0));
        var slot = side.member == joint.receivingMember ? receivingFace.slot : clampedSlot;
        var support = edgeSupportFraction(body, at, side.inner, side.outer, side.from, side.to, slot);
        if (support < 0.99999) diagnostics.warning("fasteners.edge-support", joint.id,
          '${joint.id}: ${side.member} lacks actual face support around ${joint.size} hole '
          + '(${edgeRound(point.x)}, ${edgeRound(point.y)}), ${side.section}: ${edgeRound(100 * support)}% of required wall; '
          + 'required supported radius ${side.outer} mm; made-part clearance edge >=1.5d, steel thread edge >=d '
          + '(wall beyond thread major >=0.5d), head/counterbore wall >=${HEAD_EDGE_WALL} mm');
      }
    }
  }

  /** Minimum material beyond a head/counterbore; stated design requirement, mm. */
  public static inline final HEAD_EDGE_WALL = 1.0;

  public static function requiredEdgeRadius(screwDiameter:Float, bearingDiameter:Float):Float {
    if (!(screwDiameter > 0) || !(bearingDiameter > 0)
      || !Math.isFinite(screwDiameter + bearingDiameter)) throw "Invalid fastener edge dimensions";
    return Math.max(EDGE_DIAMETERS * screwDiameter, bearingDiameter / 2 + HEAD_EDGE_WALL);
  }

  /** Fraction of the required annular wall present in the actual solid.
   * A section just inside the face avoids counting a neighbouring part as support.
   * The input body is borrowed; every temporary native shape is released. */
  public static function edgeSupportFraction(body:Part, frame:AssemblyFrame, innerRadius:Float,
    outerRadius:Float, from:Float, to:Float, ?slot:machinekit.component.BoltedFace.BoltSlot):Float {
    if (!(innerRadius > 0) || !(outerRadius > innerRadius) || !(to > from)
      || !Math.isFinite(innerRadius + outerRadius + from + to)) throw "Invalid edge-support annulus";
    if (slot != null && slot.width / 2 + 0.01 >= outerRadius) throw "Slot leaves no edge-support section";
    var ring = slot == null ? Solids.cut(Part.cylinderSpan(outerRadius, from, to),
      [Part.cylinderSpan(innerRadius, from - 1, to + 1)])
      : Solids.cut(edgeCapsule(outerRadius, slot, from, to),
        [edgeCapsule(Math.max(innerRadius, slot.width / 2 + 0.01), slot, from - 1, to + 1)]);
    var probe:Null<Part> = null;
    var overlap:Null<Part> = null;
    try {
      probe = PosedParts.moved(ring, frame);
      overlap = body.intersect(probe);
      var expected = ring.volume();
      var result = Math.max(0.0, Math.min(1.0, overlap.volume() / expected));
      overlap.close();
      probe.close();
      ring.close();
      return result;
    } catch (e:Dynamic) {
      if (overlap != null) overlap.close();
      if (probe != null) probe.close();
      ring.close();
      throw e;
    }
  }

  static function edgeCapsule(radius:Float, slot:machinekit.component.BoltedFace.BoltSlot, from:Float, to:Float):Part {
    var h = slot.halfTravel;
    var alongX = slot.axis == "x";
    var bridge = Part.box(alongX ? 2*h : 2*radius, alongX ? 2*radius : 2*h, to-from);
    var shifted = bridge.translated(new cadkit.modeling.Vector(0, 0, from));
    bridge.close();
    return Solids.union([shifted,
      Part.cylinderSpan(radius, from, to, alongX ? -h : 0, alongX ? 0 : -h),
      Part.cylinderSpan(radius, from, to, alongX ? h : 0, alongX ? 0 : h)]);
  }

  static function edgeRound(value:Float):Float return Math.round(value * 1000) / 1000;

  public static function derive(assembly:MachineAssembly, ?options:FastenerOptions):FastenerDerivation {
    var result = new FastenerDerivation();
    var tolerance = options == null || options.tolerance == null ? TOLERANCE : options.tolerance;
    var propertyClass = options == null || options.propertyClass == null ? PropertyClass.Steel129 : options.propertyClass;
    var access = options == null || options.accessLength == null ? ACCESS_LENGTH : options.accessLength;
    var selection = options == null ? null : options.clampedMembers;
    if (selection != null) {
      var names = [for (entry in assembly.components()) entry.id];
      for (id in selection) if (names.indexOf(id) < 0)
        result.diagnostics.error("fasteners.selection", id, 'Unknown clamped member "$id"');
    }
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
        if (!face.isClamped() || selection == null || selection.indexOf(entry.id) >= 0)
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
    derivePins(result, assembly, poses, group, members, tolerance, selection);
    if (options != null && options.loads != null) slip(result, options.loads, members);
    return result;
  }

  /** The slip check of the declared loads (`JointSlip`): each meets the joints of its member pair; a load that meets none is an error. */
  public static function slip(result:FastenerDerivation, loads:Array<JointLoad>, ?members:Map<String, MachineComponent>):Void {
    for (load in loads) {
      if (load.between == null || load.between.length != 2) {
        result.diagnostics.error("fasteners.slip", load.label, '${load.label}: a joint load names the two members it joins');
        continue;
      }
      function pair(a:String, b:String):Bool return (a == load.between[0] && b == load.between[1]) || (a == load.between[1] && b == load.between[0]);
      function facePasses(a:String, b:String):Bool return load.face == null || a == load.face || b == load.face;
      var screws:Array<SlipScrew> = [], pins:Array<SlipPin> = [], keys:Array<JointSlip.SlipKey> = [];
      for (joint in result.joints) if (pair(joint.clampedMember, joint.receivingMember) && facePasses(joint.clampedFace, joint.receivingFace)) {
        screws.push({id: joint.id, size: joint.size, propertyClass: joint.propertyClass, points: joint.worldHoles});
        // The shoulders and keys a member declares for the face of this joint locate it (counted once per joint).
        if (members != null) for (side in [{m: joint.clampedMember, f: joint.clampedFace}, {m: joint.receivingMember, f: joint.receivingFace}]) {
          var component = members.get(side.m);
          if (component != null) for (k in component.locatingKeys()) if (k.face == side.f && [for (e in keys) if (e.id == side.m + "#" + k.face) e].length == 0)
            keys.push({id: side.m + "#" + k.face, height: k.height, length: k.length, thickness: k.thickness});
        }
      }
      for (joint in result.dowels) if (pair(joint.memberA, joint.memberB) && facePasses(joint.faceA, joint.faceB))
        pins.push({id: joint.id, diameter: joint.diameter, engagement: Math.min(joint.engagementA, joint.engagementB), points: joint.worldHoles});
      var checked = JointSlip.check(load, screws, pins, keys);
      if (checked.length == 0) result.diagnostics.error(
        "fasteners.slip",
        load.label,
        '${load.label}: no bolted or dowelled joint between ${load.between[0]} and ${load.between[1]}${load.face == null ? "" : " at face " + load.face}, or no load declared'
      );
      for (slip in checked) {
        result.slips.push(slip);
        if (!slip.passes) result.diagnostics.error(
          "fasteners.slip",
          load.label,
          JointSlip.describe(slip) + ' (needs x${JointSlip.PEAK_FACTOR} on the peak and x${JointSlip.MOMENTARY_FACTOR} on a momentary load)'
        );
      }
    }
  }

  /**
	 * The dowelled joints: pinned faces that meet (opposed normals on one plane) pair when their holes coincide. A face with some
	 * holes in common with another but not all, or of another diameter, is an error; a face with no partner is one too.
	 */
  static function derivePins(result:FastenerDerivation, assembly:MachineAssembly, poses:Map<String, AssemblyFrame>, group:Map<String, Int>,
      members:Map<String, MachineComponent>, tolerance:Float, selection:Null<Array<String>>):Void {
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
      if (selection != null && selection.indexOf(a.member) < 0 && selection.indexOf(b.member) < 0) continue;
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
        var dowelHole = 0;
        for (hole in holes) cutsOf(result, side.member).push({
          name: cutName(id, ++dowelHole, "dowel"),
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
        worldHoles: [for (hole in b.face.holes) AssemblyFrames.transformPoint(b.world, hole.x, hole.y, 0)],
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
    for (i in 0...pins.length) if (!paired[i] && (selection == null || selection.indexOf(pins[i].member) >= 0)) diagnostics.error(
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
    var spec = a.face.lowHead ? SocketHeadCapScrew.lowHeadSpec(size) : catalog.get(size);
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
        var requiredEdge = receivingEdgeRadius(b.face, spec.diameter);
        if (!over(b.face, at.x, at.y, tolerance, requiredEdge)) diagnostics.error(
          "fasteners.edge",
          id,
          '$id: hole ${index + 1} at (${round(at.x)}, ${round(at.y)}) needs ${round(requiredEdge)} mm to the edge of the '
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
    var screw = a.face.lowHead ? SocketHeadCapScrew.lowHead(size, length, propertyClass) : SocketHeadCapScrew.metric(size, length, propertyClass);
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
      else for (k => point in points) cutsOf(result, b.member).push({
        name: cutName(id, k + 1, "tap"),
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
      else for (k => point in points) cutsOf(result, b.member).push({
        name: cutName(id, k + 1, "clearance"),
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
    var aHoles:Array<BoltHole> = a.face.holes == null ? [] : a.face.holes;
    if (!a.face.drawn) {
      if (!members.get(a.member).machinable()) diagnostics.error(
        "fasteners.machining",
        id,
        '$id: ${a.member} has clearance holes its geometry does not draw and does not cut derived holes'
      );
      else for (k => hole in aHoles) {
        var back = grip + (recess == null ? 0.0 : recess.depth);
        cutsOf(result, a.member).push({name: cutName(id, k + 1, "clearance"), frame: a.face.frame, x: hole.x, y: hole.y, diameter: clearance, from: -1.0, to: back + 1, kind: "clearance", thread: size});
        if (recess != null) cutsOf(result, a.member).push({
          name: cutName(id, k + 1, "counterbore"),
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
      worldHoles: [for (p in points) AssemblyFrames.transformPoint(b.world, p.x, p.y, 0)],
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

  /**
   * The stable name of hole `hole` (from 1) of joint `joint`, cut as `role` (tap, clearance, counterbore, dowel): `host.top..cover.bottom.h2.tap`.
   * The joint id's member and face separators become dots and everything else outside a name's alphabet an underscore, so the
   * cylinder's faces are named strongly under it (`MachinedCut.name`).
   */
  public static function cutName(joint:String, hole:Int, role:String):String {
    var id = StringTools.replace(StringTools.replace(StringTools.replace(joint, " -> ", ".."), " ~ ", ".."), "#", ".");
    var out = new StringBuf();
    for (i in 0...id.length) {
      var c = id.charCodeAt(i);
      var ok = (c >= 48 && c <= 57) || (c >= 65 && c <= 90) || (c >= 97 && c <= 122) || c == 46 || c == 95 || c == 43 || c == 45;
      out.add(ok ? id.charAt(i) : "_");
    }
    return out.toString() + ".h" + hole + "." + role;
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
