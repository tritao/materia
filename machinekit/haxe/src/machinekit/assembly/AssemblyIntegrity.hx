package machinekit.assembly;

import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.PosedParts.PartBox;
import machinekit.component.MountingFace;
import machinekit.design.DesignRequirement;
import machinekit.motion.LeadScrew;
import machinekit.transmission.Rack;
import machinekit.transmission.SpurGear;
import materia.assembly.AssemblyBodies;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** One sampled joint state of a sweep: the bodies' motion from rest, the joint values, and the pairs already reported. */
typedef SweepSample = {moved:Map<String, AssemblyFrame>, sample:Map<String, Float>, reported:Map<String, Bool>};

/** The root leader of a drive, where it rests and the small step the drive check moves it by. */
typedef LeaderMove = {id:String, from:Float, step:Float};

typedef IntegrityOptions = {
  /** Two members closer than this (mm) touch. */
  @:optional var contactTolerance:Float;
  /** Shared volume (mm³) above which two members interfere. */
  @:optional var overlapLimit:Float;
  /** Shared volume (mm³) a gear or rack mesh may have: the tooth outlines are approximations of involutes. */
  @:optional var meshOverlapLimit:Float;
  /**
	 * Height of the floor the machine stands on. When given, every member must be held up, through mates whose members
	 * touch, by a member that stands on the floor. When absent, the members no joint carries are the ground.
	 */
  @:optional var floorZ:Float;
  /** Joint states to sweep, replacing the default sampling (see `AssemblyIntegrity.sweepStates`). */
  @:optional var sweepStates:Array<Map<String,
  Float>>;
  /**
	 * Least distance (mm) bodies that move relative to each other keep over the sampled sweep (`AssemblyIntegrity.RUNNING_GAP`).
	 * Off when absent: overlap is always reported, a face that merely rubs is not. Pairs a moving mate joins (a block on its
	 * rail) and pairs the model says mesh are exempt.
	 */
  @:optional var runningGap:Float;
  /**
	 * Least contact area (mm²) a fixed mate must have where its child carries other members (`AssemblyIntegrity.MIN_CONTACT_AREA`).
	 * Off when absent. Measured by pushing the child a little into its parent, so it counts faces normal to a world axis.
	 */
  @:optional var minContactArea:Float;
  /**
	 * Every part with a `mountingFace` that is fixed to a host finds its cut-out in the host: the host has no material in
	 * the pilot and bolt holes, and lies under the flange.
	 */
  @:optional var flangeCutouts:Bool;
  /**
	 * Every joint `MachineAssembly.deriveFasteners` derived has its fastener fit: the holes it passes through exist in the
	 * parts (`fasteners.hole`), its head, washer and nut stand clear of the other members (`fasteners.head-clearance`), and a
	 * driver reaches it along the axis (`fasteners.access`, among the members held rigidly with the clamped part: a moving
	 * member can be moved away). Absent when the assembly derived none.
	 */
  @:optional var fasteners:Bool;
  /**
	 * Members the operator takes away to reach a screw (a workpiece and its clamps): they do not block a driver's access
	 * (`fasteners.access`) though they are fixed to the clamped part. They still count for holes and heads.
	 */
  @:optional var removable:Array<String>;
  /** Pairs of members the machine's process brings together (a hob in the work, a tool in the stock); they may share volume. */
  @:optional var processContacts:Array<{a:String, b:String}>;
  /** All three of the above at their defaults. */
  @:optional var strict:Bool;
}

/** What one `AssemblyIntegrity.check` found, and how much it looked at. */
class IntegrityReport {
  public final diagnostics:Diagnostics;
  /** Joint states the sweep posed, the rest state included. */
  public final statesChecked:Int;

  public function new(diagnostics:Diagnostics, statesChecked:Int) {
    this.diagnostics = diagnostics;
    this.statesChecked = statesChecked;
  }

  public function errors():Array<Diagnostic> return [for (item in diagnostics.items) if (item.severity == Error) item];

  /** Throws every error found, one per line. */
  public function require():Void diagnostics.throwIfErrors();
}

/**
 * Checks that an assembly is physically put together, whatever machine it is:
 *
 * - **support**: the two members of every mate touch, and every member is held up by the ground through mates whose
 *   members touch, so a part "fixed" to something it does not reach is found;
 * - **rest interference**: no two members share volume at rest, members of one rigid body included, except where the
 *   model says they mesh: a lead screw in its nut, a belt on its pulley or in its clamp, a chain on its sprocket, and
 *   gear and rack teeth up to `meshOverlapLimit`. Nothing is excused by a hand-written list; a shaft in a bore must fit;
 * - **sweeps**: bodies that move relative to each other stay clear at sampled joint states (see `sweepStates`). A
 *   sampled sweep, not a swept volume: a collision between samples can be missed;
 * - **drive consistency**: each transmission's coupling agrees with its parts' geometry. The root leader is moved a
 *   little and the parts' material points are compared: a pinion's pitch point stays on its rack, meshing gears'
 *   pitch circles roll on each other, and a nut travels one signed lead per turn of its screw (the thread's hand
 *   included). A wrong ratio or sense is found here. Belts and chains are not checked this way: their strands are not
 *   part geometry.
 *
 * Three further checks are off unless asked for (`IntegrityOptions.strict` turns them all on): a running gap between
 * bodies that move relative to each other, a minimum contact area for fixed mates that carry other members, and the
 * cut-out a flanged part (`MachineComponent.mountingFace`) needs in the host it is fixed to.
 */
class AssemblyIntegrity {
  public static inline var CONTACT_TOLERANCE:Float = 0.05;
  public static inline var OVERLAP_LIMIT:Float = 1e-3;
  public static inline var MESH_OVERLAP_LIMIT:Float = 0.05;
  /** Default running gap between bodies that move relative to each other, mm. */
  public static inline var RUNNING_GAP:Float = 1.0;
  /** Default contact area for a fixed mate that carries other members, mm² (25 x 25). */
  public static inline var MIN_CONTACT_AREA:Float = 625.0;
  /** How far the contact area probe pushes the child into its parent, mm. */
  static inline var CONTACT_PROBE:Float = 0.5;
  /** Depth of the layer of the host a flange's cut-out is checked in, mm. */
  static inline var FLANGE_PROBE:Float = 1.0;
  /** Share of the footprint's material a flange's host must supply. */
  static inline var FLANGE_SEAT_SHARE:Float = 0.5;
  /** Leader step for the drive consistency check, in the leader's units (mm or rad). */
  static inline var DRIVE_STEP:Float = 0.01;
  /** A drive's material points may part by this share of their travel (the test is first order). */
  static inline var DRIVE_SLIP:Float = 0.01;

  public static function check(assembly:MachineAssembly, ?options:IntegrityOptions):IntegrityReport {
    var checker = new AssemblyIntegrity(assembly, options == null ? ({} : IntegrityOptions) : options);
    var states = 0;
    PosedParts.scope(posed -> {states = checker.run(posed);
    });
    return new IntegrityReport(checker.diagnostics, states);
  }

  /**
	 * Only the fit of the derived fasteners (`IntegrityOptions.fasteners`), at the joints' initial values: the cheap check
	 * for a machine whose other integrity checks are not run.
	 */
  public static function fasteners(assembly:MachineAssembly, ?options:IntegrityOptions):IntegrityReport {
    var checker = new AssemblyIntegrity(assembly, options == null ? ({} : IntegrityOptions) : options);
    PosedParts.scope(posed -> checker.runFasteners(posed));
    return new IntegrityReport(checker.diagnostics, 1);
  }

  /**
	 * Only the declared service removals and guarded channels (`ServiceChecks`), at the joints' initial values: the cheap check
	 * for a machine whose other integrity checks are not run. Every finding is `integrity.service.<id>` or `integrity.drain.<id>`.
	 */
  public static function service(assembly:MachineAssembly, ?options:IntegrityOptions):IntegrityReport {
    var checker = new AssemblyIntegrity(assembly, options == null ? ({} : IntegrityOptions) : options);
    PosedParts.scope(posed -> checker.runService(posed));
    return new IntegrityReport(checker.diagnostics, 1);
  }

  /**
	 * The default sweep: every corner of the leader joints' travel when there are at most six leaders, and each leader at
	 * its quarter points with the others at their initial values. A leader is a movable tree joint (prismatic or
	 * revolute, with finite limits) that no coupling drives.
	 */
  public static function sweepStates(definition:AssemblyDefinition):Array<Map<String, Float>> {
    var leaders = leaderJoints(definition);
    var states:Array<Map<String, Float>> = [];
    if (leaders.length <= 6) for (corner in 0...1 << leaders.length) {
      var state = new Map<String, Float>();
      for (index in 0...leaders.length) state.set(
        leaders[index].id,
        ((corner >> index) & 1) == 0 ? leaders[index].lower : leaders[index].upper
      );
      states.push(state);
    }
    for (joint in leaders) for (share in [0.25, 0.5, 0.75]) {
      var state = new Map<String, Float>();
      state.set(joint.id, joint.lower + share * (joint.upper - joint.lower));
      states.push(state);
    }
    return states;
  }

  /** The leader joints and their travel. */
  static function leaderJoints(definition:AssemblyDefinition):Array<{id:String, lower:Float, upper:Float}> {
    var driven = new Map<String, Bool>();
    if (definition.couplings != null) for (coupling in definition.couplings) driven.set(coupling.target, true);
    var result:Array<{id:String, lower:Float, upper:Float}> = [];
    for (joint in definition.joints) {
      if (joint.role != AssemblyJointRole.Tree || driven.exists(joint.id) || joint.passive == true) continue;
      if (joint.type != AssemblyJointType.Prismatic && joint.type != AssemblyJointType.Revolute) continue;
      var limits = joint.limits;
      if (limits == null) continue;
      var lower = limits.lower, upper = limits.upper;
      if (lower == null || upper == null || !Math.isFinite(lower) || !Math.isFinite(upper)) continue;
      result.push({id: joint.id, lower: lower, upper: upper});
    }
    return result;
  }

  final assembly:MachineAssembly;
  final flat:FlatAssembly;
  final definition:AssemblyDefinition;
  final contactTolerance:Float;
  final overlapLimit:Float;
  final meshOverlapLimit:Float;
  final floorZ:Null<Float>;
  final requestedStates:Null<Array<Map<String, Float>>>;
  final runningGap:Null<Float>;
  final minContactArea:Null<Float>;
  final flangeCutouts:Bool;
  final fastenerFit:Bool;
  final removable = new Map<String, Bool>();
  /** Pairs joined by a moving mate, keyed "a|b" both ways: they run in contact by design. */
  final movingPairs = new Map<String, Bool>();
  final gapReported = new Map<String, Bool>();
  final diagnostics = new Diagnostics();
  /** Pairs the model says mesh, keyed "a|b" both ways, with the volume each may share. */
  final interfaces = new Map<String, Float>();
  final ids:Array<String> = [];
  final bodyOf = new Map<String, String>();
  /** Bodies a revolute or continuous joint turns relative to their parent, to the parent's body. */
  final turning = new Map<String, Map<String, Bool>>();

  function new(assembly:MachineAssembly, options:IntegrityOptions) {
    this.assembly = assembly;
    flat = assembly.derived();
    definition = flat.definition;
    contactTolerance = options.contactTolerance == null ? CONTACT_TOLERANCE : options.contactTolerance;
    overlapLimit = options.overlapLimit == null ? OVERLAP_LIMIT : options.overlapLimit;
    meshOverlapLimit = options.meshOverlapLimit == null ? MESH_OVERLAP_LIMIT : options.meshOverlapLimit;
    floorZ = options.floorZ;
    requestedStates = options.sweepStates;
    var strict = options.strict == true;
    runningGap = options.runningGap ?? (strict ? RUNNING_GAP : null);
    minContactArea = options.minContactArea ?? (strict ? MIN_CONTACT_AREA : null);
    flangeCutouts = options.flangeCutouts ?? strict;
    fastenerFit = options.fasteners ?? strict;
    if (options.removable != null) for (id in options.removable) removable.set(id, true);
    for (joint in definition.joints) if (joint.type != AssemblyJointType.Fixed) {
      movingPairs.set(joint.parent + "|" + joint.child, true);
      movingPairs.set(joint.child + "|" + joint.parent, true);
    } else if (joint.role == AssemblyJointRole.Tree && (Std.isOfType(flat.member(joint.parent), machinekit.standard.ScrewFastener)
      || Std.isOfType(flat.member(joint.child), machinekit.standard.ScrewFastener))) {
      // A screw or bolt fixed to its part is threaded into it: the thread overlaps the hole's.
      allow(joint.parent, joint.child, Math.POSITIVE_INFINITY);
    }
    for (contact in options.processContacts ?? []) allow(contact.a, contact.b, Math.POSITIVE_INFINITY);
    for (member in flat.members) ids.push(member.id);
    for (body in AssemblyBodies.of(definition)) for (id in body.occurrences) bodyOf.set(id, body.id);
    for (joint in definition.joints) if (joint.role == AssemblyJointRole.Tree
      && (joint.type == AssemblyJointType.Continuous || joint.type == AssemblyJointType.Revolute)) {
      var body = bodyOf.get(joint.child), parent = bodyOf.get(joint.parent);
      if (body == null || parent == null) continue;
      if (!turning.exists(body)) turning.set(body, new Map());
      turning.get(body).set(parent, true);
    }
    for (record in flat.transmissions) switch record.source {
      case LeadScrew(screw, nut):
        allow(screw, nut, Math.POSITIVE_INFINITY);
      case GearMesh(driver, driven):
        allow(driver, driven, meshOverlapLimit);
      case RackAndPinion(pinion, rack):
        if (rack != null) allow(pinion, rack, meshOverlapLimit);
      case TimingBelt(belt, pulley) | BeltIdler(belt, pulley):
        allow(belt, pulley, Math.POSITIVE_INFINITY);
      case BeltReduction(belt, driver, driven):
        allow(belt, driver, Math.POSITIVE_INFINITY);
        allow(belt, driven, Math.POSITIVE_INFINITY);
      case RollerChain(chain, sprocket):
        if (chain != null) allow(chain, sprocket, Math.POSITIVE_INFINITY);
    }
    for (path in flat.beltPaths) if (path.clamp != null) allow(path.belt,
      path.clamp.instanceId, Math.POSITIVE_INFINITY);
  }

  function allow(a:String, b:String, volume:Float):Void {
    interfaces.set(a + "|" + b, volume);
    interfaces.set(b + "|" + a, volume);
  }

  function allowed(a:String, b:String):Float {
    var volume = interfaces.get(a + "|" + b);
    return volume == null ? overlapLimit : Math.max(volume, overlapLimit);
  }

  function run(posed:PosedParts):Int {
    var model = new AssemblyModel("mm");
    assembly.addTo(model, "");
    var state = new AssemblyState(model.definition("integrity"));
    state.forwardKinematics();
    // The driving joints at rest: movable tree joints no coupling drives (the state solves the others from them).
    var coupled = new Map<String, Bool>();
    if (definition.couplings != null) for (coupling in definition.couplings) coupled.set(coupling.target, true);
    var rest = new Map<String, Float>();
    for (joint in definition.joints) if (joint.role == AssemblyJointRole.Tree
      && joint.type != AssemblyJointType.Fixed && joint.passive != true && !coupled.exists(joint.id)) rest.set(
        joint.id,
        state.joint(joint.id)
      );
    var restPoses = new Map<String, AssemblyFrame>();
    for (id in ids) restPoses.set(id, state.worldPose(id));

    var parts = poseAll(posed, state);
    try {
      checkSupport(parts.parts, parts.boxes);
      if (minContactArea != null) checkContactAreas(parts.parts, parts.boxes);
      if (flangeCutouts) checkFlangeCutouts(parts.parts, parts.boxes, restPoses);
      if (fastenerFit) checkFasteners(parts.parts, parts.boxes, restPoses);
      checkInterference(parts.parts, parts.boxes, null);
    } catch (error:Dynamic) {
      closeAll(parts.parts);
      throw error;
    }
    closeAll(parts.parts);

    var states = requestedStates != null ? requestedStates : sweepStates(definition);
    var reported = new Map<String, Bool>();
    for (sample in states) {
      for (id => value in rest) state.setJoint(id, value);
      for (id => value in sample) state.setJoint(id, value);
      state.forwardKinematics();
      var moved = movedBodies(state, restPoses);
      var posedNow = poseAll(posed, state);
      try checkInterference(posedNow.parts,
        posedNow.boxes, {moved: moved, sample: sample, reported: reported}) catch (error:Dynamic) {
        closeAll(posedNow.parts);
        throw error;
      }
      closeAll(posedNow.parts);
    }
    for (id => value in rest) state.setJoint(id, value);
    state.forwardKinematics();
    ServiceChecks.run(assembly, flat, state, posed, diagnostics, overlapLimit, runningGap, rest);
    checkDrives(state, rest);
    return states.length + 1;
  }

  function runService(posed:PosedParts):Void {
    var model = new AssemblyModel("mm");
    assembly.addTo(model, "");
    var state = new AssemblyState(model.definition("integrity"));
    state.forwardKinematics();
    ServiceChecks.run(assembly, flat, state, posed, diagnostics, overlapLimit, runningGap, new Map<String, Float>());
  }

  function runFasteners(posed:PosedParts):Void {
    var model = new AssemblyModel("mm");
    assembly.addTo(model, "");
    var state = new AssemblyState(model.definition("integrity"));
    state.forwardKinematics();
    var restPoses = new Map<String, AssemblyFrame>();
    for (id in ids) restPoses.set(id, state.worldPose(id));
    var parts = poseAll(posed, state);
    try checkFasteners(parts.parts, parts.boxes, restPoses) catch (error:Dynamic) {
      closeAll(parts.parts);
      throw error;
    }
    closeAll(parts.parts);
  }

  function poseAll(posed:PosedParts, state:AssemblyState): {parts: Map<String, Part>, boxes: Map<String, PartBox>} {
    var parts = new Map<String, Part>(), boxes = new Map<String, PartBox>();
    try {
      for (member in flat.members) {
        var part = posed.posed(member.component, state.worldPose(member.id));
        parts.set(member.id, part);
        boxes.set(member.id, PosedParts.boxOf(part));
      }
    } catch (error:Dynamic) {
      closeAll(parts);
      throw error;
    }
    return {parts: parts, boxes: boxes};
  }

  static function closeAll(parts:Map<String, Part>):Void for (part in parts) part.close();

  /** The value at `key`, which the check put there. */
  static function need<T>(map:Map<String, T>, key:String):T {
    var value = map.get(key);
    if (value == null) throw 'Assembly integrity lost track of "$key"';
    return value;
  }

  /** Each body's world pose against its rest pose: the bodies that moved. */
  function movedBodies(state:AssemblyState, restPoses:Map<String, AssemblyFrame>):Map<String, AssemblyFrame> {
    var moved = new Map<String, AssemblyFrame>();
    for (id in ids) {
      var body = need(bodyOf, id);
      if (body != id) continue;
      var now = state.worldPose(id), before = need(restPoses, id);
      moved.set(id, AssemblyFrames.compose(now, AssemblyFrames.inverse(before)));
    }
    return moved;
  }

  static function sameMotion(a:AssemblyFrame,
    b:AssemblyFrame):Bool return Math.abs(a.x - b.x) < 1e-9 && Math.abs(a.y - b.y) < 1e-9
    && Math.abs(a.z - b.z) < 1e-9 && Math.abs(a.qx * b.qx + a.qy * b.qy + a.qz * b.qz + a.qw * b.qw) > 1 - 1e-12;

  // ---- support ---------------------------------------------------------------------------------------------------

  function touching(parts:Map<String, Part>, boxes:Map<String, PartBox>, a:String, b:String):Float {
    if (PosedParts.apart(need(boxes, a), need(boxes, b), contactTolerance)) return Math.POSITIVE_INFINITY;
    return need(parts, a).distance(need(parts, b));
  }

  function checkSupport(parts:Map<String, Part>, boxes:Map<String, PartBox>):Void {
    var parent = new Map<String, String>();
    function find(id:String):String {
      var current = id;
      while (true) {
        var next = parent.get(current);
        if (next == null || next == current) return current;
        current = next;
      }
    }
    function join(a:String, b:String):Void parent.set(find(a), find(b));
    for (id in ids) parent.set(id, id);
    var carried = new Map<String, Bool>();
    for (joint in definition.joints) if (joint.role == AssemblyJointRole.Tree) {
      carried.set(joint.child, true);
      var gap = touching(parts, boxes, joint.parent, joint.child);
      if (gap <= contactTolerance) join(joint.parent, joint.child);
      else diagnostics.error(
        "integrity.support.gap",
        joint.id,
        'Mate ${joint.id} (${joint.type}) holds ${joint.child} on ${joint.parent}, ' + (Math.isFinite(gap)
        ? 'which it does not touch: ${round(gap)} mm apart' : 'which it does not reach')
      );
    }
    var grounded = new Map<String, Bool>();
    var floor = floorZ;
    for (id in ids) {
      var bottom = need(boxes, id).minZ;
      var ground = if (floor == null) !carried.exists(id) else {
        if (bottom < floor - contactTolerance) diagnostics.error(
          "integrity.support.floor",
          id,
          '$id reaches ${round(floor - bottom)} mm below the floor'
        );
        Math.abs(bottom - floor) <= contactTolerance;
      }
      if (ground) grounded.set(find(id), true);
    }
    var loose = new Map<String, Array<String>>();
    for (id in ids) {
      var root = find(id);
      if (grounded.exists(root)) continue;
      if (!loose.exists(root)) loose.set(root, []);
      need(loose, root).push(id);
    }
    for (group in loose) diagnostics.error(
      "integrity.support.ground",
      group[0],
      'Nothing holds up ${group.length} member(s) from the ' + (floorZ
      == null ? "ground" : "floor") + ': ${group.slice(0, 6).join(", ")}${group.length > 6 ? ", ..." : ""}'
    );
  }

  // ---- interference -----------------------------------------------------------------------------------------------

  /**
	 * Pairs that share more volume than they may. At rest (`sweep` null) every pair; in a sweep only pairs whose bodies moved
	 * relative to each other, and each pair once.
	 */
  function checkInterference(parts:Map<String, Part>, boxes:Map<String, PartBox>, sweep:Null<SweepSample>):Void {
    for (i in 0...ids.length) for (j in i + 1...ids.length) {
      var a = ids[i], b = ids[j];
      if (sweep != null) {
        var bodyA = need(bodyOf, a), bodyB = need(bodyOf, b);
        if (bodyA == bodyB || sameMotion(need(sweep.moved, bodyA), need(sweep.moved, bodyB))) continue;
        if (sweep.reported.exists(a + "|" + b)) continue;
      }
      var shared = PosedParts.commonVolume(need(parts, a), need(boxes, a), need(parts, b), need(boxes, b));
      var limit = allowed(a, b);
      if (shared <= limit) {
        if (sweep != null && runningGap != null) checkRunningGap(parts, boxes, a, b, sweep);
        continue;
      }
      if (sweep == null) diagnostics.error(
        "integrity.overlap",
        '$a|$b',
        '$a and $b share ${round(shared)} mm³ at rest' + (limit
        > overlapLimit ? ' (their mesh may share ${round(limit)})' : '')
      );
      else {
        sweep.reported.set(a + "|" + b, true);
        diagnostics.error(
          "integrity.sweep",
          '$a|$b',
          '$a and $b share ${round(shared)} mm³ at ${describe(sweep.sample)} (sampled sweep)'
        );
      }
    }
  }

  /** Bodies that move relative to each other keep `runningGap` apart, unless a moving mate joins them or they mesh. */
  function checkRunningGap(
    parts:Map<String, Part>,
    boxes:Map<String, PartBox>,
    a:String,
    b:String,
    sweep:SweepSample
  ):Void {
    var margin = runningGap;
    if (margin == null || interfaces.exists(a + "|" + b) || movingPairs.exists(a + "|" + b)
      || gapReported.exists(a + "|" + b) || PosedParts.apart(need(boxes, a), need(boxes, b), margin)) return;
    var gap = need(parts, a).distance(need(parts, b));
    if (gap >= margin) return;
    // A shaft in the bearing it turns in (a turning body next to a part that carries a journal) and a guide block on its rail
    // run close by design; an overlap is still an error, found by the interference check.
    if (journalAgainst(a, b) || journalAgainst(b, a) || guided(a, b)) return;
    gapReported.set(a + "|" + b, true);
    diagnostics.error(
      "integrity.gap",
      '$a|$b',
      '$a and $b move relative to each other and come within ${round(gap)} mm (running gap ${round(margin)} mm) at '
      + describe(sweep.sample) + " (sampled sweep)"
    );
  }

  /**
	 * Parts that run close by design: a guide block and its guide rail (the block rides the rail it touches), and a sealed scale's
	 * reading head in its housing's slot (they keep the datasheet's running gap and never touch).
	 */
  function guided(a:String, b:String):Bool {
    var first = flat.member(a), second = flat.member(b);
    if (Std.isOfType(first, machinekit.motion.SealedScalePart) && Std.isOfType(second, machinekit.motion.SealedScalePart)) {
      var one:machinekit.motion.SealedScalePart = cast first, two:machinekit.motion.SealedScalePart = cast second;
      return one.variant != two.variant;
    }
    return (Std.isOfType(first, machinekit.motion.LinearRail) && Std.isOfType(second, machinekit.motion.LinearRailBlock))
      || (Std.isOfType(second, machinekit.motion.LinearRail) && Std.isOfType(first, machinekit.motion.LinearRailBlock));
  }

  /**
	 * `bearing` carries a journal and `other` is in a body that turns relative to the bearing's own body: the shaft runs in its
	 * bearing. A turning body next to some other part with bearings (another axis's motor) is still a rubbing contact.
	 */
  function journalAgainst(bearing:String, other:String):Bool {
    if (!flat.member(bearing).carriesJournal()) return false;
    var body = bodyOf.get(other), host = bodyOf.get(bearing);
    var parents = body == null ? null : turning.get(body);
    return parents != null && host != null && parents.exists(host);
  }

  static function describe(sample:Map<String, Float>):String {
    var entries = [for (key => value in sample) '$key = ${round(value)}'];
    entries.sort(Reflect.compare);
    return entries.join(", ");
  }

  static function round(value:Float):Float return Math.round(value * 1000) / 1000;

  // ---- fixed mates ------------------------------------------------------------------------------------------------

  static final PUSHES:Array<Array<Float>> = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]];

  /**
	 * A fixed mate whose child carries other members must touch its parent over `minContactArea`: a block on a plate's edge
	 * that holds up a frame is a mounting in name only. The area is the volume the child shares with the parent when pushed
	 * `CONTACT_PROBE` into it, over the push, summed over the six axis directions.
	 */
  function checkContactAreas(parts:Map<String, Part>, boxes:Map<String, PartBox>):Void {
    var least = minContactArea;
    if (least == null) return;
    var carries = new Map<String, Bool>();
    for (joint in definition.joints) if (joint.role == AssemblyJointRole.Tree) carries.set(joint.parent, true);
    for (joint in definition.joints) {
      if (joint.role != AssemblyJointRole.Tree || joint.type != AssemblyJointType.Fixed
        || !carries.exists(joint.child) || flat.member(joint.child).fastensByForm()) continue;
      var gap = touching(parts, boxes, joint.parent, joint.child);
      if (gap > contactTolerance) continue;
      var area = contactArea(parts, boxes, joint.child, joint.parent, gap);
      if (area < least) diagnostics.error(
        "integrity.contact.area",
        joint.id,
        'Mate ${joint.id} holds ${joint.child}, which carries other members, on ${joint.parent} over only '
        + '${round(area)} mm² (at least ${round(least)} mm² wanted)'
      );
    }
  }

  function contactArea(
    parts:Map<String, Part>,
    boxes:Map<String, PartBox>,
    child:String,
    parent:String,
    gap:Float
  ):Float {
    var total = 0.0;
    for (push in PUSHES) {
      var moved = need(parts, child).translated(new Vector(push[0] * CONTACT_PROBE, push[1] * CONTACT_PROBE,
        push[2] * CONTACT_PROBE));
      try {
        var shared = PosedParts.commonVolume(moved, PosedParts.boxOf(moved), need(parts, parent), need(boxes, parent));
        total += shared / Math.max(CONTACT_PROBE - gap, 1e-6);
      } catch (error:Dynamic) {
        moved.close();
        throw error;
      }
      moved.close();
    }
    return total;
  }

  // ---- flanges ----------------------------------------------------------------------------------------------------

  /** A part with a `mountingFace` fixed to a host: the host is cut for the flange's pilot and bolts, and lies under it. */
  function checkFlangeCutouts(parts:Map<String, Part>, boxes:Map<String, PartBox>, poses:Map<String, AssemblyFrame>):Void {
    for (joint in definition.joints) {
      if (joint.role != AssemblyJointRole.Tree || joint.type != AssemblyJointType.Fixed) continue;
      checkFlange(parts, boxes, poses, joint.child, joint.parent, joint.id);
      checkFlange(parts, boxes, poses, joint.parent, joint.child, joint.id);
    }
  }

  function checkFlange(
    parts:Map<String, Part>,
    boxes:Map<String, PartBox>,
    poses:Map<String, AssemblyFrame>,
    flanged:String,
    host:String,
    mate:String
  ):Void {
    var face:Null<MountingFace> = flat.member(flanged).mountingFace();
    if (face == null) return;
    var pose = AssemblyFrames.compose(need(poses, flanged), face.pose);
    var owned:Array<Part> = [];
    try {
      var localTool = face.hostTool(FLANGE_PROBE);
      owned.push(localTool);
      var tool = PosedParts.moved(localTool, pose);
      owned.push(tool);
      var slab = Part.box(face.width, face.height, FLANGE_PROBE);
      owned.push(slab);
      var seat = PosedParts.moved(slab, pose);
      owned.push(seat);
      var hostPart = need(parts, host), hostBox = need(boxes, host);
      var cut = PosedParts.commonVolume(hostPart, hostBox, tool, PosedParts.boxOf(tool));
      var under = PosedParts.commonVolume(hostPart, hostBox, seat, PosedParts.boxOf(seat));
      var available = slab.volume() - localTool.volume();
      // A host with no material at the flange at all is some other mate of this part (a motor on a mount, a ring on a support).
      if (cut <= overlapLimit && under <= overlapLimit) {
        closeList(owned);
        return;
      }
      if (cut > overlapLimit) diagnostics.error(
        "integrity.mount.cutout",
        mate,
        '$flanged is bolted to $host, which has ${round(cut)} mm³ of material in the pilot and bolt holes the flange needs '
        + "(the host is not cut for it)"
      );
      else if (under < FLANGE_SEAT_SHARE * available) diagnostics.error(
        "integrity.mount.seat",
        mate,
        '$flanged is bolted to $host, which supplies ${round(under)} of the ${round(available)} mm³ under the flange face'
      );
    } catch (error:Dynamic) {
      closeList(owned);
      throw error;
    }
    closeList(owned);
  }

  // ---- fasteners -----------------------------------------------------------------------------------------------------

  /**
	 * The envelope of every derived fastener (`FastenedJoint.volumes`), carried to the receiving face's pose, against the
	 * members: the holes must be free of material, heads and nuts must have room, and a driver must reach them.
	 */
  function checkFasteners(parts:Map<String, Part>, boxes:Map<String, PartBox>, poses:Map<String, AssemblyFrame>):Void {
    var derivation = assembly.derivedFasteners();
    if (derivation == null) return;
    var group = JointFasteners.rigidGroups(assembly);
    var faces = new Map<String, machinekit.component.BoltedFace>();
    for (joint in derivation.joints) {
      var key = joint.receivingMember + "#" + joint.receivingFace;
      if (!faces.exists(key)) for (face in flat.member(joint.receivingMember).fasteningFaces()) if (face.id == joint.receivingFace)
        faces.set(key, face);
      var face = faces.get(key);
      if (face == null) throw 'Assembly integrity lost the face "$key" of joint ${joint.id}';
      var frame = AssemblyFrames.compose(need(poses, joint.receivingMember), face.frame);
      var reported = new Map<String, Bool>();
      for (volume in joint.volumes) for (point in joint.hostHoles) {
        var local = Part.cylinderSpan(volume.diameter / 2, volume.from, volume.to, point.x, point.y);
        var tool = PosedParts.moved(local, frame);
        local.close();
        var box = PosedParts.boxOf(tool);
        try {
          for (id => part in parts) {
            if (reported.exists(volume.name + "|" + id)) continue;
            if (volume.access && (group.get(id) != group.get(joint.clampedMember) || removable.exists(id))) continue;
            var shared = PosedParts.commonVolume(part, need(boxes, id), tool, box);
            if (shared <= overlapLimit) continue;
            reported.set(volume.name + "|" + id, true);
            diagnostics.error(
              volume.code,
              joint.id,
              '${joint.id}: the ${joint.size} x ${Math.round(joint.length)} fastener\'s ${volume.name} '
              + (volume.access ? 'cannot be reached: $id stands in its ${Math.round(volume.to - volume.from)} mm access' : 'meets $id')
              + ' (${round(shared)} mm³ at (${round(point.x)}, ${round(point.y)}))'
            );
          }
        } catch (error:Dynamic) {
          tool.close();
          throw error;
        }
        tool.close();
      }
    }
  }

  static function closeList(parts:Array<Part>):Void for (part in parts) part.close();

  // ---- drive consistency -------------------------------------------------------------------------------------------

  function checkDrives(state:AssemblyState, rest:Map<String, Float>):Void {
    if (definition.couplings == null) return;
    var bySource = new Map<String, AssemblyJointCoupling>();
    for (coupling in definition.couplings) bySource.set(coupling.id, coupling);
    var drivenBy = new Map<String, String>();
    for (coupling in definition.couplings) drivenBy.set(coupling.target, coupling.source);
    var limitsOf = new Map<String, AssemblyJointLimits>();
    for (joint in definition.joints) limitsOf.set(joint.id, joint.limits);
    for (record in flat.transmissions) {
      var coupling = bySource.get(record.coupling);
      if (coupling == null) continue;
      var leader = coupling.source;
      for (_ in 0...definition.joints.length) {
        var up = drivenBy.get(leader);
        if (up == null) break;
        leader = up;
      }
      var at = rest.get(leader);
      if (at == null) continue;
      var limits = limitsOf.get(leader);
      var upper = limits == null ? null : limits.upper;
      var step = upper != null && at + DRIVE_STEP > upper ? -DRIVE_STEP : DRIVE_STEP;
      var leaderMove = {id: leader, from: at, step: step};
      switch record.source {
        case RackAndPinion(pinion, rack):
          if (rack != null) checkRack(state, rest, leaderMove, record.coupling, pinion, rack);
        case GearMesh(driver, driven):
          checkGears(state, rest, leaderMove, record.coupling, driver, driven);
        case LeadScrew(screw, nut):
          checkScrew(state, rest, leaderMove, record.coupling, screw, nut);
        case _:
      }
    }
  }

  /** World poses of `members` at rest and with the leader moved by its step. */
  function stepped(state:AssemblyState, rest:Map<String, Float>, leader:LeaderMove, members:Array<String>): {
    before: Array<AssemblyFrame>,
    after: Array<AssemblyFrame>
  } {
    for (id => value in rest) state.setJoint(id, value);
    state.forwardKinematics();
    var before = [for (id in members) state.worldPose(id)];
    state.setJoint(leader.id, leader.from + leader.step);
    state.forwardKinematics();
    var after = [for (id in members) state.worldPose(id)];
    for (id => value in rest) state.setJoint(id, value);
    state.forwardKinematics();
    return {before: before, after: after};
  }

  /**
	 * Where the material point at world `point` (at rest) of each of two members is after the step: how far apart the two
	 * copies end up, against how far the members moved. A pitch point is at rest on a fixed rack to first order, so the
	 * scale is the larger of the point's travel and each member's own origin's travel, which is first order in the step.
	 */
  static function slip(
    point: {x: Float, y: Float, z: Float},
    poses: {before: Array<AssemblyFrame>, after: Array<AssemblyFrame>}
  ): {
    apart: Float,
    travel: Float
  } {
    var moved = [for (index in 0...2) {
      var local = AssemblyFrames.transformPoint(AssemblyFrames.inverse(poses.before[index]), point.x, point.y, point.z);
      AssemblyFrames.transformPoint(poses.after[index], local.x, local.y, local.z);
    }
    ];
    var travel = Math.max(distance(moved[0], point), distance(moved[1], point));
    for (index in 0...2) {
      var before = poses.before[index], after = poses.after[index];
      travel = Math.max(travel,
        distance({x: before.x, y: before.y, z: before.z}, {x: after.x, y: after.y, z: after.z}));
    }
    return {apart: distance(moved[0], moved[1]), travel: travel};
  }

  static function distance(a: {x: Float,
    y: Float, z: Float}, b: {x: Float, y: Float, z: Float}):Float return Math.sqrt((a.x
    - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));

  function gear(id:String):Null<SpurGear> {
    var member = flat.member(id);
    return Std.isOfType(member, SpurGear) ? cast member : null;
  }

  function checkRack(
    state:AssemblyState,
    rest:Map<String, Float>,
    leader:LeaderMove,
    coupling:String,
    pinion:String,
    rack:String
  ):Void {
    var wheel = flat.member(pinion), bar = flat.member(rack);
    if ((!Std.isOfType(wheel,
      SpurGear) && !Std.isOfType(wheel, machinekit.transmission.HelicalPinion)) || (!Std.isOfType(
        bar,
        Rack
      ) && !Std.isOfType(
        bar,
        machinekit.transmission.HelicalRack
      ) && !Std.isOfType(
        bar,
        machinekit.transmission.GearRack
      ) && !Std.isOfType(
        bar,
        machinekit.transmission.GearRackRun
      ))) return;
    var radius = machinekit.transmission.RackGeometry.pinion(wheel).diameter / 2;
    var poses = stepped(state, rest, leader, [pinion, rack]);
    var wheelPose = poses.before[0], rackPose = poses.before[1];
    var centre = AssemblyFrames.transformPoint(wheelPose, 0, 0, 0);
    var axis = AssemblyFrames.transformVector(wheelPose, 0, 0, 1);
    var up = AssemblyFrames.transformVector(rackPose, 0, 1, 0), across = AssemblyFrames.transformVector(
      rackPose,
      1,
      0,
      0
    );
    // The pitch line is the rack's local y = 0; its teeth point along local +Y, its face runs along local +X.
    var height = (centre.x - rackPose.x) * up.x + (centre.y - rackPose.y) * up.y + (centre.z - rackPose.z) * up.z;
    if (Math.abs(height - radius) > contactTolerance
      || Math.abs(Math.abs(axis.x * across.x + axis.y * across.y + axis.z * across.z) - 1) > 1e-6) diagnostics.error(
        "integrity.drive.mesh",
        coupling,
        '$pinion does not stand on $rack\'s pitch line: its centre is ${round(height)} mm '
      + 'above it, its pitch radius ${round(radius)} mm'
    );
    var contact = {x: centre.x - radius * up.x, y: centre.y - radius * up.y, z: centre.z - radius * up.z};
    reportSlip(coupling, '$pinion on $rack', slip(contact, poses));
  }

  function checkGears(
    state:AssemblyState,
    rest:Map<String, Float>,
    leader:LeaderMove,
    coupling:String,
    driver:String,
    driven:String
  ):Void {
    var a = gear(driver), b = gear(driven);
    if (a == null || b == null) return;
    var poses = stepped(state, rest, leader, [driver, driven]);
    var ca = AssemblyFrames.transformPoint(poses.before[0], 0, 0, 0), cb = AssemblyFrames.transformPoint(
      poses.before[1],
      0,
      0,
      0
    );
    var axis = AssemblyFrames.transformVector(poses.before[0], 0, 0, 1);
    var dx = cb.x - ca.x, dy = cb.y - ca.y, dz = cb.z - ca.z;
    var along = dx * axis.x + dy * axis.y + dz * axis.z;
    dx -= along * axis.x;
    dy -= along * axis.y;
    dz -= along * axis.z;
    var centres = Math.sqrt(dx * dx + dy * dy + dz * dz);
    if (Math.abs(centres - (a.pitchDiameter + b.pitchDiameter) / 2) > contactTolerance) diagnostics.error(
      "integrity.drive.mesh",
      coupling,
      '$driver and $driven stand ${round(centres)} mm apart, their pitch radii sum to '
      + '${round((a.pitchDiameter + b.pitchDiameter) / 2)} mm'
    );
    var r = a.pitchDiameter / 2 / centres;
    reportSlip(coupling, '$driver and $driven', slip({x: ca.x + r * dx, y: ca.y + r * dy, z: ca.z + r * dz}, poses));
  }

  function reportSlip(coupling:String, what:String, result: {apart: Float, travel: Float}):Void {
    if (!(result.travel > 0)) {
      diagnostics.error("integrity.drive.still", coupling, 'Moving the leader of $coupling does not move $what');
      return;
    }
    if (result.apart > DRIVE_SLIP * result.travel) diagnostics.error(
      "integrity.drive.slip",
      coupling,
      'The coupling $coupling disagrees with its parts: $what part by '
      + '${round(result.apart / result.travel * 100)} % of their travel at the pitch point (a wrong sense or ratio)'
    );
  }

  function checkScrew(
    state:AssemblyState,
    rest:Map<String, Float>,
    leader:LeaderMove,
    coupling:String,
    screw:String,
    nut:String
  ):Void {
    var member = flat.member(screw);
    if (!Std.isOfType(member, LeadScrew)) return;
    var thread = (cast member:LeadScrew).thread;
    var poses = stepped(state, rest, leader, [screw, nut]);
    // The nut in the screw's frame before and after: it must turn about the screw's +Z and travel along it as the
    // thread says.
    var before = AssemblyFrames.compose(AssemblyFrames.inverse(poses.before[0]), poses.before[1]);
    var after = AssemblyFrames.compose(AssemblyFrames.inverse(poses.after[0]), poses.after[1]);
    if (Math.sqrt(before.x * before.x + before.y * before.y) > contactTolerance) diagnostics.error(
      "integrity.drive.mesh",
      coupling,
      '$nut is ${round(Math.sqrt(before.x * before.x + before.y * before.y))} mm off $screw\'s axis'
    );
    var turn = AssemblyFrames.compose(after, AssemblyFrames.inverse(before));
    // The nut's turn relative to the screw, about the screw's +Z.
    var nutTurn = 2 * Math.atan2(turn.qz, turn.qw);
    if (nutTurn > Math.PI) nutTurn -= 2 * Math.PI;
    if (nutTurn < -Math.PI) nutTurn += 2 * Math.PI;
    var travel = after.z - before.z;
    // A screw turning by θ against its nut moves the nut by signedLead·θ/2π along +Z; the nut turning by -θ is the same.
    var expected = -thread.signedLead() * nutTurn / (2 * Math.PI);
    var scale = Math.max(Math.abs(travel), Math.abs(expected));
    if (!(scale > 0)) {
      diagnostics.error(
        "integrity.drive.still",
        coupling,
        'Moving the leader of $coupling neither turns $screw nor moves $nut'
      );
      return;
    }
    if (Math.abs(travel - expected) > DRIVE_SLIP * scale) diagnostics.error(
      "integrity.drive.slip",
      coupling,
      'The coupling $coupling disagrees with $screw\'s thread: $nut travels ${round(travel)} mm '
      + 'for a turn that should move it ${round(expected)} mm (a wrong sense or lead)'
    );
  }
}
