package machinekit.motion;

import machinekit.assembly.AxisBuilder;
import machinekit.component.MachineComponent;
import machinekit.standard.DeepGrooveBearing;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/**
 * Where a supported screw drive sits, in world mm. `start` is the screw's origin, its fixed end; `direction` the unit
 * vector along the screw to its floating end; `normal` the unit vector from the surface the supports stand on toward the
 * screw axis (the mounting side), perpendicular to `direction`.
 */
typedef ScrewDriveAxis = {
  var start:Array<Float>;
  var direction:Array<Float>;
  var normal:Array<Float>;
}

/** The members that carry a drive's parts: the fixed mount's and the floating support's hosts, and the nut's carriage. */
typedef ScrewDriveHosts = {
  var fixedHost:String;
  var floatingHost:String;
  var nutHost:String;
}

/**
 * A hole the drive needs in the seat it stands on: `tapped` (bolt thread, drilled to `depth`), `clearance` (bolt from the
 * far side) or `dowel` (reamed at assembly once the support is aligned to the nut). `direction` points into the seat.
 */
typedef ScrewDriveSeatHole = {
  var kind:String;
  var support:String;
  var thread:Null<String>;
  var diameter:Float;
  var depth:Float;
  var position:{x:Float, y:Float, z:Float};
  var direction:{x:Float, y:Float, z:Float};
}

/** A supported ball-screw drive: a motor mount with its fixed bearings, the floating support and its retaining ring, servo and coupling.
 * Every mounting datum is read from the parts' catalogue and STEP review data; only the coupling
 * (outside diameter, length, clamps and where it seats) is stated here. Nothing is sized for a load.
 *
 * `poses` places every part from an axis (screw start, direction, mounting side) and a screw length, `place` adds them to an
 * `AxisBuilder`, and `seatHoles` lists the holes the seats need, all from the same datums.
 */
class SupportedScrewDrive {
  /** Integrated motor mount with the fixed bearing pair. */
  public final fixed:MotorMount;
  public final floating:ScrewSupportUnit;
  /** The DIN 471 ring that holds the floating bearing on its journal, and where along the support it sits. */
  public final ring:machinekit.standard.RetainingRing;
  public final ringSeatZ:Float;
  public final motor:LeadshineServo;
  public final coupling:ShaftCoupling;
  /** Motor face to the coupling origin, along the screw. */
  public final couplingSeat:Float;
  /** HZMotion screw end machined for the fixed mount. */
  public final endProfile:String;
  /** Motor face to fixed-bearing centre. */
  public final bearingDrop:Float;
  /** Screw's fixed end, shoulder to thread: the screw starts `fixed.spec.shoulderZ - fixedEnd` from the motor face. */
  public final fixedEnd:Float;
  /** Screw end to the floating bearing centre: the journal less half the bearing, which seats on its shoulder. */
  public final floatingEnd:Float;
  /** Floating support's bolt line and bolt offset. */
  public final floatingBoltZ:Float;
  public final floatingBoltX:Float;
  /** Bolt counterbore seats above each part's mounting base. */
  public final fixedBoltSeat:Float;
  public final floatingBoltSeat:Float;

  public function new(
    fixed:MotorMount,
    floating:ScrewSupportUnit,
    motor:LeadshineServo,
    couplingLength:Float,
    couplingSeat:Float,
    ?couplingOutside:Float,
    ?couplingClamps:Array<ShaftCoupling.ShaftCouplingSetScrew>
  ) {
    this.fixed = fixed;
    this.floating = floating;
    this.motor = motor;
    this.couplingSeat = couplingSeat;
    if (!Math.isFinite(couplingSeat)
      || couplingSeat <= 0) throw "Screw-drive coupling seat must be finite and positive";
    var profile = fixed.spec.endProfile;
    if (profile == null || !StringTools.startsWith(profile, "C-")) throw "Screw drive needs a C-profile fixed mount";
    endProfile = profile.substr(2);
    if (fixed.spec.oppositeSupport
      != floating.spec.model) throw "Floating support must be the fixed mount's opposite support";
    if (fixed.spec.shoulderZ == null) throw "Screw drive mount needs a screw shoulder datum";
    bearingDrop = fixed.bearingCentreZ();
    fixedEnd = HzMotionDrawingScrew.fixedEndFor(endProfile);
    coupling = new ShaftCoupling(
      motor.rating.shaftDiameter,
      HzMotionDrawingScrew.couplingDiameterFor(endProfile),
      couplingOutside,
      couplingLength,
      couplingClamps
    );
    var bearing = floating.bearing;
    if (!StringTools.endsWith(bearing, "ZZ")) throw "Floating support needs a shielded deep-groove bearing";
    floatingEnd = HzMotionDrawingScrew.floatingEndFor(endProfile) - DeepGrooveBearing.catalog().get(bearing.substr(
      0,
      bearing.length - 2
    )).width / 2;
    var holes = floating.footHoles();
    if (holes.length != 2
      || holes[0].z != holes[1].z || holes[0].x != -holes[1].x) throw "Floating support needs one symmetric bolt pair";
    floatingBoltZ = holes[0].z;
    floatingBoltX = Math.abs(holes[0].x);
    fixedBoltSeat = fixed.boltSeatY();
    floatingBoltSeat = floating.boltSeatY();
    var seat = floating.ringSeatZ();
    if (seat == null) throw 'Floating support ${floating.spec.model} has no reviewed retaining-ring seat';
    ringSeatZ = seat;
    ring = machinekit.standard.RetainingRing.forShaft(floating.boreDiameter);
  }

  // ---- placement --------------------------------------------------------------------------------------------------

  /** Motor face to the screw's start along the screw: where the shoulder sits in the mount, less the screw's fixed end. */
  public function faceToStart():Float {
    var shoulder = fixed.spec.shoulderZ;
    if (shoulder == null) throw "Screw drive mount needs a screw shoulder datum";
    return shoulder - fixedEnd;
  }

  /** A screw of this drive's end profile whose floating bearing centre is `faceToBolts` mm from the motor face. */
  public function screwFor(faceToBolts:Float):HzMotionDrawingScrew {
    if (!Math.isFinite(faceToBolts) || faceToBolts <= 0) throw "Screw drive needs a positive motor face to floating bearing distance";
    return new HzMotionDrawingScrew(faceToStart() + faceToBolts + floatingEnd, endProfile);
  }

  static function along(point:Array<Float>, direction:Array<Float>, distance:Float):Array<Float> return [
    point[0] + direction[0] * distance, point[1] + direction[1] * distance, point[2] + direction[2] * distance];

  static function checkAxis(axis:ScrewDriveAxis):Void {
    for (vector in [axis.start, axis.direction, axis.normal]) {
      if (vector == null || vector.length != 3) throw "A screw drive axis needs 3-vectors";
      for (value in vector) if (!Math.isFinite(value)) throw "A screw drive axis must be finite";
    }
    function length(v:Array<Float>):Float return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (Math.abs(length(axis.direction) - 1) > 1e-9 || Math.abs(length(axis.normal) - 1) > 1e-9
      || Math.abs(axis.direction[0] * axis.normal[0] + axis.direction[1] * axis.normal[1]
      + axis.direction[2] * axis.normal[2]) > 1e-9) throw "A screw drive axis needs a unit direction and a unit normal perpendicular to it";
  }

  /** The motor face's centre on the axis. */
  public function motorFace(axis:ScrewDriveAxis):Array<Float> {
    checkAxis(axis);
    return along(axis.start, axis.direction, -faceToStart());
  }

  /** The floating support's bolt line on the axis, for a screw `length` long: its bearing centre. */
  public function floatingBoltLine(axis:ScrewDriveAxis, length:Float):Array<Float> {
    checkAxis(axis);
    return along(axis.start, axis.direction, length - floatingEnd);
  }

  static function frame(point:Array<Float>, axis:ScrewDriveAxis):AssemblyFrame return AxisBuilder.orient(
    point[0], point[1], point[2], axis.normal, axis.direction);

  /**
	 * World poses of the drive's parts for a screw `length` long, keyed `mount`, `motor` (the motor, its shaft and the
	 * coupling's reference), `screw`, `coupling`, `floating` (the support in its own frame), `ring` and `nut`
	 * (`nutStation` along the screw from its start; omitted when null).
	 */
  public function poses(axis:ScrewDriveAxis, length:Float, ?nutStation:Float):Map<String, AssemblyFrame> {
    var face = motorFace(axis);
    var result = new Map<String, AssemblyFrame>();
    // Mount and support stand on the seat: their frames sit a shaft height below the axis.
    result.set("mount", frame(along(face, axis.normal, -fixed.spec.shaftHeight), axis));
    result.set("motor", frame(face, axis));
    result.set("screw", frame(axis.start, axis));
    result.set("coupling", AssemblyFrames.compose(frame(face, axis), AssemblyFrames.translation(0, 0, couplingSeat)));
    var footOrigin = along(along(floatingBoltLine(axis, length), axis.direction, -floatingBoltZ), axis.normal,
      -floating.spec.shaftHeight);
    var foot = AssemblyFrames.compose(frame(footOrigin, axis), floating.supportInFootFrame());
    result.set("floating", foot);
    result.set("ring", AssemblyFrames.compose(foot, AssemblyFrames.translation(0, 0, ringSeatZ)));
    if (nutStation != null) {
      if (!Math.isFinite(nutStation)) throw "Screw drive nut station must be finite";
      result.set("nut", frame(along(axis.start, axis.direction, nutStation), axis));
    }
    return result;
  }

  /**
	 * Adds the drive to `builder` as members `<name><suffix>` (`motorMount`, `motor`, `motorShaft`, `screw`, `coupling`,
	 * `floatingBearing`, `floatingRing`, `nut`): the mount on `hosts.fixedHost`, the floating support on
	 * `hosts.floatingHost`, the nut (when given) on `hosts.nutHost`, the screw turning in the mount about `direction`
	 * as joint `screw<suffix>-turn`. Bolts are not added: the seats' holes come from `seatHoles`.
	 */
  public function place(
    builder:AxisBuilder,
    axis:ScrewDriveAxis,
    hosts:ScrewDriveHosts,
    screw:HzMotionDrawingScrew,
    ?nut:MachineComponent,
    nutStation:Float = 0,
    suffix:String = ""
  ):Void {
    if (screw.endProfile != endProfile) throw 'Screw drive takes a $endProfile screw, not ${screw.endProfile}';
    var at = poses(axis, screw.totalLength, nut == null ? null : nutStation);
    var mount = "motorMount" + suffix, motorId = "motor" + suffix, screwId = "screw" + suffix;
    builder.attach(mount, fixed, need(at, "mount"), hosts.fixedHost);
    builder.attach(motorId, motor, need(at, "motor"), mount);
    builder.hang(screwId, screw, need(at, "screw"), mount);
    builder.addMateOnAxis(
      screwId + "-turn",
      "continuous",
      mount,
      "to-" + screwId,
      screwId,
      "attach-" + screwId,
      {x: axis.direction[0], y: axis.direction[1], z: axis.direction[2]}
    );
    builder.attach("motorShaft" + suffix, motor.shaft(), need(at, "motor"), screwId);
    builder.attach("coupling" + suffix, coupling, need(at, "coupling"), screwId);
    builder.attach("floatingBearing" + suffix, floating, need(at, "floating"), hosts.floatingHost);
    builder.attach("floatingRing" + suffix, ring, need(at, "ring"), "floatingBearing" + suffix);
    if (nut != null) builder.attach("nut" + suffix, nut, need(at, "nut"), hosts.nutHost);
  }

  static function need(poses:Map<String, AssemblyFrame>, key:String):AssemblyFrame {
    var pose = poses.get(key);
    if (pose == null) throw 'Screw drive lost its "$key" pose';
    return pose;
  }

  /** Dowel diameter and how far inboard of the two farthest bolts it sits (assumed: the catalogue feet carry no dowel holes). */
  public static inline var DOWEL_DIAMETER:Float = 5;
  public static inline var DOWEL_INBOARD:Float = 10;

  /**
	 * The holes the seats need, in world mm: a tapped hole (or, with `throughBolts`, a clearance hole) for each foot bolt of
	 * the fixed mount and of the floating support, and for each support two dowels on the line between its two farthest
	 * bolts, `DOWEL_INBOARD` inboard of each, reamed through the foot at assembly once the support is aligned to the nut.
	 */
  public function seatHoles(axis:ScrewDriveAxis, length:Float, throughBolts:Bool = false):Array<ScrewDriveSeatHole> {
    var at = poses(axis, length);
    var down = {x: -axis.normal[0], y: -axis.normal[1], z: -axis.normal[2]};
    var result:Array<ScrewDriveSeatHole> = [];
    function support(name:String, origin:AssemblyFrame, holes:Array<{x:Float, z:Float, diameter:Float}>):Void {
      var base = [origin.x, origin.y, origin.z];
      var across = AssemblyFrames.transformVector(origin, 1, 0, 0);
      var points = [for (hole in holes) {
        x: base[0] + across.x * hole.x + axis.direction[0] * hole.z,
        y: base[1] + across.y * hole.x + axis.direction[1] * hole.z,
        z: base[2] + across.z * hole.x + axis.direction[2] * hole.z
      }];
      for (index in 0...holes.length) {
        var thread = machinekit.component.MountingFace.boltFor(holes[index].diameter);
        result.push({
          kind: throughBolts ? "clearance" : "tapped",
          support: name,
          thread: thread.name,
          diameter: throughBolts ? holes[index].diameter : thread.diameter,
          depth: throughBolts ? 0 : 2 * thread.diameter,
          position: points[index],
          direction: down
        });
      }
      var first = 0, second = 0, farthest = -1.0;
      for (i in 0...points.length) for (j in i + 1...points.length) {
        var dx = points[j].x - points[i].x, dy = points[j].y - points[i].y, dz = points[j].z - points[i].z;
        var span = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (span > farthest) {
          farthest = span;
          first = i;
          second = j;
        }
      }
      if (farthest > 2 * DOWEL_INBOARD) for (pair in [[first, second], [second, first]]) {
        var a = points[pair[0]], b = points[pair[1]];
        var share = DOWEL_INBOARD / farthest;
        result.push({
          kind: "dowel",
          support: name,
          thread: null,
          diameter: DOWEL_DIAMETER,
          depth: 2 * DOWEL_DIAMETER,
          position: {x: a.x + (b.x - a.x) * share, y: a.y + (b.y - a.y) * share, z: a.z + (b.z - a.z) * share},
          direction: down
        });
      }
    }
    support("fixed", need(at, "mount"), [for (hole in fixed.spec.baseHoles) {x: hole.x, z: hole.z, diameter: hole.diameter}]);
    // The floating support's holes are in the foot frame, which `poses` composes `supportInFootFrame` onto.
    var foot = AssemblyFrames.compose(need(at, "floating"), AssemblyFrames.inverse(floating.supportInFootFrame()));
    support("floating", foot, floating.footHoles());
    return result;
  }

  /** Mill Z: M80E-BK15-C / BF15, braked ELM2H-0750 servo, 48 mm coupling. */
  public static function m80eBk15():SupportedScrewDrive return new SupportedScrewDrive(
    MotorMount.of("M80E-BK15-C", null, "hzmotion"),
    ScrewSupportUnit.model("BF15", "hzmotion"),
    LeadshineServo.model("ELM2H-0750LA80E"),
    48,
    17
  );

  /** Small mill Y: M60D-BK12-C / BF12, ELM2H-0400 servo, 25 × 32 clamp coupling. */
  public static function m60dBk12(driven:Bool = false):SupportedScrewDrive return new SupportedScrewDrive(
    MotorMount.of("M60D-BK12-C", null, "hzmotion"),
    ScrewSupportUnit.model("BF12", "hzmotion"),
    driven ? LeadshineServo.driven("ELM2H-0400LA60F") : LeadshineServo.model("ELM2H-0400LA60F"),
    32,
    16,
    25,
    [{z: 8, angle: Math.PI / 2}, {z: 24, angle: Math.PI / 2}]
  );

  /** The same drive with the braked ELM2H-0400LA60E servo, for a vertical axis that must hold when the power drops. */
  public static function m60dBk12Braked(driven:Bool = false):SupportedScrewDrive return new SupportedScrewDrive(
    MotorMount.of("M60D-BK12-C", null, "hzmotion"),
    ScrewSupportUnit.model("BF12", "hzmotion"),
    driven ? LeadshineServo.driven("ELM2H-0400LA60E") : LeadshineServo.model("ELM2H-0400LA60E"),
    32,
    16,
    25,
    [{z: 8, angle: Math.PI / 2}, {z: 24, angle: Math.PI / 2}]
  );
}
