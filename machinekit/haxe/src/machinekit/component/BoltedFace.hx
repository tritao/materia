package machinekit.component;

import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** A bolt axis in a face frame, mm. */
typedef BoltHole = {x:Float, y:Float}

/** The material a thread is cut in; it sets the least thread engagement in nominal diameters. */
enum ThreadMaterial {
  Steel;
  CastIron;
  Aluminium;
}

/** The counterbore a fastener head sinks into: the recess floor is the bearing seat. */
typedef BoltRecess = {diameter:Float, depth:Float}

/**
 * One round hole or recess `JointFasteners` derives for a part: a cylinder of `diameter` on the axis through (`x`, `y`) of
 * `frame` (a frame in the part's own frame), spanning `from` to `to` along that frame's +Z, mm. A part that `machinable()`
 * cuts these into its geometry (`MachineComponent.machined`), so the solid, the exports and the checks see the same holes.
 */
@:wire typedef MachinedCut = {
  @:id(1) var frame:AssemblyFrame;
  @:id(2) var x:Float;
  @:id(3) var y:Float;
  @:id(4) var diameter:Float;
  @:id(5) var from:Float;
  @:id(6) var to:Float;
  /** Tapped, clearance or counterbore; the thread names the screw. */
  @:id(7) @:optional var kind:String;
  @:id(8) @:optional var thread:String;
  /** Reamed fit (H7 for a dowel) and whether the hole exits the stock. */
  @:id(9) @:optional var fit:String;
  @:id(10) @:optional var through:Bool;
  /**
   * The cut's stable name, `<joint>.h<k>.<role>` (`J3.h2.tap`, `J3.h1.counterbore`): the cylinder's faces are named under
   * it, so a mate or drawing picks "the bore of hole 2 of joint J3" whatever else the part's geometry does.
   */
  @:id(11) @:optional var name:String;
};

enum BoltedRole {
  /**
	 * This side is clamped: the fastener passes through a clearance hole of `clearance` mm. `grip` is the material from the
	 * bearing seat (the back face, or the recess floor) to the mating plane. `size` is the fastener the hole was made for
	 * (a catalogue rail names its screw), or null when any fastener that fits is acceptable.
	 */
  Clamped(size:Null<String>, clearance:Float, grip:Float, recess:Null<BoltRecess>);
  /**
	 * This side holds the thread. `size` is the thread already cut (null: the host is drilled and tapped to suit the joint).
	 * `depth` is the deepest thread the material takes (the thread already cut, or the most a seat can be tapped) and
	 * `thickness` the material behind the mating plane; `through` means the thread leaves the far face (the screw tip may
	 * stand flush with it) rather than ending blind.
	 */
  Receiving(size:Null<String>, depth:Float, thickness:Float, through:Bool, material:ThreadMaterial);
  /**
	 * This side is a plate with clearance holes of `clearance` mm, `thickness` thick, listing its holes: the fastener passes
	 * through it and a nut with a washer clamps it on the far side (the screw is a bolt). `size` as for `Clamped`.
	 */
  Through(size:Null<String>, clearance:Float, thickness:Float);
}

/**
 * One face of a part where it is bolted to another, and what it knows of that bolting. The face frame lies in the part's
 * frame with its origin on the mating plane and +Z pointing into this part's own material, so two faces that meet have
 * opposed Z axes. A clamped face lists its clearance holes; a receiving face lists the holes already tapped, or none and a
 * footprint (`width` by `height`, centred on the origin): the host takes whatever holes the joint needs there, and
 * the derivation reports them. `JointFasteners` pairs faces that meet in an assembly, validates them against each other
 * and derives the fasteners and the host's drilling from the pairing.
 */
class BoltedFace {
  public final id:String;
  public final frame:AssemblyFrame;
  public final role:BoltedRole;
  /** The holes in the face frame; null on a receiving face drilled to suit. */
  public final holes:Null<Array<BoltHole>>;
  /** The footprint of a face drilled to suit, mm; 0 where `holes` give the extent. */
  public final width:Float;
  public final height:Float;
  /** The material behind the plane at (x, y) of a face drilled to suit, where a bore or pocket makes it thinner than `thickness`. */
  public final thicknessAt:Null<(Float, Float) -> Float>;
  /**
	 * Whether the part's own geometry already draws this face's holes (a catalogue rail with its counterbores, a datasheet
	 * flange). A clamped face that does not (`drawn` false) has its clearance holes, and its counterbore where it declares a
	 * recess, cut by the derivation; a receiving face drilled to suit is always cut.
	 */
  public final drawn:Bool;
  /** Whether a clamped face wants a plain washer under the head (a soft or thin bearing surface); a nut always has one. */
  public final washer:Bool;
  /**
	 * Names a set of faces of one part that are alternatives (a nut flange bolts on by either of its faces): the part is
	 * fastened if one of them meets a receiving face, and only when none does is it unmatched.
	 */
  public final alternative:Null<String>;
  /**
	 * A tapped face whose thread depth is the maker's own, rated on its drawing (a catalogue actuator's flange): the least engagement is
	 * one diameter, the steel rule, whatever the material, and the derivation reports a warning where the material's rule asks for more.
	 */
  public final rated:Bool;

  function new(id:String, frame:AssemblyFrame, role:BoltedRole, holes:Null<Array<BoltHole>>, width:Float, height:Float,
      ?thicknessAt:(Float, Float) -> Float, drawn:Bool = true, washer:Bool = false, ?alternative:String, rated:Bool = false) {
    this.alternative = alternative;
    this.rated = rated;
    this.thicknessAt = thicknessAt;
    this.drawn = drawn;
    this.washer = washer;
    if (id == null || StringTools.trim(id) == "") throw "A bolted face needs an id";
    this.id = id;
    this.frame = frame;
    this.role = role;
    this.holes = holes;
    this.width = width;
    this.height = height;
    switch role {
      case Clamped(_, clearance, grip, recess):
        if (holes == null && (!(width > 0) || !(height > 0))) throw 'Clamped face "$id" needs its holes, or a footprint to be drilled to suit';
        if (holes != null && holes.length == 0) throw 'Clamped face "$id" lists no holes';
        if (!(clearance > 0) || !(grip > 0) || !Math.isFinite(clearance + grip)) throw 'Clamped face "$id" needs a positive clearance hole and grip';
        if (recess != null && (!(recess.diameter > clearance) || !(recess.depth > 0))) throw 'Clamped face "$id" has an invalid recess';
      case Through(_, clearance, thickness):
        if (holes == null || holes.length == 0) throw 'Through face "$id" needs its holes';
        if (!(clearance > 0) || !(thickness > 0) || !Math.isFinite(clearance + thickness))
          throw 'Through face "$id" needs a positive clearance hole and thickness';
      case Receiving(size, depth, thickness, _, _):
        if (!(depth > 0) || !(thickness >= depth) || !Math.isFinite(depth + thickness))
          throw 'Receiving face "$id" needs a positive thread depth within its thickness';
        if (holes == null && (!(width > 0) || !(height > 0))) throw 'Receiving face "$id" drilled to suit needs a footprint';
        if (holes != null && holes.length == 0) throw 'Receiving face "$id" lists no holes';
        if (holes != null && size == null) throw 'Receiving face "$id" with tapped holes needs their thread size';
    }
    if (holes != null) for (hole in holes) if (!Math.isFinite(hole.x) || !Math.isFinite(hole.y)) throw 'Face "$id" has a non-finite hole';
  }

  /**
	 * A clamped face whose holes are not listed: the part is drilled to suit whatever it is bolted to. It lies over a footprint
	 * (`width` by `height`, centred on the origin) and meets every receiving face with tapped holes inside it, a carriage
	 * seat that takes several rail blocks; each pairing takes the receiving face's hole pattern (see `paired`). Its holes are
	 * never drawn: the derivation cuts them, with the recess where there is one.
	 */
  public static function clampedToSuit(id:String, frame:AssemblyFrame, width:Float, height:Float, clearance:Float, grip:Float, ?size:String,
      ?recess:BoltRecess, washer:Bool = false):BoltedFace return new BoltedFace(id, frame, Clamped(size, clearance, grip, recess), null, width,
      height, null, false, washer);

  /** A clamped face drilled to suit, with the holes one receiving face gave it. */
  public function paired(holes:Array<BoltHole>):BoltedFace return new BoltedFace(id, frame, role, holes, width, height, thicknessAt, drawn,
      washer, alternative, rated);

  public static function clamped(id:String, frame:AssemblyFrame, holes:Array<BoltHole>, clearance:Float, grip:Float,
      ?size:String, ?recess:BoltRecess, drawn:Bool = true, washer:Bool = false, ?alternative:String):BoltedFace return new BoltedFace(id, frame,
      Clamped(size, clearance, grip, recess), holes, 0, 0, null, drawn, washer, alternative);

  /** A plate the bolt passes through, with a nut and washer on its far side (`BoltedRole.Through`). */
  public static function through(id:String, frame:AssemblyFrame, holes:Array<BoltHole>, clearance:Float, thickness:Float, ?size:String,
      drawn:Bool = true):BoltedFace return new BoltedFace(id, frame, Through(size, clearance, thickness), holes, 0, 0, null, drawn);

  public static function tapped(id:String, frame:AssemblyFrame, holes:Array<BoltHole>, size:String, depth:Float, thickness:Float,
      through:Bool = false, ?material:ThreadMaterial, rated:Bool = false):BoltedFace return new BoltedFace(id, frame,
      Receiving(size, depth, thickness, through, material ?? Steel), holes, 0, 0, null, true, false, null, rated);

  public static function drilledToSuit(id:String, frame:AssemblyFrame, width:Float, height:Float, depth:Float, thickness:Float,
      ?material:ThreadMaterial, ?size:String, through:Bool = false, ?thicknessAt:(Float, Float) -> Float):BoltedFace return new BoltedFace(id, frame,
      Receiving(size, depth, thickness, through, material ?? Steel), null, width, height, thicknessAt);

  public function isClamped():Bool return switch role {
    case Clamped(_, _, _, _): true;
    case _: false;
  };

  /**
	 * A face frame at `origin` with +Z along `into` (into the part's material) and +X along `across`; Y follows as Z cross X.
	 * `into` and `across` must be perpendicular.
	 */
  public static function frameOf(origin:{x:Float, y:Float, z:Float}, into:{x:Float, y:Float, z:Float},
      across:{x:Float, y:Float, z:Float}):AssemblyFrame {
    var z = unit(into), x = unit(across);
    if (Math.abs(z.x * x.x + z.y * x.y + z.z * x.z) > 1e-9) throw "A bolted face's axes must be perpendicular";
    var y = {x: z.y * x.z - z.z * x.y, y: z.z * x.x - z.x * x.z, z: z.x * x.y - z.y * x.x};
    return AssemblyFrames.fromRotationMatrix(origin.x, origin.y, origin.z, [x.x, y.x, z.x, x.y, y.y, z.y, x.z, y.z, z.z]);
  }

  static function unit(v:{x:Float, y:Float, z:Float}):{x:Float, y:Float, z:Float} {
    var length = Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (!(length > 1e-9)) throw "A bolted face's axis has zero length";
    return {x: v.x / length, y: v.y / length, z: v.z / length};
  }

  /** Nominal diameters of engagement a thread in `material` needs. */
  public static function engagementFactor(material:ThreadMaterial):Float return switch material {
    case Steel: 1.0;
    case CastIron: 1.5;
    case Aluminium: 2.0;
  };
}

/**
 * A shoulder or key that locates a bolted part against a thrust in both directions: a step of the part that carries it, `height` proud of the
 * plane the other part seats on, that the other part butts against on each side (a rib either side of a block's base, or a rebate the base sits in).
 * `face` names the `BoltedFace` of this part that the key belongs to (the joint it locates); `length` is the key's extent across the thrust and
 * `thickness` the step's root, the material that carries it in shear, mm. `JointSlip` counts the least of its bearing (`height` x `length`) and its
 * root shear (`thickness` x `length`) as the capacity of a shear load, as it counts a dowel; the friction is then not counted.
 */
typedef LocatingKey = {face:String, height:Float, length:Float, thickness:Float}

/** A dowel hole position in a face frame, mm. */
typedef PinHole = BoltHole;

/**
 * One face of a part where it is located by dowel pins: a pattern of holes of `diameter` (the pin's nominal diameter), reamed H7 for a
 * m6 pin, `depth` deep into this part (the material behind the plane when `through`), in a face frame like `BoltedFace`'s (origin on
 * the mating plane, +Z into the part). Two pinned faces that meet with the same holes and diameter are one dowelled joint:
 * `JointFasteners` derives the pin (ISO 8734, the shortest standard length that engages `ENGAGEMENT_DIAMETERS` diameters in each part), cuts
 * the reamed bore into each part whose geometry does not draw it, and adds the pins to the BOM. `atAssembly` marks a bore the maker
 * leaves to be reamed through the pair once aligned (a catalogue part, whose geometry is not ours to cut, or a joint aligned in the build).
 */
class PinnedFace {
  public final id:String;
  public final frame:AssemblyFrame;
  public final holes:Array<PinHole>;
  public final diameter:Float;
  public final depth:Float;
  public final through:Bool;
  /** The part's own geometry already has these holes (or, a catalogue part, they are made at assembly and not modelled); the derivation cuts none. */
  public final drawn:Bool;
  public final fit:String;

  /** Diameters of engagement each part gives a dowel, at least. */
  public static inline var ENGAGEMENT_DIAMETERS:Float = 1.0;
  /** Air relief left at the bottom of a blind dowel hole, mm. */
  public static inline var RELIEF:Float = 1.0;

  public function new(id:String, frame:AssemblyFrame, holes:Array<PinHole>, diameter:Float, depth:Float, through:Bool = false,
      drawn:Bool = false, fit:String = "H7") {
    if (id == null || StringTools.trim(id) == "") throw "A pinned face needs an id";
    if (holes == null || holes.length == 0) throw 'Pinned face "$id" lists no holes';
    if (!(diameter > 0) || !(depth > 0) || !Math.isFinite(diameter + depth)) throw 'Pinned face "$id" needs a positive diameter and depth';
    for (hole in holes) if (!Math.isFinite(hole.x) || !Math.isFinite(hole.y)) throw 'Pinned face "$id" has a non-finite hole';
    this.id = id;
    this.frame = frame;
    this.holes = holes;
    this.diameter = diameter;
    this.depth = depth;
    this.through = through;
    this.drawn = drawn;
    this.fit = fit;
  }

  /** The holes given as points of the part's frame, which `frame` carries onto the face. */
  public static function atPoints(id:String, frame:AssemblyFrame, points:Array<{x:Float, y:Float, z:Float}>, diameter:Float, depth:Float,
      through:Bool = false, drawn:Bool = false, fit:String = "H7"):PinnedFace {
    var inverse = AssemblyFrames.inverse(frame);
    return new PinnedFace(id, frame, [for (p in points) {
      var at = AssemblyFrames.transformPoint(inverse, p.x, p.y, p.z);
      {x: at.x, y: at.y};
    }], diameter, depth, through, drawn, fit);
  }

  /** The depth a pin can use in this part: the hole less the relief of a blind one. */
  public function usable():Float return through ? depth : depth - RELIEF;
}
