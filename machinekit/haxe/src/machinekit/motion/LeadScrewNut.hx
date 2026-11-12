package machinekit.motion;

import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentValue.*;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.Dimension;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.threads.Threads;
import cadkit.threads.ThreadedFace.ThreadStart;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.MountingFace;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import machinekit.component.Solids;
import machinekit.standard.ClearanceFit;
import machinekit.standard.SocketHeadCapScrew;

/** Explicit flange dimensions, mm, for catalogue nuts with independently specified geometry. */
typedef ScrewNutDimensions = {
  var bodyDiameter:Float;
  var bodyLength:Float;
  var flangeDiameter:Float;
  var flangeThickness:Float;
  var boltCircleDiameter:Float;
  var mountScrew:String;
  /** Flange start on the barrel; absent retains the legacy end flange. */
  var ?flangeOffset:Float;
}

/** ACME/trapezoidal lead screw nut: a flanged block with a bore matching the screw diameter and
 * a mounting bolt pattern on the flange face, for driving a carriage. The thread itself is
 * semantic (`LeadScrewThread` family, diameter, pitch, starts and hand), not modelled.
 * `travelPerRevolution()`/`rotationFor()` convert between screw rotation and nut travel; for a
 * right-hand thread, positive rotation about +Z moves the nut toward -Z.
 * CAD frame: axis along +Z, body from z=0 to z=bodyLength, flange from there to
 * z=bodyLength+flangeThickness. Connectors: `bore` (axis, mid-body) and `mount1`..`mountN` (on
 * the flange face), all with +Y along +Z.
 */
class LeadScrewNut extends MachineComponent {
  public final thread:LeadScrewThread;
  public final screwDiameter:Float;
  public final lead:Float;
  public final bodyDiameter:Float;
  public final bodyLength:Float;
  public final flangeDiameter:Float;
  public final flangeThickness:Float;
  public final flangeOffset:Float;
  public final overallLength:Float;
  public final boltCircleDiameter:Float;
  public final boltCount:Int;
  public final mountScrew:String;
  /** A barrel nut has no flange and is retained by its bracket. */
  public final flanged:Bool;
  public final kind:LeadScrewNutKind;
  final explicitDimensions:Bool;

  public function new(
    thread:LeadScrewThread,
    boltCount:Int = 4,
    flanged:Bool = true,
    kind:LeadScrewNutKind = AntiBacklash,
    ?dimensions:ScrewNutDimensions
  ) {
    if (kind == null) throw "Lead screw nut needs a kind";
    if (thread == null) throw "Lead screw nut needs a thread specification";
    var screwDiameter = thread.screwDiameter;
    var lead = thread.lead;
    if (flanged && boltCount < 3) throw "Lead screw nut needs at least 3 mounting bolts";
    // Proportioned on the common T8 nut (10.2 mm body, 16 mm bolt circle, 22 mm flange, M3).
    // A barrel nut fits a narrow carriage gap; a flanged body follows the common T8 proportions.
    var bodyDia = dimensions == null ? screwDiameter * (flanged ? 1.3 : 1.2) : dimensions.bodyDiameter;
    var bodyLen = dimensions == null ? screwDiameter * 2 : dimensions.bodyLength;
    var flangeThick = dimensions == null ? Math.max(3, screwDiameter * 0.3) : dimensions.flangeThickness;
    var offset = dimensions == null || dimensions.flangeOffset == null ? bodyLen : dimensions.flangeOffset;
    if (!Math.isFinite(offset) || offset < 0 || offset > bodyLen) throw "Nut flange must start on the barrel";
    var mountScrewSize = dimensions
      == null ? (screwDiameter <= 8 ? "M3" : screwDiameter <= 12 ? "M4" : "M5") : dimensions.mountScrew;
    var screw = SocketHeadCapScrew.catalog().get(mountScrewSize);
    // Holes keep 1 mm of material to the body and the screw heads 1 mm to the flange rim.
    var boltRadius = dimensions
      == null ? bodyDia / 2 + screw.clearanceMedium / 2 + 1 : dimensions.boltCircleDiameter / 2;
    if (flanged && 2 * boltRadius * Math.sin(Math.PI / boltCount)
      < screw.clearanceMedium + 1) throw "Lead screw nut bolt count leaves too little material between mounting holes";
    var flangeDia = dimensions == null ? 2 * Math.max(screwDiameter * 1.5,
      boltRadius + screw.headDiameter / 2 + 1) : dimensions.flangeDiameter;
    if (!(bodyDia > screwDiameter) || !(bodyLen > 0) || !(flangeThick > 0)
      || (flanged && (!(boltRadius > bodyDia / 2 + screw.clearanceMedium / 2) || !(flangeDia / 2 > boltRadius
      + screw.clearanceMedium / 2)))) throw "Nut dimensions must leave material around its bore and mounting holes";
    var diameterText = Dimension.format(screwDiameter), leadText = Dimension.format(lead);
    super(
      'LEADNUT-${thread.designation}${flanged ? "" : "-BARREL"}${kind == AntiBacklash ? "" : "-" + Std.string(kind)}',
      'Lead screw nut, ${thread.designation}, $leadText mm lead',
      "bronze"
    );
    if (Std.isExactType(this, LeadScrewNut)) {
      addFacet(machinekit.component.EngineeringMetadata.assumed(""));
      if (dimensions != null) addFacet(new machinekit.component.ProcurementIntent(Buy(null, null, null), [
        machinekit.component.Procurement.ProcurementCodec.choice("family", Std.string(thread.family)),
        machinekit.component.Procurement.ProcurementCodec.requirement("screwDiameter", "mm", null, null, screwDiameter),
        machinekit.component.Procurement.ProcurementCodec.requirement("pitch", "mm", null, null, thread.pitch),
        machinekit.component.Procurement.ProcurementCodec.requirement("starts", "1", null, null, thread.starts),
        machinekit.component.Procurement.ProcurementCodec.choice("hand", Std.string(thread.hand)),
        machinekit.component.Procurement.ProcurementCodec.choice("kind", Std.string(kind)),
        machinekit.component.Procurement.ProcurementCodec.requirement("bodyDiameter", "mm", null, null, dimensions.bodyDiameter),
        machinekit.component.Procurement.ProcurementCodec.requirement("bodyLength", "mm", null, null, dimensions.bodyLength)
      ]));
    }
    this.thread = thread;
    this.screwDiameter = screwDiameter;
    this.lead = lead;
    bodyDiameter = bodyDia;
    bodyLength = bodyLen;
    flangeDiameter = flanged ? flangeDia : bodyDia;
    flangeThickness = flanged ? flangeThick : 0;
    flangeOffset = offset;
    overallLength = Math.max(bodyLength, flanged ? flangeOffset + flangeThickness : bodyLength);
    boltCircleDiameter = flanged ? 2 * boltRadius : 0;
    this.boltCount = flanged ? boltCount : 0;
    this.flanged = flanged;
    this.kind = kind;
    explicitDimensions = dimensions != null;
    mountScrew = mountScrewSize;
    addConnector("bore", Axis, Solids.axial(0, 0, bodyLength / 2));
    addConnector("mountFace", Face, Solids.axial(0, 0, flangeOffset + flangeThickness));
    var i = 1;
    for (point in boltPattern()) addConnector(
      'mount${i++}',
      Mount,
      Solids.axial(point.x, point.y, flangeOffset + flangeThickness)
    );
  }

  /** Assumed reversal clearance, mm: plain bronze 0.15, preloaded 0.05, ball nut 0.01. */
  public function backlash():Float return switch kind {
    case PlainBronze:
      0.15;
    case AntiBacklash:
      0.05;
    case BallNut:
      0.01;
    case PreloadedBallNut:
      0;
  };

  /** Assumed running torque, N m, including nut preload and support bearing drag. */
  public function drag():Float return switch kind {
    case PlainBronze:
      0.01;
    case AntiBacklash:
      0.02;
    case BallNut:
      0.005;
    case PreloadedBallNut:
      0.02;
  };

  /**
	 * Assumed axial stiffness of the nut, N/mm (catalogue-typical rigidity of a 16 mm class ball nut: about 100 N/um unpreloaded, 300 N/um
	 * preloaded). A sliding nut is taken as rigid against the shaft (null): its thread stiffness is far above the screw's.
	 */
  public function axialStiffness():Null<Float> return switch kind {
    case BallNut:
      100000;
    case PreloadedBallNut:
      300000;
    default:
      null;
  };

  /** Sliding nuts use a greased steel/bronze friction of 0.1; ball nuts assume 90% efficiency. */
  public function efficiency():Float return (kind == BallNut
    || kind == PreloadedBallNut) ? LeadScrewThread.BALL_EFFICIENCY : thread.efficiency(0.1);

  static function parseKind(value:String):LeadScrewNutKind return switch value {
    case "PlainBronze":
      PlainBronze;
    case "AntiBacklash":
      AntiBacklash;
    case "BallNut":
      BallNut;
    case "PreloadedBallNut":
      PreloadedBallNut;
    default:
      throw 'Unknown lead screw nut kind "$value"';
  };

  /**
	 * The flange face a block seats this nut on, as a `MountingFace`: the flange's barrel-side face, with +Z into the block
	 * (against the nut's own +Z) and +Y reversed. Null for a barrel nut.
	 */
  override public function mountingFace():Null<MountingFace> {
    if (!flanged) return null;
    return new MountingFace(
      flangeDiameter,
      flangeDiameter,
      depth -> mountingCutout(depth),
      AssemblyFrames.fromRotationMatrix(0, 0, flangeOffset, [1, 0, 0, 0, -1, 0, 0, 0, -1])
    );
  }

  /**
	 * The flange's two faces, as alternatives: a bolted flange sits on a plate by its barrel-side face (the bolts tap into a
	 * block ahead of the barrel, head on the flange back) or by its back face (the bolts pass the plate, head on the barrel
	 * side). Clearance holes as drawn (`geometry`); the grip is the flange.
	 */
  override public function fasteningFaces():Array<machinekit.component.BoltedFace> {
    if (!flanged) return [];
    var screw = SocketHeadCapScrew.catalog().get(mountScrew);
    if (screw == null) throw 'No ${mountScrew} screw row for the nut flange';
    var pattern = boltPattern();
    var barrel = machinekit.component.BoltedFace.frameOf({x: 0, y: 0, z: flangeOffset}, {x: 0, y: 0, z: 1}, {x: 1, y: 0, z: 0});
    var back = machinekit.component.BoltedFace.frameOf({x: 0, y: 0, z: flangeOffset + flangeThickness}, {x: 0, y: 0, z: -1}, {x: 1, y: 0, z: 0});
    return [
      machinekit.component.BoltedFace.clamped("barrel", barrel, [for (p in pattern) {x: p.x, y: p.y}], screw.clearanceMedium, flangeThickness,
        mountScrew, null, true, false, "flange"),
      machinekit.component.BoltedFace.clamped("back", back, [for (p in pattern) {x: p.x, y: -p.y}], screw.clearanceMedium, flangeThickness,
        mountScrew, null, true, false, "flange")
    ];
  }

  /** Diameter of the core hole the block taps for the flange bolts. */
  public function mountingTapDrill():Float {
    var screw = SocketHeadCapScrew.catalog().get(mountScrew);
    if (screw == null) throw 'No ${mountScrew} screw row for the nut flange';
    return screw.tapDrill;
  }

  /** The flange bolt centres in the face frame (`mountingFace().pose`): the nut's pattern with +Y reversed. */
  public function mountingHoles():Array<{x:Float, y:Float}> return [for (point in boltPattern()) {x: point.x, y: -point.y}];

  /**
	 * Tool for a block ahead of the flange face (face frame z = 0..`depth`): the barrel's bore, with `clearance` on its
	 * radius, and the tap-drill holes for the flange bolts. The block is cut with it before the nut is bolted on.
	 */
  public function mountingCutout(depth:Float, clearance:Float = 0.2):Part {
    if (!flanged || !(depth > 0) || !(clearance >= 0)) throw "A nut cut-out needs a flanged nut, a positive depth and a non-negative clearance";
    var tools = [Solids.named(Part.cylinderSpan(bodyDiameter / 2 + clearance, 0, depth), "bore")];
    var index = 1;
    for (point in mountingHoles()) tools.push(Solids.named(
      Part.cylinderSpan(mountingTapDrill() / 2, 0, depth, point.x, point.y),
      "bolt" + index++
    ));
    return Solids.union(tools);
  }

  /** Mount bolt centres on the flange face, counter-clockwise from angle 0. */
  public function boltPattern():Array<{x:Float, y:Float}> {
    var r = boltCircleDiameter / 2;
    return [for (i in 0...boltCount) {
      var angle = 2 * Math.PI * i / boltCount;
      {
        x:r * Math.cos(angle), y:r * Math.sin(angle)
      };
    }
    ];
  }

  /** Signed axial travel for one positive screw revolution. */
  public function travelPerRevolution():Float return thread.signedLead();

  /** Screw rotation, in radians, needed to travel `distance`. */
  public function rotationFor(distance:Float):Float return distance / thread.signedLead() * 2 * Math.PI;

  /** Screw that fits the nut's mounting holes. */
  public function mountScrewPart(length:Float):SocketHeadCapScrew return SocketHeadCapScrew.metric(mountScrew, length);

  override public function hasGeometry():Bool return true;

  /** The barrel and its flange about the screw axis, with the threaded bore; the preview adds the flange's bolt holes. */
  override public function record(b:cadkit.parametric.recording.DocumentBuilder, detail:ComponentDetail):Null<cadkit.parametric.Feature> {
    machinekit.component.PartRecordings.recipeInputs(this, b);
    var axis = new Vector(0, 0, 0), up = Vector.Z();
    var body = b.label(b.cylinderAlong(axis, up, bodyDiameter / 2, 0, bodyLength), "Barrel");
    if (!flanged) return b.label(b.cutAll(body, [threadedBore(b, bodyLength)], ["threaded bore"]), "Threaded bore");
    var nut = b.label(b.unionAll([body, b.label(b.cylinderAlong(axis, up, flangeDiameter / 2, flangeOffset, flangeOffset + flangeThickness), "Flange")]),
      "Barrel and flange");
    var tools = [threadedBore(b, overallLength)], labels = ["threaded bore"];
    if (detail != Envelope) {
      var radius = mountScrewPart(10).clearanceDiameter(Medium) / 2;
      for (k => point in boltPattern()) {
        tools.push(b.cylinderAlong(new Vector(point.x, point.y, 0), up, radius, flangeOffset - 0.1, flangeOffset + flangeThickness + 0.1));
        labels.push('bolt ${k + 1}');
      }
    }
    return b.label(b.cutAll(nut, tools, labels), detail == Envelope ? "Threaded bore" : "Threaded bore and bolt holes");
  }

  /** The bore along the axis over `length`, carrying the nut's internal thread. */
  function threadedBore(b:cadkit.parametric.recording.DocumentBuilder, length:Float):cadkit.parametric.Feature
    return b.thread(b.cylinderAlong(new Vector(0, 0, 0), Vector.Z(), screwDiameter / 2, -0.1, length + 0.1), "cyl.side", thread.threadSpec(true),
      AxisMinimum);

  private static final recipeTypeCache = new machinekit.cache.Once<ComponentType>();

  public static function recipeType():ComponentType {
    return recipeTypeCache.get(() -> new ComponentType(
      "machinekit.motion.lead-screw-nut",
      ComponentRecipeSupport.threadParameters().concat([
        ComponentRecipeSupport.count("boltCount", 4),
        ComponentRecipeSupport.flag("flanged", true),
        ComponentRecipeSupport.choice(
          "kind",
          ["PlainBronze", "AntiBacklash", "BallNut", "PreloadedBallNut"],
          "AntiBacklash"
        )
      ]),
      v -> new LeadScrewNut(
        ComponentRecipeSupport.thread(v),
        v.integer("boltCount"),
        v.boolean("flanged"),
        parseKind(v.token("kind"))
      ),
      false,
      true
    ).numbered(v -> {
      var kind = switch v.token("kind") {
        case "PlainBronze": "PLAIN";
        case "AntiBacklash": "ANTI";
        case "BallNut": "BALL";
        case "PreloadedBallNut": "PRELOAD";
        case _: throw "Unknown lead screw nut kind";
      };
      var material = v.token("material");
      return 'NUT-${ComponentRecipeSupport.threadPartNumber(v)}-$kind-${v.integer("boltCount")}'
        + (v.boolean("flanged") ? "F" : "N")
        + (material == "bronze" ? "" : material == "steel" ? "-STEEL"
          : "-" + materia.project.MaterialLibrary.fromSpec(material).toUpperCase());
    }).procured(_ -> Buy(null, null, null))
      .requiringInputs(["family", "screwDiameter", "pitch", "starts", "hand", "kind", "boltCount", "flanged"]));
  }

  override public function componentType():Null<ComponentType> return Std.isExactType(this,
    LeadScrewNut) && !explicitDimensions ? recipeType() : null;

  override public function values():ComponentValues {
    return ComponentRecipeSupport.threadValues(this.thread).setInteger("boltCount", this.boltCount).setBoolean(
      "flanged",
      flanged
    ).setToken(
      "kind",
      Std.string(kind)
    ).setToken(
      "material",
      materialSpec()
    );
  }

}
