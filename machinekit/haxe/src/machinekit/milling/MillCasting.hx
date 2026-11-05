package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.BoltedFace;
import machinekit.component.BoltedFace.PinnedFace;
import machinekit.component.BoltedFace.ThreadMaterial.CastIron;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import materia.assembly.AssemblyFrames;

/** The five cast bodies of the bench mill; origins are centred on their bottom faces. */
enum MillCastingKind {
  Base;
  Column;
  Saddle;
  Table;
  Head;
}

/**
 * A steel boss cast into the column's +x side wall, the seat of a Z screw support's foot: a pocket in the wall that the boss fills,
 * from its `back` face (x) out to the wall's face (a boss stands proud of the wall by what its foot plane needs), `width` across and
 * `length` along the column, centred at `y` and `z` in the column's frame (origin at the middle of its bottom face). The pocket runs on
 * out through the column's front face: the boss's foot stands partly in front of the column, beside the head.
 */
typedef ColumnBoss = {y:Float, z:Float, width:Float, length:Float, back:Float};

/** Machined cast-iron bodies with rail seats, screw clearances and mounting holes.
 * Dimensions describe this example's designed bodies, rather than a vendor catalogue.
 */
class MillCasting extends MachineComponent {
  public final kind:MillCastingKind;
  public final width:Float;
  public final depth:Float;
  public final height:Float;
  public final railSpacing:Float;
  public final screwHeight:Float;
  /** The column's cast-in bosses (empty for the other bodies). */
  public final bosses:Array<ColumnBoss>;
  /** Lower Z drive clearance for the canonical cover seats, mm. */
  public static inline var COVER_Z_SHORTENING:Float = 180;
  public static inline var SLOT_WIDTH:Float = 10;
  public static inline var SLOT_SPACING:Float = 40;

  public function new(kind:MillCastingKind, ?bosses:Array<ColumnBoss>) {
    super('MILL-${Std.string(kind).toUpperCase()}', 'Bench mill ${Std.string(kind).toLowerCase()}', "cast iron");
    if (bosses != null && bosses.length > 0 && kind != Column) throw "Only the column carries cast-in bosses";
    this.bosses = bosses == null ? [] : bosses.copy();
    addFacet(new machinekit.component.EngineeringMetadata(new ComponentValues()
      .setToken("ratingsBasis", "Stated").setToken("errorsBasis", "Assumed").setToken("source", "")));
    this.kind = kind;
    var dims = dimensions(kind);
    width = dims[0];
    depth = dims[1];
    height = dims[2];
    railSpacing = dims[3];
    screwHeight = height / 2;
    addConnector("base", Mount, Solids.axial(0, 0, 0));
    addConnector("top", Face, Solids.axial(0, 0, height));
    addConnector("nut", Mount, AssemblyFrames.alongY(0, 0, screwHeight, 0, 1, 0));
    addConnector("motor", Mount, AssemblyFrames.alongY(0, -depth / 2, screwHeight, 0, -1, 0));
    for (side in [-1, 1]) {
      var name = side < 0 ? "Left" : "Right";
      if (kind == Column) addConnector(
        'rail$name',
        Mount,
        AssemblyFrames.alongY(side * railSpacing / 2, -depth / 2, 0, 0, -1, 0)
      );
      else if (kind == Saddle) addConnector('rail$name', Mount, Solids.axial(0, side * railSpacing / 2, height));
      else addConnector('rail$name', Mount, Solids.axial(side * railSpacing / 2, 0, height));
    }
    if (kind == Base) addConnector("column", Mount, Solids.axial(0, depth / 2 - new MillColumn().depth / 2, height));
    if (kind == Head) addConnector("spindle",
      Axis, Solids.axial(0, -depth / 2 + SpindleCartridge.DIAMETER / 2 + 10, 0));
  }

  static function dimensions(kind:MillCastingKind):Array<Float> return switch kind {
    case Base:
      [520.0, 550, 80, 140];
    case Column:
      [180.0, 120, 670, 110];
      // The 50 mm nut-clearance bore leaves a continuous lower web and upper rail-side webs.
    case Saddle:
      [450.0, 130, 70, 90];
    case Table:
      [400.0, 130, 40, 90];
    case Head:
      [180.0, 220, 130, 110];
  };

  /** The material left under a carriage block's screw heads, mm: the grip, with one nominal diameter of thread engagement. */
  public static inline var BLOCK_SOLE:Float = 12;
  /** The carriage block the bench mill rides on (all three axes). */
  public static inline var BLOCK:String = "HGH15CA";

  /** The deepest rail screw thread a pad is tapped, mm. */
  public static inline var SEAT_TAP_DEPTH:Float = 20;

  /**
	 * The rail pads, where the rails are bolted down: the pads are drilled and tapped to suit the rails, in the wall behind
	 * them (the base's top skin, the saddle's top above the nut bore, the column's front wall).
	 */
  override public function fasteningFaces():Array<BoltedFace> {
    var faces:Array<BoltedFace> = [];
    if (kind == Saddle || kind == Table || kind == Head) blockSeats(faces);
    coverFaces(faces);
    if (kind != Base && kind != Saddle && kind != Column) return faces;
    var nut = new machinekit.motion.BallNut();
    var wall = switch kind {
      case Base: 21.0;
      case Column: 20.0;
      default: height;
    };
    for (side in [-1, 1]) {
      var name = side < 0 ? "railLeft" : "railRight";
      faces.push(switch kind {
        case Base:
          BoltedFace.drilledToSuit(name, BoltedFace.frameOf({x: side * railSpacing / 2, y: 0, z: height}, {x: 0, y: 0, z: -1}, {x: 1, y: 0, z: 0}),
            30, depth, Math.min(SEAT_TAP_DEPTH, wall), wall, CastIron);
        case Saddle:
          // The nut bore runs under the pad at x = 0: the wall above it is thin only where the bore is.
          var radius = nut.flangeDiameter / 2 + 1;
          BoltedFace.drilledToSuit(name, BoltedFace.frameOf({x: 0, y: side * railSpacing / 2, z: height}, {x: 0, y: 0, z: -1}, {x: 1, y: 0, z: 0}),
            width, 30, SEAT_TAP_DEPTH, height, CastIron, null, false,
            (u, _) -> Math.abs(u) < radius ? height - screwHeight - Math.sqrt(radius * radius - u * u) : height);
        default:
          BoltedFace.drilledToSuit(name, BoltedFace.frameOf({x: side * railSpacing / 2, y: -depth / 2, z: height / 2}, {x: 0, y: 1, z: 0}, {x: 1, y: 0, z: 0}),
            30, height, Math.min(SEAT_TAP_DEPTH, wall), wall, CastIron);
      });
    }
    return faces;
  }

  /**
   * The faces the moving flanges of the way covers are bolted to: the saddle's front and rear faces (Y covers; the flange legs' lips at
   * either side of the nut bore) and the table's two end faces (X covers; the roof lip along the top). Drilled and tapped to suit whichever
   * flange is bolted there.
   */
  function coverFaces(faces:Array<BoltedFace>):Void {
    // Independent receiving stock: the actual base skins and casting side walls, not artificial mating planes on the mounts.
    if (kind == Base) {
      faces.push(BoltedFace.drilledToSuit("coverFrontMount",
        BoltedFace.frameOf({x: 0, y: -depth / 2, z: height - 20}, {x: 0, y: 1, z: 0}, {x: 1, y: 0, z: 0}),
        264, 32, 12, 20, CastIron));
      var columnFront = depth / 2 - dimensions(Column)[1];
      faces.push(BoltedFace.drilledToSuit("coverRearMount",
        BoltedFace.frameOf({x: 0, y: columnFront - 11, z: height - BODY_PAD}, {x: 0, y: 0, z: -1}, {x: 1, y: 0, z: 0}),
        264, 28, 12, 21 - BODY_PAD, CastIron));
    }
    if (kind == Column || kind == Head) faces.push(BoltedFace.drilledToSuit("coverMount",
      BoltedFace.frameOf({x: width / 2, y: 0, z: height / 2}, {x: -1, y: 0, z: 0}, {x: 0, y: 1, z: 0}),
      depth, height, 12, 20, CastIron));

    if (kind == Saddle) for (side in [-1, 1]) faces.push(BoltedFace.drilledToSuit(
      side < 0 ? "coverFront" : "coverRear",
      BoltedFace.frameOf({x: 0, y: side * depth / 2, z: height / 2}, {x: 0, y: -side, z: 0}, {x: 1, y: 0, z: 0}),
      340, height, COVER_TAP_DEPTH, 40, CastIron
    ));
    if (kind == Table) for (side in [-1, 1]) faces.push(BoltedFace.drilledToSuit(
      side < 0 ? "coverLeft" : "coverRight",
      BoltedFace.frameOf({x: side * width / 2, y: 0, z: height / 2}, {x: -side, y: 0, z: 0}, {x: 0, y: 1, z: 0}),
      depth, height, COVER_TAP_DEPTH, 30, CastIron
    ));
  }

  /** Drawing datums of the cover mounts' two 4 H7 locating bores per seat. Unmatched or shifted mates are rejected by F1. */
  override public function pinnedFaces():Array<PinnedFace> {
    var pair = (half:Float) -> [{x: -half, y: 0.0}, {x: half, y: 0.0}];
    if (kind == Base) return [
      new PinnedFace("coverFrontMount", BoltedFace.frameOf({x: 0, y: -depth / 2, z: height - 26},
        {x: 0, y: 1, z: 0}, {x: 1, y: 0, z: 0}), pair(110), 4, 7),
      new PinnedFace("coverRearMount", BoltedFace.frameOf({x: 0, y: depth / 2 - dimensions(Column)[1] - 23, z: height - BODY_PAD},
        {x: 0, y: 0, z: -1}, {x: 1, y: 0, z: 0}), pair(110), 4, 7)
    ];
    var block = machinekit.motion.LinearRailBlock.metric(BLOCK).spec, nut = new machinekit.motion.BallNut();
    if (kind == Column) {
      var bk = machinekit.motion.ScrewSupportUnit.bk12();
      var topWall = height + block.railEndMargin - machinekit.motion.BallScrew.INPUT_JOURNAL_LENGTH + bk.length / 2
        + Math.max(bk.length, bk.spec.longitudinalPitch + 16) / 2 + 1;
      return [
        new PinnedFace("coverTopMount", BoltedFace.frameOf({x: width / 2, y: -depth / 2 - block.blockHeight / 2 + 91, z: topWall - 24},
          {x: -1, y: 0, z: 0}, {x: 0, y: 1, z: 0}), pair(12), 4, 7),
        new PinnedFace("coverBottomMount", BoltedFace.frameOf({x: width / 2, y: -depth / 2 - block.blockHeight / 2 + 72,
          z: block.railEndMargin + COVER_Z_SHORTENING - 4 - 8},
          {x: -1, y: 0, z: 0}, {x: 0, y: 1, z: 0}), pair(12), 4, 7)
      ];
    }
    if (kind == Head) {
      var y = depth / 2 + block.blockHeight / 2 - 72;
      return [
        new PinnedFace("coverTopMount", BoltedFace.frameOf({x: width / 2, y: y, z: height / 2 + nut.bodyLength / 2 + 14 - 8},
          {x: -1, y: 0, z: 0}, {x: 0, y: 1, z: 0}), pair(12), 4, 7),
        new PinnedFace("coverBottomMount", BoltedFace.frameOf({x: width / 2, y: y,
          z: height / 2 - nut.bodyLength / 2 - nut.flangeThickness - 14},
          {x: -1, y: 0, z: 0}, {x: 0, y: 1, z: 0}), pair(12), 4, 7)
      ];
    }
    return [];
  }

  /**
   * The base's chip gutters under the Y covers: the strip between the screw-support pad and each rail pad, `GUTTER_WIDTH` wide at `GUTTER_CENTRE`
   * from the middle, falling from the body top at `GUTTER_REAR` to `GUTTER_FALL` of its length lower at `GUTTER_FRONT`, where the drain hole
   * of `DRAIN_DIAMETER` goes through the top skin into the hollow base.
   */
  public static inline var GUTTER_CENTRE:Float = 42.5;
  public static inline var GUTTER_WIDTH:Float = 21;
  public static inline var GUTTER_REAR:Float = 122;
  public static inline var GUTTER_FRONT:Float = -248;
  public static inline var GUTTER_FALL:Float = 0.015;
  public static inline var DRAIN_DIAMETER:Float = 14;
  /** The body top of the base (its rail pads stand `4` mm above it), mm. */
  public static inline var BODY_PAD:Float = 4;

  /** The cut that makes the gutter on `side`: a wedge along the base, the sloped floor and the body top above it. */
  function gutter(side:Int):Part {
    var top = height - BODY_PAD, drop = (GUTTER_REAR - GUTTER_FRONT) * GUTTER_FALL;
    // The wedge is drawn in (y, z) and extruded across x: local x is y, local y is z, local z is x.
    var wedge = Part.prism([
      new Vector(GUTTER_REAR, top + 1), new Vector(GUTTER_FRONT - 8, top + 1), new Vector(GUTTER_FRONT - 8, top - drop - GUTTER_FALL * 8), new Vector(GUTTER_REAR, top)
    ], 0, GUTTER_WIDTH);
    var frame = AssemblyFrames.fromRotationMatrix(side * GUTTER_CENTRE - GUTTER_WIDTH / 2, 0, 0, [0, 0, 1, 1, 0, 0, 0, 1, 0]);
    var placed = machinekit.assembly.PosedParts.moved(wedge, frame);
    wedge.close();
    return placed;
  }

  /** The column's bay for the Y screw's rear bearing: half its width, depth into the front face and height from the base, mm. */
  public static inline var COVER_BAY_HALF_WIDTH:Float = 40;
  public static inline var COVER_BAY_DEPTH:Float = 10;
  public static inline var COVER_BAY_HEIGHT:Float = 92;

  /** The deepest thread tapped in a casting for a cover flange's screws, mm. */
  public static inline var COVER_TAP_DEPTH:Float = 12;

  /**
	 * Where the carriage blocks are bolted to this casting: the catalogue block is tapped from its top only, so the screws come
	 * through the casting from the side away from the guide, through clearance holes and counterbored pockets that leave
	 * `BLOCK_SOLE` of material under the heads (the casting is drilled and counterbored to suit the blocks; the pocket opens on
	 * the casting's far face and the assembly checks that it is clear). One seat per guide rail, as long as the casting.
	 * The pocket's diameter is the head and two millimetres for the driver (assumed).
	 */
  function blockSeats(faces:Array<BoltedFace>):Void {
    var block = machinekit.motion.LinearRailBlock.metric(BLOCK);
    var screw = machinekit.standard.SocketHeadCapScrew.catalog().get(block.spec.blockMountThread);
    var seatWidth = block.spec.blockHoleSpacingAcross + 4 * screw.diameter;
    var guide = dimensions(switch kind {
      case Saddle:
        Base;
      case Table:
        Saddle;
      default:
        Column;
    })[3];
    // The wall the pocket is cut through, from the seat to the far face.
    var thickness = kind == Head ? depth : height;
    var recess = {diameter: screw.headDiameter + 2, depth: thickness - BLOCK_SOLE};
    for (side in [-1, 1]) {
      var name = side < 0 ? "blockLeft" : "blockRight";
      var at = side * guide / 2;
      var along = switch kind {
        case Saddle:
          depth;
        case Table:
          width;
        default:
          height;
      };
      var frame = switch kind {
        case Saddle:
          BoltedFace.frameOf({x: at, y: 0, z: 0}, {x: 0, y: 0, z: 1}, {x: 1, y: 0, z: 0});
        case Table:
          BoltedFace.frameOf({x: 0, y: at, z: 0}, {x: 0, y: 0, z: 1}, {x: 0, y: -1, z: 0});
        default:
          BoltedFace.frameOf({x: at, y: depth / 2, z: height / 2}, {x: 0, y: -1, z: 0}, {x: 1, y: 0, z: 0});
      };
      faces.push(BoltedFace.clampedToSuit(name, frame, seatWidth, along, screw.clearanceMedium, BLOCK_SOLE, block.spec.blockMountThread, recess));
    }
  }

  override public function hasGeometry():Bool return true;
  override public function machinable():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part return detail == Envelope ? drawnBody(detail) : machined(drawnBody(detail));

  function drawnBody(detail:ComponentDetail):Part {
    var body:Part;
    var pad = 4.0;
    if (kind == Base || kind == Saddle) {
      var pieces = [Solids.named(Part.box(width, depth, height - pad), "casting")];
      for (side in [-1, 1]) {
        var railPad = kind == Saddle ? Part.box(width, 30, pad).translated(new Vector(
          0,
          side * railSpacing / 2,
          height - pad
        )) : Part.box(
          30,
          depth,
          pad
        ).translated(new Vector(
          side * railSpacing / 2,
          0,
          height - pad
        ));
        pieces.push(Solids.named(railPad, side < 0 ? "rail-pad-left" : "rail-pad-right"));
      }
      if (kind == Base) {
        var column = new MillColumn();
        pieces.push(Solids.named(
          Part.box(column.width,
          column.depth, pad).translated(new Vector(0, depth / 2 - column.depth / 2, height - pad)),
          "column-pad"
        ));
        pieces.push(Solids.named(
          Part.box(machinekit.motion.ScrewSupportUnit.bk12().width, depth, pad).translated(new Vector(
            0,
            0,
            height - pad
          )),
          "screw-support-pad"
        ));
      }
      body = Solids.union(pieces);
    } else if (kind == Column) {
      var pieces = [Solids.named(Part.box(width,
        depth - pad, height).translated(new Vector(0, pad / 2, 0)), "casting")];
      for (side in [-1, 1]) pieces.push(Solids.named(
        Part.box(30, pad, height).translated(new Vector(side * railSpacing / 2, -depth / 2 + pad / 2, 0)),
        side < 0 ? "rail-pad-left" : "rail-pad-right"
      ));
      body = Solids.union(pieces);
    } else body = Solids.named(Part.box(width, depth, height), "casting");
    if (detail == Envelope) return body;
    var tools:Array<Part> = [];
    if (kind == Base) tools.push(Solids.named(
      Part.box(width - 50, depth - 50, height - 20).translated(new Vector(0, 0, -1)),
      "underside-core"
    ));
    if (kind == Column) tools.push(Solids.named(
      Part.box(width - 40, depth - 40, height - 50).translated(new Vector(0, 0, 25)),
      "column-core"
    ));
    // The pockets the cast-in steel bosses fill; the boss's face stands 1 mm proud of the wall.
    for (i in 0...bosses.length) {
      var boss = bosses[i], face = width / 2 + 1;
      tools.push(Solids.named(
        Part.box(face - boss.back, boss.width, boss.length).translated(new Vector((boss.back + face) / 2, boss.y, boss.z - boss.length / 2)),
        'z-boss-pocket${i + 1}'
      ));
    }
    // The chip gutters of the Y covers: either side of the screw-support pad, the base's top falls to a drain hole at the front.
    if (kind == Base) for (side in [-1, 1]) {
      tools.push(Solids.named(gutter(side), side < 0 ? "chip-gutter-left" : "chip-gutter-right"));
      tools.push(Solids.named(
        Part.cylinderSpan(DRAIN_DIAMETER / 2, height - 30, height + 1, side * GUTTER_CENTRE, GUTTER_FRONT),
        side < 0 ? "chip-drain-left" : "chip-drain-right"
      ));
    }
    // The bay for the Y screw's rear bearing: its housing and stand stand back into the column's lower front, a little wider than they are.
    if (kind == Column) tools.push(Solids.named(
      Part.box(2 * COVER_BAY_HALF_WIDTH, COVER_BAY_DEPTH + 1, COVER_BAY_HEIGHT).translated(new Vector(0, -depth / 2 + (COVER_BAY_DEPTH - 1) / 2, -1)),
      "y-bearing-bay"
    ));
    // Clearance for the screw along the body; the rail pads remain above it.
    if (kind == Saddle) tools.push(Solids.named(
      Part.cylinderAlongY(
        new machinekit.motion.BallNut().flangeDiameter / 2 + 1,
        -depth / 2 - 1,
        depth / 2 + 1,
        0,
        screwHeight
      ),
      "screw-bore"
    ));
    // The flanged X nut crosses the rail gap; recess both adjacent faces.
    if (kind == Saddle || kind == Table) {
      var nut = new machinekit.motion.BallNut();
      var block = machinekit.motion.LinearRailBlock.metric("HGH15CA");
      var recess = (nut.flangeDiameter + 2) / 2 - block.spec.blockHeight / 2;
      tools.push(Solids.named(
        Part.box(width + 2, nut.flangeDiameter + 2, recess + 1).translated(new Vector(
          0,
          0,
          kind == Saddle ? height - recess : -1
        )),
        "x-nut-clearance"
      ));
    }
    if (kind == Head) tools.push(Solids.named(
      Part.cylinderSpan(
        SpindleCartridge.DIAMETER / 2,
        -1,
        height + 1,
        0,
        -depth / 2 + SpindleCartridge.DIAMETER / 2 + 10
      ),
      "spindle-bore"
    ));
    if (kind == Table) for (i in 0...3) {
      var y = (i - 1) * SLOT_SPACING;
      tools.push(Solids.named(
        Part.box(width + 2, SLOT_WIDTH, 9).translated(new Vector(0, y, height - 8)),
        'slot${i + 1}-mouth'
      ));
      tools.push(Solids.named(Part.box(width + 2,
        18, 6).translated(new Vector(0, y, height - 14)), 'slot${i + 1}-foot'));
    }
    for (sx in [-1, 1]) for (sy in [-1, 1]) tools.push(Solids.named(
      Part.cylinderSpan(4.5, -1, height + 1, sx * (width / 2 - 20), sy * (depth / 2 - 20)),
      'mount-${sx < 0 ? "left" : "right"}-${sy < 0 ? "front" : "back"}'
    ));
    return Solids.cut(body, tools);
  }

  static function parse(value:String):MillCastingKind return switch value {
    case "Base":
      Base;
    case "Column":
      Column;
    case "Saddle":
      Saddle;
    case "Table":
      Table;
    case "Head":
      Head;
    default:
      throw 'Unknown mill casting "$value"';
  };
  static var castingRecipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if (castingRecipe == null) castingRecipe = new ComponentType(
      "machinekit.milling.casting",
      [ComponentRecipeSupport.choice("kind", ["Base", "Column", "Saddle", "Table", "Head"], "Base"), ComponentRecipeSupport.optionalText("bosses")],
      v -> new MillCasting(parse(v.token("kind")), bossesOf(v.optionalToken("bosses")))
    ).versioned(3).numbered(v -> 'MILL-${v.token("kind").toUpperCase()}')
      .procured(_ -> Make("casting and CNC machining"));
    return castingRecipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  static function bossesOf(text:Null<String>):Array<ColumnBoss> {
    if (text == null) return [];
    var raw:Array<Dynamic> = haxe.Json.parse(text);
    return [for (b in raw) {y: (b.y : Float), z: (b.z : Float), width: (b.width : Float), length: (b.length : Float), back: (b.back : Float)}];
  }

  override public function values():ComponentValues {
    var out = new ComponentValues().setToken("kind", Std.string(kind)).setToken("material", materialSpec());
    if (bosses.length == 0) out.setUnset("bosses");
    else out.setToken("bosses", haxe.Json.stringify(bosses));
    return out;
  }
}

class MillBase extends MillCasting {
  public function new() super(Base);
}
class MillColumn extends MillCasting {
  public function new(?bosses:Array<ColumnBoss>) super(Column, bosses);
}
class MillSaddle extends MillCasting {
  public function new() super(Saddle);
}
class MillTable extends MillCasting {
  public function new() super(Table);
}
class MillHead extends MillCasting {
  public function new() super(Head);
}
