package machinekit.structural;

import cadkit.modeling.Part;
import cadkit.modeling.Polygon;
import cadkit.threads.ThreadSpec;
import cadkit.threads.Threads;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.structural.LevelingFootSpec;
import machinekit.structural.LevelingFootSpec.LevelingFootCodec;
import machinekit.structural.LevelingFootSpec.FootStemEnvelope;

/** Shared nominal foot hardware: bought foot, pad, screw, removable plate or nut.
 * Saved dimensions are layout assumptions. Supplier geometry and load qualification remain pending.
 */
class LevelingFoot extends AssumedCatalogComponent {
  static var recipe:Null<ComponentType>;
  final spec:LevelingFootSpec;

  public static function parameters():Array<ComponentParameter> return [P.text(
    "part",
    LevelingFootCodec.encode(Foot(
      100,
      20,
      machinekit.standard.FastenerThreads.metric(20, 2.5),
      20,
      120,
      100,
      Nominal,
      NoCollar
    ))
  )];

  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType(
      "machinekit.structural.leveling-foot",
      parameters(),
      v -> new LevelingFoot(v)
    ).numbered(v -> partNumber(LevelingFootCodec.decode(v.token("part"))))
      .procured(_ -> Buy(null, null, null)).requiring(v -> requirementsFor(LevelingFootCodec.decode(v.token("part"))));
    return recipe;
  }

  static function requirementsFor(part:LevelingFootSpec):Array<machinekit.component.Procurement.ProcurementRequirement> {
    var req = machinekit.component.Procurement.ProcurementCodec.requirement;
    return switch part {
      case Foot(d, _, thread, bottom, top, _, _, _):
        [req("padDiameter", "mm", null, null, d), req("threadDiameter", "mm", null, null, thread.majorDiameter),
          req("threadPitch", "mm", null, null, thread.pitch), req("stemLength", "mm", top - bottom)];
      case Pad(d, h):
        [req("diameter", "mm", null, null, d), req("height", "mm", null, null, h)];
      case Screw(thread, length, _):
        [req("threadDiameter", "mm", null, null, thread.majorDiameter),
          req("threadPitch", "mm", null, null, thread.pitch), req("length", "mm", null, null, length)];
      case Plate(w, d, h, bore):
        [req("width", "mm", null, null, w), req("depth", "mm", null, null, d),
          req("height", "mm", null, null, h), req("bore", "mm", null, null, bore)];
      case Nut(profile, h, thread):
        var size = switch profile { case Round(d): d; case Hexagon(af): af; };
        var shape = switch profile { case Round(_): "round"; case Hexagon(_): "hexagon"; };
        [machinekit.component.Procurement.ProcurementCodec.choice("profile", shape),
          req("size", "mm", null, null, size), req("height", "mm", null, null, h),
          req("threadDiameter", "mm", null, null, thread.majorDiameter),
          req("threadPitch", "mm", null, null, thread.pitch)];
    };
  }

  static function partNumber(part:LevelingFootSpec):String return switch part {
    case Foot(d, _, thread, bottom, top, _, _, _):
      'FOOT-ASSUMED-${thread.designation}-D${Dimension.format(d)}-L${Dimension.format(top - bottom)}';
    case Pad(d, h):
      'FOOT-PAD-ASSUMED-${Dimension.format(d)}x${Dimension.format(h)}';
    case Screw(thread, length, _):
      'FOOT-SCREW-ASSUMED-${thread.designation}-L${Dimension.format(length)}';
    case Plate(w, d, h, bore):
      'FOOT-PLATE-ASSUMED-${Dimension.format(w)}x${Dimension.format(d)}x${Dimension.format(h)}-D${Dimension.format(bore)}';
    case Nut(profile, h, thread):
      var shape = switch profile {
        case Round(d): 'R${Dimension.format(d)}';
        case Hexagon(af): 'H${Dimension.format(af)}';
      };
      'FOOT-NUT-ASSUMED-${thread.designation}-${shape}x${Dimension.format(h)}';
  }

  public static function of(spec:LevelingFootSpec):LevelingFoot return new LevelingFoot(new ComponentValues().setToken(
    "part",
    LevelingFootCodec.encode(spec)
  ));

  override public function fastensByForm():Bool return switch spec {
    case Nut(_, _, _), Foot(_, _, _, _, _, _, _, _): true;
    default: false;
  };

  public function new(?overrides:ComponentValues) {
    var v = P.resolve(parameters(), overrides);
    var parsed = LevelingFootCodec.decode(v.token("part"));
    v.setToken("part", LevelingFootCodec.encode(parsed));
    super("leveling-foot", partLabel(parsed) + "; capacity unverified", "steel", parameters(), v);
    spec = parsed;
    var top = switch spec {
      case Foot(_, _, _, _, _, mount, _, _):
        mount;
      case Pad(_, height), Nut(_, height, _):
        height;
      case Screw(_, length, _):
        length;
      case Plate(_, _, height, _):
        height;
    };
    addConnector("floor", Mount, Solids.axial(0, 0, 0));
    addConnector("mount", Mount, Solids.axial(0, 0, specIsFoot() ? top : 0));
    addConnector("frame", Mount, Solids.axial(0, 0, top));
    var physicalTop = switch spec {
      case Foot(_, _, _, _, stemTop, _, _, _):
        stemTop;
      default:
        top;
    };
    addConnector("top", Mount, Solids.axial(0, 0, physicalTop));
  }

  static function partLabel(part:LevelingFootSpec):String return switch part {
    case Foot(d, _, thread, _, _, _, _, _):
      '${thread.designation} levelling foot, pad D$d';
    case Pad(d, h):
      'Levelling pad D$d x $h';
    case Screw(thread, length, _):
      '${thread.designation} levelling stud L$length';
    case Plate(w, d, h, bore):
      'Removable foot bearing plate $w x $d x $h, clearance D$bore';
    case Nut(profile, height, thread):
      var outside = switch profile {
        case Round(d):
          'round D$d';
        case Hexagon(af):
          'hexagon AF$af';
      };
      '${thread.designation} foot nut, $outside x $height';
  };

  public function specification():LevelingFootSpec return LevelingFootCodec.decode(parameterToken("part"));

  function specIsFoot():Bool return switch spec {
    case Foot(_, _, _, _, _, _, _, _):
      true;
    default:
      false;
  };

  override public function componentType():Null<ComponentType> return recipeType();

  static function radius(thread:ThreadSpec, envelope:FootStemEnvelope):Float return switch envelope {
    case Nominal:
      thread.majorDiameter / 2;
    case ThreadRoot:
      thread.externalMinorDiameter() / 2;
  };

  static function threadedCylinder(radius:Float, bottom:Float, top:Float, thread:ThreadSpec):Part {
    var raw = Solids.named(Part.cylinderSpan(radius, bottom, top), "thread");
    try {
      var result = new Part(Threads.attach(raw.shape, "thread:cyl.side", thread, AxisMinimum));
      raw.close();
      return result;
    } catch (error:Dynamic) {
      raw.close();
      throw error;
    }
  }

  override public function geometry(detail:ComponentDetail = Preview):Part return switch spec {
    case Foot(d, h, thread, bottom, top, _, envelope, collar):
      Solids.building([], owned -> {var pad = Part.cylinderSpan(d / 2, 0, h);
      owned.push(pad);
      var stem = threadedCylinder(radius(thread, envelope), bottom, top, thread);
      owned.push(stem);
      var pieces = [pad, stem];
      switch collar {
        case NoCollar:
        case RoundCollar(cd, ch, z):
          var ring = Part.cylinderSpan(cd / 2, z, z + ch);
          owned.push(ring);
          pieces.push(ring);
      }
      return Solids.union(pieces);
      });
    case Pad(d, h):
      Part.cylinderSpan(d / 2, 0, h);
    case Screw(thread, length, envelope):
      threadedCylinder(radius(thread, envelope), 0, length, thread);
    case Plate(w, d, h, bore):
      Solids.building([], owned -> {var blank = Part.box(w, d, h);
      owned.push(blank);
      var hole = Part.cylinderSpan(bore / 2, -1, h + 1);
      owned.push(hole);
      return Solids.cut(blank, [hole]);
      });
    case Nut(profile, height, thread):
      Solids.building([], owned -> {var blank = switch profile {
        case Round(d):
          Part.cylinderSpan(d / 2, 0, height);
        case Hexagon(af):
          Part.prism(Polygon.regular(6, af), 0, height);
      };
      owned.push(blank);
      var hole = threadedCylinder(thread.majorDiameter / 2, -1, height + 1, thread.mating());
      owned.push(hole);
      return Solids.cut(blank, [hole]);
      });
  };
}
