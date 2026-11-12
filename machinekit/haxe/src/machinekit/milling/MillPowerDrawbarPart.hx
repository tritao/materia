package machinekit.milling;

import cadkit.modeling.Plane;
import cadkit.parametric.Feature;
import cadkit.parametric.recording.DocumentBuilder;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Separately itemised study hardware, in the cartridge gauge frame: the rotor, drawbar, disc springs, gripper and
 * pull stud, the tool holder, and the compact release cylinder (MillDrawbarLayout) on its reaction frame. Spring
 * preload, gripper details and cylinder pressure/force remain selection inputs. */
class MillPowerDrawbarPart extends MachineComponent {
  static final recipe = new machinekit.cache.Once<ComponentType>();
  static final roles = [
    "drawbar",
    "springs",
    "gripper",
    "pullStud",
    "holder",
    "reactionFrame",
    "cylinder",
    "pistonRod",
    "rotor",
    "piston"
  ];
  public static function parameters():Array<ComponentParameter> return [P.text("role", "drawbar")];
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.milling.power-drawbar-study-part",
      parameters(),
      v -> new MillPowerDrawbarPart(v)
    ).numbered(v -> 'BT30-STUDY-${v.token("role").toUpperCase()}'));
  }
  public function new(?values:ComponentValues) {
    var v = P.resolve(parameters(), values);
    if (roles.indexOf(v.token("role")) < 0) throw "Unknown drawbar study part";
    super(
      'BT30-STUDY-${v.token("role")}',
      'BT30 ${v.token("role")} — design assumption',
      v.token("role") == "cylinder" ? "aluminium 6061" : "steel",
      false,
      v
    );
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    if (v.token("role") == "holder") addConnector("tool", Axis, Solids.axial(0, 0, -35));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function hasGeometry():Bool return true;

  /** A cylinder of radius `r` up the Z axis through (x, y), from z = lo to hi. */
  static function post(b:DocumentBuilder, r:Float, lo:Float, hi:Float, x:Float = 0, y:Float = 0):Feature
    return b.cylinderAlong(new Vector(x, y, 0), Vector.Z(), r, lo, hi);

  /** A tube of outer radius `r` and bore `bore` on the Z axis, from z = lo to hi. */
  static function ring(b:DocumentBuilder, r:Float, bore:Float, lo:Float, hi:Float, name:String):Feature
    return b.label(b.cutAll(b.label(post(b, r, lo, hi), name), [post(b, bore, lo - 1, hi + 1)], [name + " bore"]), name);

  /** A plate `w` x `d` centred on the Z axis, from z = lo to hi. */
  static function plate(b:DocumentBuilder, w:Float, d:Float, lo:Float, hi:Float):Feature
    return b.boxBetween(-w / 2, w / 2, -d / 2, d / 2, lo, hi);

  /** A solid of revolution about Z through the (radius, z) section `points`. */
  static function turned(b:DocumentBuilder, points:Array<Array<Float>>, name:String):Feature
    return b.label(b.revolve(b.polygon([for (p in points) new Vector(p[0], p[1], 0)], Plane.XZ()), new Vector(0, 0, 0), Vector.Z(), Math.PI * 2), name);

  override public function record(b:DocumentBuilder, detail:ComponentDetail):Null<Feature> {
    PartRecordings.recipeInputs(this, b);
    var bottom = MillDrawbarLayout.CYLINDER_BOTTOM, top = bottom + MillDrawbarLayout.HEIGHT, cap = MillDrawbarLayout.CAP;
    var bridge = MillDrawbarLayout.BRIDGE_BOTTOM, piston = MillDrawbarLayout.PISTON_BOTTOM;
    var mounts = [for (x in [-MillDrawbarLayout.MOUNT_X, MillDrawbarLayout.MOUNT_X]) for (y in [-MillDrawbarLayout.MOUNT_Y, MillDrawbarLayout.MOUNT_Y]) [x, y]];
    function corner(p:Array<Float>):String return (p[1] < 0 ? "front " : "back ") + (p[0] < 0 ? "left" : "right");
    return switch values().token("role") {
      case "drawbar":
        b.label(b.unionAll([
          b.label(post(b, 5, 62, 285), "Bar"),
          b.label(post(b, 11, 222, 227), "Spring seat"),
          b.label(post(b, 7, 68, 71), "Gripper collar")
        ]), "Drawbar");
      case "springs":
        b.label(b.unionAll([for (i in 0...16) ring(b, 12.5, 5.5, 181 + i * 2.6, 183 + i * 2.6, 'Disc ${i + 1}')]), "Disc spring stack");
      case "gripper":
        b.label(b.unionAll([ring(b, 9, 4.2, 48, 51, "Jaw lip"), ring(b, 9, 5.5, 51, 68, "Collet")]), "Gripper");
      case "pullStud":
        b.label(b.unionAll([b.label(post(b, 4, 40, 51), "Shank"), b.label(post(b, 5.4, 51, 55), "Knob")]), "Pull stud");
      case "holder":
        var body = b.label(b.unionAll([
          turned(b, [[0, 0], [15.875, 0], [10.042, 40], [0, 40]], "Taper"),
          b.label(post(b, 23, -10, 0), "Flange"),
          b.label(post(b, 17, -35, -10), "Nose")
        ]), "Holder blank");
        b.label(b.cutAll(body, [
          post(b, 3, -36, -5),
          b.boxBetween(18, 28, -4, 4, -10, 1),
          b.boxBetween(-28, -18, -4, 4, -10, 1)
        ], ["tool bore", "drive slot right", "drive slot left"]), "Tool bore and drive slots");
      case "reactionFrame":
        var parts = [ring(b, 70, 22, 242, 250, "Collar flange"), b.label(plate(b, 140, 100, bridge, bridge + MillDrawbarLayout.BRIDGE_THICKNESS), "Bridge")];
        for (x in [-60.0, 60.0]) for (y in [-35.0, 35.0]) parts.push(b.label(post(b, 6, 249, bridge + 1, x, y), "Pillar " + corner([x, y])));
        var tools = [post(b, 12, bridge - 1, bottom + 1)], labels = ["rod bore"];
        for (p in mounts) {
          tools.push(post(b, 3.3, bridge - 1, bottom + 1, p[0], p[1]));
          labels.push("cylinder screw " + corner(p));
        }
        for (p in [[-37.5, 0.0], [37.5, 0.0], [0.0, -37.5], [0.0, 37.5]]) {
          var at = p[0] < 0 ? "left" : p[0] > 0 ? "right" : p[1] < 0 ? "front" : "back";
          tools.push(post(b, 2.75, 241, 251, p[0], p[1]));
          labels.push('collar screw $at');
          tools.push(post(b, 5, 244.6, 251, p[0], p[1]));
          labels.push('collar counterbore $at');
        }
        b.label(b.cutAll(b.label(b.unionAll(parts), "Frame"), tools, labels), "Rod bore and screw holes");
      case "cylinder":
        var width = MillDrawbarLayout.WIDTH;
        var parts = [
          b.label(plate(b, width, width, bottom, bottom + cap), "Lower cap"),
          b.label(plate(b, width, width, top - cap, top), "Upper cap"),
          ring(b, MillDrawbarLayout.BORE / 2 + 5, MillDrawbarLayout.BORE / 2, bottom + cap, top - cap, "Barrel")
        ];
        for (x in [-34.0, 34.0]) for (y in [-34.0, 34.0]) parts.push(b.label(post(b, 3, bottom + cap - 1, top - cap + 1, x, y), "Tie rod " + corner([x, y])));
        var tools = [post(b, 10.5, bottom - 1, bottom + cap + 1)], labels = ["rod bore"];
        for (p in mounts) {
          tools.push(post(b, 2.5, bottom - 1, bottom + cap + 1, p[0], p[1]));
          labels.push("mount tap " + corner(p));
        }
        for (z in [bottom + 11, top - 11]) {
          tools.push(b.cylinderAlong(new Vector(0, 0, z), Vector.Y(), 3, -46, -39));
          labels.push(z < (bottom + top) / 2 ? "lower air port" : "upper air port");
        }
        b.label(b.cutAll(b.label(b.unionAll(parts), "Cylinder body"), tools, labels), "Rod bore, taps and air ports");
      case "piston":
        ring(b, MillDrawbarLayout.BORE / 2 - 0.5, 10, piston, piston + MillDrawbarLayout.PISTON_THICKNESS, "Piston");
      case "pistonRod":
        b.label(post(b, 10, MillDrawbarLayout.ROD_TIP, piston + MillDrawbarLayout.PISTON_THICKNESS), "Piston rod");
      case "rotor":
        var blank = b.label(b.unionAll([b.label(post(b, 30, 0, 39), "Nose"), b.label(post(b, 25, 38, 242), "Shaft")]), "Rotor blank");
        var tube = b.label(b.cutAll(blank, [
          turned(b, [[0, -1], [16.041, -1], [9.479, 44], [0, 44]], "Taper socket"),
          post(b, 13, 43, 243)
        ], ["taper socket", "drawbar bore"]), "Taper socket and drawbar bore");
        b.label(b.unionAll([
          tube,
          ring(b, 25, 5.5, 178, 181, "Spring seat"),
          ring(b, 25, 5.5, 238, 242, "Top cap"),
          ring(b, 10, 5.5, 242, 275, "Drawbar guide")
        ]), "Rotor");
      default:
        throw "Unknown drawbar role";
    };
  }
}
