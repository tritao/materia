package machinekit.milling;

import cadkit.parametric.Feature;
import cadkit.parametric.recording.DocumentBuilder;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Parametric reconstruction, not supplier CAD. RKE92 external dimensions are
 * published; internal slides, mounting accessories and recesses are nominal.
 */
class Rke92VisePart extends MachineComponent {
  static final recipe = new machinekit.cache.Once<ComponentType>();
  static function parameters():Array<ComponentParameter> return [
    P.choice("role", ["body", "jaw-back", "slide", "jaw-plate", "drive", "stop", "clamp", "key"], "body"),
    P.length("depth", 39)
  ];
  public function new(role:String = "body", depth:Float = 39) {
    var v = P.resolve(parameters(), new ComponentValues().setToken("role", role).setNumber("depth", depth));
    super("RKE92-REFERENCE-" + role, "RKE92 drawing reference: " + role, "steel", false, v);
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
  }
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.milling.rke92-reference-part",
      parameters(),
      v -> new Rke92VisePart(v.token("role"), v.number("depth"))
    ).numbered(v -> 'RKE92-REFERENCE-${v.token("role").toUpperCase()}'));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  /** A box `w` x `d` x `h` centred on (x, y), rising from z. */
  static function slab(b:DocumentBuilder, w:Float, d:Float, h:Float, x:Float = 0, y:Float = 0, z:Float = 0):Feature
    return b.boxBetween(x - w / 2, x + w / 2, y - d / 2, y + d / 2, z, z + h);

  /** A bore of radius `r` along Y from y0 to y1, through (x, z). */
  static function boreY(b:DocumentBuilder, r:Float, y0:Float, y1:Float, x:Float, z:Float):Feature
    return b.cylinderAlong(new Vector(x, 0, z), Vector.Y(), r, y0, y1);

  /** A bore of radius `r` along Z from z0 to z1, through (x, y). */
  static function boreZ(b:DocumentBuilder, r:Float, z0:Float, z1:Float, x:Float, y:Float):Feature
    return b.cylinderAlong(new Vector(x, y, 0), Vector.Z(), r, z0, z1);

  override public function record(b:DocumentBuilder, detail:ComponentDetail):Null<Feature> {
    PartRecordings.recipeInputs(this, b);
    var d = parameterNumber("depth"), role = values().token("role");
    var envelope = detail == Envelope;
    function cut(body:Feature, tools:Array<Feature>, labels:Array<String>, label:String):Feature
      return envelope ? body : b.label(b.cutAll(body, tools, labels), label);
    return switch role {
      case "body":
        var body = b.label(slab(b, 94, 276, 80, 0, 138), "Stock · Body");
        var tools = [slab(b, 52, 197, 57, 0, 178.5, 24), slab(b, 20, 278, 5.2, 0, 138, -1)], labels = ["jaw channel", "key slot"];
        for (side in [-1, 1]) {
          tools.push(slab(b, 12, 278, 16, side * 42, 138, 12));
          labels.push(side < 0 ? "clamp ledge left" : "clamp ledge right");
        }
        cut(body, tools, labels, "Channel, key slot and clamp ledges");
      case "jaw-back", "slide":
        var block = b.label(slab(b, 92, d, 32, 0, -d / 2), "Stock · Jaw");
        var body = role == "slide" ? b.label(b.unionAll([block, b.label(slab(b, 50, d, 50, 0, -d / 2, -50), "Slide foot")]), "Jaw and foot") : block;
        var tools:Array<Feature> = [], labels:Array<String> = [];
        for (x in [-31.5, 31.5]) {
          tools.push(boreY(b, 2.5, -14, 1, x, 14));
          labels.push(x < 0 ? "plate tap left" : "plate tap right");
        }
        if (role == "slide") {
          tools.push(boreY(b, 10.5, -d - 1, 1, 0, -32));
          labels.push("drive bore");
        }
        cut(body, tools, labels, role == "slide" ? "Plate taps and drive bore" : "Plate taps");
      case "jaw-plate":
        var plate = b.label(slab(b, 92, 25, 32, 0, -12.5), "Stock · Plate");
        var tools:Array<Feature> = [], labels:Array<String> = [];
        for (x in [-31.5, 31.5]) {
          var side = x < 0 ? "left" : "right";
          tools.push(boreY(b, 3.3, -26, 1, x, 14));
          labels.push('screw clearance $side');
          tools.push(boreY(b, 5.5, -6.4, 1, x, 14));
          labels.push('screw counterbore $side');
        }
        cut(plate, tools, labels, "Counterbored screw holes");
      case "drive":
        var shaft = b.label(b.unionAll([b.label(boreY(b, 10, 0, 70, 0, 0), "Shaft"), b.label(boreY(b, 24, 20, 50, 0, 0), "Collar")]), "Shaft and collar");
        cut(shaft, [boreY(b, 4, 62, 71, 0, 0)], ["drive socket"], "Drive socket");
      case "stop":
        var body = b.label(b.unionAll([b.label(slab(b, 16, 16, 6, -2), "Foot"), b.label(slab(b, 6, 8, 30), "Post")]), "Foot and post");
        cut(body, [boreZ(b, 2.25, -1, 7, -6, 0)], ["screw hole"], "Screw hole");
      case "clamp":
        cut(b.label(slab(b, 60, 24, 10), "Stock · Clamp"), [boreZ(b, 3.3, -1, 11, 10, 0)], ["screw hole"], "Screw hole");
      case "key":
        b.label(b.unionAll([b.label(slab(b, 19.8, 20, 4), "Key"), b.label(slab(b, 9.6, 20, 4, 0, 0, -4), "Tenon")]), "Key and tenon");
      default:
        throw "Unknown vise part";
    };
  }
}
