package machinekit.hmi;

import CadKit;
import cadkit.Shape;
import cadkit.modeling.Footprint;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentDetail;
import materia.assembly.AssemblyFrames;
import machinekit.component.ComponentParameter;
import machinekit.component.PanelBoss;
import machinekit.component.PanelMountFacet;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.transmission.ValueBasis;

/**
 * A bought operator control for a panel. Unlike an assumed catalogue part its numbers may be a maker's drawing or a
 * measurement; the recipe's `basis` input says which, and travels in its designation and BOM line. Parts
 * are in their own frame: origin at the centre of the panel's front face, +z toward the operator.
 */
class HmiPart extends MachineComponent {
  /** How trustworthy the part's numbers are. */
  public final basis:ValueBasis;
  final stored:ComponentValues;

  function new(id:String,
    label:String, material:String, parameters:Array<ComponentParameter>, ?inputs:ComponentValues) {
    var v = ComponentRecipeSupport.resolve(parameters, inputs);
    if (v.get("material") == null) v.setToken("material", material);
    var tag = v.token("basis");
    super(ComponentRecipeSupport.designationFor(id, tag, v), label, v.token("material"), false, v);
    stored = v.copy();
    basis = switch tag {
      case "catalog":
        Catalog;
      case "stated":
        Stated;
      case "measured":
        Measured;
      case _:
        Assumed;
    };
  }

  /** The strong name of the one planar face of `shape` that looks toward the operator (+z); `what` names the part in the error. */
  public static function frontFace(shape:Shape, what:String):String {
    var names = shape.elementNames(CadKit.ShapeKind.Face);
    var found:Null<String> = null;
    for (index in 0...names.length) {
      var face = shape.subshape(CadKit.ShapeKind.Face, index);
      var front = face.surfaceKind() == CadKit.SurfaceKind.Plane && face.faceNormal().get_z() > 0.99;
      face.close();
      if (front) {
        if (found != null) throw "The " + what + " has more than one front face";
        found = names[index];
      }
    }
    if (found == null) throw "The " + what + " has no front face";
    return found;
  }

  /** The provenance input every part declares. */
  public static function basisParameter(value:String =
    "assumed"):ComponentParameter return ComponentRecipeSupport.choice(
      "basis",
      ["catalog", "stated", "assumed", "measured"],
      value
    );

  public function n(name:String):Float return parameterNumber(name);
  public function flag(name:String):Bool return stored.boolean(name);
  /** An integer input (a count). */
  public function count(name:String):Int return stored.integer(name);
  public function positive(names:Array<String>):Void {
    for (name in names) if (!Math.isFinite(n(name))
      || n(name) <= 0) throw designation + ": " + name + " must be finite and positive";
  }
  public function require(ok:Bool, message:String):Void {
    if (!ok) throw designation + ": " + message;
  }
  override public function hasGeometry():Bool return true;

  /**
   * The device as its separately finished bodies, the purchase first; null for a part that is one solid. The whole device's
   * `geometry` is these bodies fused, so there is one description of the shape.
   */
  public function bodies():Null<Array<DeviceBodySpec>> return null;

  /**
   * Inserts and screws that hold the part to its panel, with their origins in the part's frame (z = 0 at the panel's front
   * face). The panel assembly places them in its `fasteners` group.
   */
  public function hardware():Array<PanelHardware> return [];

  /** Joints between the bodies, each about the device's z axis: the encoder of a wheel (free), the shaft of a switch (detented). */
  public function joints():Array<DeviceJointSpec> return [];

  /** A cylinder of radius `r` up Z through (x, y), from z0 to z1. */
  public static function post(b:cadkit.parametric.recording.DocumentBuilder, r:Float, z0:Float, z1:Float, x:Float = 0,
      y:Float = 0):cadkit.parametric.Feature return b.cylinderAlong(new Vector(x, y, 0), Vector.Z(), r, z0, z1);

  /** A tube of outer radius `r` and bore `bore` up Z, from z0 to z1. */
  public static function tube(b:cadkit.parametric.recording.DocumentBuilder, r:Float, bore:Float, z0:Float, z1:Float):cadkit.parametric.Feature
    return b.cutAll(post(b, r, z0, z1), [post(b, bore, z0 - 1, z1 + 1)], ["bore"]);

  /** The solid of revolution about Z through the (r, z) section `points`; `sweptFace` names the face each segment sweeps. */
  public static function turned(b:cadkit.parametric.recording.DocumentBuilder, points:Array<{r:Float, z:Float}>,
      ?angle:Float):cadkit.parametric.Feature
    return b.revolve(b.polygon([for (p in points) new Vector(p.r, p.z, 0)], cadkit.modeling.Plane.XZ()), new Vector(0, 0, 0), Vector.Z(),
      angle == null ? 2 * Math.PI : angle);

  /**
   * The name, within `revolved` (a `turned` solid), of the face that the profile's segment from point k to point k + 1 sweeps:
   * the face a detail on it names.
   */
  public static function sweptFace(revolved:cadkit.parametric.Feature, k:Int):String {
    var revolve = Std.downcast(revolved, cadkit.parametric.features.RevolveFeature);
    if (revolve == null) throw "A swept face belongs to a revolved profile";
    return 'side(f${revolve.source.id.toInt()}:seg.$k)';
  }

  /** A box `w` x `h` x `d` centred on (x, y), from z0. */
  public static function block(b:cadkit.parametric.recording.DocumentBuilder, w:Float, h:Float, d:Float, x:Float, y:Float,
      z0:Float):cadkit.parametric.Feature return b.boxBetween(x - w / 2, x + w / 2, y - h / 2, y + h / 2, z0, z0 + d);

  /** A rounded-corner block `w` x `h`, centred on (x, y), from z0 to z1. */
  public static function roundedBlock(b:cadkit.parametric.recording.DocumentBuilder, w:Float, h:Float, radius:Float, z0:Float, z1:Float, x:Float = 0,
      y:Float = 0):cadkit.parametric.Feature
    return b.extrude(b.roundedRectangle(w, h, radius, new cadkit.modeling.Plane(new Vector(x, y, z0), Vector.X(), Vector.Z())), z1 - z0);

  /** `parts` fused into one step, or the one part itself. */
  public static function fuse(b:cadkit.parametric.recording.DocumentBuilder, parts:Array<cadkit.parametric.Feature>):cadkit.parametric.Feature
    return parts.length == 1 ? parts[0] : b.unionAll(parts);

  /** A device with bodies records them fused, one labelled step each. */
  override public function record(b:cadkit.parametric.recording.DocumentBuilder, detail:ComponentDetail):Null<cadkit.parametric.Feature> {
    var specs = bodies();
    if (specs == null) return null;
    machinekit.component.PartRecordings.recipeInputs(this, b);
    var parts = [for (spec in specs) b.label(spec.record(b), spec.label)];
    return b.label(fuse(b, parts), "Device");
  }

  /**
   * The device as an assembly of its bodies in the device's own frame, or null when it is one solid. The first body is the
   * purchase and carries the BOM line; the others are included in it. Each body takes a share of the declared mass by volume.
   */
  public function expand():Null<MachineAssembly> {
    var specs = bodies();
    if (specs == null) return null;
    var volumes:Array<Float> = [], centres:Array<Vector> = [];
    var total = 0.0;
    for (spec in specs) {
      var solid = spec.solid();
      var physical = solid.massProperties();
      volumes.push(physical.volume);
      centres.push(physical.centerOfMass);
      total += physical.volume;
      solid.close();
    }
    var assembly = new MachineAssembly();
    var type = componentType();
    if (type == null) throw "Expanded HMI bodies require a declared recipe";
    var definingKey = type.key(stored);
    for (i in 0...specs.length) {
      var spec = specs[i];
      var name = i == 0 ? designation : machinekit.component.PartNumbers.geometry("HMI", spec.id, definingKey);
      assembly.addComponent(
        spec.id,
        new DeviceBody(
          name,
          spec.label,
          spec.material,
          spec,
          this,
          i == 0,
          n("massGrams") / 1000.0 * volumes[i] / total,
          centres[i],
          spec.ports
        )
      );
    }
    for (joint in joints()) {
      assembly.addMemberConnector(joint.parent, joint.id + "-fixed", AssemblyFrames.identity());
      assembly.addMemberConnector(joint.child, joint.id + "-turning", AssemblyFrames.identity());
      assembly.addMateOnAxis(
        joint.id,
        joint.kind,
        joint.parent,
        joint.id + "-fixed",
        joint.child,
        joint.id + "-turning",
        {x: 0, y: 0, z: 1},
        joint.defaultValue,
        joint.limits
      );
    }
    return assembly;
  }

  public function dataSource():String return switch basis {
    case Catalog:
      "vendor-drawing";
    case Stated:
      "listing";
    case Measured:
      "measured";
    case _:
      "assumed";
  };

  /** States how the part is fixed to a panel (see PanelMountFacet), from the part's own numbers. */
  function mount(
    cutout:Footprint,
    keepOut:Footprint,
    holes:Array<{x:Float, y:Float, diameter:Float}>,
    above:Float,
    behind:Float,
    body:Footprint,
    maxPanelThickness:Float = 1e9,
    ?bosses:Array<PanelBoss>
  ):Void addFacet(new PanelMountFacet(
    cutout,
    keepOut,
    holes,
    above,
    behind,
    body,
    maxPanelThickness,
    dataSource(),
    bosses
  ));
}
