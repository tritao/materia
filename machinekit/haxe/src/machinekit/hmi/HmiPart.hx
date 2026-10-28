package machinekit.hmi;

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

  override public function geometry(detail:ComponentDetail = Preview):Part {
    var specs = bodies();
    if (specs == null) throw designation + ": no geometry";
    return Solids.union([for (spec in specs) spec.build()]);
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
      var solid = spec.build();
      var physical = solid.massProperties();
      volumes.push(physical.volume);
      centres.push(physical.centerOfMass);
      total += physical.volume;
      solid.close();
    }
    var assembly = new MachineAssembly();
    for (i in 0...specs.length) {
      var spec = specs[i];
      var name = i == 0 ? designation : designation + "-" + spec.id.toUpperCase();
      assembly.addComponent(
        spec.id,
        new DeviceBody(
          name,
          spec.label,
          spec.material,
          spec.build,
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
