package machinekit.pneumatic;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.PortInterface;
import machinekit.component.PortKind;
import machinekit.component.PortRole;
import machinekit.component.Solids;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;

/** Generic air manifold with one required input and N bridged outlets. */
class PneumaticManifold extends MachineComponent {
  public final outlets:Int;

  public function new(outlets:Int) {
    if (outlets < 1) throw "Pneumatic manifold needs at least one outlet";
    super('MANIFOLD-$outlets', 'Generic $outlets-outlet pneumatic manifold', "aluminium 6061", true);
    if (Std.isExactType(this, PneumaticManifold)) addFacet(new machinekit.component.EngineeringMetadata(new machinekit.component.ComponentValues()
      .setToken("ratingsBasis", "Assumed").setToken("errorsBasis", "Assumed").setToken("source", "")));
    this.outlets = outlets;
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addPort({name: "input", kind: Pneumatic, role: Consumer, iface: PushIn(6), required: true});
    for (i in 1...outlets + 1) {
      addPort({name: 'out$i', kind: Pneumatic, role: Supply, iface: PushIn(6), required: false});
      addBridge("input", 'out$i');
    }
  }

  static final recipeTypeCache = new machinekit.cache.Once<ComponentType>();

  public static function recipeType():ComponentType {
    return recipeTypeCache.get(() -> new ComponentType(
      "machinekit.pneumatic.manifold",
      [ComponentRecipeSupport.count("outlets", 2)],
      v -> new PneumaticManifold(v.integer("outlets")),
      true
    ).procured(_ -> Buy(null, null, null)).requiring(v -> [
      machinekit.component.Procurement.ProcurementCodec.requirement("outlets", "1", v.integer("outlets")),
      machinekit.component.Procurement.ProcurementCodec.requirement("tubeOutsideDiameter", "mm", null, null, 6)
    ]));
  }

  /** Subclasses must declare their own recipe and saved values. */
  override public function componentType():Null<ComponentType> return Std.isExactType(this,
    PneumaticManifold) ? recipeType() : null;

  override public function values():machinekit
    .component.ComponentValues return new machinekit.component.ComponentValues().setInteger(
      "outlets",
      outlets
    ).setToken(
      "material",
      materialSpec()
    );

  override public function hasGeometry():Bool return true;

  /** A bar with its outlets drilled into the front (`PneumaticBodies.manifold`). The envelope box is also the mass model. */
  override public function geometry(detail:ComponentDetail = Preview):Part return detail == Envelope ? Part.box(
    20,
    10,
    12 + 8 * outlets
  ) : PneumaticBodies.manifold(
    20,
    10,
    12 + 8 * outlets,
    outlets
  );

  override public function massGeometry():Null<Part> return Part.box(20, 10, 12 + 8 * outlets);
}
