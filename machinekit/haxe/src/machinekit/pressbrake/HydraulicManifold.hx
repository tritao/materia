package machinekit.pressbrake;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Assumed hydraulic distribution block with one inlet and distinct G1/2 outlets. */
class HydraulicManifold extends MachineComponent {
  public final outlets:Int;
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public function new(outlets:Int) {
    if (outlets < 1 || outlets > 64) throw "Hydraulic manifold needs 1 to 64 outlets";
    super('HYDRAULIC-MANIFOLD-$outlets', 'Assumed $outlets-outlet hydraulic distribution block', "steel", true);
    addFacet(machinekit.component.EngineeringMetadata.assumed(""));
    this.outlets = outlets;
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addPort({name: "input", kind: Hydraulic, role: Consumer, iface: Thread("G1/2"), required: true});
    for (i in 1...outlets + 1) {
      addPort({name: 'out$i', kind: Hydraulic, role: Supply, iface: Thread("G1/2"), required: false});
      addBridge("input", 'out$i');
    }
  }
  override public function hasGeometry():Bool return true;
  /** The distribution bar with its outlets drilled. Envelope detail is the bar, which also sets the mass. */
  override public function geometry(detail:ComponentDetail = Preview):Part return detail
    == Envelope ? massGeometry() : machinekit.pneumatic.PneumaticBodies.manifold(
      24,
      24,
      16 + 12 * outlets,
      outlets
    );

  override public function massGeometry():Null<Part> return Solids.named(
    Part.box(24, 24, 16 + 12 * outlets),
    "hydraulic-manifold"
  );
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.pressbrake.hydraulic-manifold",
      [new machinekit.component.ComponentParameter("outlets", Count, Integer(2), null, 1, 64)],
      v -> new HydraulicManifold(v.integer("outlets"))
    ).numbered(v -> 'HYD-MANIFOLD-ASSUMED-${v.integer("outlets")}OUT').procured(_ -> Buy(null, null, null)).requiring(v -> [
      machinekit.component.Procurement.ProcurementCodec.requirement("outlets", "1", v.integer("outlets")),
      machinekit.component.Procurement.ProcurementCodec.choice("portThread", "G1/2")
    ]));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function values():ComponentValues return new ComponentValues().setInteger("outlets",
    outlets).setToken(
      "material",
      materialSpec()
    );
}
