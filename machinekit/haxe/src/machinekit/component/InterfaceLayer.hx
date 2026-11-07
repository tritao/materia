package machinekit.component;

/** A physical shim or replicated layer may transmit a fixed mate through its
 * two real contact faces. It supplies no thread or slip-capacity exemption. */
class InterfaceLayer implements ComponentFacet {
  public final thickness:Float;
  public function new(thickness:Float) {
    if (!(thickness > 0) || !Math.isFinite(thickness)) throw "Invalid interface layer thickness";
    this.thickness = thickness;
  }
  public function check(component:MachineComponent):Void {
    if (!component.hasGeometry()) throw "Interface layer requires physical geometry";
  }
  public function describe():String return 'physical interface layer, $thickness mm; both contact faces require verification';
  public static function of(component:MachineComponent):Null<InterfaceLayer> {
    for (facet in component.facets()) if (Std.isOfType(facet, InterfaceLayer)) return cast facet;
    return null;
  }
}
