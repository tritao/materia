package machinekit.component;

import machinekit.component.Procurement.ProcurementCodec;
import machinekit.component.Procurement.ProcurementRequirement;

/** Explicit procurement intent for a component without a recipe. */
class ProcurementIntent implements ComponentFacet {
  public final value:Procurement;
  final terms:Array<ProcurementRequirement>;

  public function new(value:Procurement, ?requirements:Array<ProcurementRequirement>) {
    ProcurementCodec.document(value);
    this.value = value;
    terms = ProcurementCodec.copyRequirements(requirements);
  }
  public function requirements():Array<ProcurementRequirement> return ProcurementCodec.copyRequirements(terms);
  public function check(component:MachineComponent):Void {}
  public function describe():String return "procurement: " + ProcurementCodec.document(value).kind;
  public static function of(component:MachineComponent):Null<ProcurementIntent> {
    var result:Null<ProcurementIntent> = null;
    for (facet in component.facets()) if (Std.isOfType(facet, ProcurementIntent)) {
      if (result != null) throw 'Duplicate procurement intent on "${component.designation}"';
      result = cast facet;
    }
    return result;
  }
}
