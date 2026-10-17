package machinekit.motion;

import machinekit.component.ComponentFacet;
import machinekit.component.MachineComponent;
import machinekit.assembly.MachineAssembly;
import machinekit.transmission.ValueBasis;

/** A stated error bound, not an injected waveform. Units and interpretation travel with it.
    Runout bounds are TIR; eccentricity and cogging are amplitudes. */
class PrecisionError {
  public final quantity:String;
  public final bound:Float;
  public final unit:String;
  public final basis:ValueBasis;
  public function new(quantity:String,bound:Float,unit:String,basis:ValueBasis) {
    if(quantity.length==0||!Math.isFinite(bound)||bound<0)throw "invalid precision error";
    this.quantity=quantity;this.bound=bound;this.unit=unit;this.basis=basis;
  }
}

/** Typed component facet, persisted through each component's recipe inputs. */
class PrecisionErrors implements ComponentFacet {
  final entries:Array<PrecisionError>;
  public function new(entries:Array<PrecisionError>)this.entries=entries.copy();
  public function all():Array<PrecisionError>return entries.copy();
  public function check(component:MachineComponent):Void {
    var seen:Map<String,Bool> = [];
    for(e in entries){if(seen.exists(e.quantity))throw "duplicate precision error";seen.set(e.quantity,true);}
  }
  public function describe():String return [for(e in entries)'${e.quantity} <= ${e.bound} ${e.unit} (${e.basis})'].join(", ");
  public static function of(component:MachineComponent):Null<PrecisionErrors> {
    for(f in component.facets())if(Std.isOfType(f,PrecisionErrors))return cast f;return null;
  }
  /** Path-qualified readout, including nested assemblies, without recentering or applying errors. */
  public static function inAssembly(assembly:MachineAssembly):Array<{part:String,error:PrecisionError}> {
    var result:Array<{part:String,error:PrecisionError}> = [];
    for(m in assembly.components()){var f=of(m.component);if(f!=null)for(e in f.all())result.push({part:m.id,error:e});}
    result.sort((a,b)->{var order=Reflect.compare(a.part,b.part);return order==0?Reflect.compare(a.error.quantity,b.error.quantity):order;});
    return result;
  }
}
