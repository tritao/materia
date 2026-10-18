package machinekit.component;

import machinekit.transmission.ValueBasis;
import machinekit.catalog.Catalog;
import machinekit.component.ComponentRecipeSupport as P;

/** Optional engineering provenance, independent of shape and recipe storage. */
class EngineeringMetadata implements ComponentFacet {
  public final ratingsBasis:ValueBasis;
  public final errorsBasis:ValueBasis;
  public final source:String;
  public function new(values:ComponentValues){
    ratingsBasis=basis(values.token("ratingsBasis"));errorsBasis=basis(values.token("errorsBasis"));source=values.token("source");
    if((ratingsBasis==Catalog||errorsBasis==Catalog)&&StringTools.trim(source).length==0)throw "Catalog basis requires a source";
  }
  public static function parameters():Array<ComponentParameter>return [
    P.choice("ratingsBasis",["Assumed","Catalog","Stated","Derived"],"Assumed"),
    P.choice("errorsBasis",["Assumed","Catalog","Stated","Derived"],"Assumed"),P.text("source","")];
  public static function basis(value:String):ValueBasis return switch value {
    case "Assumed":Assumed;case "Catalog":Catalog;case "Stated":Stated;case "Derived":Derived;
    default:throw "invalid value basis";
  };
  public function check(component:MachineComponent):Void {}
  public function describe():String return 'ratings: $ratingsBasis; errors: $errorsBasis; source: $source';
  public static function of(component:MachineComponent):Null<EngineeringMetadata>{
    for(f in component.facets())if(Std.isOfType(f,EngineeringMetadata))return cast f;return null;
  }
  /** An unverified reference row carries its source instead of implying catalogue verification. */
  public static function assumedCatalog(name:String,designation:String,parameters:Array<ComponentParameter>,source:String):Catalog<ComponentValues>{
    var row=P.resolve(parameters);
    return new Catalog(name,_ -> designation,[row],_ -> ({source:source,standard:null,standardEdition:null,
      dimensionKind:Unverified,conformance:GenericApproximation,verifiedFields:[]}));
  }
}
