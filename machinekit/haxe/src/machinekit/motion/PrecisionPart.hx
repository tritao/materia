package machinekit.motion;

import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.catalog.Catalog;
import machinekit.transmission.ValueBasis;
import machinekit.motion.PrecisionErrors.PrecisionError;

/** Recipe-backed reference envelopes. CAD mm; mechanical ratings SI unless the input unit says mm.
    Dimensions/ratings and error bounds have separate provenance; no catalogue verification implied. */
class PrecisionPart extends MachineComponent {
  final input:ComponentValues;
  final round:Bool;
  public final ratingsBasis:ValueBasis;
  public final errorsBasis:ValueBasis;
  public final source:String;
  function new(designation:String,label:String,material:String,input:ComponentValues,round:Bool,
      errors:Array<{name:String,unit:String}>) {
    super(designation,label,material);
    this.input=input.copy();this.round=round;
    if(input.get("material")!=null)setMaterial(input.token("material"));
    ratingsBasis=basis(input.token("ratingsBasis"));errorsBasis=basis(input.token("errorsBasis"));source=input.token("source");
    if((ratingsBasis==Catalog||errorsBasis==Catalog)&&StringTools.trim(source).length==0)throw "Catalog basis requires a source";
    if(round){if(n("bore")>=n("outside")||n("outside")<=0||n("height")<=0)throw "invalid annular envelope";}
    else for(k in ["width","depth","height"])if(n(k)<=0)throw "invalid block envelope";
    addConnector("mount",Mount,Solids.axial(0,0,0));
    addConnector("axis",Axis,Solids.axial(0,0,0));
    addConnector("top",Mount,Solids.axial(0,0,n("height")));
    addFacet(new PrecisionErrors([for(e in errors)new PrecisionError(e.name,n(e.name),e.unit,errorsBasis)]));
  }
  public function n(name:String):Float return input.number(name);
  public function token(name:String):String return input.token(name);
  public function count(name:String):Int return input.integer(name);
  override public function values():ComponentValues return input.copy().setToken("material",materialSpec());
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail=Preview):Part {
    if(!round)return Solids.named(Part.box(n("width"),n("depth"),n("height")),"body");
    var body=Solids.named(Part.cylinderSpan(n("outside")/2,0,n("height")),"body");
    return n("bore")==0?body:Solids.cut(body,[Solids.named(Part.cylinderSpan(n("bore")/2,-0.1,n("height")+0.1),"bore")]);
  }
  public static function quantity(name:String,value:Float,unit:String,minimum:Float=0):ComponentParameter
    return new ComponentParameter(name,Scalar,Number(value),unit,minimum);
  public static function common():Array<ComponentParameter>return [
    P.choice("ratingsBasis",["Assumed","Catalog","Stated","Derived"],"Assumed"),
    P.choice("errorsBasis",["Assumed","Catalog","Stated","Derived"],"Assumed"),P.text("source","")];
  public static function resolve(parameters:Array<ComponentParameter>,?overrides:ComponentValues):ComponentValues {
    var result=new ComponentValues(),names:Map<String,Bool> = [];
    for(p in parameters){names.set(p.name,true);var value=overrides==null?null:overrides.get(p.name);
      if(value==null)value=p.defaultValue;p.validate(value);result.set(p.name,value);}
    if(overrides!=null)for(name in overrides.names())if(name!="material"&&!names.exists(name))throw 'unknown precision-part input $name';
    if(overrides!=null&&overrides.get("material")!=null)result.set("material",overrides.get("material"));
    return result;
  }
  public static function basis(value:String):ValueBasis return switch value {
    case "Assumed":Assumed;case "Catalog":Catalog;case "Stated":Stated;case "Derived":Derived;
    default:throw "invalid value basis";
  };
  public static function assumedCatalog(name:String,designation:String,parameters:Array<ComponentParameter>):Catalog<ComponentValues> {
    var row=resolve(parameters);
    return new Catalog(name,_ -> designation,[row],_ -> ({source:"machinekit/examples/flexspline-hobber/PLAN.md (assumed reference design)",
      standard:null,standardEdition:null,dimensionKind:Unverified,conformance:GenericApproximation,verifiedFields:[]}));
  }
}
