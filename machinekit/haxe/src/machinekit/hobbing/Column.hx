package machinekit.hobbing;

import machinekit.motion.PrecisionPart;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Hobber mineral-cast column. Reference row is Assumed; dimensions are simplified envelopes. */
class Column extends PrecisionPart {
  public static function parameters():Array<ComponentParameter>return PrecisionPart.common().concat([
    PrecisionPart.quantity("width",180,"mm"),
    PrecisionPart.quantity("depth",160,"mm"),
    PrecisionPart.quantity("height",600,"mm"),
    PrecisionPart.quantity("railSpacing",110,"mm")]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.column",parameters(),v -> new Column(v));
    return recipe;
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues>
    return PrecisionPart.assumedCatalog("Hobber mineral-cast column","HOBBER-COLUMN",parameters());
  public function new(?overrides:ComponentValues) {
    var v=PrecisionPart.resolve(parameters(),overrides);
    super("HOBBER-COLUMN","Hobber mineral-cast column (reference envelope)","mineral cast",v,false,[]);
    if(n("railSpacing")>=n("width"))throw "column rail spacing exceeds width";
    addConnector("railLeft",Mount,materia.assembly.AssemblyFrames.alongY(-n("railSpacing")/2,-n("depth")/2,0,0,-1,0));
    addConnector("railRight",Mount,materia.assembly.AssemblyFrames.alongY(n("railSpacing")/2,-n("depth")/2,0,0,-1,0));
  }
  override public function componentType():Null<ComponentType>return recipeType();


}
