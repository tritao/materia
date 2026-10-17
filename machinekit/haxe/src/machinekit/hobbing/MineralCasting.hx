package machinekit.hobbing;

import machinekit.motion.PrecisionPart;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.PosedParts;

/** Thick-walled, cross-ribbed reference body with insert recesses. No machined casting datums.
    Enclosed design cavities require process/core design later; these are geometric design bodies. */
class MineralCasting extends PrecisionPart {
  public final kind:String;
  public static function parameters():Array<ComponentParameter>return PrecisionPart.common().concat([
    P.choice("kind",["bed","column"],"bed"),P.length("width",650),P.length("depth",550),P.length("height",120)]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.mineral-casting-body",parameters(),v->new MineralCasting(v));return recipe;
  }
  public function new(?values:ComponentValues){
    var v=PrecisionPart.resolve(parameters(),values),k=v.token("kind"),dims=FoundationLayout.dimensions(k);
    for(i in 0...3)if(v.number(["width","depth","height"][i])!=dims[i])throw "foundation body uses its designed insert layout dimensions";
    super("MINERAL-CAST-"+k.toUpperCase(),"Ribbed mineral casting "+k+" (purchase by casting mass, kg)","mineral cast",v,false,[]);
    kind=k;
  }
  override public function componentType():Null<ComponentType>return recipeType();
  override public function geometry(detail:ComponentDetail=Preview):Part {
    var tools:Array<Part> = [],body=super.geometry(detail);
    try {
      var t=kind=="bed"?30.0:25.0,rib=25.0;
      var w=(n("width")-2*t-rib)/2,d=(n("depth")-2*t-rib)/2;
      for(x in [-1,1])for(y in [-1,1]){
        var p=Part.box(w,d,n("height")-40),m=p.translated(new Vector(x*(w+rib)/2,y*(d+rib)/2,20));p.close();tools.push(m);
      }
      for(row in FoundationLayout.inserts(kind)){
        var p=Part.box(row.width,row.depth,row.height),m=PosedParts.moved(p,row.pose);p.close();tools.push(m);
      }
      return Solids.cut(body,tools);
    }catch(e:Dynamic){if(!body.shape.isClosed())body.close();for(t in tools)if(!t.shape.isClosed())t.close();throw e;}
  }
  override function get_bom():BomItem {
    var item=super.get_bom();
    return {partNumber:item.partNumber,description:item.description+", "+Dimension.format(castingByWeight().kg)+" kg",
      quantity:item.quantity,material:item.material,typeId:item.typeId,valuesKey:item.valuesKey};
  }
  override public function setMaterial(spec:String):Void {
    if(spec!="mineral cast")throw "mineral casting requires mineral-cast material";super.setMaterial(spec);
  }
  /** BOM purchasing quantity from actual native CAD volume and material density. */
  public function castingByWeight():{material:String,kg:Float}return {material:materialSpec(),kg:massProperties().mass};
}
