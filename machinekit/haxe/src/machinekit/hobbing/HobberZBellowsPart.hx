package machinekit.hobbing;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.cover.*;
import machinekit.cover.WayCoverSection.WayCoverRole;
import machinekit.component.ComponentRecipeSupport as P;

/** Custom RFQ version of the shared bellows: trim the end flanges to the
 * declared host face, never the nested skins. All dimensions remain ASSUMED. */
class HobberZBellowsPart extends WayCoverSection {
  static final recipe=new machinekit.cache.Once<ComponentType>();
  public static function parameters():Array<ComponentParameter> return WayCoverSection.recipeType().parameters().filter(p->p.name!="material").map(p->switch p.name {
    case "row":P.choice("row",[HobberZBellowsLayout.ROW],HobberZBellowsLayout.ROW);
    case "section":P.choice("section",["Box"],"Box");
    case "nesting":P.choice("nesting",["GrowsToFixed"],"GrowsToFixed");
    case "height":P.length("height",17);
    case "stroke":P.length("stroke",210);
    case "space":P.length("space",30);
    case "hostMax":P.length("hostMax",24);
    case _:p;
  });
  public static function recipeType():ComponentType return recipe.get(()->new ComponentType(
    "machinekit.hobbing.z-bellows",parameters(),v->new HobberZBellowsPart(v),true,false,null,false)
    .numbered(v->{
      var base=WayCoverSection.purchaseNumber(WayCoverCatalog.get(v.token("row")),v.number("width"),v.number("height"),v.number("stroke"),v.number("space"),
        {open:true,hostMin:v.number("hostMin"),hostMax:v.number("hostMax"),hostHalfWidth:v.number("hostHalfWidth"),reversed:true});
      return v.token("role")=="Fixed"?base:v.token("role")=="Moving"?base+"-M":base+"-S"+v.integer("level");
    }).procured(_->Buy(null,null,null)).requiring(v->WayCoverSection.recipeType().requirements(v).concat([
      ProcurementCodec.requirement("flangeMaxHeight","mm",null,v.number("hostMax"),null)
    ])));
  public function new(?overrides:ComponentValues) {
    var v=P.resolve(parameters(),overrides);
    var shape={open:v.token("section")=="Box",hostMin:v.number("hostMin"),hostMax:v.number("hostMax"),hostHalfWidth:v.number("hostHalfWidth"),reversed:v.token("nesting")=="GrowsToFixed"};
    var row=WayCoverCatalog.get(v.token("row")),p=new WayCoverPlan(row,v.number("width"),v.number("height"),v.number("stroke"),v.number("space"),shape);
    var choice:WayCoverRole=switch(v.token("role")){case "Sleeve":Sleeve;case "Moving":Moving;case _:Fixed;};
    super(row,choice,choice==Fixed?p.fixedLevel():choice==Moving?p.movingLevel():v.integer("level"),p.width,p.height,p.stroke,p.space,shape);
    if(choice!=Sleeve)setMaterial("steel"); // Conservative end-flange/attached-skin density; custom RFQ steel end plates.
    if(p.outerHigh(level)>shape.hostMax)throw "Z bellows flange trimming would cut its skin";
  }
  override public function values():ComponentValues {
    var all=super.values(),out=new ComponentValues();
    for(p in parameters()) {var value=all.get(p.name);if(value!=null)out.set(p.name,value);}
    return out;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function geometry(detail:ComponentDetail=Preview):Part {
    var body=super.geometry(detail);if(role==Sleeve)return body;
    var half=plan.fullPlateHalf()+1,high=plan.shape.hostMax;
    var raw=Part.box(2*half,high+1,plan.flangeSkin()+plan.row.flangeThickness+plan.row.wiper+2);
    var keep=raw.translated(new Vector(0,(high-1)/2,-1));raw.close();
    var trimmed=body.intersect(keep);keep.close();body.close();return trimmed;
  }
}
