package hobber;

import machinekit.hobbing.*;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.cover.*;
import machinekit.motion.ScrewLinearModuleSpec;
import machinekit.cover.WayCoverSection.WayCoverRole;
import machinekit.component.ComponentRecipeSupport as P;

/** Custom RFQ version of the shared bellows: trim the end flanges to the
 * declared host face, never the nested skins. All dimensions remain ASSUMED. */
class HobberZBellowsPart extends WayCoverSection {
  static final recipe=new machinekit.cache.Once<ComponentType>();
  public static function parameters():Array<ComponentParameter> return WayCoverSection.recipeType().parameters().filter(p->p.name!="material").map(p->switch p.name {
    case "row":P.choice("row",[HobberSkeleton.reference().zBellowsRow],HobberSkeleton.reference().zBellowsRow);
    case "section":P.choice("section",["Box"],"Box");
    case "nesting":P.choice("nesting",["GrowsToFixed"],"GrowsToFixed");
    case "height":P.length("height",17.2);
    case "stroke":P.length("stroke",210);
    case "space":P.length("space",30);
    case "hostMax":P.length("hostMax",24);
    case _:p;
  });
  public static function recipeType():ComponentType return recipe.get(()->new ComponentType(
    "machinekit.hobbing.z-bellows",parameters(),v->new HobberZBellowsPart(v),true,false,null,false)
    .versioned(3).numbered(v->{
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
  /** Custom RFQ pattern: retain complete M3 washer lands after trimming the
   * flange clear of the catalogue supports. The joint owns this single pattern. */
  override public function holes():Array<{x:Float,y:Float}> {
    var screw=machinekit.standard.SocketHeadCapScrew.catalog().get(plan.row.screw);
    var half=plan.fullPlateHalf()-ScrewLinearModuleSpec.GAP,low=plan.row.flangeThickness+0.2,edge=1.5*screw.diameter;
    return [for(x in [-half+plan.row.lip/2,half-plan.row.lip/2])for(y in [low+edge,plan.shape.hostMax-edge]){x:x,y:y}];
  }
  override public function fasteningFaces():Array<BoltedFace> {
    if(role==Sleeve)return [];
    // These custom end plates are 3 mm steel, not cloth: the socket head
    // bears directly on steel. A plain washer would foul the attached skin.
    return [BoltedFace.clamped(role==Fixed?"fixed":"moving",
      BoltedFace.frameOf({x:0.0,y:0.0,z:0.0},{x:0.0,y:0.0,z:1.0},{x:1.0,y:0.0,z:0.0}),
      holes(),plan.row.clearance,plan.row.flangeThickness,plan.row.screw,null,false,false)];
  }
  override public function geometry(detail:ComponentDetail=Preview):Part {
    var body=super.geometry(detail);if(role==Sleeve)return body;
    var half=plan.fullPlateHalf()-ScrewLinearModuleSpec.GAP,high=plan.shape.hostMax,low=plan.row.flangeThickness+0.2;
    var raw=Part.box(2*half,high-low,plan.flangeSkin()+plan.row.flangeThickness+plan.row.wiper+2);
    var keep=raw.translated(new Vector(0,(high+low)/2,-1));raw.close();
    var trimmed=body.intersect(keep);keep.close();body.close();return trimmed;
  }
}
