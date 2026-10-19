package machinekit.hobbing;

import machinekit.motion.PrecisionErrors;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Hydraulic expanding mandrel. Reference row is Assumed; dimensions are simplified envelopes. */
class ExpandingMandrel extends MachineComponent {
  public static function parameters():Array<ComponentParameter>return EngineeringMetadata.parameters().concat([
    P.quantity("outside",59,"mm"),
    P.quantity("bore",12,"mm"),
    P.quantity("height",50,"mm"),P.length("baseDiameter",0),P.length("baseHeight",0),P.length("baseBore",0),P.length("baseBoreDepth",0),P.length("basePilotDiameter",0),P.length("basePilotDepth",0),P.length("baseBoltPcd",0),
    P.quantity("workSeat",30,"mm"),P.quantity("stopDiameter",0,"mm"),P.flag("separateSleeve",false),
    P.quantity("expansion",0.05,"mm"),
    P.quantity("sleeveWall",1.5,"mm"),
    P.quantity("sleeveLength",16,"mm"),
    P.quantity("sleeveStart",26,"mm"),
    P.quantity("sleeveYoungs",210000,"N/mm²"),
    P.quantity("sleevePoisson",0.3,""),
    P.quantity("oilCavity",0.5,"mm"),
    P.quantity("oilBulkModulus",1500,"N/mm²"),
    P.quantity("contactStiffnessFactor",1,""),
    P.quantity("radialFitGap",0.025,"mm"),
    P.quantity("clampingPressure",10000000,"Pa"),
    P.quantity("maxPressure",15000000,"Pa"),
    P.quantity("eccentricity",0.004,"mm"),
    P.quantity("clampingDistortion",0.002,"mm")]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.expanding-mandrel",parameters(),v -> new ExpandingMandrel(v));
    return recipe;
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues>
    return EngineeringMetadata.assumedCatalog("Hydraulic expanding mandrel","HYDRAULIC-MANDREL-59",parameters(),"machinekit/examples/flexspline-hobber/PLAN.md (assumed reference design)");
  public function new(?overrides:ComponentValues) {
    var v=P.resolve(parameters(),overrides);
    super("HYDRAULIC-MANDREL-59","Hydraulic expanding mandrel (reference envelope)","steel",false,v);
    if(v.get("material")!=null)setMaterial(v.token("material"));
    ComponentEnvelopes.validateAnnular(v);
    addConnector("mount",Mount,Solids.axial(0,0,0));
    addConnector("axis",Axis,Solids.axial(0,0,0));
    addConnector("top",Mount,Solids.axial(0,0,parameterNumber("height")));
    addFacet(new EngineeringMetadata(v));
    addFacet(PrecisionErrors.fromValues(v,[{name:"eccentricity",unit:"mm"},{name:"clampingDistortion",unit:"mm"}]));
    if(parameterNumber("clampingPressure")<=0||parameterNumber("clampingPressure")>parameterNumber("maxPressure")||parameterNumber("workSeat")>=parameterNumber("height")||parameterNumber("expansion")<=0)throw "invalid mandrel ratings";
    for(key in ["sleeveWall","sleeveLength","sleeveYoungs","oilCavity","oilBulkModulus","contactStiffnessFactor"])
      if(parameterNumber(key)<=0)throw 'invalid mandrel support $key';
    if(parameterNumber("sleeveStart")+parameterNumber("sleeveLength")>parameterNumber("height")||parameterNumber("sleeveWall")>=parameterNumber("outside")/2||parameterNumber("radialFitGap")<0
      ||parameterNumber("sleevePoisson")<=0||parameterNumber("sleevePoisson")>=0.5)throw "invalid sleeve/contact design";
    addConnector("work",Mount,Solids.axial(0,0,parameterNumber("workSeat")));
    // Separate axial envelopes preserve real external shoulders. A single hull
    // bridges the large base to the sleeve cap and invents material at the cut.
    // Internal bores/oil cavities remain conservatively filled in collision only.
    var hulls:Array<Array<Float>> = [],r=parameterNumber("outside")/2,baseHeight=parameterNumber("baseHeight"),height=parameterNumber("height");
    function section(radius:Float,lo:Float,hi:Float):Void {if(hi>lo)hulls.push(CollisionHullFacet.cylinder(radius,lo,hi));}
    if(baseHeight>0){
      if(parameterNumber("baseBoltPcd")>0){
        section(parameterNumber("baseDiameter")/2-25,0,baseHeight);
        for(i in 0...6){var a=(i+0.5)*Math.PI/3,d=parameterNumber("baseBoltPcd")/2,h=CollisionHullFacet.cylinder(15,0,baseHeight);
          for(j in 0...Std.int(h.length/3)){h[j*3]+=d*Math.cos(a);h[j*3+1]+=d*Math.sin(a);}hulls.push(h);}
      }else section(parameterNumber("baseDiameter")/2,0,baseHeight);
    }
    if(v.boolean("separateSleeve")){
      var start=parameterNumber("sleeveStart"),end=start+parameterNumber("sleeveLength");
      section(r,baseHeight,start);section(r-parameterNumber("sleeveWall")-parameterNumber("oilCavity"),Math.max(baseHeight,start),end);section(r,Math.max(baseHeight,end),height);
    }else section(r,baseHeight,height);
    if(parameterNumber("stopDiameter")>0)section(parameterNumber("stopDiameter")/2,parameterNumber("workSeat")-2,parameterNumber("workSeat"));
    addFacet(new CollisionHullFacet(hulls));
  }
  override public function componentType():Null<ComponentType>return recipeType();
  public var clampingPressure(get,never):Float;
  function get_clampingPressure():Float return parameterNumber("clampingPressure");
  public var eccentricity(get,never):Float;
  function get_eccentricity():Float return parameterNumber("eccentricity");
  public var clampingDistortion(get,never):Float;
  function get_clampingDistortion():Float return parameterNumber("clampingDistortion");

  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail=Preview):cadkit.modeling.Part
    {
      var stem=ComponentEnvelopes.annular(values());
      if(parameterNumber("stopDiameter")>0)stem=Solids.union([stem,Solids.cut(cadkit.modeling.Part.cylinderSpan(parameterNumber("stopDiameter")/2,parameterNumber("workSeat")-2,parameterNumber("workSeat")),
        [cadkit.modeling.Part.cylinderSpan(parameterNumber("bore")/2,parameterNumber("workSeat")-2.1,parameterNumber("workSeat")+0.1)])]);
      if(values().boolean("separateSleeve"))stem=Solids.cut(stem,[Solids.cut(
        cadkit.modeling.Part.cylinderSpan(parameterNumber("outside")/2+0.1,parameterNumber("sleeveStart"),parameterNumber("sleeveStart")+parameterNumber("sleeveLength")),
        [cadkit.modeling.Part.cylinderSpan(parameterNumber("outside")/2-parameterNumber("sleeveWall")-parameterNumber("oilCavity"),parameterNumber("sleeveStart")-0.1,parameterNumber("sleeveStart")+parameterNumber("sleeveLength")+0.1)])]);
      if(parameterNumber("baseHeight")==0)return stem;
      if(parameterNumber("baseDiameter")<parameterNumber("outside")||parameterNumber("baseHeight")>=parameterNumber("workSeat"))throw "invalid integral mandrel base";
      var outerBase:cadkit.modeling.Part;
      if(parameterNumber("baseBoltPcd")>0){
        var pieces=[cadkit.modeling.Part.cylinderSpan(parameterNumber("baseDiameter")/2-25,0,parameterNumber("baseHeight"))];
        for(i in 0...6){var a=(i+0.5)*Math.PI/3,r=parameterNumber("baseBoltPcd")/2;pieces.push(cadkit.modeling.Part.cylinderSpan(15,0,parameterNumber("baseHeight"),r*Math.cos(a),r*Math.sin(a)));}
        outerBase=Solids.union(pieces);
      }else outerBase=cadkit.modeling.Part.cylinderSpan(parameterNumber("baseDiameter")/2,0,parameterNumber("baseHeight"));
      var base=Solids.cut(outerBase,[cadkit.modeling.Part.cylinderSpan(parameterNumber("bore")/2,-0.1,parameterNumber("baseHeight")+0.1)]);
      var complete=Solids.union([stem,base]);
      if(parameterNumber("basePilotDiameter")>0){
        if(parameterNumber("basePilotDepth")<=0||parameterNumber("basePilotDiameter")>=parameterNumber("baseDiameter"))throw "invalid provisional fixture pilot";
        complete=Solids.union([complete,Solids.cut(cadkit.modeling.Part.cylinderSpan(parameterNumber("basePilotDiameter")/2,-parameterNumber("basePilotDepth"),0.1),
          [cadkit.modeling.Part.cylinderSpan(Math.max(parameterNumber("baseBore"),parameterNumber("bore"))/2,-parameterNumber("basePilotDepth")-1,1)])]);
      }
      var cuts:Array<cadkit.modeling.Part> = [];
      if(parameterNumber("baseBore")>0){
        if(parameterNumber("baseBoreDepth")<=0||parameterNumber("baseBoreDepth")>=parameterNumber("baseHeight")||parameterNumber("baseBore")>=parameterNumber("baseDiameter"))throw "invalid rotary union recess";
        cuts.push(cadkit.modeling.Part.cylinderSpan(parameterNumber("baseBore")/2,-parameterNumber("basePilotDepth")-1,parameterNumber("baseBoreDepth")));
      }
      if(parameterNumber("baseBoltPcd")>0)for(i in 0...6){var angle=(i+0.5)*Math.PI/3,r=parameterNumber("baseBoltPcd")/2;
        if(r+4.5>=parameterNumber("baseDiameter")/2)throw "provisional M8 face holes outside fixture flange";
        cuts.push(cadkit.modeling.Part.cylinderSpan(4.5,-1,parameterNumber("baseHeight")+1,r*Math.cos(angle),r*Math.sin(angle)));
        cuts.push(cadkit.modeling.Part.cylinderSpan(7,parameterNumber("baseHeight")-9,parameterNumber("baseHeight")+1,r*Math.cos(angle),r*Math.sin(angle)));
      }
      return cuts.length==0?complete:Solids.cut(complete,cuts);
    }
}
