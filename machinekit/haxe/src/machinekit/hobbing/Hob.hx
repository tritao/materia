package machinekit.hobbing;

import gearkit.*;
import machinekit.motion.PrecisionErrors;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Physical tool envelope, generated from the same rack/HobDesign as the rake cutter.
    The optional detailed B-rep uses its rake sections, with explicit assumed relief. */
class Hob extends MachineComponent {
  public final design:HobDesign;
  public final target:GearSpec;
  public static function parameters():Array<ComponentParameter>return EngineeringMetadata.parameters().concat([
    P.length("outside",32),P.length("bore",16),P.length("height",40),P.scalar("module",0.3,0.000001),
    P.count("teeth",200),P.length("faceWidth",8),P.scalar("pressureAngle",20*Math.PI/180,0.000001),
    P.scalar("profileShift",0),P.length("cutterTipRadius",0.114),P.length("tipFilletRadius",0.015),
    P.count("starts",1),P.count("gashes",12),P.scalar("rake",0),P.scalar("rootHeight",0.3),
    P.flag("detailed",false),P.flag("renderTeeth",false),P.scalar("landFraction",0.6,0.05),P.scalar("reliefAngle",3*Math.PI/180,0),
    P.length("fluteExtraDepth",0.5),P.length("edgeTolerance",0.0005),P.length("surfaceTolerance",0.01),P.length("keywayWidth",0),P.length("keywayDepth",2.3),
    P.length("radialRunout",0.002),P.length("axialRunout",0.002),P.length("profileError",0.001)]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.hob",parameters(),v -> new Hob(v));return recipe;
  }
  public function new(?overrides:ComponentValues) {
    var v=P.resolve(parameters(),overrides);
    super("HOB-"+Dimension.format(v.number("outside"))+"-"+v.integer("gashes")+"-"+v.integer("starts"),"Conjugate rack hob; optional detailed relieved teeth","carbide",false,v);
    if(v.get("material")!=null)setMaterial(v.token("material"));
    ComponentEnvelopes.validateAnnular(v);
    addConnector("mount",Mount,Solids.axial(0,0,0));
    addConnector("axis",Axis,Solids.axial(0,0,0));
    addConnector("top",Mount,Solids.axial(0,0,parameterNumber("height")));
    addFacet(new EngineeringMetadata(v));
    addFacet(PrecisionErrors.fromValues(v,[{name:"radialRunout",unit:"mm"},{name:"axialRunout",unit:"mm"},{name:"profileError",unit:"mm"}]));
    target=new GearSpec(parameterNumber("module"),parameterCount("teeth"),parameterNumber("faceWidth"),parameterNumber("pressureAngle"),false,parameterNumber("profileShift"),parameterNumber("cutterTipRadius"),parameterNumber("tipFilletRadius"));
    design=new HobDesign(ConjugateRack.of(new ToothProfile(target)),parameterCount("starts"),parameterNumber("outside"),parameterCount("gashes"),parameterNumber("rake"),parameterNumber("rootHeight"));
    if(parameterNumber("bore")>=design.pitchDiameter-2*design.rootHeight)throw "hob bore breaks into cutter roots";
    if(parameterNumber("landFraction")>=0.95||parameterNumber("reliefAngle")>=Math.PI/6||parameterNumber("edgeTolerance")<=0||parameterNumber("surfaceTolerance")<=0||parameterNumber("surfaceTolerance")>=design.outsideDiameter/4)throw "invalid hob land geometry/tolerance";
    if(parameterNumber("keywayWidth")>0&&(parameterNumber("keywayWidth")>=parameterNumber("bore")||parameterNumber("keywayDepth")+parameterNumber("bore")/2>=design.pitchDiameter/2-design.rootHeight-parameterNumber("fluteExtraDepth")))throw "hob keyway breaks into cutter roots";
    if(v.boolean("detailed")||v.boolean("renderTeeth")){
      var core=design.pitchDiameter/2-design.rootHeight-parameterNumber("fluteExtraDepth");
      if(parameterNumber("bore")/2>=core)throw "hob flutes break into bore";
      if(core<=design.outsideDiameter/2*Math.abs(Math.sin(design.rake)))throw "hob flute floor misses rake plane";
      if(parameterNumber("height")/design.axialPitch>512)throw "detailed hob exceeds 512 teeth per land";
    }
    if(v.boolean("renderTeeth")&&!v.boolean("detailed"))addFacet(new ComponentDisplayMesh(()->HobMesh.build(this)));
    addConnector("centre",Axis,Solids.axial(0,0,parameterNumber("height")/2));
  }
  /** Captures the supplied design and enough generating geometry for an exact recipe rebuild. */
  public static function fromDesign(design:HobDesign,target:GearSpec,length:Float=40,bore:Float=16,detailed:Bool=false,renderTeeth:Bool=false):Hob {
    if(target.internal)throw "hob tool part requires an external generating target";
    var v=new ComponentValues().setBoolean("detailed",detailed).setBoolean("renderTeeth",renderTeeth).setNumber("outside",design.outsideDiameter).setNumber("height",length).setNumber("bore",bore)
      .setNumber("module",target.module).setInteger("teeth",target.teeth).setNumber("faceWidth",target.faceWidth)
      .setNumber("pressureAngle",target.pressureAngle).setNumber("profileShift",target.profileShift)
      .setNumber("cutterTipRadius",target.cutterTipRadius).setNumber("tipFilletRadius",target.tipFilletRadius)
      .setInteger("starts",design.starts).setInteger("gashes",design.gashes).setNumber("rake",design.rake).setNumber("rootHeight",design.rootHeight);
    var result=new Hob(v);
    // Reject a target that happens to share module/angle but has a different recovered rack.
    for(g in 0...design.gashes){var a=design.rakeFaceRegion(g),b=result.design.rakeFaceRegion(g);
      if(a.length!=b.length)throw "hob design and target rack differ";
      for(i in 0...a.length)if(Math.abs(a[i].x-b[i].x)+Math.abs(a[i].y-b[i].y)+Math.abs(a[i].z-b[i].z)>1e-8)throw "hob design and target rack differ";}
    return result;
  }
  override public function componentType():Null<ComponentType>return recipeType();
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail=Preview):cadkit.modeling.Part
    return detail==Envelope||!values().boolean("detailed")?ComponentEnvelopes.annular(values()):HobSolid.build(this);
}
