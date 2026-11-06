package machinekit.standard;
import cadkit.modeling.Part;
import machinekit.component.*;
import materia.assembly.AssemblyFrames as F;

/** Assumed procurement envelope, not a vendor claim. A moulded NBR push-in
 * plug closes a 10 mm tool-access port in 2 mm sheet. Compliant retention
 * barbs are omitted from the nominal collision envelope. */
class PanelHolePlug extends MachineComponent {
  public static inline final HOLE=10.0;
  public static inline final PANEL=2.0;
  public static inline final FLANGE=12.0;
  public static inline final FLANGE_THICKNESS=1.5;
  public static inline final STEM=9.8;
  public static inline final STEM_LENGTH=3.0;
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public static function parameters():Array<ComponentParameter> return EngineeringMetadata.parameters();
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType("machinekit.standard.panel-hole-plug",parameters(),v->new PanelHolePlug(v),false,false,null,false)
      .numbered(_->"HOLE-PLUG-ASSUMED-D10-P2-NBR").procured(_->Buy(null,null,null)).requiring(_->{
        var out=[for(p in [{name:"holeDiameter",value:HOLE},{name:"panelThickness",value:PANEL},{name:"flangeDiameter",value:FLANGE},
          {name:"flangeThickness",value:FLANGE_THICKNESS},{name:"stemDiameter",value:STEM},{name:"stemLength",value:STEM_LENGTH}])
          machinekit.component.Procurement.ProcurementCodec.requirement(p.name,"mm",null,null,p.value)];
        out.push(machinekit.component.Procurement.ProcurementCodec.choice("material","NBR, oil-compatible grade to qualify"));return out;
      }));
  }
  public function new(?overrides:ComponentValues) {
    var v=machinekit.component.ComponentRecipeSupport.resolve(parameters(),overrides);
    v.setToken("ratingsBasis","Assumed").setToken("errorsBasis","Assumed").setToken("source","FRAME_PLAN P4: assumed panel-plug procurement envelope");
    super("HOLE-PLUG-ASSUMED-D10-P2-NBR","Push-in NBR tool-port plug, assumed D10 / panel 2","nitrile rubber NBR",false,v);
    addFacet(new EngineeringMetadata(v));addConnector("mount",Mount,Solids.axial(0,0,0));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function fastensByForm():Bool return true;
  public static function holeTool(depth:Float):Part return Part.cylinderSpan(HOLE/2,-0.1,depth+0.1);
  override public function mountingFace():Null<MountingFace> return new MountingFace(FLANGE,FLANGE,holeTool,
    F.fromRotationMatrix(0,0,0,[1.0,0,0,0,-1,0,0,0,-1]));
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail=Preview):Part return Solids.union([
    Part.cylinderSpan(STEM/2,-STEM_LENGTH,0),Part.cylinderSpan(FLANGE/2,0,FLANGE_THICKNESS)
  ]);
}
