package machinekit.enclosure;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;

/** Removable stepped toe clamp. Local +X points over the panel; M6 clearance bore is explicit. */
class TrayClamp extends MachineComponent {
  static var recipe:Null<ComponentType>;
  public final height:Float;
  public function new(height:Float=TraySeat.TOP) {
    if(!(height>0)||!Math.isFinite(height))throw "Clamp needs a finite positive seat height";
    super("TRAY-TOE-CLAMP-M6-"+height,"Stepped laminate tray toe clamp, M6 clearance","steel");
    this.height=height;
    addConnector("mount",Mount,Solids.axial(0,0,0));
    addConnector("head",Mount,Solids.axial(0,0,height+4));
  }
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.enclosure.tray-toe-clamp",[ComponentRecipeSupport.length("height",TraySeat.TOP)],v->new TrayClamp(v.number("height")),true);
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function values():ComponentValues return new ComponentValues().setNumber("height",height).setToken("material",materialSpec());
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail=Preview):Part {
    var rawHeel=Part.box(20,20,height),rawToe=Part.box(40,20,4);
    var heel=rawHeel.translated(new Vector(-5,0,0)),toe=rawToe.translated(new Vector(5,0,height));
    rawHeel.close();rawToe.close();
    return Solids.cut(Solids.union([heel,toe]),[Part.cylinderSpan(3.3,-1,height+5)]);
  }
}
