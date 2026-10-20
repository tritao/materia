package machinekit.structural;
import cadkit.modeling.Part;
import cadkit.modeling.Polygon;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
/** Removable steel bearing plate and separate nominal M24x3 support/lock nuts.
 * Nut dimensions are assumed envelopes; threads are semantic, not helical solids.
 */
class LevelingSeat extends MachineComponent {
  public static inline final PLATE_WIDTH=110.0;
  static var recipe:Null<ComponentType>;
  public static function parameters():Array<ComponentParameter>return [P.choice("variant",["plate","nut"],"plate")];
  public static function recipeType():ComponentType {if(recipe==null)recipe=new ComponentType("machinekit.structural.leveling-seat",parameters(),v->new LevelingSeat(v));return recipe;}
  public function new(?overrides:ComponentValues){var v=P.resolve(parameters(),overrides),plate=v.token("variant")=="plate";
    super(plate?"LEVELING-PLATE-110-15":"LEVELING-NUT-M24X3-ASSUMED",plate?"Removable ground steel foot load plate, 26 mm clearance bore":"M24x3 support/lock nut; assumed 36 mm flats, 19 mm height","steel",false,v);
    addConnector("mount",Mount,Solids.axial(0,0,0));addConnector("top",Mount,Solids.axial(0,0,plate?15:19));
  }
  override public function componentType():Null<ComponentType>return recipeType();
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail=Preview):Part {
    var plate=parameterToken("variant")=="plate",h=plate?15.0:19.0;
    return Solids.cut(plate?Part.box(PLATE_WIDTH,PLATE_WIDTH,h):Part.prism(Polygon.regular(6,36),0,h),
      [Part.cylinderSpan(plate?13:12,-1,h+1)]);
  }
}
