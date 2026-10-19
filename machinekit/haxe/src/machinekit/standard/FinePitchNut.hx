package machinekit.standard;
import cadkit.modeling.Part;
import cadkit.threads.ThreadSpec;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Explicit fine-pitch hex retaining nut. Dimensions are design inputs, not ISO4032 conformance.
 * The SFU20 example uses BK15's SW22 wrench size and an assumed 6 mm height.
 * Anti-loosening treatment and preload are installation specifications, not inferred geometry.
 */
class FinePitchNut extends MachineComponent {
 public final diameter:Float;
 public final pitch:Float;
 public final acrossFlats:Float;
 public final height:Float;
 static var recipe:Null<ComponentType>;
 public function new(diameter:Float=15,pitch:Float=1,acrossFlats:Float=22,height:Float=6) {
  if(!Math.isFinite(diameter+pitch+acrossFlats+height) || diameter<=0 || pitch<=0 || acrossFlats<=diameter || height<=0)throw "Invalid fine-pitch nut dimensions";
  super('FINE-NUT-M${Dimension.format(diameter)}x${Dimension.format(pitch)}-AF${Dimension.format(acrossFlats)}-H${Dimension.format(height)}',"Fine-pitch retaining nut (explicit dimensions)","steel 8");
  this.diameter=diameter;this.pitch=pitch;this.acrossFlats=acrossFlats;this.height=height;
  addConnector("front",Face,Solids.axial(0,0,0));addConnector("back",Face,Solids.axial(0,0,height));
 }
 override public function hasGeometry():Bool return true;
 override public function geometry(detail:ComponentDetail=Preview):Part {
  var thread=new ThreadSpec('M${Dimension.format(diameter)}x${Dimension.format(pitch)}',diameter,pitch,1,RightHand,true);
  return HexNut.custom({size:thread.designation,diameter:diameter,acrossFlats:acrossFlats,height:height},thread).geometry(detail);
 }
 public static function recipeType():ComponentType {
  if(recipe==null)recipe=new ComponentType("machinekit.standard.fine-pitch-nut",[P.length("diameter",15),P.length("pitch",1),P.length("acrossFlats",22),P.length("height",6)],v -> new FinePitchNut(v.number("diameter"),v.number("pitch"),v.number("acrossFlats"),v.number("height")),true);
  return recipe;
 }
 override public function componentType():Null<ComponentType> return recipeType();
 override public function values():ComponentValues return new ComponentValues().setNumber("diameter",diameter).setNumber("pitch",pitch).setNumber("acrossFlats",acrossFlats).setNumber("height",height).setToken("material",materialSpec());
}
