package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Steel Y-drive risers, mm. Z up, Y toward motor. Separate receiving holes
 * match HZMotion's checked pattern; external anchor bores meet the frame insert.
 * Tap bores are nominal machining requirements, not a joint strength calculation.
 */
class MillYDriveSeat extends MachineComponent {
 public final floating:Bool;
 static var recipe:Null<ComponentType>;
 public static function recipeType():ComponentType {
  if(recipe==null)recipe=new ComponentType("machinekit.milling.hzmotion-y-drive-seat",[P.flag("floating",false)],v->new MillYDriveSeat(v.boolean("floating")));
  return recipe;
 }
 public function new(floating:Bool=false){
  this.floating=floating;
  super(floating?"MILL-Y-BF12-SEAT-v1":"MILL-Y-BK12-C-SEAT-v1","Machined steel seat for selected HZMotion Y drive","steel");
  addConnector("mount",Mount,Solids.axial(0,0,0));
 }
 override public function componentType():Null<ComponentType> return recipeType();
 override public function values():ComponentValues return new ComponentValues().setBoolean("floating",floating);
 override public function hasGeometry():Bool return true;
 override public function geometry(detail:ComponentDetail=Preview):Part {
  var depth=floating?20.0:83.0;
  var body=Part.box(100,depth,15),tools:Array<Part> = [];
  if(!floating)tools.push(Part.box(101,14.01,16).translated(new Vector(0,depth/2-7,0)));
  if(floating)for(x in [-23.0,23])tools.push(Part.cylinderSpan(2.5,0,16,x,0));
  else for(hole in machinekit.assembly.HzMotionCatalog.get("M60D-BK12-C").baseHoles)
   tools.push(Part.cylinderSpan(2.5,0,16,hole.x,41.5-hole.z));
  for(x in [-42.0,42])tools.push(Part.cylinderSpan(3.3,-1,16,x,floating?0:-10));
  return Solids.cut(body,tools);
 }
}
