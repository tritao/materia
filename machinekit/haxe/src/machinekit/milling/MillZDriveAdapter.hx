package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** One flat T plate. Origin at column rail tops, centre X/Y.
 * Rear face +Y supports the pinned M80E STEP base; front M6 blind taps.
 * Stem clears both rails. Plate strength and casting anchors remain unqualified.
 */
class MillZDriveAdapter extends MachineComponent {
 static var recipe:Null<ComponentType>;
 static function parameters():Array<ComponentParameter> return [P.length("motorOffset",60),P.length("stemHeight",60),P.length("width",220),P.length("spacing",180),P.length("thickness",20),P.length("stemWidth",110)];
 public function new(?values:ComponentValues) {
  var v=P.resolve(parameters(),values);
  if(v.number("width")<=v.number("spacing")+12 || v.number("stemWidth")>=v.number("spacing")-30 || v.number("thickness")<16 || v.number("stemHeight")<114-v.number("motorOffset") || v.number("motorOffset")>76)throw "Z adapter needs rail and anchor clearance";
  super("MILL-Z-DRIVE-ADAPTER","Flat T adapter for HZMotion M80E-BK15-C","steel",false,v);
  addConnector("mount",Mount,Solids.axial(0,0,0));
 }
 public static function recipeType():ComponentType {
  if(recipe==null)recipe=new ComponentType("machinekit.milling.z-drive-adapter",parameters(),v->new MillZDriveAdapter(v));return recipe;
 }
 override public function componentType():Null<ComponentType> return recipeType();
 override public function hasGeometry():Bool return true;
 override public function geometry(detail:ComponentDetail=Preview):Part {
  var t=parameterNumber("thickness"),body=Solids.union([Part.box(parameterNumber("width"),t,40),Part.box(parameterNumber("stemWidth"),t,parameterNumber("stemHeight")).translated(new Vector(0,0,-parameterNumber("stemHeight")))]);
  var tools:Array<Part> =[];
  for(side in [-1,1])tools.push(Part.cylinderAlongY(3.3,-t/2-1,t/2+1,side*parameterNumber("spacing")/2,10));
  var asset=machinekit.assembly.HzMotionCatalog.cadAsset("M80E-BK15-C");
  if(asset.interfaces==null)throw "Z adapter requires measured STEP holes";
  for(h in asset.interfaces.baseHoles)tools.push(Part.cylinderAlongY(2.5,t/2-15,t/2+1,-h.x,parameterNumber("motorOffset")-h.z));
  return Solids.cut(body,tools);
 }
}
