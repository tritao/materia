package machinekit.milling;
import cadkit.modeling.*;
import cadkit.threads.Threads;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.standard.SocketHeadCapScrew;
/** Fabricated plate: four table through holes and four top taps, centred on rectangular patterns. */
class MillFixtureAdapter extends MachineComponent {
 static var recipe:Null<ComponentType>;
 public final height:Float;
 public final pneumatic:Bool;
 public function new(height:Float=20,pneumatic:Bool=false) {
  if(!Math.isFinite(height)||height<18)throw "Fixture adapter needs at least 18 mm thickness";
  this.height=height;this.pneumatic=pneumatic;super("MILL-FIXTURE-ADAPTER-"+(pneumatic?"200x190-128":"180x190-82")+"x"+height,"Tapped workholding table adapter","steel");
  addConnector("mount",Mount,Solids.axial(0,0,0));
 }
 override public function hasGeometry():Bool return true;
 override public function geometry(detail:ComponentDetail=Preview):Part {
  var body=Part.box(pneumatic?200:180,190,height),cuts:Array<Part> = [];
  for(x in pneumatic?[-85,85]:[-70,70])for(y in [-80,80])cuts.push(Part.cylinderSpan(3.3,-1,height+1,x,y));
  var bolt=SocketHeadCapScrew.metric("M10",25);
  for(x in pneumatic?[-64,64]:[-41,41])for(y in pneumatic?[-64,64]:[-41,41]) {
   var tap=Threads.tappedBore(bolt.thread.mating(),bolt.spec.tapDrill,16);
   var placed=tap.translated(new Vector(x,y,height));tap.close();
   cuts.push(Solids.named(placed,"tap"+x+"-"+y));
  }
  return Solids.cut(body,cuts);
 }
 public static function recipeType():ComponentType {
  if(recipe==null)recipe=new ComponentType("machinekit.milling.fixture-adapter",[P.length("height",20),P.choice("pattern",["82-square","128-square"],"82-square")],v->new MillFixtureAdapter(v.number("height"),v.token("pattern")=="128-square"));
  return recipe;
 }
 override public function componentType():Null<ComponentType> return recipeType();
 override public function values():ComponentValues return new ComponentValues().setNumber("height",height).setToken("pattern",pneumatic?"128-square":"82-square");
}
