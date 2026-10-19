package machinekit.motion;
import cadkit.modeling.*;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.motion.SfuFamilyData.DsgHousingReference;
import machinekit.standard.SocketHeadCapScrew;
import materia.assembly.AssemblyFrames as F;

/** DSG housing: base Y=0, shaft +Z at Y=h, nut flange seats on Z=0.
 * SFU nut must be rotated 90 degrees about Z so its flats match housing height.
 * Published tables omit blind thread depths: caller supplies them for clearance/engagement modelling.
 */
class DsgNutHousing extends MachineComponent {
 static var recipe:Null<ComponentType>;
 public final spec:DsgHousingReference;
 public final baseThreadDepth:Float;
 public final flangeThreadDepth:Float;
 public function new(model:String="DSG16H",baseThreadDepth:Float=0,flangeThreadDepth:Float=0) {
  var s=SfuFamilyData.housing(model);
  if(!Math.isFinite(baseThreadDepth+flangeThreadDepth) || baseThreadDepth<=0 || flangeThreadDepth<=0 || baseThreadDepth>s.height || flangeThreadDepth>s.length)throw "DSG needs explicit finite thread depths within the housing";
  super("DSG-"+model+"-BD"+Dimension.format(baseThreadDepth)+"-FD"+Dimension.format(flangeThreadDepth),"DSG SFU nut housing "+model,"unspecified");
  spec=s;this.baseThreadDepth=baseThreadDepth;this.flangeThreadDepth=flangeThreadDepth;lockManufacturingMaterial("unspecified");
  addConnector("base",Mount,F.translation(0,0,0));
  addConnector("shaft",Axis,Solids.axial(0,s.shaftHeight,0));
  addConnector("nutFace",Mount,Solids.axial(0,s.shaftHeight,0));
  var i=1;
  for(x in [-s.holePitchX/2,s.holePitchX/2])for(z in [s.firstHole,s.firstHole+s.holePitchZ])addConnector("baseBolt"+i++,Mount,F.alongY(x,0,z,0,-1,0));
  i=1;for(p in flangePattern())addConnector("nutBolt"+i++,Mount,Solids.axial(p.x,s.shaftHeight+p.y,0));
 }
 public function flangePattern():Array<{x:Float,y:Float}> return [for(a in [135,180,225,315,0,45]){x:spec.flangePcd/2*Math.cos(a*Math.PI/180),y:spec.flangePcd/2*Math.sin(a*Math.PI/180)}];
 override public function hasGeometry():Bool return true;
 override public function geometry(detail:ComponentDetail=Preview):Part {
  var s=spec,w=s.width/2,h=s.height,e=s.chamfer;
  var body=Part.prism([new Vector(-w,0),new Vector(w,0),new Vector(w,h-e),new Vector(w-e,h),new Vector(-w+e,h),new Vector(-w,h-e)],0,s.length);
  return Solids.building([body],owned -> {
   var bore=Part.cylinderSpan(s.bore/2,-0.1,s.length+0.1,0,s.shaftHeight);owned.push(bore);var tools=[bore];
   if(detail!=Envelope) {
    for(x in [-s.holePitchX/2,s.holePitchX/2])for(z in [s.firstHole,s.firstHole+s.holePitchZ]) {
     var tap=SocketHeadCapScrew.metric(s.baseThread,baseThreadDepth).tapHole(baseThreadDepth);
     var posed=tap.placed(new Location(new Plane(new Vector(x,0,z),Vector.X(),Vector.Y().scale(-1))));tap.close();var named=Solids.named(posed,"tap-"+tools.length);owned.push(named);tools.push(named);
    }
    for(p in flangePattern()) {
     var tap=SocketHeadCapScrew.metric(s.flangeThread,flangeThreadDepth).tapHole(flangeThreadDepth);
     var posed=tap.placed(new Location(new Plane(new Vector(p.x,s.shaftHeight+p.y,0),Vector.X(),Vector.Z().scale(-1))));tap.close();var named=Solids.named(posed,"tap-"+tools.length);owned.push(named);tools.push(named);
    }
   }
   return Solids.cut(body,tools);
  });
 }
 public static function familyRecipeType():ComponentType {
  if(recipe==null)recipe=new ComponentType("machinekit.motion.dsg-nut-housing",[P.choice("model",SfuFamilyData.housingModels(),"DSG16H"),P.length("baseThreadDepth",10),P.length("flangeThreadDepth",10)],v -> new DsgNutHousing(v.token("model"),v.number("baseThreadDepth"),v.number("flangeThreadDepth")),true,false,null,false);
  return recipe;
 }
 override public function componentType():Null<ComponentType> return familyRecipeType();
 override public function values():ComponentValues return new ComponentValues().setToken("model",spec.model).setNumber("baseThreadDepth",baseThreadDepth).setNumber("flangeThreadDepth",flangeThreadDepth);
}
