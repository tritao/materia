package machinekit.assembly;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.Plane;
import cadkit.modeling.Location;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.assembly.HzMotionCatalog.HzMotionSpec;
import materia.assembly.AssemblyFrames as F;

/** Drawing-backed purchased assemblies. Interfaces follow the pinned individual
 * drawings; webs, corners, seals and internal bearings are simplified. These are
 * not supplier STEP solids and must not be used to infer close tool clearances.
 */
class HzMotionComponent extends MachineComponent {
 public final spec:HzMotionSpec;
 public final bearingGrade:String;
 static var recipe:Null<ComponentType>;
 public static function recipeType():ComponentType {
  if(recipe==null)recipe=new ComponentType("machinekit.assembly.hzmotion-drawing-component",[
   P.choice("model",HzMotionCatalog.models(),"M60D-BK12-C"),
   P.choice("revision",[HzMotionCatalog.revision()],HzMotionCatalog.revision()),
   P.choice("bearingGrade",["unspecified","C3","C4","C5"],"unspecified")],v->new HzMotionComponent(v.token("model"),v.token("revision"),v.token("bearingGrade")),true,false,null,false);
  return recipe;
 }
 public function new(model:String="M60D-BK12-C",?revision:String,bearingGrade:String="unspecified") {
  if(revision!=null&&revision!=HzMotionCatalog.revision())throw "Unknown HZMotion drawing revision";
  spec=HzMotionCatalog.get(model);
  if(["unspecified","C3","C4","C5"].indexOf(bearingGrade)<0)throw "Unknown bearing grade";
  if(bearingGrade!="unspecified"&&(spec.family=="A"||(spec.family=="BF"||spec.family=="EF")||spec.kind=="adapter"))throw "Part has no selectable fixed-end bearing grade";
  this.bearingGrade=bearingGrade;
  super('HZMOTION-$model-${spec.drawingRevision}-$bearingGrade','HZMotion $model — drawing model',"unspecified");
  lockManufacturingMaterial("unspecified");
  if(spec.kind=="mount"||spec.kind=="adapter") {
   addConnector("motor",Mount,Solids.axial(0,spec.shaftHeight,0));
   var h=spec.pcd/(2*Math.sqrt(2)),i=0;
   for(x in [-h,h])for(y in [-h,h])addConnector('motorBolt${++i}',Face,Solids.axial(x,spec.shaftHeight+y,0));
  }
  addConnector("shaftAxis",Axis,Solids.axial(0,spec.shaftHeight,0));
  addConnector("base",Mount,F.alongY(0,0,spec.length/2,0,1,0));
  for(i in 0...spec.baseHoles.length){var h=spec.baseHoles[i];addConnector('baseBolt${i+1}',Face,F.alongY(h.x,0,h.z,0,1,0));}
  if(spec.shoulderZ!=null)addConnector("screwShoulder",Axis,Solids.axial(0,spec.shaftHeight,spec.shoulderZ));
  if(spec.kind=="adapter")addConnector("body",Mount,Solids.axial(0,0,spec.length));
  if(spec.family=="A")addConnector("fixedSupport",Mount,F.alongY(0,spec.shaftHeight,spec.rearEnd,0,0,-1));
  if(spec.family=="FK")addConnector("mount",Mount,Solids.axial(0,0,10));
  if((spec.family=="BF"||spec.family=="EF")) {
   var half=floatingBearing().width/2;
   addConnector("bearingFront",Axis,Solids.axial(0,spec.shaftHeight,10-half));
   addConnector("bearingBack",Axis,Solids.axial(0,spec.shaftHeight,10+half));
  }
 }
 override public function componentType():Null<ComponentType> return recipeType();
 override public function values():ComponentValues return new ComponentValues().setToken("model",spec.model).setToken("revision",spec.drawingRevision).setToken("bearingGrade",bearingGrade);
 override public function hasGeometry():Bool return true;
 override public function massProperties():MassProperties {
  var publishedMass=spec.massKg;
  if(publishedMass==null)throw 'Published assembly mass unavailable for ${spec.model}; do not assign a homogeneous material to its bearings';
  // Catalog mass, with a geometric-volume centroid estimate; no inertia declaration.
  var p=geometry();try {var center=p.massProperties().centerOfMass;p.close();declareMass(publishedMass,center);}
  catch(e:Dynamic){p.close();throw e;}
  return super.massProperties();
 }
 function floatingBearing():machinekit.standard.DeepGrooveBearing.DeepGrooveBearingSpec {
  var designation=spec.bearing;if(designation==null)throw "Floating support has no bearing designation";
  return machinekit.standard.DeepGrooveBearing.catalog().get(StringTools.replace(designation,"ZZ",""));
 }
 static function box(width:Float,y0:Float,y1:Float,z0:Float,z1:Float,x:Float=0):Part {
  if(!(width>0)||!(y1>y0)||!(z1>z0))throw 'Invalid HZMotion box: $width $y0 $y1 $z0 $z1';
  return Part.prism([new Vector(x-width/2,y0),new Vector(x+width/2,y0),new Vector(x+width/2,y1),new Vector(x-width/2,y1)],z0,z1);
 }
 static function vertical(radius:Float,y0:Float,y1:Float,x:Float,z:Float):Part {
  var raw=Part.cylinderSpan(radius,y0,y1);
  try {var p=raw.placed(new Location(new Plane(new Vector(x,0,z),Vector.X(),Vector.Y())));raw.close();return p;}
  catch(e:Dynamic){raw.close();throw e;}
 }
 override public function geometry(detail:ComponentDetail=Preview):Part {
  return Solids.building([],tracked->{
   function keep(p:Part):Part {tracked.push(p);return p;}
   var s=spec,pieces:Array<Part> = [],cuts:Array<Part> = [],h=s.shaftHeight;
   if(s.kind=="adapter") {
    pieces.push(keep(box(80,-40,40,0,12)));
    pieces.push(keep(Part.cylinderSpan(25,12,15.5)));
    cuts.push(keep(Part.cylinderSpan(23,-1,17)));
    // Recess depth is unpublished; this is an explicitly visual approximation.
    cuts.push(keep(Part.cylinderSpan(35,-1,6)));
   } else if(s.family=="FK") {
    // FK12 installation option B: flange front Z=0, locating boss Z=10..27.
    pieces.push(keep(box(44,-22,22,0,10)));
    pieces.push(keep(Part.cylinderSpan(18,10,27)));
    cuts.push(keep(Part.cylinderSpan(6,-1,28)));
    for(x in [-44/(2*Math.sqrt(2)),44/(2*Math.sqrt(2))])for(y in [-44/(2*Math.sqrt(2)),44/(2*Math.sqrt(2))])
     cuts.push(keep(Part.cylinderSpan(2.25,-1,11,x,y)));
   } else if((s.family=="BF"||s.family=="EF")) {
    var side=s.family=="EF"?24.0:s.model=="BF12"?32.5:38.0;
    pieces.push(keep(box(s.width,0,side,0,20)));
    pieces.push(keep(box(s.rearWidth,0,s.rearHeight,0,20)));
    cuts.push(keep(Part.cylinderSpan(s.shaftBore/2,-1,21,0,h)));
    // Keep the actual bearing's narrow bore only over its own width. Housing
    // recesses must admit the screw race/shoulder outside that bearing interval.
    var bearing=floatingBearing();
    cuts.push(keep(Part.cylinderSpan(bearing.outside/2,-1,10-bearing.width/2,0,h)));
    cuts.push(keep(Part.cylinderSpan(bearing.outside/2,10+bearing.width/2,21,0,h)));
   } else {
    var bottom=h-s.face/2;
    if(s.family=="KK")pieces.push(keep(box(s.width,0,s.rearHeight,0,s.flange)));
    else pieces.push(keep(box(s.face,bottom,h+s.face/2,0,s.flange)));
    var web=(s.width-s.rearWidth)/2;
    if(s.family=="KK"||s.family=="A")web=8; // Undimensioned side-wall thickness; approximation.
    var inner=s.width/2-web;
    for(side in [-1,1])pieces.push(keep(box(web,0,s.family=="KK"?10:24,s.flange,s.length,side*(inner+web/2))));
    pieces.push(keep(box(s.rearWidth,0,s.rearHeight,s.rearStart,s.rearEnd)));
    cuts.push(keep(Part.cylinderSpan(s.pilot/2,-1,s.flange+0.01,0,h)));
    cuts.push(keep(Part.cylinderSpan(s.shaftBore/2,s.rearStart-1,s.rearEnd+1,0,h)));
    if(s.family=="A")cuts.push(keep(Part.cylinderSpan(18,s.rearStart-1,s.rearEnd+1,0,h)));
   }
   if(s.kind=="mount"||s.kind=="adapter") {
    var half=s.pcd/(2*Math.sqrt(2)),root=Std.parseFloat(s.thread.substr(1))==6?2.5:2.1;
    for(x in [-half,half])for(y in [-half,half])cuts.push(keep(Part.cylinderSpan(root,-1,s.flange+1,x,h+y)));
   }
   for(hole in s.baseHoles)cuts.push(keep(vertical(hole.diameter/2,-1,s.rearHeight+1,hole.x,hole.z)));
   var all=keep(Solids.union(pieces));return Solids.cut(all,cuts);
  });
 }
}
