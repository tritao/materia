import machinekit.assembly.*;
import machinekit.component.*;
import machinekit.motion.*;
import cadkit.modeling.Part;
import materia.assembly.AssemblyFrames as F;

class HzMotionChecks {
 static function check(ok:Bool,label:String):Void {if(!ok)throw label;}
 static function near(a:Float,b:Float,label:String):Void {check(Math.isFinite(a)&&Math.abs(a-b)<1e-5,'$label: $a != $b');}
 static function overlap(a:Part,b:Part):Float {
  var common=a.intersect(b);try {var volume=common.volume();common.close();return volume;}catch(e:Dynamic){common.close();throw e;}
 }
 public static function main():Void {
  var registry=MachineKitComponents.standard(),rows:Array<Dynamic> = [];
  check(registry.has(HzMotionComponent.recipeType().id),"HZMotion recipe is in the standard registry");
  check(HzMotionCatalog.coverage().length==135,"Full six-family coverage index retained");
  for(model in HzMotionCatalog.models()) {
   var component=new HzMotionComponent(model),s=component.spec;
   Sys.println('Checking $model: face=${s.face} width=${s.width} flange=${s.flange} height=${s.shaftHeight}');
   var p=component.geometry();
   check(p.valid(),'Valid drawing solid: $model');check(p.solidCount()==1,'One connected purchased component: $model');
   if(s.kind=="mount"||s.kind=="adapter") {
    var depth=s.pilotDepth==null?3.0:Math.min(3,s.pilotDepth);
    var probe=Part.cylinderSpan(s.pilot/2-0.1,0.1,depth-0.1,0,s.shaftHeight);
    near(overlap(p,probe),0,'Pilot passage $model');probe.close();
   }
   for(hole in s.baseHoles) {
    var id='baseBolt${s.baseHoles.indexOf(hole)+1}',datum=component.connector(id).frame;
    near(datum.x,hole.x,"Base hole X datum");near(datum.z,hole.z,"Base hole Z datum");
    var probe=Part.cylinderAlong(hole.diameter/2-0.1,new cadkit.modeling.Vector(hole.x,-1,hole.z),cadkit.modeling.Vector.Y(),s.rearHeight+2);
    near(overlap(p,probe),0,'Actual base bore $model/$id');probe.close();
   }
   var publishedMass=s.massKg;
   if(publishedMass==null) {
    var rejected=false;try component.massProperties()catch(_:Dynamic)rejected=true;
    check(rejected,"Unknown supplier mass must not become an engineering estimate");
   }else near(component.massProperties().mass,publishedMass,"Published purchased assembly mass");
   var saved=component.componentType().create(component.values());
   check(saved.designation==component.designation,'Recipe identity $model');
   rows.push({model:model,solidCount:p.solidCount(),volumeMm3:p.volume(),source:HzMotionCatalog.sourceUrl(model),geometryBasis:s.geometryBasis});p.close();
  }
  for(model in HzMotionCatalog.cadModels()) {
   if(model=="M60D-BK12-C"||model=="M80E-BK15-C")continue;
   var component=new HzMotionComponent(model,null,"unspecified","supplier-step"),s=component.spec,p=component.geometry();
   check(p.valid()&&p.solidCount()>1,"Supplier bearing assembly keeps separate solids");
   var bounds=p.shape.bounds();
   near(bounds.get_min().get_y(),0,"Normalized supplier base plane");
   near(bounds.get_min().get_z(),0,"Normalized supplier front plane");
   near(bounds.get_max().get_z(),20,"Normalized supplier support length");
   for(hole in s.baseHoles){
    var probe=Part.cylinderAlong(hole.diameter/2-0.1,new cadkit.modeling.Vector(hole.x,-1,hole.z),cadkit.modeling.Vector.Y(),s.rearHeight+2);
    near(overlap(p,probe),0,"Supplier base hole matches connector");probe.close();
   }
   var bore=Part.cylinderSpan(s.shaftBore/2-0.1,6.5,13.5,0,s.shaftHeight);
   near(overlap(p,bore),0,"Supplier shaft passage matches connector");bore.close();
   var rebuilt=component.componentType().create(component.values());
   check(rebuilt.designation==component.designation,"Saved supplier recipe retains pinned identity");
   var rebuiltType:ComponentType=rebuilt.componentType();
   check(rebuiltType.id!=HzMotionComponent.recipeType().id,"Drawing recipe remains distinct");
   var unknown=false;try component.massProperties()catch(_:Dynamic)unknown=true;
   check(unknown,"Detailed STEP does not invent engineering mass");p.close();
  }
  var blocked=false;try new HzMotionComponent("M60D-EK12-C",null,"unspecified","supplier-step")catch(_:Dynamic)blocked=true;
  check(blocked,"Discrepant mount STEP is excluded from recipes");
  var oldDrawing=new HzMotionComponent("M60D-BK12-C","hzmotion-pdf-2025-04-09-v1");
  near(oldDrawing.spec.baseHoles[2].x,-25,"Saved v1 drawing retains its historical transcription");
  check(oldDrawing.componentType().create(oldDrawing.values()).designation==oldDrawing.designation,"Legacy drawing revision still reconstructs");
  var copy=HzMotionCatalog.get("M60D-BK12-C");copy.pilot=999;
  near(HzMotionCatalog.get("M60D-BK12-C").pilot,50,"Catalog records cannot mutate the shared source");
  var modular=new MachineAssembly();modular.addComponent("bracket",new HzMotionComponent("M60D-FK12-A"));
  modular.addComponent("bearing",new HzMotionComponent("FK12"));
  modular.addMate("bearing-seat","fixed","bracket","fixedSupport","bearing","mount");
  modular.validate();
  var solved:Map<String,materia.assembly.AssemblyRecord.AssemblyFrame> = modular.solvedPoses();
  var cache=new PosedParts(),members=modular.components();
  near(cache.overlap(members.filter(m->m.id=="bracket")[0].component,solved.get("bracket"),members.filter(m->m.id=="bearing")[0].component,solved.get("bearing")),0,"Modular FK12 pilot fits bracket rear seat");cache.close();
  var kk=new MachineAssembly();kk.addComponent("body",new HzMotionComponent("KK86-FK10L"));
  kk.addComponent("adapter",new HzMotionComponent("KK86-80EP-adapter"));
  kk.addMate("adapter-seat","fixed","body","motor","adapter","body");kk.validate();
  var kkPoses:Map<String,materia.assembly.AssemblyRecord.AssemblyFrame> = kk.solvedPoses();
  var kkCache=new PosedParts(),kkMembers=kk.components();
  near(kkCache.overlap(kkMembers.filter(m->m.id=="body")[0].component,kkPoses.get("body"),
   kkMembers.filter(m->m.id=="adapter")[0].component,kkPoses.get("adapter")),0,"KK adapter locating spigot seats without interference");kkCache.close();
  var zFindings=HzMotionCatalog.motorFindings("KK86-80EP-adapter",LeadshineServo.model("ELM2H-0750LA80E"));
  check(zFindings.length==2,"Z locating dimensions match but pilot depth and M5 joint remain unqualified");
  var motor=LeadshineServo.model("ELM2H-0400LA60F");
  var findings=HzMotionCatalog.motorFindings("M60D-BK12-C",motor);
  check(findings.length==1&&findings[0].indexOf("thread depth")>=0,"400 W nominal mounting dimensions match; thread depth remains unknown");
  check(HzMotionCatalog.motorFindings("M80E-BK15-C",motor).length>1,"Wrong frame is rejected");
  var detailed=new HzMotionAxis(true,"as-supplied"),detailedPoses:Map<String,materia.assembly.AssemblyRecord.AssemblyFrame> = detailed.solvedPoses();
  var detailedParts=detailed.components(),detailedCache=new PosedParts();
  var interference=detailedCache.overlap(detailedParts.filter(m->m.id=="floating")[0].component,detailedPoses.get("floating"),
   detailedParts.filter(m->m.id=="screw")[0].component,detailedPoses.get("screw"));
  detailedCache.close();
  check(interference>3.6&&interference<3.8,"Supplier BF12 retaining clip interference remains visible; do not silently replace assembly geometry");
  rows.push({model:"BF12",supplierScrewIntersectionMm3:interference,status:"Blocked: STEP retaining clip intersects drawing screw groove"});
  HzMotionAxis.checkAndWrite(rows);
  Sys.println("HZMotion catalog geometry, compatibility and drive-stack checks passed");
 }
}
