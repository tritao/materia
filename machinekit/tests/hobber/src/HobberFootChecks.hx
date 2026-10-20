import machinekit.hobbing.FoundationLayout;
import machinekit.hobbing.FoundationLayout as Pocket;
import machinekit.structural.LevelingSeat;
import machinekit.component.MachineKitComponents;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.PosedParts;
import materia.assembly.AssemblyFrames as F;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
/** Verify the load-bearing roof, wrench access and plate extraction using exact solids. */
class HobberFootChecks {
  public static function run():Void {
    var n=0;
    function check(ok:Bool,label:String):Void {n++;if(!ok)throw label;}
    var machine=new FlexsplineHobber(),members=machine.components(),feet=FoundationLayout.feet();
    check(feet.length==3,"three primary support points");
    check(machine.bed.insertLocations().filter(p->StringTools.startsWith(p.id,"foot")).length==0,"no permanently embedded foot anchors");
    var restored=MachineAssembly.decode(machine.encode(),MachineKitComponents.standard());
    check(restored.components().length==members.length,"separate foot hardware persists");
    var body=machine.bed.body.geometry(),plate=new LevelingSeat(),cache=new PosedParts();
    var plateShape=plate.geometry(),began=Sys.time();
    function volumeInBody(shape:Part):Float {var common=shape.intersect(body);var v=common.volume();common.close();return v;}
    function box(w:Float,d:Float,h:Float,x:Float,y:Float,z:Float):Part {var raw=Part.box(w,d,h),p=raw.translated(new Vector(x,y,z));raw.close();return p;}
    try {
      for(i in 0...feet.length){var p=feet[i];
        var support=box(110,110,70,p.x,p.y,Pocket.FOOT_ROOF),hole=box(Pocket.FOOT_STEM_CLEARANCE,Pocket.FOOT_STEM_CLEARANCE,72,p.x,p.y,Pocket.FOOT_ROOF-1);
        var roof=support.subtract(hole);support.close();hole.close();
        var expected=roof.volume();check(Math.abs(volumeInBody(roof)-expected)<1e-5,"continuous 70 mm roof around stem "+i);roof.close();
        var passage=box(28,28,Pocket.FOOT_SUPPORT_TOP+1,p.x,p.y,0);
        check(volumeInBody(passage)<0.001,"stem has casting clearance "+i);passage.close();
        // An 80 mm-wide, 12 mm-thick wrench corridor reaches both nuts.
        for(z in [Pocket.FOOT_ROOF-Pocket.FOOT_PLATE_THICKNESS-9.5,Pocket.FOOT_ROOF-Pocket.FOOT_PLATE_THICKNESS-28.5]){
          var alongX=i==2,edge=alongX?machine.bed.body.parameterNumber("width")/2+40:(p.y<0?-440.0:440.0);
          var start=alongX?p.x:p.y,low=Math.min(start-40,edge),high=Math.max(start+40,edge);
          var wrench=box(alongX?high-low:80,alongX?80:high-low,12,alongX?(low+high)/2:p.x,alongX?p.y:(low+high)/2,z-6);
          check(volumeInBody(wrench)<0.001,"side wrench corridor clear "+i);wrench.close();
        }
        // Once unloaded, nuts lowered and stem withdrawn, lower the plate then slide out.
        // Both motions are checked; removal is not possible under machine weight.
        for(j in 0...11){var placed=PosedParts.moved(plateShape,F.translation(p.x,p.y,Pocket.FOOT_ROOF-Pocket.FOOT_PLATE_THICKNESS-2*j));
          check(volumeInBody(placed)<0.001,"plate lowering clear "+i+":"+j);placed.close();}
        for(j in 0...16){var dx=i==2?15.0*j:0.0,dy=i==2?0.0:(i==0?-15.0*j:15.0*j);
          var placed=PosedParts.moved(plateShape,F.translation(p.x+dx,p.y+dy,Pocket.FOOT_ROOF-Pocket.FOOT_PLATE_THICKNESS-20));
          check(volumeInBody(placed)<0.001,"plate side extraction clear "+i+":"+j);placed.close();}
        var loaded=PosedParts.moved(plateShape,F.translation(p.x,p.y,Pocket.FOOT_ROOF-Pocket.FOOT_PLATE_THICKNESS));
        check(volumeInBody(loaded)<0.001,"installed plate fits registered roof recess "+i);loaded.close();
        check(members.filter(m->m.id=="footSeat"+i||m.id=="footNut"+i||m.id=="footLocknut"+i||m.id=="foot"+i).length==4,"four separate physical components per foot");
      }
    }catch(e:Dynamic){plateShape.close();body.close();cache.close();throw e;}
    plateShape.close();body.close();cache.close();
    var base=FlexsplineHobberBaseReport.measure(machine);
    check(base.feet!=null&&base.feet.worstReactionN!=null,"gravity reactions are reported independently of beam support");
    var reaction:Float=base.feet.worstReactionN;
    check(Math.isFinite(reaction)&&reaction>0,"worst foot reaction is finite and positive");
    var samples:Array<Dynamic> = base.beam.samples;
    check(samples.length==21,"gravity study covers the full declared X positions");
    for(sample in samples){
      var forces:Array<Float> = sample.footReactionN,sum=0.0;
      for(force in forces){check(Math.isFinite(force)&&force>0,"each primary foot carries a positive finite load");sum+=force;}
      check(Math.abs(sum-base.machineKg*9.80665)<base.machineKg*9.80665*1e-9,"foot reactions balance total machine weight");
    }
    check(base.beam.status=="unsupported"&&base.beam.newMinimumIyMm4==null&&base.beam.passed==false,
      "round socket sections do not become a passing beam screen");
    var contactArea=machinekit.structural.LevelingSeat.PLATE_WIDTH*machinekit.structural.LevelingSeat.PLATE_WIDTH-Pocket.FOOT_STEM_CLEARANCE*Pocket.FOOT_STEM_CLEARANCE;
    var result={assertions:n,seconds:Sys.time()-began,footCount:3,pocketRoofMm:Pocket.FOOT_ROOF,pocketFloorMm:Pocket.FOOT_FLOOR,
      roofThicknessMm:Pocket.FOOT_SUPPORT_TOP-Pocket.FOOT_ROOF,plateMm:[110,110,15],stemClearanceMm:28,thread:"M24x3 nominal, semantic threads",
      worstFootReactionN:reaction,meanRoofBearingStressMPa:reaction/contactArea,
      basis:"Static gravity reaction at 21 X positions; nominal full plate contact assumed. Bearing stress is a preliminary average, not a roof bending, creep, fatigue, thread or dynamic stability qualification.",
      replacement:"Unload and lift base before withdrawing foot stem, lowering plate and extracting through side opening; never remove a loaded support.",base:base};
    var output=Sys.getEnv("HOBBER_FOOT_EVIDENCE");if(output!=null)sys.io.File.saveContent(output,haxe.Json.stringify(result));
    Sys.println("HOBBER_FOOT_PASS "+haxe.Json.stringify(result));
  }
}
