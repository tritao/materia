import machinekit.hobbing.FoundationLayout as Layout;
import machinekit.hobbing.HobberXLayout;
import materia.casting.BoxAlgebra;
import materia.casting.CastingCavities;
import machinekit.assembly.PosedParts;
import materia.assembly.AssemblyFrames as F;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
/** Exact support volumes, independently of assembly mates or collision exclusions. */
class HobberDriveSupportChecks {
  static function box(b:Array<Float>):Part {
    var raw=Part.box(b[3]-b[0],b[4]-b[1],b[5]-b[2]),p=raw.translated(new Vector((b[0]+b[3])/2,(b[1]+b[4])/2,b[2]));raw.close();return p;
  }
  public static function ribProbes(body:Part):Array<{label:String,missingMm3:Float}> {
    var l=HobberXLayout.front(),c=l.bedCentre,start=l.deckStart-c,end=l.motorFace-c;
    var probes:Array<{label:String,bounds:Array<Float>}> = [];
    for(side in [-1,1])probes.push({label:'longitudinal web '+side,bounds:[start,side*112.5-17.5,85,end,side*112.5+17.5,660]});
    for(station in [l.floatingCentre,l.screwStart])probes.push({label:'bearing cross-rib '+station,bounds:[station-c-15,-275,230,station-c+15,275,660]});
    var rear=Layout.feet()[2];
    for(side in [-1,1])probes.push({label:'rear foot boss/web junction '+side,bounds:[rear.x-40,side<0?-95.0:90.0,160,rear.x+40,side<0?-90.0:95.0,230]});
    return [for(p in probes){var solid=box(p.bounds),gap=solid.subtract(body),v=gap.volume();gap.close();solid.close();{label:p.label,missingMm3:v};}];
  }
  static function section(path:String,cells:Array<Array<Float>>,workX:Float):Void {
    var x=workX-HobberXLayout.front().bedCentre,parts=['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="1050" viewBox="0 0 1000 1050"><rect width="1000" height="1050" fill="white"/><g font-family="sans-serif" fill="#222"><text x="50" y="35" font-size="21">Bed transverse section: work X = '+workX+' mm</text><text x="50" y="62" font-size="15">Dark: actual casting material. White: cavities. Dimensions in mm.</text>'];
    for(b in cells)if(x>b[0]&&x<b[3])parts.push('<rect x="'+(500+b[1])+'" y="'+(920-b[5])+'" width="'+(b[4]-b[1])+'" height="'+(b[5]-b[2])+'" fill="#515862"/>');
    for(z in [0,85,660,710,720,800]){parts.push('<path d="M 80 '+(920-z)+' H 920" stroke="#aab1ba" stroke-dasharray="3 5"/><text x="20" y="'+(925-z)+'" font-size="14">'+z+'</text>');}
    parts.push('<g fill="#087eab" stroke="#087eab" stroke-width="1.5"><path d="M350 300 H650 M350 294 V306 M650 294 V306 M925 200 V260 M919 200 H931 M919 260 H931"/><text x="410" y="325" stroke="none" font-size="16">300 mm cap width</text><text x="934" y="236" stroke="none" font-size="16">60</text></g>');
    parts.push('<text x="50" y="970" font-size="16">Z measured from casting bottom; transverse Y spans -400 to +400.</text><text x="50" y="998" font-size="15">Cap Z=660..720; 35 mm webs at Y=±112.5. Structural/modal stiffness unqualified.</text></g></svg>');
    sys.io.File.saveContent(path,parts.join("\n"));
  }
  public static function run():Void {
    var machine=new FlexsplineHobber(),body=machine.bed.body.geometry(),cells=machine.bed.body.materialBoxes();
    var row=machine.bed.insertLocations().filter(p->p.id=="driveSeat")[0],raw=Part.box(row.part.parameterNumber("width"),row.part.parameterNumber("depth"),50);
    var backing=PosedParts.moved(raw,F.compose(row.pose,F.translation(0,0,-50)));raw.close();
    var missing=backing.subtract(body),missingVolume=missing.volume(),required=backing.volume();missing.close();backing.close();
    var n=0,measureOnly=Sys.args().indexOf("measure-only")>=0,probes:Array<Dynamic> =[];
    try {
      n++;if(!measureOnly&&missingVolume>0.001)throw 'X drive seat lacks 50 mm full-footprint casting backing: $missingVolume mm3';
      for(p in ribProbes(body)){n++;probes.push(p);if(!measureOnly&&p.missingMm3>0.001)throw 'Missing casting support ${p.label}: ${p.missingMm3} mm3';}
      n++;if(body.solidCount()!=1)throw "Bed casting is disconnected";
      n++;if(Math.abs(BoxAlgebra.volume(cells)-body.volume())>body.volume()*1e-9)throw "Section cells disagree with exact CAD";
      var compact=BoxAlgebra.compact(cells),cuts=machine.bed.body.finishedCavities();
      n++;if(Math.abs(BoxAlgebra.volume(compact)-body.volume())>body.volume()*1e-9)throw "Collision compaction changes exact CAD volume";
      for(cell in compact){var b=BoxAlgebra.limits(cell);n++;if(b[0]<-machine.bed.body.parameterNumber("width")/2||b[3]>machine.bed.body.parameterNumber("width")/2||b[1]<-400||b[4]>400||b[2]<0||b[5]>800)throw "Collision box leaves casting envelope";
        for(cut in cuts)for(piece in CastingCavities.boxes(cut)){var c=BoxAlgebra.limits(piece),v=1.0;for(a in 0...3)v*=Math.max(0,Math.min(b[a+3],c[a+3])-Math.max(b[a],c[a]));
          n++;if(v>0.001)throw "Compacted collision box fills casting void "+cut.featureId;}
      }
      var output=Sys.getEnv("HOBBER_DRIVE_SUPPORT_EVIDENCE");if(output!=null){section(output+"-motor-flange.svg",[for(b in cells)BoxAlgebra.limits(b)],HobberXLayout.front().motorFace-10);section(output+"-under-motor.svg",[for(b in cells)BoxAlgebra.limits(b)],(HobberXLayout.front().motorFace+HobberXLayout.front().motorRear)/2);}
      var result={assertions:n,collisionBoxesBefore:cells.length,collisionBoxesAfter:compact.length,driveSeatRequiredBackingMm3:required,driveSeatMissingBackingMm3:missingVolume,backingDepthMm:50,
        probes:probes,solidCount:body.solidCount(),castingMassKg:machine.bed.castingByWeight().kg,
        scope:"Exact volume continuity only; not stiffness, fatigue or casting-process qualification."};
      if(output!=null)sys.io.File.saveContent(output+".json",haxe.Json.stringify(result));
      Sys.println("HOBBER_DRIVE_SUPPORT "+haxe.Json.stringify(result));
    }catch(e:Dynamic){body.close();throw e;}body.close();
  }
}
