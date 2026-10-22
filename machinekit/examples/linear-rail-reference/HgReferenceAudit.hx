import cadkit.Shape;
import cadkit.PlanarRegion;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;

/** Extract exact line/arc running sections between holes; never scale another rail. */
class HgReferenceAudit {
 static function main():Void {
  var root=Sys.getEnv("MATERIA_ROOT"),cache=root+"/build/suppliers/vallder/hg-reference/";
  var rows:Array<Dynamic> =[];
  for(size in [15,20,25,30,35,45]) for(prefix in ["HGR","HGH","HGW"]) {
   var model=prefix+size+(prefix=="HGR" ? "R" : prefix=="HGH" ? "CA" : "CC"),path=cache+model+".step";
   var raw=Shape.importStep(path),b=raw.bounds();
   var row:Dynamic={model:model,sha256:machinekit.catalog.PinnedCad.sha256(path),
    min:[b.get_min().get_x(),b.get_min().get_y(),b.get_min().get_z()],
    max:[b.get_max().get_x(),b.get_max().get_y(),b.get_max().get_z()],volume:raw.volume(),valid:CadKit.shapeValidChecked(raw.borrowHandle())!=0};
   if(prefix=="HGR") {
    // Raw rails are centred longitudinally. Z=10 is between between the reference holes.
    var region=PlanarRegion.section(raw,10),loops=region.loops();
    if(loops.length!=1 || !loops[0].outer)throw model+": section crosses a bore or multiple regions";
    var segments:Array<Array<Float>> =[];
    for(s in loops[0].segments) {
     var mid=s.pointAt(0.5);
     segments.push(s.arc ? [s.x0,s.y0,s.x1,s.y1,mid.x,mid.y] : [s.x0,s.y0,s.x1,s.y1]);
    }
    Reflect.setField(row,"section",segments);region.close();
   }
   rows.push(row);raw.close();
  }
  sys.io.File.saveContent(cache+"geometry-audit.json",haxe.Json.stringify(rows,null,"  "));
  trace("Audited 18 HG references and extracted six exact rail sections");
 }
}
