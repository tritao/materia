import materia.project.SceneArtifact;
import cadbridge.AssemblySimulationBridge;
import cadbridge.AssemblyPhysicalPartView;
import robotkit.manipulation.Manipulator;
import robotkit.collision.CollisionSceneBuilder;

/** Fast conservative map; exact CAD diagnostic checks remain independent. */
class StraightBoundaryReview {
  static function digest(bytes:haxe.io.Bytes):String {
    var hash=haxe.crypto.Sha256.make(bytes),out=new StringBuf();
    for(i in 0...hash.length) out.add(StringTools.hex(hash.get(i),2).toLowerCase());
    return out.toString();
  }
  public static function run():Void {
    var directory=ReferenceAssetLocation.DIRECTORY+"/../../zeroerr-arm";
    var configPath=directory+"/clearance-review-config.json";
    var config:Dynamic=haxe.Json.parse(sys.io.File.getContent(configPath));
    var label="revised";
    for(arg in Sys.args()) if(StringTools.startsWith(arg,"--review-label=")) label=arg.substr(15);
    if(["baseline","revised"].indexOf(label)<0) throw "Unknown boundary review label";
    var stem=directory+"/build/straight/"+(label=="baseline" ? "clearance-baseline/" : "")+"zeroerr5-straight80"+(label=="baseline" ? "" : "-geometry")+".mtrg";
    var bytes=sys.io.File.getBytes(stem);
    if(label=="baseline" && digest(bytes)!=Std.string(config.baselineArtifactSha256)) throw "Changed baseline arm";
    var scene=SceneArtifact.decode(bytes);
    var definition:materia.assembly.AssemblyDefinition=scene.assemblyDefinition;
    var current:Dynamic=haxe.Json.parse(sys.io.File.getContent(directory+"/build/straight/current.json"));
    var layoutHash=label=="baseline" ? Std.string(config.baselineLayoutSha256) :
      haxe.crypto.Sha256.encode(sys.io.File.getContent(Std.string(current.layoutPath)));
    // Mapping probes deliberately extend outside the proposed planning window.
    for(joint in definition.joints) if(joint.type=="revolute") {joint.limits.lower=null;joint.limits.upper=null;}
    var converted=AssemblySimulationBridge.toRobotModel(scene.assemblyDefinition,AssemblyPhysicalPartView.fromSceneArtifact(scene));
    var frames=[for(f in converted.model.frames) if(f.name=="toolFlange robot flange") f];
    if(frames.length!=1) throw "Missing semantic flange";
    var arm=new Manipulator(converted.model,converted.model.links[0].id,frames[0].id);
    if(arm.group.count()!=6) throw "Expected six joints";
    var builder=new CollisionSceneBuilder(arm,[for(_ in 0...6) 0.0],"arm",config.marginMm/1000);
    var clearance=builder.clearance(() -> new collisionkit.native.NativeCollisionWorld());
    var scans:Array<Dynamic> =[],queries=0;
    try {
      var definitions:Array<Dynamic> =config.scans;
      for(scan in definitions) {
        var seed:Array<Float> =scan.seedRad,pairs:Array<Array<String>> =scan.pairs,index:Int=scan.joint;
        var samples:Array<Dynamic> =[],boundaries:Array<Dynamic> =[],cache:Map<String,Dynamic> =[];
        function at(angle:Float):Dynamic {
          var key=Std.string(angle);
          if(cache.exists(key)) return cache.get(key);
          var q=seed.copy();q[index]=angle*Math.PI/180;
          var observations:Array<Dynamic> =[],clear=true;
          for(pair in pairs) {
            var distance=clearance.distanceBetween(q,"arm/"+pair[0],"arm/"+pair[1])*1000;
            if(!Math.isFinite(distance)) throw "Invalid hull distance";
            observations.push({a:pair[0],b:pair[1],distanceMm:distance});
            if(distance<config.marginMm) clear=false;
          }
          queries++;
          var row={joints:q,clear:clear,pairs:observations};cache.set(key,row);return row;
        }
        var range:Array<Float> =config.rangeDeg,count=Std.int(Math.round((range[1]-range[0])/config.coarseStepDeg));
        for(i in 0...count+1) {
          var angle=range[0]+i*config.coarseStepDeg,row=at(angle);
          samples.push({angleDeg:angle,observation:row});
          if(i>0 && row.clear!=samples[i-1].observation.clear) {
            var low:Float=samples[i-1].angleDeg,high=angle,lowClear:Bool=samples[i-1].observation.clear;
            while(high-low>config.boundaryToleranceDeg) {
              var mid=(low+high)/2;
              if(at(mid).clear==lowClear) low=mid;else high=mid;
            }
            boundaries.push({lowDeg:low,highDeg:high,low:at(low),high:at(high)});
          }
        }
        scans.push({id:scan.id,joint:index+1,seedRad:seed,pairs:pairs,samples:samples,boundaries:boundaries});
        Sys.println('Hull slice ${scan.id}: ${boundaries.length} clearance transitions');Sys.stdout().flush();
      }
    } catch(error:Dynamic) {clearance.world.dispose();throw error;}
    clearance.world.dispose();
    var report={status:"conditional_slices_checked",method:"enclosing_visual_hulls",layoutSha256:layoutHash,artifactSha256:digest(bytes),
      configSha256:haxe.crypto.Sha256.encode(sys.io.File.getContent(configPath)),
      producerSha256:haxe.crypto.Sha256.encode(sys.io.File.getContent(ReferenceAssetLocation.DIRECTORY+"/../StraightBoundaryReview.hx")),
      marginMm:config.marginMm,coarseStepDeg:config.coarseStepDeg,boundaryToleranceDeg:config.boundaryToleranceDeg,
      uniqueQueries:queries,scope:config.scope,hardwareQualified:false,scans:scans};
    sys.io.File.saveContent(directory+"/build/straight/clearance-boundaries-"+label+".json",haxe.Json.stringify(report,null,"  "));
  }
}
