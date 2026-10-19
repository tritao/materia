import cadkit.Shape;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.Plane;
import cadkit.modeling.Location;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import machinekit.robotics.CobotArm;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact;

/** Exact static service CAD at previously solved tending joint configurations.
    Does not inherit their collision verdicts or claim a new reach certificate. */
class DhExpertRouteAudit {
  static var boundsCache:haxe.ds.ObjectMap<Shape,{low:Vector,high:Vector}> = new haxe.ds.ObjectMap();
  static function bounds(s:Shape):{low:Vector,high:Vector} {
    var b=boundsCache.get(s);if(b!=null)return b;
    var raw=s.bounds();b={low:Vector.fromNative(raw.get_min()),high:Vector.fromNative(raw.get_max())};boundsCache.set(s,b);return b;
  }
  static function bound(a:Shape,b:Shape):Float {
    var x=bounds(a),y=bounds(b),dx=Math.max(0,Math.max(x.low.x-y.high.x,y.low.x-x.high.x)),
      dy=Math.max(0,Math.max(x.low.y-y.high.y,y.low.y-x.high.y)),dz=Math.max(0,Math.max(x.low.z-y.high.z,y.low.z-x.high.z));
    return Math.sqrt(dx*dx+dy*dy+dz*dz);
  }
  static function hashFile(path:String):String {
    var bytes=haxe.crypto.Sha256.make(sys.io.File.getBytes(path)),out=new StringBuf();
    for(i in 0...bytes.length)out.add(StringTools.hex(bytes.get(i),2).toLowerCase());return out.toString();
  }
  static function n(v:Dynamic):Float return Std.parseFloat(Std.string(v));
  static function f(v:Dynamic):AssemblyFrame return {x:n(Reflect.field(v,"x")),y:n(Reflect.field(v,"y")),z:n(Reflect.field(v,"z")),
    qx:n(Reflect.field(v,"qx")),qy:n(Reflect.field(v,"qy")),qz:n(Reflect.field(v,"qz")),qw:n(Reflect.field(v,"qw"))};
  static function place(s:Shape,p:AssemblyFrame):Shape {
    var r=AssemblyFrames.toRotationMatrix(p);
    var result=new Location(new Plane(new Vector(p.x,p.y,p.z),new Vector(r[0],r[3],r[6]),new Vector(r[2],r[5],r[8]))).apply(s);
    var b=bounds(s),low=new Vector(1e30,1e30,1e30),high=new Vector(-1e30,-1e30,-1e30);
    for(x in [b.low.x,b.high.x]) for(y in [b.low.y,b.high.y]) for(z in [b.low.z,b.high.z]) {
      var v=AssemblyFrames.transformPoint(p,x,y,z);
      low=new Vector(Math.min(low.x,v.x),Math.min(low.y,v.y),Math.min(low.z,v.z));
      high=new Vector(Math.max(high.x,v.x),Math.max(high.y,v.y),Math.max(high.z,v.z));
    }
    boundsCache.set(result,{low:low,high:high});return result;
  }
  public static function main():Void {
    var directory=DhGripperPreview.directory()+"/build/expert";
    var viseAccess=Sys.args().indexOf("--vise-access")>=0;
    var currentAccess=viseAccess || Sys.args().indexOf("--current-access")>=0;
    var sourceReport=DhGripperPreview.directory()+"/../machine-tending/build/"+(viseAccess ? "electric-expert-vise" : currentAccess ? "electric-expert-access" : "electric-access")+"/report.json";
    var armDirectory=viseAccess ? "robot-source-vise" : currentAccess ? "robot-source-current" : "robot-source";
    var text=sys.io.File.getContent(sourceReport);
    var report:Dynamic=haxe.Json.parse(text),rows:Array<Dynamic> = cast Reflect.field(report,"results");
    if(currentAccess && Sys.args().indexOf("--approved-only")>=0) {
      var provenance:Dynamic=Reflect.field(report,"provenance");
      if(Std.string(Reflect.field(provenance,"toolSha256"))!=hashFile(directory+"/dual-expert-tool.mtrg"))
        throw "Access approvals were computed for different tool geometry; rerun Expert access study";
    }
    var arm=new CobotArm(Reach1300),model=new AssemblyModel("mm");arm.addTo(model,"");
    var state=model.initialState("expert-service-audit"),placement=f(Reflect.field(report,"armPlacement"));
    var services:Array<{id:String,shape:Shape}> = [];
    var sourceHashes:Array<{id:String,sha256:String}> = [];
    for(file in sys.FileSystem.readDirectory(directory)) if(StringTools.endsWith(file,".step")) {
      services.push({id:file.substr(0,file.length-5),shape:Shape.importStep(directory+"/"+file)});
      sourceHashes.push({id:file,sha256:hashFile(directory+"/"+file)});
    }
    var checks:Array<{pose:String,service:String,arm:String,distanceMm:Float,distanceKind:String,overlapMm3:Float}> = [];
    if(Sys.args().indexOf("--poses-only")>=0) {
      var prior:Dynamic=haxe.Json.parse(sys.io.File.getContent(directory+"/tool-cad-audit.json"));
      if(Std.string(Reflect.field(prior,"sourceSceneSha256"))!=hashFile(directory+"/dual-expert-tool.mtrg"))
        throw "Retained tool audit does not match current Expert scene";
    } else {
    // Independently check the installed service against actual retained tool CAD at three jaw openings.
    var toolScene=SceneArtifact.decode(sys.io.File.getBytes(directory+"/dual-expert-tool.mtrg"));
    var toolDefinition:materia.assembly.AssemblyDefinition=toolScene.assemblyDefinition;
    var ts=new AssemblyState(toolDefinition,toolScene.assemblyState);
    var toolCad=new Map<String,Shape>();
    var toolChecks:Array<{openingMm:Float,service:String,tool:String,distanceMm:Float,distanceKind:String,overlapMm3:Float}> = [];
    for(opening in [0.0,17.0,25.0]) {
      Sys.println("Checking retained tool at opening "+opening);Sys.stdout().flush();
      ts.setJoint("a-"+DhGripperJaws.MASTER,opening);ts.setJoint("b-"+DhGripperJaws.MASTER,opening);
      for(o in toolDefinition.occurrences) {
        if(opening>0 && (o.id=="adapter" || StringTools.endsWith(o.id,"pgc-140-50") || StringTools.endsWith(o.id,"blank-60")))continue;
        var file=o.id=="adapter" ? "/build/dual/adapter.step" : "";
        if(StringTools.startsWith(o.id,"a-") || StringTools.startsWith(o.id,"b-")) {
          var local=o.id.substr(2);
          if(local=="blank-60")file="/build/blank-60-local.step";
          if(local=="pgc-140-50")file="/build/pgc-140-50-local.step";
          if(local=="jaw-left" || local=="jaw-right")file="/build/"+local+".step";
          if(local=="finger-left" || local=="finger-right")file="/build/cnc-aluminium/"+local+".step";
        }
        if(file=="")continue;
        var raw=toolCad.get(file);
        if(raw==null){raw=Shape.importStep(DhGripperPreview.directory()+file);toolCad.set(file,raw);}
        var target=place(raw,ts.worldPose(o.id));
        for(s in services) {
          var lower=bound(s.shape,target),gap=lower>5 ? lower : s.shape.distance(target),volume=0.0;
          if(gap<0.01){var c=s.shape.common(target);if(c.subshapeCount(CadKit.ShapeKind.Solid)>0)volume=c.volume();c.close();}
          toolChecks.push({openingMm:opening,service:s.id,tool:o.id,distanceMm:gap,distanceKind:lower>5 ? "bounds lower limit" : "exact CAD",overlapMm3:volume});
        }
        target.close();
      }
    }
    for(raw in toolCad)raw.close();
    sys.io.File.saveContent(directory+"/tool-cad-audit.json",haxe.Json.stringify({sourceSceneSha256:hashFile(directory+"/dual-expert-tool.mtrg"),scope:"Retained tool CAD at three jaw openings; supplier housing is a surface reference",checks:toolChecks},null,"  "));
    }
    if(Sys.args().indexOf("--tool-only")>=0){for(s in services)s.shape.close();Sys.println("Retained tool CAD audit complete");return;}
    var armParts:Array<{id:String,shape:Shape}> = [];
    var armHashes:Array<{id:String,sha256:String}> = [];
    sys.FileSystem.createDirectory(directory+"/"+armDirectory);
    for(e in arm.components()) if(e.id!="powerSupply" && !StringTools.startsWith(e.id,"driver")) {
      var p=e.component.geometry();sys.FileSystem.createDirectory(haxe.io.Path.directory(directory+"/"+armDirectory+"/"+e.id+".step"));
      p.exportStep(directory+"/"+armDirectory+"/"+e.id+".step");
      armHashes.push({id:e.id,sha256:hashFile(directory+"/"+armDirectory+"/"+e.id+".step")});
      armParts.push({id:e.id,shape:p.shape.cloneShape()});p.close();
    }
    var used=new Map<String,Bool>(),count=0;
    var configurations:Array<{pose:String,q:Array<Float>,wrist:AssemblyFrame}> = [];
    for(row in rows) {
      if(Sys.args().indexOf("--all-profiles")<0 && Std.string(Reflect.field(row,"profile"))!="tending-top-entry")continue;
      var q:Array<Dynamic> = cast Reflect.field(row,"q");if(q.length!=6)continue;
      if(Sys.args().indexOf("--approved-only")>=0 && (Std.string(Reflect.field(row,"geometricClear"))!="true"
        || Std.string(Reflect.field(row,"ikStatus"))!="sampled route solved and clear"))continue;
      var key=haxe.Json.stringify(q);if(used.exists(key))continue;used.set(key,true);count++;
      for(i in 0...6)state.setJoint('j${i+1}',n(q[i]));
      var wrist=AssemblyFrames.compose(placement,state.worldPose("toolFlange"));
      if(currentAccess && Sys.args().indexOf("--approved-only")>=0) {
        var wanted=f(Reflect.field(row,"wrist")),dx=wrist.x-wanted.x,dy=wrist.y-wanted.y,dz=wrist.z-wanted.z;
        var dot=Math.abs(wrist.qx*wanted.qx+wrist.qy*wanted.qy+wrist.qz*wanted.qz+wrist.qw*wanted.qw);
        if(Math.sqrt(dx*dx+dy*dy+dz*dz)>0.06 || 2*Math.acos(Math.min(1,dot))>0.0003)
          throw "Saved approved joints no longer reach the source wrist pose";
      }
      var name=Std.string(Reflect.field(row,"profile"))+"/"+Std.string(Reflect.field(row,"station"))+"/"+Std.string(Reflect.field(row,"side"));
      configurations.push({pose:name,q:[for(v in q)n(v)],wrist:wrist});
      Sys.println("Expert service CAD check: "+name);Sys.stdout().flush();
      for(s in services) {
        var tool=place(s.shape,wrist);
        for(a in armParts) {
          var body=place(a.shape,AssemblyFrames.compose(placement,state.worldPose(a.id))),lower=bound(tool,body),gap=lower>5 ? lower : tool.distance(body),volume=0.0;
          if(gap<0.01){var common=tool.common(body);if(common.subshapeCount(CadKit.ShapeKind.Solid)>0)volume=common.volume();common.close();}
          checks.push({pose:name,service:s.id,arm:a.id,distanceMm:gap,distanceKind:lower>5 ? "bounds lower limit" : "exact CAD",overlapMm3:volume});body.close();
        }
        tool.close();
      }
    }
    for(s in services)s.shape.close();for(a in armParts)a.shape.close();
    if(count==0)throw "No solved tending poses to audit";
    sys.io.File.saveContent(directory+(viseAccess ? "/vise-tending-pose-audit.json" : currentAccess ? "/current-tending-pose-audit.json" : "/tending-pose-audit.json"),haxe.Json.stringify({status:"CAD service checks at saved solved poses; bounds separation beyond 5 mm, exact CAD for closer pairs; no full-motion or flexible-cable certificate",
      approvedOnly:Sys.args().indexOf("--approved-only")>=0,sourceReportPath:sourceReport,armSourceDirectory:armDirectory,configurations:configurations,armSources:armHashes,sources:sourceHashes,sourceSceneSha256:hashFile(directory+"/dual-expert-tool.mtrg"),
      sourceReportSha256:haxe.crypto.Sha256.encode(text),profileSelection:Sys.args().indexOf("--all-profiles")<0 ? "tending-top-entry" : "all",uniquePoses:count,checks:checks},null,"  "));
    Sys.println('Audited $count poses, ${checks.length} CAD pairs');
  }
}
