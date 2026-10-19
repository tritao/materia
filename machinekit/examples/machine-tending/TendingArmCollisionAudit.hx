import cadkit.Shape;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import collisionkit.CollisionPose;
import collisionkit.native.NativeCollisionWorld;
import machinekit.component.CollisionHullFacet;
import machinekit.robotics.CobotArm;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Independent fine-mesh coverage and exact-solid audit at the previously flagged self-clearance. */
class TendingArmCollisionAudit {
  static function number(value:Dynamic):Float return Std.parseFloat(Std.string(value));
  static function frame(v:Dynamic):AssemblyFrame return {x:number(v.x),y:number(v.y),z:number(v.z),
    qx:number(v.qx),qy:number(v.qy),qz:number(v.qz),qw:number(v.qw)};
  static function placed(shape:Shape,f:AssemblyFrame):Shape {
    var m=AssemblyFrames.toRotationMatrix(f);
    return new Location(new Plane(new Vector(f.x,f.y,f.z),new Vector(m[0],m[3],m[6]),new Vector(m[2],m[5],m[8]))).apply(shape);
  }
  static function adapterAudit(source:CobotArm):Void {
    var directory=DhGripperPreview.directory()+"/../machine-tending/build/electric-expert-access-setback-50-trays-case-infeedTray-1-2-b";
    var current=Sys.args().indexOf("--current-adapter")>=0;
    var sourceFile=!current && sys.FileSystem.exists(directory+"/report-before-extended-seeds.json") ? directory+"/report-before-extended-seeds.json" : directory+"/report.json";
    var bytes=sys.io.File.getBytes(sourceFile),report:Dynamic=haxe.Json.parse(bytes.toString());
    var rows:Array<Dynamic> = cast report.results;
    var chosen=[for(row in rows)if(Std.string(row.ikStatus)=="endpoint solved; conservative clearance flag" || Std.string(row.ikStatus)=="sampled route solved and clear")row][0];
    if(chosen==null)throw "No solved adapter posture";
    var model=new AssemblyModel("mm");source.addTo(model,"");var state=model.initialState("adapter-audit");
    var q:Array<Dynamic> = cast chosen.q;for(j in 0...6)state.setJoint("j"+(j+1),number(q[j]));
    var placement=frame(report.armPlacement),wrist=AssemblyFrames.compose(placement,state.worldPose("toolFlange"));
    var tool=materia.project.SceneArtifact.decode(sys.io.File.getBytes(DhGripperPreview.directory()+"/build/expert/dual-expert-tool.mtrg"));
    var ts=new AssemblyState(tool.assemblyDefinition,tool.assemblyState);
    var raw=Shape.importStep(DhGripperPreview.directory()+"/build/dual/adapter.step");
    var adapter=placed(raw,AssemblyFrames.compose(wrist,ts.worldPose("adapter")));raw.close();var checks:Array<Dynamic> = [];
    for(entry in source.components())if(entry.id=="joint4" || entry.id=="link4") {
      var part=entry.component.geometry(),shape=placed(part.shape,AssemblyFrames.compose(placement,state.worldPose(entry.id)));part.close();
      var gap=adapter.distance(shape),overlap=0.0;
      if(gap<0.001){var common=adapter.common(shape);if(common.subshapeCount(CadKit.ShapeKind.Solid)>0)overlap=common.volume();common.close();}
      if(current && (gap<5 || overlap>0))throw "Solved adapter posture violates exact CAD clearance";
      checks.push({arm:entry.id,exactGapMm:gap,overlapMm3:overlap});shape.close();
      Sys.println('adapter/${entry.id}: exact $gap mm; overlap $overlap mm3');Sys.stdout().flush();
    }
    adapter.close();sys.io.File.saveContent(directory+(current ? "/adapter-current-cad-audit.json" : "/adapter-collision-audit.json"),haxe.Json.stringify({sourceReportSha256:haxe.crypto.Sha256.encode(bytes.toString()),checks:checks},null,"  "));
  }
  static function tableAudit(source:CobotArm):Void {
    var directory=DhGripperPreview.directory()+"/../machine-tending/build/electric-expert-vise-setback-50";
    var witnessFile=directory+"/table-collision-witness.json";
    var bytes=sys.io.File.getBytes(witnessFile),report:Dynamic=haxe.Json.parse(bytes.toString());
    var layout=TendingReachLayout.decode(TendingReachCertificate.JSON).result;
    var tables=new TendingCellTable(layout.tableWidth,layout.tableDepth,layout.tableTop,layout.tableOffset);
    var armComponent=[for(e in source.components())if(e.id=="link2")e.component][0];
    var armPart=armComponent.geometry();
    var top=[for(e in tables.components())if(e.id=="right-table/worktop/top")e.component][0].geometry();
    var checks:Array<Dynamic> = [],rows:Array<Dynamic> = cast report.checks;
    if(rows.length!=2)throw "Table regression needs both captured flagged poses";
    for(row in rows) {
      var armPose=frame(row.armPose),tablePose=frame(row.tablePose);
      var a=placed(armPart.shape,armPose),b=placed(top.shape,tablePose);
      var gap=a.distance(b),overlap=0.0;
      var world=new NativeCollisionWorld(),armBody=world.addBody(0),tableBody=world.addBody(1);
      world.setBodyPose(armBody,new CollisionPose(armPose.x,armPose.y,armPose.z,armPose.qx,armPose.qy,armPose.qz,armPose.qw));
      world.setBodyPose(tableBody,new CollisionPose(tablePose.x,tablePose.y,tablePose.z,tablePose.qx,tablePose.qy,tablePose.qz,tablePose.qw));
      var facet:CollisionHullFacet=CollisionHullFacet.of(armComponent);
      for(hull in facet.hulls)world.add(armBody,CollisionPose.identity(),Convex(hull));
      var mesh=top.shape.tessellate(0.01);
      world.add(tableBody,CollisionPose.identity(),Convex(cadkit.ConvexHullVertices.enclosingFromMesh(mesh.vertices,mesh.vertexCount,0.001,0.001).vertices));
      var proxy=Math.POSITIVE_INFINITY;for(distance in world.distances(1000))proxy=Math.min(proxy,distance.distance);world.dispose();
      if(proxy>gap+0.01)throw "Arm/table proxy understates exact geometry";
      if(proxy<5)throw "Aligned arm hull regressed to a table clearance flag";
      if(gap<0.001){var common=a.common(b);if(common.subshapeCount(CadKit.ShapeKind.Solid)>0)overlap=common.volume();common.close();}
      checks.push({station:Std.string(row.station),side:"a",failedEdge:row.failedEdge,proxyGapMm:proxy,exactGapMm:gap,overlapMm3:overlap,
        armPose:armPose,tablePose:tablePose});
      Sys.println('${row.station}: exact link2/table gap $gap mm; overlap $overlap mm3');Sys.stdout().flush();a.close();b.close();
    }
    armPart.close();top.close();
    sys.io.File.saveContent(directory+"/table-collision-audit.json",haxe.Json.stringify({witnessFile:witnessFile,witnessSha256:haxe.crypto.Sha256.encode(bytes.toString()),
      scope:"Exact CAD at failed continuous-edge endpoint; not a swept-edge certificate",checks:checks},null,"  "));
  }
  public static function main():Void {
    var directory=DhGripperPreview.directory()+"/../machine-tending/build/electric-expert-vise";
    var report:Dynamic=haxe.Json.parse(sys.io.File.getContent(directory+"/report.json"));
    var prior:Dynamic=haxe.Json.parse(sys.io.File.getContent(directory+"/planner-before-arm-refinement.json"));
    var source=new CobotArm(Reach1300),model=new AssemblyModel("mm");source.addTo(model,"");
    var state=model.initialState("refined-arm-audit"),placement=frame(report.armPlacement);
    if(Sys.args().indexOf("--adapter-only")>=0){adapterAudit(source);return;}
    var locals:Map<String,Shape> = [],hulls:Map<String,Array<Array<Float>>> = [],coverage:Array<Dynamic> = [];
    var directions:Array<Array<Float>> = [];
    for(x in [-1.0,0.0,1.0])for(y in [-1.0,0.0,1.0])for(z in [-1.0,0.0,1.0])
      if(x!=0 || y!=0 || z!=0)directions.push([x,y,z]);
    for(entry in source.components())if(entry.id=="link2" || entry.id=="link3" || entry.id=="link5") {
      var part=entry.component.geometry(),shape=part.shape.cloneShape();part.close();locals.set(entry.id,shape);
      var facet=CollisionHullFacet.of(entry.component),mesh=shape.tessellate(0.01);
      if(entry.id!="link5" && (facet==null || facet.hulls.length<2))throw "Long arm needs routed collision pieces";
      var pieces=facet==null ? [cadkit.ConvexHullVertices.enclosingFromMesh(mesh.vertices,mesh.vertexCount,0.001,0.05).vertices] : facet.hulls;
      hulls.set(entry.id,pieces);
      if(entry.id!="link5") {
        var link:machinekit.robotics.CobotArmLink=cast entry.component;
        var axes=directions.slice(0,26);
        for(d in axes) {
          var v=AssemblyFrames.transformVector(link.serviceFrame,d[0],d[1],d[2]);directions.push([v.x,v.y,v.z]);
        }
      }
      var supports:Array<Array<Float>> = [];
      for(hull in pieces) {
        var planes:Array<Float> = [];
        for(d in directions) {
          var most=Math.NEGATIVE_INFINITY;
          for(i in 0...Std.int(hull.length/3))most=Math.max(most,d[0]*hull[i*3]+d[1]*hull[i*3+1]+d[2]*hull[i*3+2]);
          planes.push(most);
        }
        supports.push(planes);
      }
      var membership:Array<Int> = [];
      for(i in 0...mesh.vertexCount) {
        var x=mesh.vertices.getDouble(i*24),y=mesh.vertices.getDouble(i*24+8),z=mesh.vertices.getDouble(i*24+16),mask=0;
        for(h in 0...supports.length) {
          var inside=true;
          for(j in 0...directions.length) {
            var d=directions[j];if(x*d[0]+y*d[1]+z*d[2]>supports[h][j]+0.000001){inside=false;break;}
          }
          if(inside)mask|=1<<h;
        }
        if(mask==0)throw 'Collision proxy misses fine CAD vertex $i of ${entry.id}';
        membership.push(mask);
      }
      var common=0;
      for(i in 0...Std.int(mesh.indexCount/3)) {
        var a=mesh.indices.getInt32(i*12),b=mesh.indices.getInt32(i*12+4),c=mesh.indices.getInt32(i*12+8);
        if((membership[a]&membership[b]&membership[c])!=0)common++;
      }
      if(common!=Std.int(mesh.indexCount/3))throw 'Collision pieces do not cover every fine CAD triangle of ${entry.id}';
      coverage.push({part:entry.id,pieces:pieces.length,vertices:mesh.vertexCount,coveredTriangles:common,deflectionMm:0.01});
      Sys.println('${entry.id}: ${pieces.length} pieces cover $common fine CAD triangles');Sys.stdout().flush();
    }
    var rows:Array<Dynamic> = cast prior.results,checks:Array<Dynamic> = [];
    for(row in rows)if(row.side=="b") {
      var route:Array<Dynamic> = cast row.routeQ,values:Array<Dynamic> = cast route[Std.int(number(row.failedApproachEdge))+1];
      for(j in 0...6)state.setJoint("j"+(j+1),number(values[j]));
      var world=new NativeCollisionWorld(),a=world.addBody(0),b=world.addBody(1);
      var first=AssemblyFrames.compose(placement,state.worldPose("link3")),second=AssemblyFrames.compose(placement,state.worldPose("link5"));
      for(pair in [{id:"link3",body:a,pose:first},{id:"link5",body:b,pose:second}]) {
        world.setBodyPose(pair.body,new CollisionPose(pair.pose.x,pair.pose.y,pair.pose.z,pair.pose.qx,pair.pose.qy,pair.pose.qz,pair.pose.qw));
        for(hull in hulls.get(pair.id))world.add(pair.body,CollisionPose.identity(),Convex(hull));
      }
      var gap=Math.POSITIVE_INFINITY;for(distance in world.distances(1000))gap=Math.min(gap,distance.distance);
      var x=placed(locals.get("link3"),first),y=placed(locals.get("link5"),second),exact=x.distance(y),overlap=0.0;
      if(exact<0.001){var shared=x.common(y);if(shared.subshapeCount(CadKit.ShapeKind.Solid)>0)overlap=shared.volume();shared.close();}
      x.close();y.close();world.dispose();
      if(gap>exact+0.001)throw "Refined proxy is not conservative at flagged arm pose";
      checks.push({station:Std.string(row.station),side:"b",q:[for(v in values)number(v)],oldProxyGapMm:4.98353484484348,
        refinedProxyGapMm:gap,exactGapMm:exact,overlapMm3:overlap});
      Sys.println('${row.station}: refined $gap mm, exact $exact mm, overlap $overlap mm3');Sys.stdout().flush();
    }
    tableAudit(source);
    for(shape in locals)shape.close();
    sys.io.File.saveContent(directory+"/arm-collision-audit.json",haxe.Json.stringify({coverage:coverage,checks:checks},null,"  "));
  }
}
