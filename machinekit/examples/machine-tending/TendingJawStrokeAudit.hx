import cadkit.modeling.AssemblyState;
import collisionkit.CollisionGeometry;
import collisionkit.CollisionMargins;
import collisionkit.CollisionPose;
import collisionkit.native.NativeCollisionWorld;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact.SceneArtifactData;

/** Full translation sweeps, not a sampling of the open and closed endpoints. Units: mm. */
class TendingJawStrokeAudit {
  public static function checkConstruction():Array<Dynamic> {
    var result:Array<Dynamic> = [];
    var reference:Dynamic=haxe.Json.parse(sys.io.File.getContent(DhGripperPreview.directory()+"/build/reference-frame.json"));
    var offset:Array<Dynamic> = cast reference.offset;
    for(variant in ["cnc-aluminium","sls-pa12"])for(left in [true,false]) {
      var part=machinekit.robotics.OffsetFingerGeometry.make(left,variant),boxes=machinekit.robotics.OffsetFingerGeometry.collisionBoxes(left,variant);
      var mesh=part.shape.tessellate(0.01);
      for(i in 0...mesh.vertexCount) {
        var inside=false;
        for(b in boxes){var fits=true;for(a in 0...3){var v=mesh.vertices.getDouble(i*24+a*8);if(v<b[a*2]-1e-6 || v>b[a*2+1]+1e-6)fits=false;}if(fits)inside=true;}
        if(!inside)throw "Finger construction envelope misses fine CAD";
      }
      var difference=0.0;
      if(variant=="cnc-aluminium") {
        var local=part.shape.translate(new cadkit.modeling.Vector(offset[0],offset[1],offset[2]).native());
        var cached=cadkit.Shape.importStep(DhGripperPreview.directory()+"/build/cnc-aluminium/finger-"+(left ? "left" : "right")+".step");
        var extra=local.cut(cached),missing=cached.cut(local);
        if(extra.subshapeCount(CadKit.ShapeKind.Solid)>0)difference+=extra.volume();
        if(missing.subshapeCount(CadKit.ShapeKind.Solid)>0)difference+=missing.volume();
        extra.close();missing.close();cached.close();local.close();
        if(difference>0.001)throw "Authored finger differs from the cached STEP used in the tool";
      }
      result.push({variant:variant,side:left ? "left" : "right",fineVerticesCovered:mesh.vertexCount,
        tessellationMm:0.01,cachedStepDifferenceMm3:variant=="cnc-aluminium" ? difference : null});part.close();
    }
    return result;
  }
  static function pose(f:AssemblyFrame):CollisionPose return new CollisionPose(f.x,f.y,f.z,f.qx,f.qy,f.qz,f.qw);
  static function bounds(vertices:Array<Float>):Array<Float> {
    var b=[Math.POSITIVE_INFINITY,Math.NEGATIVE_INFINITY,Math.POSITIVE_INFINITY,Math.NEGATIVE_INFINITY,Math.POSITIVE_INFINITY,Math.NEGATIVE_INFINITY];
    for(i in 0...Std.int(vertices.length/3))for(a in 0...3){b[a*2]=Math.min(b[a*2],vertices[i*3+a]);b[a*2+1]=Math.max(b[a*2+1],vertices[i*3+a]);}
    return b;
  }
  static function points(b:Array<Float>):Array<Float> {
    var out:Array<Float> =[];
    for(x in [b[0],b[1]])for(y in [b[2],b[3]])for(z in [b[4],b[5]]){out.push(x);out.push(y);out.push(z);}return out;
  }
  public static function run(tool:SceneArtifactData,ts:AssemblyState,wrist:AssemblyFrame,arm:AssemblyState,
      placement:AssemblyFrame,fixtures:AssemblyState,hulls:Map<String,Array<Array<Float>>>,hidden:Null<String>,
      side:String,prefix:String,offset:Array<Float>):Dynamic {
    var world=new NativeCollisionWorld(),fixed=world.addBody(0);world.setBodyStatic(fixed,true);
    var names:Map<Int,String> =[],obstacles:Array<{object:Int,id:String,kind:String}> =[];
    var closed=new AssemblyState(ts.definition,ts.record()),opened=new AssemblyState(ts.definition,ts.record());
    for(s in ["a","b"]){closed.setJoint(s+"-jaw-opening",17);opened.setJoint(s+"-jaw-opening",s==side ? 25 : 17);}
    var moving=[for(m in ["jaw-left","jaw-right","finger-left","finger-right"])side+"-"+m];
    for(group in [{state:fixtures,key:"cell:",kind:"fixture"},{state:arm,key:"arm:",kind:"arm"}])
      for(o in group.state.definition.occurrences) {
        if(group.kind=="fixture" && o.id==hidden)continue;
        var pieces=hulls.get(group.key+o.definition);if(pieces==null)continue;
        var f=group.state.worldPose(o.id);if(group.kind=="arm")f=AssemblyFrames.compose(placement,f);
        for(piece in pieces){var id=world.add(fixed,pose(f),Convex(piece));names.set(id,group.kind+"/"+o.id);obstacles.push({object:id,id:o.id,kind:group.kind});}
      }
    var stockVertices:Array<Float> =[],stockPose=AssemblyFrames.identity();
    for(o in closed.definition.occurrences) {
      if(moving.indexOf(o.id)>=0)continue;
      var part=[for(p in tool.parts)if(p.id==o.definition)p][0];
      var vertices=[for(i in 0...part.vertexCount*3)part.vertices.getDouble(i*8)];
      var geometry:CollisionGeometry=StringTools.endsWith(o.id,"blank-60") ? Convex(vertices)
        : Mesh(vertices,[for(i in 0...part.indexCount)part.indices.getInt32(i*4)]);
      var f=AssemblyFrames.compose(wrist,closed.worldPose(o.id)),id=world.add(fixed,pose(f),geometry);
      names.set(id,"tool/"+o.id);obstacles.push({object:id,id:o.id,kind:"tool"});
      if(o.id==side+"-blank-60"){stockVertices=vertices;stockPose=f;}
    }
    var sweeps:Array<{object:Int,id:String}> =[],proofs:Array<Dynamic> =[],exclusions:Array<String> =[];
    for(member in moving) {
      var occurrence=[for(o in closed.definition.occurrences)if(o.id==member)o][0];
      var part=[for(p in tool.parts)if(p.id==occurrence.definition)p][0];
      var from=AssemblyFrames.compose(wrist,closed.worldPose(member)),to=AssemblyFrames.compose(wrist,opened.worldPose(member));
      var delta=AssemblyFrames.compose(AssemblyFrames.inverse(from),to);
      if(Math.abs(delta.qx)+Math.abs(delta.qy)+Math.abs(delta.qz)>1e-9)throw "Jaw stroke must be a translation";
      var pieces:Array<Array<Float>> =[];
      if(member.indexOf("finger-")>=0) {
        var boxes=machinekit.robotics.OffsetFingerGeometry.collisionBoxes(StringTools.endsWith(member,"left"));
        for(b in boxes)for(a in 0...3){b[a*2]+=offset[a];b[a*2+1]+=offset[a];}
        // Reject stale imported fingers before using the authored construction bounds.
        for(i in 0...part.vertexCount) {
          var inside=false;
          for(b in boxes){var fits=true;for(a in 0...3){var v=part.vertices.getDouble(i*24+a*8);if(v<b[a*2]-1e-6 || v>b[a*2+1]+1e-6)fits=false;}if(fits)inside=true;}
          if(!inside)throw "Imported finger falls outside authored bounds";
        }
        for(b in boxes)pieces.push(points(b));
      } else {
        pieces.push(cadkit.ConvexHullVertices.enclosingFromMesh(part.vertices,part.vertexCount,0.001,0.151).vertices);
      }
      var body=world.addBody(0);world.setBodyPose(body,pose(from));
      var relativeStock=AssemblyFrames.compose(AssemblyFrames.inverse(from),stockPose),stock:Array<Float> =[];
      for(i in 0...Std.int(stockVertices.length/3)){var p=AssemblyFrames.transformPoint(relativeStock,stockVertices[i*3],stockVertices[i*3+1],stockVertices[i*3+2]);stock.push(p.x);stock.push(p.y);stock.push(p.z);}
      var stockBox=bounds(stock);
      for(piece in pieces) {
        var sweep=piece.copy();for(i in 0...Std.int(piece.length/3)){sweep.push(piece[i*3]+delta.x);sweep.push(piece[i*3+1]+delta.y);sweep.push(piece[i*3+2]+delta.z);}
        var id=world.add(body,CollisionPose.identity(),Convex(sweep));names.set(id,"sweep/"+member);sweeps.push({object:id,id:member});
        for(o in obstacles) {
          var slider=member.indexOf("jaw-")>=0 && o.id==side+"-pgc-140-50";
          if(slider){world.setObjectRule(id,o.object,Allow,DeclaredContact);exclusions.push(member+" / supplier carriage guide");continue;}
          if(member.indexOf("finger-")>=0 && o.id==side+"-blank-60") {
            var box=bounds(sweep),separation=Math.NEGATIVE_INFINITY;
            for(a in 0...3)separation=Math.max(separation,Math.max(stockBox[a*2]-box[a*2+1],box[a*2]-stockBox[a*2+1]));
            if(separation < -1e-6)throw "Finger stroke has no separating plane from stock";
            world.setObjectRule(id,o.object,Allow,DeclaredContact);
            proofs.push({member:member,separationMm:separation,method:"Exact construction boxes and cuboid stock; affine translation keeps the entire stroke outside one stock support plane"});continue;
          }
          var margin=o.kind=="tool" || (o.kind=="fixture" && StringTools.startsWith(o.id,prefix)) ? 0.5 : 5.0;
          world.setObjectMargins(id,o.object,margin,margin);
        }
      }
    }
    for(i in 0...sweeps.length)for(j in i+1...sweeps.length) {
      var a=sweeps[i],b=sweeps[j];
      if(StringTools.endsWith(a.id,"left")==StringTools.endsWith(b.id,"left"))world.setObjectRule(a.object,b.object,Allow,Rigid);
      else world.setObjectMargins(a.object,b.object,0.5,0.5);
    }
    var failure=world.violation(CollisionMargins.uniform(5)),message:Null<String> =null;
    if(failure!=null)message=names.get(failure.a)+" / "+names.get(failure.b)+" ("+failure.distance+" mm; requires "+failure.required+" mm)";
    world.dispose();return {clear:failure==null,failure:message,strokeMmPerJaw:8,sweepObjects:sweeps.length,
      stockSeparatingPlanes:proofs,declaredContacts:exclusions,method:"Convex hull of both translation endpoints of each enclosing construction piece",
      scope:"Full modeled 17–25 mm per-jaw stroke at fixed wrist; excludes supplier carriage guides and rigid finger/carriage mounts"};
  }
}
