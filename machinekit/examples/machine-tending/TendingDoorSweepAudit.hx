import cadkit.modeling.AssemblyState;
import collisionkit.CollisionMargins;
import collisionkit.CollisionPose;
import collisionkit.native.NativeCollisionWorld;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact.SceneArtifactData;

/** Robot/tool clearance through the entire prismatic door stroke, at park. Units: mm. */
class TendingDoorSweepAudit {
  static function pose(f:AssemblyFrame):CollisionPose return new CollisionPose(f.x,f.y,f.z,f.qx,f.qy,f.qz,f.qw);
  public static function run(fixtures:AssemblyState,arm:AssemblyState,placement:AssemblyFrame,tool:SceneArtifactData,
      ts:AssemblyState,hulls:Map<String,Array<Array<Float>>>,park:Array<Float>,stroke:Float):Dynamic {
    var world=new NativeCollisionWorld(),fixed=world.addBody(0),moving=world.addBody(0),names:Map<Int,String> =[];
    world.setBodyStatic(fixed,true);
    var parked=new AssemblyState(arm.definition,arm.record());for(j in 0...6)parked.setJoint("j"+(j+1),park[j]);
    for(o in parked.definition.occurrences) {
      var pieces=hulls.get("arm:"+o.definition);if(pieces==null)continue;
      for(piece in pieces){var id=world.add(fixed,pose(AssemblyFrames.compose(placement,parked.worldPose(o.id))),Convex(piece));names.set(id,"arm/"+o.id);}
    }
    var wrist=AssemblyFrames.compose(placement,parked.worldPose("toolFlange"));
    var closed=new AssemblyState(ts.definition,ts.record()),open=new AssemblyState(ts.definition,ts.record());
    for(side in ["a","b"]){closed.setJoint(side+"-jaw-opening",17);open.setJoint(side+"-jaw-opening",25);}
    for(o in closed.definition.occurrences) {
      var part=[for(p in tool.parts)if(p.id==o.definition)p][0],vertices=haxe.io.Bytes.alloc(part.vertexCount*48);
      var from=AssemblyFrames.compose(wrist,closed.worldPose(o.id)),to=AssemblyFrames.compose(wrist,open.worldPose(o.id));
      for(end in 0...2)for(i in 0...part.vertexCount) {
        var f=end==0 ? from : to,p=AssemblyFrames.transformPoint(f,part.vertices.getDouble(i*24),part.vertices.getDouble(i*24+8),part.vertices.getDouble(i*24+16));
        var index=(end*part.vertexCount+i)*24;vertices.setDouble(index,p.x);vertices.setDouble(index+8,p.y);vertices.setDouble(index+16,p.z);
      }
      var id=world.add(fixed,CollisionPose.identity(),Convex(cadkit.ConvexHullVertices.enclosingFromMesh(vertices,part.vertexCount*2,0.001,0.151).vertices));names.set(id,"tool/"+o.id);
    }
    var shut=new AssemblyState(fixtures.definition,fixtures.record()),openDoor=new AssemblyState(fixtures.definition,fixtures.record());
    shut.setJoint("mill/door",0);openDoor.setJoint("mill/door",stroke);var count=0;
    for(o in shut.definition.occurrences) {
      var a=shut.worldPose(o.id),b=openDoor.worldPose(o.id),delta=AssemblyFrames.compose(AssemblyFrames.inverse(a),b);
      if(Math.abs(delta.x)+Math.abs(delta.y)+Math.abs(delta.z)<1e-8)continue;
      if(Math.abs(delta.qx)+Math.abs(delta.qy)+Math.abs(delta.qz)>1e-9)throw "Door sweep needs prismatic motion";
      var pieces=hulls.get("cell:"+o.definition);if(pieces==null || pieces.length==0)throw "Moving door member has no collision geometry";
      for(piece in pieces) {
        var sweep=piece.copy();for(i in 0...Std.int(piece.length/3)){sweep.push(piece[i*3]+delta.x);sweep.push(piece[i*3+1]+delta.y);sweep.push(piece[i*3+2]+delta.z);}
        var id=world.add(moving,pose(a),Convex(sweep));names.set(id,"door/"+o.id);count++;
      }
    }
    if(count==0)throw "No moving door geometry found";
    var failure=world.violation(CollisionMargins.uniform(5)),message:Null<String> =null;
    if(failure!=null)message=names.get(failure.a)+" / "+names.get(failure.b)+" ("+failure.distance+" mm; requires "+failure.required+" mm)";
    world.dispose();return {clear:failure==null,failure:message,sweptPieces:count,strokeMm:stroke,park:park,
      method:"Endpoint convex hulls enclose the complete prismatic door stroke; tool includes both full jaw strokes",
      scope:"Door versus parked robot, complete tool, both held blanks and static harness; machine guide contacts excluded"};
  }
}
