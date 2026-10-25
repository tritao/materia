import materia.assembly.AssemblyFrames as F;
import cadkit.modeling.AssemblyState;
import machinekit.motion.PackedCableChainMesh;

/** Both live bracket points, including the X chain carried by moving Y. */
class CableChainBindingTests {
 static var checks=0;
 static function near(a:Float,b:Float,label:String):Void {checks++;if(Math.abs(a-b)>1e-3)throw label+": "+a+" vs "+b;}
 public static function main():Void {
  for(bed in [2600,3150]) {
   var router=new SheetRouter(new SheetRouterSpec(bed));
   // The router still has unrelated code-only members; test its live binding
   // and round-trip the scene-motion wire data without claiming whole-router reload.
   var restored=router;
   var visuals=@:privateAccess machinekit.assembly.AssemblyPreview.movingBelts(restored);
   var encoded=haxeon.wire.JsonWire.encode(visuals);
   var decoded:materia.project.SceneArtifact.SceneArtifactMachineMotion=haxeon.wire.JsonWire.decode(encoded);
   if(decoded.cableChains==null||decoded.cableChains.length!=2)throw "Both chain visuals survive scene-motion wire reload";
   visuals=decoded;
   var state=new AssemblyState(restored.definition());
   for(x in [0.0,800.0,1600.0])for(y in [0.0,bed/2.0,bed*1.0]) {
    state.setJoint("x",x);state.setJoint("y",y);state.forwardKinematics();
    for(v in visuals.cableChains) {
     var source:machinekit.gantry.GantryCableChain=cast [for(c in restored.components())if(c.id==v.occurrence)c.component][0];
     var fixed=F.compose(state.worldPose(v.fixed.occurrence),restored.memberConnectorFrame(v.fixed.occurrence,v.fixed.connector));
     var moving=F.compose(state.worldPose(v.moving.occurrence),restored.memberConnectorFrame(v.moving.occurrence,v.moving.connector));
     var position=v.axisJoint=="x"?x:y;
     var part=state.worldPose(v.occurrence),expected=F.compose(part,source.chain.movingFrame(position));
     near(fixed.x,part.x,"fixed X");near(fixed.y,part.y,"fixed Y");near(fixed.z,part.z,"fixed Z");
     near(moving.x,expected.x,"moving X bracket");near(moving.y,expected.y,"moving Y bracket");near(moving.z,expected.z,"moving Z bracket");
     var relative=F.compose(F.inverse(fixed),moving);near(relative.y,position-source.chain.n("travel")/2,"live end-derived position");
     var mesh=new PackedCableChainMesh();mesh.update(source.chain,position,1,[0.0,0.0,0.0],part);
     // Each cap contains the four source envelope corners about its bracket centre.
     for(end in [F.identity(),source.chain.movingFrame(position)])for(sx in [-1,1])for(sz in [-1,1]) {
      var corner=F.transformPoint(part,end.x+sx*(source.chain.n("innerWidth")/2+source.chain.n("wall")),end.y,
       end.z+sz*(source.chain.n("innerHeight")/2+source.chain.n("wall")));
      var distance=1e100;
      for(i in 0...mesh.vertexCount) {
       var dx=mesh.positions.getFloat(i*12)-corner.x,dy=mesh.positions.getFloat(i*12+4)-corner.y,dz=mesh.positions.getFloat(i*12+8)-corner.z;
       distance=Math.min(distance,Math.sqrt(dx*dx+dy*dy+dz*dz));
      }
      // Packed display storage is float32 at machine-scale mm coordinates.
      near(distance,0,"display cap remains at bracket");
     }
    }
   }
  }
  Sys.println('CableChainBindingTests: $checks assertions passed');
 }
}
