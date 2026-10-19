package robotkit.runtime;

import robotkit.runtime.RobotRuntimeBlueprint.RobotRuntimeLinkShape;
import robotkit.runtime.RobotRuntimeBlueprint.RobotRuntimeContactPair;

/** Pack exact convex geometry into the existing native hull buffer when primitive slots run out. */
class SimulationCollisionPacking {
  public static function pack(shapes:Array<RobotRuntimeLinkShape>,pairs:Array<RobotRuntimeContactPair>,
      maxShapes:Int,maxHulls:Int):{shapes:Array<Int>,hulls:Array<Int>,shapeMap:Array<Int>} {
    if(maxShapes<0||maxHulls<0)throw "Collision capacities must be non-negative";
    var referenced=new Map<Int,Bool>();
    for(pair in pairs)for(index in [pair.shapeA,pair.shapeB]) {
      if(index<0||index>=shapes.length)throw "Contact pair references an unknown shape";
      referenced.set(index,true);
    }
    var overflow=Std.int(Math.max(0,shapes.length-maxShapes)),hulls:Array<Int> = [],kept:Array<Int> = [];
    if(overflow>maxHulls)throw "Simulation collision geometry exceeds primitive and hull capacities";
    // Explicit pair IDs address primitive slots. Keep those shapes in that buffer.
    for(index in 0...shapes.length) {
      var convex=switch shapes[index].shape.geometry {case Convex(points):points!=null&&points.length<=192;case _:false;};
      if(hulls.length<overflow&&convex&&!referenced.exists(index))hulls.push(index);
      else kept.push(index);
    }
    if(kept.length>maxShapes)throw "Simulation has too many primitive or explicitly paired collision shapes";
    var map=[for(_ in shapes)-1];for(index in 0...kept.length)map[kept[index]]=index;
    return {shapes:kept,hulls:hulls,shapeMap:map};
  }
}
