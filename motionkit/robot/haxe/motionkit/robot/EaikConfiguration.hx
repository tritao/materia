package motionkit.robot;

import MotionKitNative;

import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;

/** Geometric labels do not depend on EAIK's solution ordering. */
class EaikConfiguration {
  public final spherical:Bool;
  public final native:mk_eaik_configuration;
  final path:Array<Joint>;
  final reference:Array<Float>;
  final signs:Array<Float>;
  final base:Transform3;
  final shoulderOffset:Float;
  final elbowOffset:Float;
  public function new(chain:EaikChain) {
    path=chain.chain.pathJoints();reference=[for(_ in 0...6)0.0];
    var g=geometry(reference);
    spherical=Math.abs(g.axes[1].dot(g.axes[3]))<1e-6;
    var z=g.axes[0],y=spherical ? g.axes[1] : g.axes[1].scale(-1);
    var x=y.cross(z).normalized();
    if(spherical){
      reference[1]=align(g.origins[2].sub(g.origins[1]),z,g.axes[1]);g=geometry(reference);
      reference[2]=align(g.axes[3],z,g.axes[2]);g=geometry(reference);
      reference[3]=align(g.axes[4],y,g.axes[3]);g=geometry(reference);
      reference[4]=align(g.axes[5],z,g.axes[4]);g=geometry(reference);
    } else {
      if(Math.abs(Math.abs(g.axes[1].dot(g.axes[2]))-1)>1e-6 ||
          Math.abs(Math.abs(g.axes[1].dot(g.axes[3]))-1)>1e-6)
        throw "EAIK configuration labels require spherical-wrist or three-parallel-axis geometry";
      reference[1]=align(g.origins[2].sub(g.origins[1]),x.scale(-1),g.axes[1]);g=geometry(reference);
      reference[2]=align(g.origins[3].sub(g.origins[2]),x.scale(-1),g.axes[2]);g=geometry(reference);
      reference[3]=align(g.axes[4],z.scale(-1),g.axes[3]);g=geometry(reference);
      reference[4]=align(g.axes[5],y.scale(-1),g.axes[4]);g=geometry(reference);
    }
    y=z.cross(x).normalized();base=new Transform3(g.origins[0],Quat.fromRotationMatrix([x.x,x.y,x.z,y.x,y.y,y.z,z.x,z.y,z.z]));
    signs=[];
    for(i in 0...6){
      var expected=spherical ? (i==0 || i==3 || i==5 ? z : y) :
        i==0 ? z : i==4 ? z.scale(-1) : y.scale(-1);
      signs.push(g.axes[i].dot(expected)<0 ? -1.0 : 1.0);
    }
    var local=[for(origin in g.origins)base.inverse().transformPoint(origin)];
    shoulderOffset=local[1].x;
    elbowOffset=spherical ? Math.atan2(local[3].x-local[2].x,local[4].z-local[2].z) : 0;
    native=new mk_eaik_configuration();native.set_struct_size(mk_eaik_configuration.size());native.set_spherical(spherical ? 1 : 0);
    for(i in 0...6){native.set_reference(i,reference[i]);native.set_signs(i,signs[i]);}
    var bp=base.translation.toArray(),br=base.rotation.toRotationMatrix();
    for(i in 0...3)native.set_base_position(i,bp[i]);for(i in 0...9)native.set_base_rotation(i,br[i]);
    native.set_shoulder_offset(shoulderOffset);native.set_elbow_offset(elbowOffset);
  }
  static function align(from:Vec3,to:Vec3,axis:Vec3):Float {
    var a=from.sub(axis.scale(from.dot(axis))),b=to.sub(axis.scale(to.dot(axis)));
    if(a.norm()<1e-12 || b.norm()<1e-12)throw "Degenerate six-axis configuration reference";
    return Math.atan2(axis.dot(a.cross(b)),a.dot(b));
  }
  function geometry(q:Array<Float>):{origins:Array<Vec3>,axes:Array<Vec3>} {
    var at=Transform3.identity(),origins:Array<Vec3> = [],axes:Array<Vec3> = [],i=0;
    for(joint in path){
      var frame=at.compose(Transform3.fromArrays(joint.parentFramePosition,joint.parentFrameRotation));
      if(joint.type!=JointType.Fixed){
        var axis=Vec3.fromArray(joint.axis).normalized();origins.push(frame.translation);axes.push(frame.transformVector(axis).normalized());
        frame=frame.compose(new Transform3(new Vec3(0,0,0),Quat.fromAxisAngle(axis,q[i++])));
      }
      at=frame.compose(Transform3.fromArrays(joint.childFramePosition,joint.childFrameRotation).inverse());
    }
    return {origins:origins,axes:axes};
  }
  public function branch(q:Array<Float>):Int {
    var t=[for(i in 0...6)(q[i]-reference[i])*signs[i]],g=geometry(q);
    var center=spherical ? g.origins[3].add(g.axes[3].scale(g.origins[4].sub(g.origins[3]).dot(g.axes[3]))) : g.origins[5];
    var wrist=base.inverse().transformPoint(center);
    var front=spherical ? wrist.x*Math.cos(t[0])+wrist.y*Math.sin(t[0])-shoulderOffset>=0 :
      Math.sin(t[0]-Math.atan2(wrist.x,-wrist.y))>=0;
    var up=Math.sin(t[2]+elbowOffset)>=0,flip=Math.sin(t[4])<0;
    // One normalized transport convention: shoulder bit 2, elbow bit 1, wrist bit 4.
    return (front ? 0 : 2)+(up ? 0 : 1)+(flip ? 4 : 0);
  }
}
