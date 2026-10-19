import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import collisionkit.CollisionDescription;
import collisionkit.CollisionPose;
import collisionkit.CollisionGeometry;
import collisionkit.CollisionPairRule;
import collisionkit.CollisionPairStatus;
import collisionkit.kinematics.ModelBodies;
import collisionkit.native.NativeCollisionWorld;
import motionkit.robot.CollisionPlannerSpace;
import robotkit.runtime.*;
import nativekit.sim.SimObject;
import kinematicskit.*;

/** Recheck only the changed released-stock pair set; never waive a live contact. */
class TendingReleasedStockReturnCheck {
  static final RESERVE=0.00012;
  static function pose(f:AssemblyFrame):CollisionPose return new CollisionPose(f.x,f.y,f.z,f.qx,f.qy,f.qz,f.qw);
  static function physical(sim:Simulation,link:Int):AssemblyFrame {
    var p=sim.linkPose(0,link);
    return {x:p.position[0],y:p.position[1],z:p.position[2],qx:p.rotation[0],qy:p.rotation[1],qz:p.rotation[2],qw:p.rotation[3]};
  }
  static function frame(t:Transform):AssemblyFrame return {x:t.x,y:t.y,z:t.z,qx:t.qx,qy:t.qy,qz:t.qz,qw:t.qw};
  public static function run(scene:TendingElectricSimScene,sim:Simulation,stock:SimObject,
      carried:Array<SimObject>,edges:Array<Dynamic>):Dynamic
    return check(scene,sim,stock,carried,edges,sim.objectPose(stock));
  static function check(scene:TendingElectricSimScene,sim:Simulation,stock:SimObject,
      carried:Array<SimObject>,edges:Array<Dynamic>,released:nativekit.sim.SimPose):Dynamic {
    var model=scene.arm.model,kinematic=robotkit.kinematics.RobotKinematics.compile(model);
    var identity:RobotRuntimeIdentity=cast scene.blueprint.identity;
    var reference=new KinematicState(kinematic),armDofs=[for(i in 1...7)kinematic.dofIndex("j"+i)];
    for(i in 0...kinematic.dofCount()) {
      var joint=identity.jointIndex(model.joints[kinematic.dofJoint[i]].id);
      var measured=sim.jointSample(0,joint).get_position();
      if(armDofs.indexOf(i)<0) {
        var projected=Math.max(kinematic.dofLower[i],Math.min(kinematic.dofUpper[i],measured));
        if(Math.abs(projected-measured)>0.00001)throw "Process pose is outside its collision model";
        measured=projected;
      }
      reference.q[i]=measured;
    }
    var root=physical(sim,identity.linkIndex(model.links[0].id));
    reference.setRootPose(kinematic.bodyIndex(model.links[0].id),new Transform(root.x,root.y,root.z,root.qx,root.qy,root.qz,root.qw));
    var snapshot=new KinematicSnapshot(kinematic);snapshot.evaluate(reference);
    var description=new CollisionDescription(0.000001);
    var bodies=ModelBodies.describe(description,kinematic,"observed-return",1,reference,"measured physical state",true);
    var flangeId=identity.linkId(scene.flangeLink),flange=kinematic.bodyIndex(flangeId);
    var observedFlange=physical(sim,scene.flangeLink),inverse=AssemblyFrames.inverse(observedFlange);
    var tool=bodies.carry(flange,"observed-tool",2),maxMismatch=0.0;
    for(piece in scene.blueprint.linkCollisionShapes) {
      var id=identity.linkId(piece.link),actual=physical(sim,piece.link),vertices=piece.shape.convexVertices();
      var isTool=StringTools.startsWith(id,"electric/"),offset=AssemblyFrames.identity();
      var owner=bodies.bodies[kinematic.bodyIndex(id)],predicted=frame(snapshot.bodyPose(kinematic.bodyIndex(id)));
      if(isTool) {
        owner=tool;offset=AssemblyFrames.compose(inverse,actual);
        predicted=AssemblyFrames.compose(frame(snapshot.bodyPose(flange)),offset);
      }
      var radius=0.0;
      for(i in 0...Std.int(vertices.length/3))radius=Math.max(radius,Math.sqrt(
        vertices[i*3]*vertices[i*3]+vertices[i*3+1]*vertices[i*3+1]+vertices[i*3+2]*vertices[i*3+2]));
      var dx=predicted.x-actual.x,dy=predicted.y-actual.y,dz=predicted.z-actual.z;
      var dot=Math.abs(predicted.qx*actual.qx+predicted.qy*actual.qy+predicted.qz*actual.qz+predicted.qw*actual.qw);
      var mismatch=Math.sqrt(dx*dx+dy*dy+dz*dz)+2*radius*Math.sqrt(Math.max(0,1-Math.min(1,dot)*Math.min(1,dot)));
      maxMismatch=Math.max(maxMismatch,mismatch);
      if(mismatch>RESERVE)throw "Physical/FK return geometry differs beyond its explicit reserve: "+id;
      description.addObject(id,owner,pose(offset),Convex(vertices),piece.shape.inflation+RESERVE,"physical collision hull");
    }
    for(object in carried) {
      if(object.handle.rawValue()==stock.handle.rawValue())throw "Released stock is still marked carried";
      var p=sim.objectPose(object),local=AssemblyFrames.compose(inverse,
        {x:p.x,y:p.y,z:p.z,qx:p.qx,qy:p.qy,qz:p.qz,qw:p.qw});
      var body=bodies.carry(flange,"observed-payload-"+object.handle.rawValue(),2);
      description.addObject("retained payload",body,pose(local),Convex(object.shape.convexVertices()),RESERVE,"observed physical grasp");
    }
    // Self/environment pairs retain their existing reviewed route policies.
    // This additional query checks every robot/tool/payload shape against the
    // newly observed released stock, including its unconstrained staging yaw.
    var moving=bodies.all();
    for(i in 0...moving.length)for(j in i+1...moving.length)description.setBodyRule(moving[i],moving[j],Allow,Allowed);
    description.addFixed("observed released stock",new CollisionPose(released.x,released.y,released.z,released.qx,released.qy,released.qz,released.qw),
      Convex(stock.shape.convexVertices()),0,0,"physical stock at measured release pose");
    bodies.declare();var world=new NativeCollisionWorld();
    try {
      var build=description.build(world);
      if(build.overlapsAtReference!=0)throw "Released-stock recheck cannot inherit overlap exemptions";
      var space=new CollisionPlannerSpace(description,build,bodies,description.margins),checked=0;
      function full(q:Array<Float>):Array<Float> {
        if(q.length!=6)throw "Return edge needs six arm coordinates";
        var value=[for(i in 0...kinematic.dofCount())reference.q[i]];
        for(i in 0...6){if(armDofs[i]<0)throw "Return arm DOF is missing";value[armDofs[i]]=q[i];}
        return value;
      }
      for(edge in edges) {
        var a=full(cast edge.from),b=full(cast edge.to);
        if(!space.edgesClear(a.concat(b),1)[0])throw "Measured released-stock return edge is blocked or unproven: "+checked;
        checked++;
      }
      var result={passed:true,edgesChecked:checked,stockPose:released,pointReserveMm:RESERVE*1000,
        maximumCarrierMismatchMm:maxMismatch*1000,continuous:true,
        scope:"measured released stock versus robot, observed tool and retained payloads; remaining pairs use reviewed routes and live guards"};
      world.dispose();return result;
    } catch(error:Dynamic){world.dispose();throw error;}
  }
  /** Positive and deliberately overlapping QUERY controls; never move a physical object. */
  public static function controls(scene:TendingElectricSimScene,sim:Simulation,stock:SimObject):Dynamic {
    var q=[for(j in scene.armJoints)sim.jointSample(0,j).get_position()],next=q.copy();next[0]+=0.01;
    var edges:Array<Dynamic> = [{from:q,to:next}],positive=run(scene,sim,stock,[],edges);
    var identity:RobotRuntimeIdentity=cast scene.blueprint.identity;
    var selected=[for(piece in scene.blueprint.linkCollisionShapes)if(StringTools.startsWith(identity.linkId(piece.link),"electric/"))piece];
    if(selected.length==0)throw "Return query control needs a moving tool hull";
    function centre(points:Array<Float>):Array<Float> {
      var result=[0.0,0,0],count=Std.int(points.length/3);
      if(count<4)throw "Return query control needs a solid convex hull";
      for(i in 0...count)for(j in 0...3)result[j]+=points[i*3+j]/count;
      return result;
    }
    var piece=selected[0],c=centre(piece.shape.convexVertices()),world=AssemblyFrames.transformPoint(physical(sim,piece.link),c[0],c[1],c[2]);
    var offset=centre(stock.shape.convexVertices());
    var synthetic=new nativekit.sim.SimPose(world.x-offset[0],world.y-offset[1],world.z-offset[2]);
    var rejected=false;
    try check(scene,sim,stock,[],edges,synthetic) catch(error:Dynamic) {
      var message=Std.string(error);
      if(message.indexOf("blocked or unproven")<0 && message.indexOf("overlap exemptions")<0)throw error;
      rejected=true;
    }
    if(!rejected)throw "Released-stock return query accepted an overlapping tool hull";
    return {positiveControl:positive,negativeControlRejectsOverlap:true,syntheticQueryOnly:true};
  }

}
