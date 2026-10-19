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

/** Observed, bounded grasp-centre registration. No stock placement or attachment. */
class TendingGraspRegistration {
  static final RESERVE=0.00012;
  static function pose(f:AssemblyFrame):CollisionPose return new CollisionPose(f.x,f.y,f.z,f.qx,f.qy,f.qz,f.qw);
  static function physical(sim:Simulation,link:Int):AssemblyFrame {
    var p=sim.linkPose(0,link);
    return {x:p.position[0],y:p.position[1],z:p.position[2],qx:p.rotation[0],qy:p.rotation[1],qz:p.rotation[2],qw:p.rotation[3]};
  }
  static function objectFrame(sim:Simulation,object:SimObject):AssemblyFrame {
    var p=sim.objectPose(object);return {x:p.x,y:p.y,z:p.z,qx:p.qx,qy:p.qy,qz:p.qz,qw:p.qw};
  }
  static function pieces(object:SimObject):Array<Array<Float>> {
    var vertices=object.shape.convexVertices(),counts=object.shape.convexPieceCounts();
    if(counts.length==0)counts=[vertices.length];
    var result=[],offset=0;
    for(count in counts){if(count<12 || count%3!=0)throw "Observed collision checking needs physical convex pieces";result.push(vertices.slice(offset,offset+count));offset+=count;}
    if(offset!=vertices.length)throw "Physical convex-piece coverage differs";
    return result;
  }
  public static function plan(scene:TendingElectricSimScene,sim:Simulation,object:SimObject,side:String,
      fixed:Array<{name:String,object:SimObject}>,carried:Array<{side:String,object:SimObject}>,maximumPointMotionMm:Float):Dynamic {
    var wrist=physical(sim,scene.flangeLink),stock=objectFrame(sim,object);
    var local:AssemblyFrame=cast scene.graspFrames.get(side),nominal=AssemblyFrames.compose(wrist,local);
    // Orientation already passed the vise's full-pose check. Register translation
    // while retaining the reviewed wrist orientation and local finger arrangement.
    var target:AssemblyFrame={x:wrist.x+stock.x-nominal.x,y:wrist.y+stock.y-nominal.y,z:wrist.z+stock.z-nominal.z,
      qx:wrist.qx,qy:wrist.qy,qz:wrist.qz,qw:wrist.qw};
    var from=[for(j in scene.armJoints)sim.jointSample(0,j).get_position()],to=solveWrist(scene,sim,target,from);
    var proof=check(scene,sim,fixed,carried,from,to,maximumPointMotionMm,-1,[]);
    return {from:from,to:to,proof:proof,observedStock:stock,targetWrist:target,
      translationMm:[(stock.x-nominal.x)*1000,(stock.y-nominal.y)*1000,(stock.z-nominal.z)*1000],
      poseSource:"simulation observation; hardware sensing uncommissioned",hardwareQualified:false};
  }
  public static function solveWrist(scene:TendingElectricSimScene,sim:Simulation,target:AssemblyFrame,from:Array<Float>):Array<Float> {
    var base:cadbridge.AssemblySimulationBridge.AssemblyPartLink=cast scene.arm.partLinks.get("baseFlange");
    var relative=AssemblyFrames.compose(AssemblyFrames.inverse(physical(sim,base.link)),target);
    var answer=scene.graspManipulator.solve(new robotkit.spatial.Transform3(
      new robotkit.spatial.Vec3(relative.x,relative.y,relative.z),new robotkit.spatial.Quat(relative.qx,relative.qy,relative.qz,relative.qw)),
      from,new robotkit.manipulation.IkOptions(1e-9,1e-8,200).reaching());
    if(!answer.converged || answer.q.length!=6)throw "Observed grasp/extraction IK failed";
    return [for(i in 0...6)answer.q[i]+2*Math.PI*Math.round((from[i]-answer.q[i])/(2*Math.PI))];
  }
  public static function wristAt(scene:TendingElectricSimScene,sim:Simulation,q:Array<Float>):AssemblyFrame {
    var base:cadbridge.AssemblySimulationBridge.AssemblyPartLink=cast scene.arm.partLinks.get("baseFlange");
    var local=scene.graspManipulator.forwardKinematics(q),p=local.translation,r=local.rotation;
    return AssemblyFrames.compose(physical(sim,base.link),{x:p.x,y:p.y,z:p.z,qx:r.x,qy:r.y,qz:r.z,qw:r.w});
  }
  public static function escape(scene:TendingElectricSimScene,sim:Simulation,deltaMm:Array<Float>,maximumStepMm:Float):Array<Array<Float>> {
    if(deltaMm.length!=3 || !Math.isFinite(maximumStepMm) || maximumStepMm<=0)throw "Invalid extraction sampling";
    var squared=0.0;for(v in deltaMm){if(!Math.isFinite(v))throw "Invalid extraction vector";squared+=v*v;}
    var count=Std.int(Math.ceil(Math.sqrt(squared)/maximumStepMm));if(count<1 || count>200)throw "Extraction exceeds its bounded local corridor";
    var start=physical(sim,scene.flangeLink),q=[for(j in scene.armJoints)sim.jointSample(0,j).get_position()],result=[q];
    for(i in 1...count+1) {
      var target:AssemblyFrame={x:start.x+deltaMm[0]*i/count*0.001,y:start.y+deltaMm[1]*i/count*0.001,z:start.z+deltaMm[2]*i/count*0.001,
        qx:start.qx,qy:start.qy,qz:start.qz,qw:start.qw};
      q=solveWrist(scene,sim,target,q);result.push(q);
    }
    return result;
  }
  public static function checkPath(scene:TendingElectricSimScene,sim:Simulation,fixed:Array<{name:String,object:SimObject}>,
      carried:Array<{side:String,object:SimObject}>,path:Array<Array<Float>>,activeObject:Int,fixtureCommandContacts:Array<String>):Dynamic {
    if(path.length<2)throw "Observed path needs endpoints";
    return check(scene,sim,fixed,carried,path[0],path[path.length-1],0,activeObject,fixtureCommandContacts,false,path);
  }
  /** Full observed geometry for a new edge; only declared working-stock contacts may be command envelopes. */
  public static function check(scene:TendingElectricSimScene,sim:Simulation,fixed:Array<{name:String,object:SimObject}>,
      carried:Array<{side:String,object:SimObject}>,from:Array<Float>,to:Array<Float>,maximumPointMotionMm:Float,
      activeObject:Int,fixtureCommandContacts:Array<String>,syntheticOverlap:Bool=false,?path:Array<Array<Float>>):Dynamic {
    if(from.length!=6 || to.length!=6 || !Math.isFinite(maximumPointMotionMm) || maximumPointMotionMm<0)throw "Invalid observed edge";
    var route:Array<Array<Float>> = path==null ? [from,to] : cast path;
    if(route.length<2)throw "Observed path needs endpoints";
    for(q in route){if(q.length!=6)throw "Observed path needs six arm coordinates";for(v in q)if(!Math.isFinite(v))throw "Observed path must be finite";}
    var model=scene.arm.model,kinematic=robotkit.kinematics.RobotKinematics.compile(model),identity:RobotRuntimeIdentity=cast scene.blueprint.identity;
    var reference=new KinematicState(kinematic),armDofs=[for(i in 1...7)kinematic.dofIndex("j"+i)];
    for(i in 0...kinematic.dofCount()) {
      var measured=sim.jointSample(0,identity.jointIndex(model.joints[kinematic.dofJoint[i]].id)).get_position();
      if(armDofs.indexOf(i)<0){var clipped=Math.max(kinematic.dofLower[i],Math.min(kinematic.dofUpper[i],measured));if(Math.abs(clipped-measured)>1e-5)throw "Observed process joint is outside its collision model";measured=clipped;}
      reference.q[i]=measured;
    }
    var root=physical(sim,0);reference.setRootPose(kinematic.bodyIndex(model.links[0].id),new Transform(root.x,root.y,root.z,root.qx,root.qy,root.qz,root.qw));
    var snapshot=new KinematicSnapshot(kinematic);snapshot.evaluate(reference);
    var description=new CollisionDescription(1e-6),bodies=ModelBodies.describe(description,kinematic,"observed-grasp",1,reference,"observed state",true);
    var flange=kinematic.bodyIndex(identity.linkId(scene.flangeLink)),inverse=AssemblyFrames.inverse(physical(sim,scene.flangeLink));
    var tool=bodies.carry(flange,"observed-constant-jaws",2),groups=KinematicBodyPairs.rigidGroups(kinematic);
    // These geometries share a constant carrier during an arm-only correction.
    for(i in 0...groups.length)if(groups[i]==groups[flange])description.setBodyRule(tool,bodies.bodies[i],Allow,Rigid);
    var travel=[for(_ in 0...kinematic.dofCount())0.0];
    for(i in 0...6){if(armDofs[i]<0 || !Math.isFinite(to[i]) || !Math.isFinite(from[i]))throw "Invalid observed arm edge";for(k in 1...route.length)travel[armDofs[i]]+=Math.abs(route[k][i]-route[k-1][i]);}
    var reach=new MotionReach(kinematic),maximumMotion=0.0,maximumMismatch=0.0,toolObjects:Array<{id:Int,link:Int}> = [];
    for(piece in scene.blueprint.linkCollisionShapes) {
      var id=identity.linkId(piece.link),body=kinematic.bodyIndex(id),actual=physical(sim,piece.link),vertices=piece.shape.convexVertices();
      var owner=bodies.bodies[body],offset=AssemblyFrames.identity(),carrier=body;
      if(StringTools.startsWith(id,"electric/")){owner=tool;offset=AssemblyFrames.compose(inverse,actual);carrier=flange;}
      var predicted=snapshot.bodyPose(carrier).compose(new Transform(offset.x,offset.y,offset.z,offset.qx,offset.qy,offset.qz,offset.qw));
      var radius=0.0;for(i in 0...Std.int(vertices.length/3))radius=Math.max(radius,Math.sqrt(vertices[i*3]*vertices[i*3]+vertices[i*3+1]*vertices[i*3+1]+vertices[i*3+2]*vertices[i*3+2]));
      var dx=predicted.x-actual.x,dy=predicted.y-actual.y,dz=predicted.z-actual.z,dot=Math.min(1,Math.abs(predicted.qx*actual.qx+predicted.qy*actual.qy+predicted.qz*actual.qz+predicted.qw*actual.qw));
      var mismatch=Math.sqrt(dx*dx+dy*dy+dz*dz)+2*radius*Math.sqrt(Math.max(0,1-dot*dot));maximumMismatch=Math.max(maximumMismatch,mismatch);
      if(mismatch>RESERVE)throw "Observed grasp FK differs from physical geometry beyond reserve";
      maximumMotion=Math.max(maximumMotion,reach.bound(carrier,radius+Math.sqrt(offset.x*offset.x+offset.y*offset.y+offset.z*offset.z),travel));
      var object=description.addObject(id,owner,pose(offset),Convex(vertices),piece.shape.inflation+RESERVE,"observed physical hull");
      toolObjects.push({id:object,link:piece.link});
    }
    var active:Array<Int> = [];
    for(payload in carried) {
      var offset=AssemblyFrames.compose(inverse,objectFrame(sim,payload.object)),owner=bodies.carry(flange,"payload-"+payload.side,2);
      for(vertices in pieces(payload.object)) {
        var object=description.addObject("payload/"+payload.side,owner,pose(offset),Convex(vertices),RESERVE,"observed grasp");
        if(payload.object.handle.rawValue()==activeObject)active.push(object);
        var jaws:Array<Int> = cast scene.jawLinks.get(payload.side);
        for(piece in toolObjects)if(jaws.indexOf(piece.link)>=0)description.setObjectRule(object,piece.id,Allow,DeclaredContact);
        var radius=payload.object.shape.boundingRadius()+Math.sqrt(offset.x*offset.x+offset.y*offset.y+offset.z*offset.z);
        maximumMotion=Math.max(maximumMotion,reach.bound(flange,radius,travel));
      }
    }
    if(maximumPointMotionMm>0 && maximumMotion*1000>maximumPointMotionMm)throw "Observed grasp correction exceeds its point-motion budget: "+maximumMotion*1000;
    var moving=bodies.all();for(i in 0...moving.length)for(j in i+1...moving.length)if(description.bodyRule(moving[i],moving[j])==null)description.setBodyRule(moving[i],moving[j],Check,Checked);
    for(item in fixed)for(vertices in pieces(item.object)) {
      var object=description.addFixed(item.name,pose(objectFrame(sim,item.object)),Convex(vertices),0,0,"observed fixture/stock");
      if(item.name=="armRiser")for(piece in toolObjects)if(piece.link==0)description.setObjectRule(piece.id,object,Allow,Rigid); // Matches the explicit physical base/riser exclusion.
      if(fixtureCommandContacts.indexOf(item.name)>=0)for(stock in active)description.setObjectRule(stock,object,Allow,DeclaredContact);
    }
    if(syntheticOverlap) {
      var added=false;
      for(piece in scene.blueprint.linkCollisionShapes)if(!added && StringTools.startsWith(identity.linkId(piece.link),"electric/")) {
        var vertices=piece.shape.convexVertices(),centre=[0.0,0,0],count=Std.int(vertices.length/3);
        if(count<4)throw "Collision control needs a solid physical hull";
        for(i in 0...count)for(j in 0...3)centre[j]+=vertices[i*3+j]/count;
        var at=AssemblyFrames.transformPoint(physical(sim,piece.link),centre[0],centre[1],centre[2]);
        description.addFixed("synthetic query-only overlapping obstacle",new CollisionPose(at.x,at.y,at.z,0,0,0,1),Box(0.001,0.001,0.001),0);added=true;
      }
      if(!added)throw "Collision control needs tool geometry";
    }
    bodies.declare();var world=new NativeCollisionWorld();
    try {
      var build=description.build(world);if(build.overlapsAtReference!=0)throw "Observed grasp checking cannot inherit reference-overlap exemptions";
      var space=new CollisionPlannerSpace(description,build,bodies,description.margins);
      function full(q:Array<Float>):Array<Float>{var value=reference.q.copy();for(i in 0...6)value[armDofs[i]]=q[i];return value;}
      var packed:Array<Float> = [];
      for(i in 1...route.length)for(q in [route[i-1],route[i]])for(v in full(q))packed.push(v);
      var checks=space.edgesClear(packed,route.length-1);
      for(i in 0...checks.length)if(!checks[i])throw "Observed grasp edge is blocked or unproven: "+i+"; "+space.invalid(full(route[i+1]));
      var result={passed:true,continuous:true,edgesChecked:route.length-1,maximumPointMotionBoundMm:maximumMotion*1000,maximumFKMismatchMm:maximumMismatch*1000,
        pointReserveMm:RESERVE*1000,fixtureCommandContacts:fixtureCommandContacts.copy(),
        scope:"observed robot, constant jaw geometry, retained grasps and all cell fixture/stock hulls; live contacts unchanged"};
      world.dispose();return result;
    } catch(error:Dynamic){world.dispose();throw error;}
  }
  public static function controls(scene:TendingElectricSimScene,sim:Simulation,fixed:Array<{name:String,object:SimObject}>,
      carried:Array<{side:String,object:SimObject}>):Dynamic {
    var time=sim.simulationTime(),q=[for(j in scene.armJoints)sim.jointSample(0,j).get_position()];
    var positive=check(scene,sim,fixed,carried,q,q,0.12,-1,[]),rejected=false;
    try check(scene,sim,fixed,carried,q,q,0.12,-1,[],true) catch(error:Dynamic) {
      if(Std.string(error).indexOf("blocked or unproven")<0)throw error;rejected=true;
    }
    if(!rejected || sim.simulationTime()!=time)throw "Observed grasp collision controls failed or advanced physics";
    return {positiveControl:positive,negativeControlRejectsOverlap:true,queryOnly:true};
  }
}
