import cadbridge.AssemblySimulationBridge;
import cadbridge.AssemblyPhysicalPartView;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactData;
import materia.assembly.AssemblyFrames;
import cadkit.modeling.AssemblyState;
import robotkit.model.*;
import robotkit.runtime.*;

/** Runtime built from reviewed meshes, with explicit collision and mass assumptions for physics. */
class TendingElectricSimScene {
  public final arm:AssemblySimulationBridge.AssemblySimulationModel;
  public final tool:AssemblySimulationBridge.AssemblySimulationModel;
  public final blueprint:RobotRuntimeBlueprint;
  public final gravity:TendingElectricGravity;
  public final armJoints:Array<Int>;
  public final jawJoints:Map<String,Int> = [];
  public final jawOriginMm:Map<String,Float> = [];
  public final jawFrameChecks:Int;
  public function jawTarget(side:String,openingMm:Float):Float return (openingMm-(cast jawOriginMm.get(side):Float))*0.001;
  public function jawOpeningMm(side:String,runtimeM:Float):Float return runtimeM*1000+(cast jawOriginMm.get(side):Float);
  public final jawLinks:Map<String,Array<Int>> = [];
  public final graspFrames:Map<String,materia.assembly.AssemblyRecord.AssemblyFrame> = [];
  public final graspManipulator:robotkit.manipulation.Manipulator;
  public final flangeLink:Int;
  public final flangeOffset:materia.assembly.AssemblyRecord.AssemblyFrame;
  public function new(armScene:SceneArtifactData,toolScene:SceneArtifactData,toolSha256:String,graspProfile:String="tending-top-entry") {
    arm=AssemblySimulationBridge.toRobotModel(armScene.assemblyDefinition,AssemblyPhysicalPartView.fromSceneArtifact(armScene));
    // Reference blanks are illustration geometry, not a second copy of the cell stock.
    var definition:materia.assembly.AssemblyDefinition=cast toolScene.assemblyDefinition;
    var referenceState=new AssemblyState(definition),referenceGrasps:Map<String,materia.assembly.AssemblyRecord.AssemblyFrame> = [];
    for(side in ["a","b"])referenceGrasps.set(side,referenceState.worldPose(side+"-blank-60"));
    var hidden=["a-blank-60","b-blank-60"];
    definition.occurrences=[for(o in definition.occurrences)if(hidden.indexOf(o.id)<0)o];
    definition.joints=[for(j in definition.joints)if(hidden.indexOf(j.parent)<0 && hidden.indexOf(j.child)<0)j];
    var authored=new AssemblyState(definition);
    for(side in ["a","b"])jawOriginMm.set(side,authored.joint(side+"-jaw-opening"));
    var physical=AssemblyPhysicalPartView.fromSceneArtifact(toolScene);
    var inflation:Map<String,Float> = [];
    var reference:Dynamic=haxe.Json.parse(sys.io.File.getContent("machinekit/examples/dh-gripper/build/reference-frame.json"));
    var offset:Array<Float> = cast reference.offset;
    for(d in definition.definitions) {
      var part=[for(p in physical.parts)if(p.id==d.id)p][0];
      if(d.id.indexOf("finger-")>=0) {
        var boxes=machinekit.robotics.OffsetFingerGeometry.collisionBoxes(StringTools.endsWith(d.id,"left"));
        var rendered=[for(p in toolScene.parts)if(p.id==d.id)p][0];
        for(i in 0...rendered.vertexCount) {
          var covered=false;
          for(box in boxes) {
            var fits=true;
            for(a in 0...3) {
              var v=rendered.vertices.getDouble(i*24+a*8)-offset[a];
              if(v<box[a*2]-0.000001 || v>box[a*2+1]+0.000001)fits=false;
            }
            if(fits)covered=true;
          }
          if(!covered)throw "Simulation finger construction no longer encloses the reviewed finger mesh";
        }
        d.collisionHulls=[for(box in boxes) {
          var points:Array<Float> = [];
          for(x in [box[0],box[1]])for(y in [box[2],box[3]])for(z in [box[4],box[5]]){
            points.push(x+offset[0]);points.push(y+offset[1]);points.push(z+offset[2]);
          }points;
        }];
      } else if(d.id.indexOf("cable")>=0 || d.id.indexOf("lead")>=0 || d.id.indexOf("branch")>=0) {
        var rendered=[for(p in toolScene.parts)if(p.id==d.id)p][0];
        var vertices=[for(i in 0...rendered.vertexCount*3)rendered.vertices.getDouble(i*8)];
        var indices=[for(i in 0...rendered.indexCount)rendered.indices.getInt32(i*4)];
        var cache="machinekit/examples/machine-tending/build/electric-simulation-inputs/convex-v1-"+toolSha256+"-"+d.id+".json";
        if(sys.FileSystem.exists(cache)) {
          var saved:Dynamic=haxe.Json.parse(sys.io.File.getContent(cache));d.collisionHulls=cast saved.pieces;inflation.set(d.id,saved.inflationMm);
        } else {
          Sys.println("Decomposing electric service geometry: "+d.id);
          var decomposed=new collisionkit.native.NativeConvexDecomposer().decompose(vertices,indices,32,0);
          d.collisionHulls=decomposed.pieces;inflation.set(d.id,decomposed.inflation);
          sys.io.File.saveContent(cache,haxe.Json.stringify({pieces:decomposed.pieces,inflationMm:decomposed.inflation}));
        }
      } else {
        var rendered=[for(p in toolScene.parts)if(p.id==d.id)p][0];
        d.collisionHulls=[cadkit.ConvexHullVertices.enclosingFromMesh(rendered.vertices,rendered.vertexCount,0.001,0.151).vertices];
      }
    }
    tool=AssemblySimulationBridge.toRobotModel(definition,physical,null,null,id -> {
      var occurrence=[for(o in definition.occurrences)if(o.id==id)o][0];
      var part=[for(p in physical.parts)if(p.id==occurrence.definition)p][0];
      if(StringTools.endsWith(id,"pgc-140-50"))return 1.0;
      // Explicit simulation mass estimates; imported reference visuals declare zero mass.
      return part.volume*(id.indexOf("finger")>=0 || id=="adapter" ? 2700 : id.indexOf("jaw-")>=0 ? 7800 : 1100)*1e-9;
    });
    for(link in tool.model.links)for(shape in link.collisionShapes)if(inflation.exists(shape.name))shape.inflation=((cast inflation.get(shape.name):Float)+0.151)*0.001;
    // The bridge rebases every joint around the CAD default, not around zero.
    var kinematic=robotkit.kinematics.RobotKinematics.compile(tool.model);
    var state=new kinematicskit.KinematicState(kinematic),snapshot=new kinematicskit.KinematicSnapshot(kinematic),checks=0;
    for(opening in [0.0,17.0,25.0]) {
      for(side in ["a","b"]) {
        authored.setJoint(side+"-jaw-opening",opening);
        state.q[kinematic.dofIndex(side+"-jaw-opening")]=jawTarget(side,opening);
      }
      snapshot.evaluate(state);
      for(side in ["a","b"])for(member in ["jaw-left","jaw-right","finger-left","finger-right"]) {
        var id=side+"-"+member,part:AssemblySimulationBridge.AssemblyPartLink=cast tool.partLinks.get(id);
        var p=snapshot.bodyPose(kinematic.bodyIndex(tool.model.links[part.link].id));
        var actual=AssemblyFrames.compose({x:p.x,y:p.y,z:p.z,qx:p.qx,qy:p.qy,qz:p.qz,qw:p.qw},part.offset),expected=authored.worldPose(id);
        var orientation=actual.qx*expected.qx+actual.qy*expected.qy+actual.qz*expected.qz+actual.qw*expected.qw;
        if(Math.abs(Math.abs(orientation)-1)>0.0000000001 || Math.abs(actual.x*1000-expected.x)+Math.abs(actual.y*1000-expected.y)+Math.abs(actual.z*1000-expected.z)>0.000001)
          throw "Electric jaw runtime/CAD coordinate conversion changed the physical pose";
        checks++;
      }
    }
    jawFrameChecks=checks;
    // Fastener socket/recess detail is enclosed by one hull per rigid fastener.
    // Keep long-link and housing construction pieces separate.
    for(link in arm.model.links) {
      var groups:Map<String,Array<CollisionShape>> = [];
      for(shape in link.collisionShapes) {
        if(!groups.exists(shape.name))groups.set(shape.name,[]);
        var values:Array<CollisionShape> = cast groups.get(shape.name);values.push(shape);
      }
      var compact:Array<CollisionShape> = [];
      for(name in groups.keys()) {
        var values:Array<CollisionShape> = cast groups.get(name);
        if(values.length>1 && (name.indexOf("Bolt")>=0 || name.indexOf("bolt")>=0 || name.indexOf("screw")>=0)) {
          var points:Array<Float> = [];
          for(shape in values)for(v in shape.convexVertices())points.push(v);
          var bytes=haxe.io.Bytes.alloc(points.length*8);for(i in 0...points.length)bytes.setDouble(i*8,points[i]);
          var hull=cadkit.ConvexHullVertices.enclosingFromMesh(bytes,Std.int(points.length/3),0.000001,0);
          var shape=new CollisionShape(Convex(hull.vertices));shape.name=name;compact.push(shape);
        } else for(shape in values)compact.push(shape);
      }
      link.collisionShapes=compact;
    }
    var model=arm.model,linkMap:Map<String,Link> = [];
    for(link in tool.model.links) {
      var copy=new Link("electric/"+link.name);copy.mass=link.mass;copy.centerOfMass=link.centerOfMass.copy();
      copy.inertiaTensor=link.inertiaTensor.copy();copy.collisionShapes=link.collisionShapes.copy();
      linkMap.set(link.id,copy);model.addLink(copy);
    }
    for(j in tool.model.joints) {
      var copy=new Joint("electric/"+j.name,j.type,linkMap.get(j.parent.id),linkMap.get(j.child.id));
      copy.limits=j.limits.copy();copy.parentFramePosition=j.parentFramePosition.copy();copy.parentFrameRotation=j.parentFrameRotation.copy();
      copy.childFramePosition=j.childFramePosition.copy();copy.childFrameRotation=j.childFrameRotation.copy();copy.axis=j.axis.copy();model.addJoint(copy);
    }
    for(c in tool.model.couplings)model.addCoupling(new JointCoupling("electric/"+c.id,"electric/"+c.leader,"electric/"+c.follower,c.ratio,c.offset));
    var carrier:AssemblySimulationBridge.AssemblyPartLink=cast arm.partLinks.get("toolFlange");
    var adapter:AssemblySimulationBridge.AssemblyPartLink=cast tool.partLinks.get("adapter");
    flangeLink=carrier.link;flangeOffset=carrier.offset;
    var mount=new Joint("electric-mount",JointType.Fixed,model.links[carrier.link],linkMap.get(tool.model.links[adapter.link].id));
    var frame=AssemblyFrames.compose(carrier.offset,AssemblyFrames.inverse(adapter.offset));
    mount.parentFramePosition=[frame.x,frame.y,frame.z];mount.parentFrameRotation=[frame.qx,frame.qy,frame.qz,frame.qw];model.addJoint(mount);
    
    for(link in model.links)for(shape in link.collisionShapes)
      shape.surface=shape.name.indexOf("finger-")>=0 ? SimulatedContactSurfaces.gripperFinger(0.5) : SimulatedContactSurfaces.robotPart();
    for(side in ["a","b"]) {
      var at=AssemblyFrames.compose(cast referenceGrasps.get(side),TendingStockDatum.local(reference,graspProfile));
      graspFrames.set(side,AssemblyFrames.compose(carrier.offset,{x:at.x*0.001,y:at.y*0.001,z:at.z*0.001,qx:at.qx,qy:at.qy,qz:at.qz,qw:at.qw}));
    }
    var graspFrame=model.addFrame(new Frame("observed-grasp-carrier",model.links[carrier.link]));
    var base:AssemblySimulationBridge.AssemblyPartLink=cast arm.partLinks.get("baseFlange");
    graspManipulator=new robotkit.manipulation.Manipulator(model,model.links[base.link].id,graspFrame.id);
    // Imported CAD hulls define collision coverage. Coordinate-only links have no fallback volume.
    model.collisionApproximation=CollisionApproximation.None;
    blueprint=RobotRuntimeCompiler.compile(model,arm.profile);
    var identity:RobotRuntimeIdentity=cast blueprint.identity;
    gravity=new TendingElectricGravity(model,identity);
    armJoints=[for(j in 1...7)identity.jointIndex("j"+j)];
    for(side in ["a","b"]) {
      var joint=identity.jointIndex("electric/"+side+"-jaw-opening");
      if(joint<0)throw "Electric jaw leader was lost";
      jawJoints.set(side,joint);blueprint.processModelJoints.push(joint);
      // Explicit simulation-only jaw loop; no vendor motor/controller model is supplied.
      blueprint.joints[joint].servoStiffness=10000;blueprint.joints[joint].servoDamping=100;
      var links=[blueprint.joints[joint].childLink];
      for(c in blueprint.couplings)if(c.leader==joint)links.push(blueprint.joints[c.follower].childLink);
      if(links.length!=2)throw "Electric jaw coupling was lost";jawLinks.set(side,links);
    }
    for(j in armJoints)if(j<0)throw "Arm axis lost in electric runtime";
  }
}
