import cadkit.modeling.AssemblyState;
import cadbridge.AssemblySimulationBridge.AssemblySimulationModel;
import collisionkit.CollisionDescription;
import collisionkit.CollisionGeometry;
import collisionkit.CollisionPose;
import collisionkit.kinematics.ModelBodies;
import collisionkit.native.NativeCollisionWorld;
import kinematicskit.Transform;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact.SceneArtifactData;
import motionkit.planner.CachedPlannerSpace;
import motionkit.planner.RrtConnect;
import motionkit.robot.CollisionPlannerSpace;
import motionkit.robot.ManipulatorPlannerSpace;
import robotkit.manipulation.Manipulator;

/** One immutable physical state/policy. Never shares query caches across stations or jaw states. */
class TendingElectricPlanner implements motionkit.planner.PlannerSpace {
  static final AIR=robotkit.collision.CollisionClearance.MARGIN;
  static final WORKING=robotkit.collision.CollisionClearance.CONTACT_MARGIN;
  static final MESH_RESERVE=0.00015;
  final native:NativeCollisionWorld;
  final manipulator:Manipulator;
  final contactPairs:Array<collisionkit.CollisionPair> = [];
  final contactNames:Array<String> = [];
  public var lastEdgeReason(default,null):Null<String> = null;
  final continuous:CollisionPlannerSpace;
  final space:CachedPlannerSpace;
  public final buildSeconds:Float;
  public final rigidQualifications:Array<{first:String,second:String,gapMm:Float,requiredMm:Float}> = [];

  static function pose(f:AssemblyFrame,scale:Float):CollisionPose
    return new CollisionPose(f.x*scale,f.y*scale,f.z*scale,f.qx,f.qy,f.qz,f.qw);

  public function new(arm:Manipulator,converted:AssemblySimulationModel,placement:AssemblyFrame,
      armState:AssemblyState,fixtures:AssemblyState,tool:SceneArtifactData,ts:AssemblyState,
      hulls:Map<String,Array<Array<Float>>>,hidden:Null<String>,side:String,working:Bool,stockContacts:Array<String>,workingPrefix:String,work:TendingElectricApproach,?commandContactPair:String) {
    manipulator=arm;
    if(commandContactPair!=null && (!working || work.station!="vise-load" || work.stockFixed() ||
      commandContactPair!="mill/mill/vise/endStop"))throw "Only the active stock/end-stop search pair can have a command envelope";
    var started=Sys.time(),description=new CollisionDescription(AIR);
    var reference=arm.stateOf([for(_ in 0...arm.dofCount())0.0]),model=arm.model;
    var root=new Transform(placement.x/1000,placement.y/1000,placement.z/1000,placement.qx,placement.qy,placement.qz,placement.qw);
    for(i in 0...model.bodyCount()) if(model.bodyParentJoint[i]<0)
      reference.setRootPose(i,root.compose(reference.rootPose(i)));
    // Validate metre conversion and every CAD-to-link carrier against independent assembly FK.
    var zero=new AssemblyState(armState.definition);
    for(i in 0...arm.dofCount())zero.setJoint("j"+(i+1),0);
    var snapshot=kinematicskit.KinematicSnapshot.of(reference);
    for(id in converted.partLinks.keys()) {
      var mapping:cadbridge.AssemblySimulationBridge.AssemblyPartLink=converted.partLinks.get(id);
      if(mapping==null)throw 'Missing arm mapping $id';
      var actual=snapshot.bodyPose(model.bodyIndex(converted.model.links[mapping.link].id)).compose(
        new Transform(mapping.offset.x,mapping.offset.y,mapping.offset.z,mapping.offset.qx,mapping.offset.qy,mapping.offset.qz,mapping.offset.qw));
      var expected=AssemblyFrames.compose(placement,zero.worldPose(id));
      var dx=actual.x-expected.x/1000,dy=actual.y-expected.y/1000,dz=actual.z-expected.z/1000;
      var dot=Math.abs(actual.qx*expected.qx+actual.qy*expected.qy+actual.qz*expected.qz+actual.qw*expected.qw);
      if(Math.sqrt(dx*dx+dy*dy+dz*dz)>0.000001 || 2*Math.acos(Math.min(1,dot))>0.00001)
        throw 'Planner/assembly carrier mismatch for $id';
    }
    var bodies=ModelBodies.describe(description,model,"electric-arm",1,reference,"fixed cell placement",true);
    // Do not convert a coarse overlap at the zero pose into a permanent exemption.
    for(i in 0...bodies.bodies.length)for(j in i+1...bodies.bodies.length)
      if(description.bodyRule(bodies.bodies[i],bodies.bodies[j])==null)
        description.setBodyRule(bodies.bodies[i],bodies.bodies[j],Check,Checked);
    var flange:cadbridge.AssemblySimulationBridge.AssemblyPartLink=converted.partLinks.get("toolFlange");
    if(flange==null)throw "Missing tool flange carrier";
    var carrier=model.bodyIndex(converted.model.links[flange.link].id);
    var toolBody=bodies.carry(carrier,"complete-electric-tool",2);
    var toolObjects:Map<String,Int> = [],armObjects:Map<String,Array<Int>> = [],fixtureObjects:Map<String,Array<Int>> = [];
    for(o in armState.definition.occurrences) {
      var pieces=hulls.get("arm:"+o.definition);if(pieces==null || pieces.length==0)continue;
      var link:cadbridge.AssemblySimulationBridge.AssemblyPartLink=converted.partLinks.get(o.id);if(link==null)throw 'No robot carrier for ${o.id}';
      var body=model.bodyIndex(converted.model.links[link.link].id),objects=[];
      for(piece in pieces)objects.push(description.addObject("arm/"+o.id,bodies.bodies[body],pose(link.offset,1),Convex([for(v in piece)v/1000])));
      armObjects.set(o.id,objects);
    }
    // Transfer space covers the complete jaw stroke, so its immutable cache
    // is shared by pickup and placement cases with the same door state.
    var closed:Null<AssemblyState> = null;
    if(work.isTransport) {
      fixtures=new AssemblyState(fixtures.definition,fixtures.record());fixtures.setJoint("mill/mill/vise/jaw",0);
      closed=new AssemblyState(fixtures.definition,fixtures.record());closed.setJoint("mill/mill/vise/jaw",6);
    }
    for(o in fixtures.definition.occurrences) {
      if(o.id==hidden)continue;
      var pieces=hulls.get("cell:"+o.definition);if(pieces==null || pieces.length==0)continue;
      var body=description.addBody("cell/"+o.id,0,true,pose(fixtures.worldPose(o.id),0.001)),objects=[];
      var delta=closed==null ? AssemblyFrames.identity() : AssemblyFrames.compose(AssemblyFrames.inverse(fixtures.worldPose(o.id)),closed.worldPose(o.id));
      for(piece in pieces) {
        var vertices=[for(v in piece)v/1000];
        if(Math.abs(delta.x)+Math.abs(delta.y)+Math.abs(delta.z)>0.000001) {
          // A prismatic translation's endpoint convex hull is its exact sweep.
          for(i in 0...Std.int(piece.length/3)){vertices.push((piece[i*3]+delta.x)/1000);vertices.push((piece[i*3+1]+delta.y)/1000);vertices.push((piece[i*3+2]+delta.z)/1000);}
        }
        objects.push(description.addObject("cell/"+o.id,body,CollisionPose.identity(),Convex(vertices)));
      }
      fixtureObjects.set(o.id,objects);
    }
    if(work.isTransport) {
      var stock=description.addBody("stock/vise-envelope",0,true,pose(fixtures.worldConnector("mill/mill/vise/body","seat"),0.001));
      description.addObject("stock/vise-envelope",stock,CollisionPose.translation(0,0,0.01),Box(0.03,0.03,0.01));
    }
    var openState:Null<AssemblyState> = null;
    if(work.isTransport) {openState=new AssemblyState(ts.definition,ts.record());openState.setJoint("a-jaw-opening",25);openState.setJoint("b-jaw-opening",25);}
    // Surface meshes preserve the cable loops; a convex hull would fill their empty interiors.
    // They remain a surface-model limitation, not a claim of exact solid penetration testing.
    for(o in ts.definition.occurrences) {
      var rendered=[for(p in tool.parts)if(p.id==o.definition)p][0];
      var vertices=[for(i in 0...rendered.vertexCount*3)rendered.vertices.getDouble(i*8)/1000];
      var geometry:CollisionGeometry=StringTools.endsWith(o.id,"blank-60")
        ? Convex([for(v in cadkit.ConvexHullVertices.fromMesh(rendered.vertices,rendered.vertexCount))v/1000])
        : Mesh(vertices,[for(i in 0...rendered.indexCount)rendered.indices.getInt32(i*4)]);
      var local=AssemblyFrames.compose(flange.offset,{x:ts.worldPose(o.id).x/1000,y:ts.worldPose(o.id).y/1000,z:ts.worldPose(o.id).z/1000,
        qx:ts.worldPose(o.id).qx,qy:ts.worldPose(o.id).qy,qz:ts.worldPose(o.id).qz,qw:ts.worldPose(o.id).qw});
      var owner=toolBody,offset=pose(local,1),name="tool/"+o.id;
      if(work.stockFixed() && o.id==side+"-blank-60") {
        owner=description.addBody("stock/"+work.station,0,true,pose(work.stockPose(ts),0.001));
        offset=CollisionPose.identity();name="stock/"+work.station;
      }
      toolObjects.set(o.id,description.addObject(name,owner,offset,geometry,0,"CAD tessellation",o.id,true));
      if(openState!=null && (StringTools.endsWith(o.id,"jaw-left") || StringTools.endsWith(o.id,"jaw-right") ||
        StringTools.endsWith(o.id,"finger-left") || StringTools.endsWith(o.id,"finger-right"))) {
        var at=openState.worldPose(o.id);
        var opened=AssemblyFrames.compose(flange.offset,{x:at.x/1000,y:at.y/1000,z:at.z/1000,qx:at.qx,qy:at.qy,qz:at.qz,qw:at.qw});
        toolObjects.set(o.id+"/open-envelope",description.addObject("tool/"+o.id+"/open-envelope",toolBody,pose(opened,1),geometry,0,"Open-jaw transport envelope",o.id,true));
      }
    }
    function toolObject(id:String):Int {var value=toolObjects.get(id);if(value==null)throw 'Missing tool object $id';return value;}
    // The native backend inflates convex shapes, not mesh BVHs. Cover mesh
    // tessellation error with pair margins; whole-edge body inflation is a
    // separate conservative motion bound handled by CollisionPlannerSpace.
    for(id in toolObjects.keys())if(!StringTools.endsWith(id,"blank-60"))
      for(other in 0...description.objects.length)if(description.objects[other].body!=toolBody)
        description.setObjectMargins(toolObject(id),other,AIR+MESH_RESERVE,AIR+MESH_RESERVE);
    for(object in armObjects.get("toolFlange"))description.setObjectRule(toolObject("adapter"),object,Allow,DeclaredContact);
    if(fixtureObjects.exists("armRiser"))for(a in armObjects.get("baseFlange"))for(b in fixtureObjects.get("armRiser"))
      description.setObjectRule(a,b,Allow,DeclaredContact);
    if(working)for(id in toolObjects.keys()) {
      var member=StringTools.startsWith(id,side+"-") ? id.substr(2) : "";
      if(["pgc-140-50","jaw-left","jaw-right","finger-left","finger-right","blank-60"].indexOf(member)<0)continue;
      for(fixture in fixtureObjects.keys()) {
        var selected=StringTools.startsWith(fixture,workingPrefix);
        if(selected)for(other in fixtureObjects.get(fixture))description.setObjectMargins(toolObject(id),other,WORKING+(member=="blank-60" ? 0 : MESH_RESERVE),WORKING+(member=="blank-60" ? 0 : MESH_RESERVE));
        if(!work.stockFixed() && member=="blank-60" && stockContacts.indexOf(fixture)>=0)
          for(other in fixtureObjects.get(fixture)) {
            // Only a bounded contact-search COMMAND envelope can cross the stop.
            // Its real stock/stop contact remains subject to live force/depth guards.
            description.setObjectRule(toolObject(id),other,fixture==commandContactPair ? Allow : Touch,DeclaredContact);
            description.setObjectMargins(toolObject(id),other,0,0);
          }
      }
    }
    if(work.stockFixed() && working)for(member in ["finger-left","finger-right"])
      description.setObjectMargins(toolObject(side+"-"+member),toolObject(side+"-blank-60"),MESH_RESERVE,MESH_RESERVE);
    // Tool and carrier share one rigid transform. Qualify only pairs whose
    // enclosing boxes in that common frame already prove the required gap.
    // No adjacency inheritance and no surface-mesh containment assumption.
    var bounds:Map<Int,collisionkit.CollisionBounds> = [];
    function box(id:Int):collisionkit.CollisionBounds {
      var value=bounds.get(id);
      if(value==null) {var object=description.objects[id];value=collisionkit.CollisionBounds.of(object.geometry,object.offset,object.inflation);bounds.set(id,value);}
      return value;
    }
    var carrierObjects=[for(i in 0...description.objects.length)if(description.objects[i].body==bodies.bodies[carrier])i];
    for(id in toolObjects.keys()) {
      var first=toolObject(id);if(description.objects[first].body!=toolBody)continue;
      var a=box(first),required=AIR+(StringTools.endsWith(id,"blank-60") ? 0 : MESH_RESERVE);
      for(second in carrierObjects) {
        var rule=description.objectRule(first,second);if(rule!=null && rule.rule==Allow)continue;
        var b=box(second);
        var x=Math.max(0,Math.abs(a.centre.x-b.centre.x)-a.halfExtents.x-b.halfExtents.x);
        var y=Math.max(0,Math.abs(a.centre.y-b.centre.y)-a.halfExtents.y-b.halfExtents.y);
        var z=Math.max(0,Math.abs(a.centre.z-b.centre.z)-a.halfExtents.z-b.halfExtents.z);
        var gap=Math.sqrt(x*x+y*y+z*z);
        if(gap>=required) {
          description.setObjectRule(first,second,Allow,Rigid);
          rigidQualifications.push({first:description.objects[first].name,second:description.objects[second].name,gapMm:gap*1000,requiredMm:required*1000});
        }
      }
    }
    bodies.declare();native=new NativeCollisionWorld();var build=description.build(native);
    if(build.overlapsAtReference!=0)throw "Electric planning must not inherit reference-overlap exemptions";
    for(rule in description.objectRules)if(rule.rule==Touch) {
      var a=description.objects[rule.a],b=description.objects[rule.b];
      if(description.bodies[a.body].fixed && description.bodies[b.body].fixed)continue;
      contactPairs.push(new collisionkit.CollisionPair(build.objects[rule.a],build.objects[rule.b],build.bodies[a.body],build.bodies[b.body]));
      contactNames.push(a.name+" / "+b.name);
    }
    continuous=new CollisionPlannerSpace(description,build,bodies,description.margins);
    space=new CachedPlannerSpace(new ManipulatorPlannerSpace(continuous,arm,[for(_ in 0...arm.dofCount())0.0],reference.q));
    buildSeconds=Sys.time()-started;
  }

  public function plan(start:Array<Float>,goal:Array<Float>,other:TendingElectricPlanner,?proposals:Array<Array<Array<Float>>>):Dynamic {
    var combined=new ElectricJawStates(space,other.space),started=Sys.time(),rejected:Array<String> = [];
    if(proposals!=null)for(path in proposals) {
      var failure=motionkit.planner.SavedPathValidation.failure(combined,path,start,goal);
      if(failure==null)return {found:true,failure:"",waypoints:path,iterations:0,nodes:0,edgeChecks:path.length-1,
        checkSeconds:Sys.time()-started,totalSeconds:Sys.time()-started,buildSeconds:buildSeconds,reused:true,
        qualifiedRigidPairs:rigidQualifications.length,proposalFailures:rejected};
      rejected.push(failure);
    }
    var planner=new RrtConnect(combined,17);planner.maxIterations=1000;planner.shortcutRounds=16;planner.visibilityCheckLimit=128;
    var result=planner.plan(start,goal);
    return {found:result.found(),failure:result.failure,waypoints:result.waypoints,iterations:result.iterations,
      nodes:result.nodes,edgeChecks:result.edgeChecks,checkSeconds:result.checkSeconds,totalSeconds:result.totalSeconds,
      buildSeconds:buildSeconds,reused:false,proposalFailures:rejected,qualifiedRigidPairs:rigidQualifications.length,continuousQueries:continuous.queries,pointCacheHits:space.pointHits,edgeCacheHits:space.edgeHits};
  }
  public function dimension():Int return space.dimension();
  public function lower():Array<Float> return space.lower();
  public function upper():Array<Float> return space.upper();
  public function edgesClear(edges:Array<Float>,count:Int):Array<Bool> return space.edgesClear(edges,count);
  function touching(q:Array<Float>):Null<String> {
    if(contactPairs.length==0)return null;
    continuous.bodies.place(continuous.build,continuous.bodies.state(manipulator.stateOf(q).q));
    var distances=native.pairDistances(contactPairs);
    // Up to one micron of numerical penetration is classified as guarded
    // contact, never accepted. Larger penetration retains its collision error.
    for(i in 0...distances.length)if(distances[i].distance>=-0.000001 && distances[i].distance<=0.00000001)
      return "intended stock/fixture contact requires a guarded transition: "+contactNames[i];
    return null;
  }
  public function edge(a:Array<Float>,b:Array<Float>):Bool {
    lastEdgeReason=touching(a);if(lastEdgeReason==null)lastEdgeReason=touching(b);
    // Isotropic motion inflation cannot prove a zero-clearance endpoint or
    // sliding contact. Return the missing process action, never waive the pair.
    return lastEdgeReason==null && space.edgesClear(a.concat(b),1)[0];
  }
  public function invalid(q:Array<Float>):Null<String> return space.invalid(q);
  public function close():Void native.dispose();
}

/** Every search edge must be clear in both retained-stock jaw states. */
private class ElectricJawStates implements motionkit.planner.PlannerSpace {
  final first:motionkit.planner.PlannerSpace;
  final second:motionkit.planner.PlannerSpace;
  public function new(first,second) {this.first=first;this.second=second;}
  public function dimension():Int return first.dimension();
  public function lower():Array<Float> return first.lower();
  public function upper():Array<Float> return first.upper();
  public function invalid(q:Array<Float>):Null<String> {
    var found=first.invalid(q);return found==null ? second.invalid(q) : found;
  }
  public function edgesClear(edges:Array<Float>,count:Int):Array<Bool> {
    var a=first.edgesClear(edges,count),b=second.edgesClear(edges,count);
    return [for(i in 0...count)a[i] && b[i]];
  }
}
