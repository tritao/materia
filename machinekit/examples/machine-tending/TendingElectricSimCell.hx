import TendingSeatPolicy.TendingSeatOrientation;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactData;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import cadkit.modeling.AssemblyState;
import cadbridge.AssemblyPhysicalPartView;
import cadbridge.AssemblySimulationBridge;
import nativekit.sim.*;
import robotkit.runtime.*;
import robotkit.core.JointTarget;
import haxe.Int64;

/** Shared arm, electric jaws, fixtures and free stock in Materia's physics world. */
class TendingElectricSimCell {
  public final harness:SimulationHarness;
  public final solverOptions:SimWorldOptions;
  public final runtime:RobotRuntime;
  public final scene:TendingElectricSimScene;
  public final names:Map<Int,String> = [];
  public final bodyNames:Map<Int,String> = [];
  public final stock:Map<Int,SimObject> = [];
  public final stockMassProperties:Map<Int,robotkit.tool.MassProperties> = [];
  public final grips:Map<String,SimulatedElectricGripper> = [];
  public final fixtureState:AssemblyState;
  public final fixtures:Map<String,SimObject> = [];
  public final driver:TendingElectricSimDriver;
  final doorStroke:Float;
  public final viseClosedPositionMm:Float;
  final plc=new cnckit.controller.RobotInterface();
  final zone:TendingElectricZone;
  final doorClosedFrame:AssemblyFrame;
  final doorOpenFrame:AssemblyFrame;
  final viseOpenFrame:AssemblyFrame;
  final viseClosedFrame:AssemblyFrame;
  final stockField:SimPose;
  public var progressPath="machinekit/examples/machine-tending/build/electric-physics-progress.json";
  public final graspObservations:Array<Dynamic> = [];
  public final seatObservations:Array<Dynamic> = [];
  public final seatingTrace:Array<Dynamic> = [];
  public final locatingObservations:Array<Dynamic> = [];
  public final placingObservations:Array<Dynamic> = [];
  public final registrationObservations:Array<Dynamic> = [];
  public final extractionObservations:Array<Dynamic> = [];
  var lastExecutionPointSpeed=0.0;
  final registeredStarts:Map<String,Bool> = [];
  public final releasedStockReturnChecks:Array<Dynamic> = [];
  final stoppedSearches:Map<String,Dynamic> = [];
  public var routeObservation(default,null):Dynamic = null;
  public var running(default,null)=false;
  public var plcAlarm(default,null)=0;
  var request=0;
  var completePulse=false;
  var lastOutput:Null<cnckit.controller.RobotInterfaceOutputs> = null;
  var tickCount=0;
  var releasing:Null<String> = null;
  public function new(scene:TendingElectricSimScene,fixturesScene:SceneArtifactData,park:Array<Float>,placement:AssemblyFrame,load:Array<Float>,backend:Int,?initialOpenSide:String,initialDoorOpen:Bool=false) {
    this.scene=scene;scene.gravity.mount(placement);
    solverOptions={integrator:2,frictionCone:2,frictionImpedanceRatio:10,noslipIterations:10,
      solverTolerance:1e-12,arenaMemoryBytes:128*1024*1024};
    harness=new SimulationHarness(0.01,10,backend,solverOptions);
    fixtureState=new AssemblyState(fixturesScene.assemblyDefinition);
    // The authored default is the preset blank-contact position, not cylinder end travel.
    viseClosedPositionMm=fixtureState.joint("mill/mill/vise/jaw");
    if(!Math.isFinite(viseClosedPositionMm) || viseClosedPositionMm<=0)throw "Missing authored vise grip position";
    for(i in 0...3)fixtureState.setJoint("mill/mill/"+["x","y","z"][i],load[i]);
    fixtureState.setJoint("mill/door",0);fixtureState.setJoint("mill/mill/vise/jaw",0);
    var definition:materia.assembly.AssemblyDefinition=cast fixturesScene.assemblyDefinition;
    doorStroke=0;for(j in definition.joints)if(j.id=="mill/door")doorStroke=cast j.limits.upper;
    doorClosedFrame=fixtureState.worldPose("mill/doorSlider");
    viseOpenFrame=fixtureState.worldPose("mill/mill/vise/movingJaw");
    fixtureState.setJoint("mill/door",doorStroke);doorOpenFrame=fixtureState.worldPose("mill/doorSlider");fixtureState.setJoint("mill/door",0);
    fixtureState.setJoint("mill/mill/vise/jaw",viseClosedPositionMm);viseClosedFrame=fixtureState.worldPose("mill/mill/vise/movingJaw");fixtureState.setJoint("mill/mill/vise/jaw",0);
    var sensorDefinitions:Array<materia.assembly.AssemblyDefinition.AssemblySensor> = cast definition.sensors;
    var sensor=[for(s in sensorDefinitions)if(s.id=="mill/stock-present")s][0];
    var stockMount=fixtureState.worldConnector(sensor.occurrence,sensor.connector);
    var centre=AssemblyFrames.transformPoint(stockMount,0,0,12.5);stockField=new SimPose(centre.x*0.001,centre.y*0.001,centre.z*0.001,stockMount.qx,stockMount.qy,stockMount.qz,stockMount.qw);
    runtime=harness.simulation.addRobotAtPose(scene.blueprint,[placement.x*0.001,placement.y*0.001,placement.z*0.001],
      [placement.qx,placement.qy,placement.qz,placement.qw]);
    harness.space.session.beginTopologyUpdate();
    var physical=AssemblyPhysicalPartView.fromSceneArtifact(fixturesScene);
    var moving:Map<String,Bool> = [];
    var initial:Map<String,AssemblyFrame> = [];
    for(o in fixtureState.definition.occurrences)initial.set(o.id,fixtureState.worldPose(o.id));
    for(joint in ["mill/door","mill/mill/vise/jaw"]) {
      fixtureState.setJoint(joint,joint=="mill/door" ? doorStroke : viseClosedPositionMm);
      for(o in fixtureState.definition.occurrences) {
        var a:AssemblyFrame=cast initial.get(o.id),b=fixtureState.worldPose(o.id);
        if(Math.abs(a.x-b.x)+Math.abs(a.y-b.y)+Math.abs(a.z-b.z)>0.000001)moving.set(o.id,true);
      }
      fixtureState.setJoint(joint,0);
    }
    if(initialDoorOpen)fixtureState.setJoint("mill/door",doorStroke);
    for(o in fixtureState.definition.occurrences) {
      if(StringTools.startsWith(o.id,"outfeed/blank-"))continue;
      var d=[for(d in definition.definitions)if(d.id==o.definition)d][0];
      var part=[for(p in physical.parts)if(p.id==o.definition)p][0];
      var pieces=AssemblySimulationBridge.collisionPieces(o.id,d,part);
      if(d.collisionHulls==null) {
        var rendered=[for(p in fixturesScene.parts)if(p.id==o.definition)p][0];
        pieces=[cadkit.ConvexHullVertices.enclosingFromMesh(rendered.vertices,rendered.vertexCount,0.001,0).vertices];
      }
      if(pieces.length==0)continue;
      var shape=SimShape.compoundConvex([for(piece in pieces)[for(v in piece)v*0.001]]);
      var blank=StringTools.startsWith(o.id,"infeed/blank-"),pose=pose(fixtureState.worldPose(o.id));
      var surface=SimulatedContactSurfaces.environmentPart();
      var properties=blank ? AssemblyPhysicalPartView.massProperties(part,physical.metresPerUnit) : null;
      var inertial:Null<SimInertialProperties> = null;
      if(properties!=null) {
        if(properties.massKg<=0 || properties.inertia==null)throw "Free stock needs positive CAD mass and inertia";
        var c=properties.centerOfMass,t:robotkit.spatial.Inertia3=cast properties.inertia;
        inertial=new SimInertialProperties([c.x,c.y,c.z],[t.xx,t.xy,t.xz,t.xy,t.yy,t.yz,t.xz,t.yz,t.zz]);
      }
      var object=harness.space.session.createObject(blank ? MotionType.Dynamic : moving.exists(o.id) ? MotionType.Kinematic : MotionType.Static,
        shape,pose,properties==null ? 0 : properties.massKg,null,surface,inertial);
      names.set(object.handle.rawValue(),o.id);bodyNames.set(harness.simulation.objectBody(object).rawValue(),o.id);
      if(blank) {
        var slot=o.id.substr("infeed/blank-".length).split("-"),index=Std.parseInt(slot[0])*3+Std.parseInt(slot[1]);
        stock.set(index,object);stockMassProperties.set(index,cast properties);
      } else fixtures.set(o.id,object);
    }
    var riser:SimObject=cast fixtures.get("armRiser");
    harness.simulation.excludeContactPair(0,0,riser);
    harness.space.session.endTopologyUpdate();
    var q=[for(_ in 0...scene.blueprint.jointCount)0.0];
    for(i in 0...6)q[scene.armJoints[i]]=park[i];
    for(side in ["a","b"]) {
      var leader:Int=cast scene.jawJoints.get(side);q[leader]=scene.jawTarget(side,side==initialOpenSide ? 25 : 17);
      for(c in scene.blueprint.couplings)if(c.leader==leader)q[c.follower]=c.ratio*q[leader]+c.offset;
      grips.set(side,new SimulatedElectricGripper(harness.simulation,0,leader,cast scene.jawLinks.get(side),scene.jawTarget(side,25),scene.jawTarget(side,0),70,0.005,scene.jawTarget(side,side==initialOpenSide ? 25 : 17)));
    }
    harness.simulation.setJointPositions(0,q);
    scene.gravity.check(q,scene.armJoints);
    var gravity=scene.gravity.torques(q);
    runtime.submitTargets([for(i in 0...6) {
      var j=scene.armJoints[i],drive=scene.blueprint.joints[j];
      if(drive.servoStiffness<=0)throw "Electric arm requires an explicit servo loop";
      JointTarget.servo(j,park[i],0,drive.servoStiffness,drive.servoDamping,gravity[j]);
    }],1);
    for(side in ["a","b"]){var grip:SimulatedElectricGripper=cast grips.get(side);grip.position(scene.jawTarget(side,side==initialOpenSide ? 25 : 17));}
    driver=new TendingElectricSimDriver(harness,runtime,scene,names,[for(g in grips)g]);
    driver.bodyNames=bodyNames;
    zone=new TendingElectricZone(scene,harness.simulation,fixtureState,stock);
    tick();
  }
  public static function pose(f:AssemblyFrame):SimPose return new SimPose(f.x*0.001,f.y*0.001,f.z*0.001,f.qx,f.qy,f.qz,f.qw);
  public function wristFrame():AssemblyFrame {
    var p=harness.simulation.linkPose(0,scene.flangeLink);
    return {x:p.position[0],y:p.position[1],z:p.position[2],qx:p.rotation[0],qy:p.rotation[1],qz:p.rotation[2],qw:p.rotation[3]};
  }
  public function tick():Void {
    driver.advance(0.01);harness.step(Int64.fromFloat(tickCount++*10000000.0));
    driver.checkRetention(releasing);
    updatePlc();
    if(runtime.snapshot().safety==RobotKitRuntimeConstants.RK_SAFETY_FAULT) {
      driver.stopAndHold();throw "Physics runtime safety fault";
    }
  }
  public function door(open:Bool):Void {
    request=open ? 1 : 2;tick();fixtureMotion("mill/door",open ? doorStroke : 0,2);request=0;tick();
  }
  function fixtureMotion(joint:String,target:Float,stepMm:Float):Void {
    var from=fixtureState.joint(joint),steps=Std.int(Math.ceil(Math.abs(target-from)/stepMm));
    if(steps<1)steps=1;
    for(i in 1...steps+1) {
      fixtureState.setJoint(joint,from+(target-from)*i/steps);
      for(id in fixtures.keys()) {var object:SimObject=cast fixtures.get(id);if(object.motion==MotionType.Kinematic)object.drive(pose(fixtureState.worldPose(id)));}
      tick();
      var contacts=harness.simulation.robotContacts(runtime);
      for(contact in contacts)if(contact.active) {
        var intended=false;
        for(grip in grips)if(contact.otherKind==RobotContactOtherKind.Object && grip.jawContact(contact.linkIndex) && grip.grippedObjects(contacts).indexOf(contact.otherObject)>=0)intended=true;
        if(!intended){driver.stopAndHold();throw "Fixture motion contacted the parked robot/tool";}
      }
    }
  }
  public function clamp(closed:Bool):Void {
    request=closed ? 3 : 4;tick();fixtureMotion("mill/mill/vise/jaw",closed ? viseClosedPositionMm : 0,0.05);request=0;tick();
  }
  function sideOf(profile:Dynamic):String {var pieces=Std.string(profile.caseReference).split("/");return pieces[pieces.length-1];}
  function prepare(selected:String):Void {
    for(side in ["a","b"])if(!driver.carrying(side)) {
      var grip:SimulatedElectricGripper=cast grips.get(side);grip.position(scene.jawTarget(side,side==selected ? 25 : 17));
    }
    for(_ in 0...500) {
      tick();var ready=true;
      for(side in ["a","b"])if(!driver.carrying(side)) {
        var grip:SimulatedElectricGripper=cast grips.get(side);
        if(Math.abs(scene.jawOpeningMm(side,grip.opening())-(side==selected ? 25 : 17))>0.01)ready=false;
      }
      var contacts=harness.simulation.robotContacts(runtime);
      for(contact in contacts)if(contact.active) {
        var intended=false;
        for(grip in grips)if(contact.otherKind==RobotContactOtherKind.Object && grip.jawContact(contact.linkIndex) && grip.grippedObjects(contacts).indexOf(contact.otherObject)>=0)intended=true;
        if(!intended){driver.stopAndHold();throw "Contact while preparing the electric grippers at park";}
      }
      if(ready)return;
    }
    driver.stopAndHold();throw "Electric jaw preparation timeout";
  }
  public function route(profile:Dynamic,withdrawal:Bool,slot:Int):Void {
    if(!withdrawal)prepare(sideOf(profile));
    driver.activeStock=cast stock.get(slot);
    if(withdrawal && Reflect.hasField(profile,"registration") && profile.registration!=null) {
      if(!registeredStarts.exists(profile.caseReference))throw "Vise extraction requires its registered grasp";
      registeredStarts.remove(profile.caseReference);extractRegistered(profile,slot);return;
    }
    var hasPlacement=Reflect.hasField(profile,"placing") && profile.placing!=null;
    var placing=!withdrawal && hasPlacement;
    var edges:Array<Dynamic> = cast (placing ? profile.placing.incoming : withdrawal && hasPlacement ? profile.placing.outgoing : withdrawal ? profile.outgoing : profile.incoming);
    var measuredRelease=withdrawal && stoppedSearches.exists(profile.caseReference);
    if(withdrawal && stoppedSearches.exists(profile.caseReference)) {
      var search:Dynamic=stoppedSearches.get(profile.caseReference);
      var samples:Array<Array<Float>> = cast search.samples,current=driver.joints();
      // A contact can stop anywhere on the certified search corridor. Reject a
      // measured posture outside its existing joint tracking allowance.
      var segment=-1,bestError=1e100;
      for(i in 1...samples.length) {
        var d=0.0,t=0.0;
        for(j in 0...6){var v=samples[i][j]-samples[i-1][j];d+=v*v;t+=(current[j]-samples[i-1][j])*v;}
        t=d==0 ? 0 : Math.max(0,Math.min(1,t/d));var fits=true,error=0.0;
        for(j in 0...6) {
          var delta=current[j]-(samples[i-1][j]+t*(samples[i][j]-samples[i-1][j]));
          if(Math.abs(delta)>0.00001)fits=false;error+=delta*delta;
        }
        if(fits && error<bestError){segment=i;bestError=error;}
      }
      if(segment<0 || search.limits.returnCorridorClear!=true ||
        search.limits.returnPolicy!="reverse executed search samples")
        throw "Measured locating stop is outside its certified reverse corridor";
      var returning:Array<Dynamic> = [],from=current;
      for(reverse in 0...segment) {
        var to=samples[segment-1-reverse];
        returning.push({kind:"jointEdge",from:from,to:to,contactPolicy:null,separating:true,stockOwner:"fixture",jawOpeningMm:25});
        from=to;
      }
      edges=returning.concat(edges);
      stoppedSearches.remove(profile.caseReference);
    }
    if(measuredRelease)releasedStockReturnChecks.push(TendingReleasedStockReturnCheck.run(scene,harness.simulation,cast stock.get(slot),driver.carriedObjects(),edges));
    executeEdges(profile,withdrawal,edges);
    if(placing)placeAtEntry(profile,slot);
  }
  function executeEdges(profile:Dynamic,withdrawal:Bool,edges:Array<Dynamic>,?pointSpeedLimitMmPerSecond:Float):Void {
    var pointLimit=pointSpeedLimitMmPerSecond==null ? 0.0 : (cast pointSpeedLimitMmPerSecond:Float);
    if(!Math.isFinite(pointLimit) || pointLimit<0)throw "Invalid carried point-speed limit";
    lastExecutionPointSpeed=0;
    var executor=new TendingElectricEdgeExecutor(edges,driver,0.00001,driver.configure);
    // Tick first admits the edge, then the shared world advances before the next feedback observation.
    for(iteration in 0...900000) {
      routeObservation={caseReference:profile.caseReference,withdrawal:withdrawal,
        edgeIndex:executor.edgeIndex(),edgeCount:edges.length};
      executor.tick(0.01);
      if(pointLimit>0) {
        var frame=harness.space.session.capture(),maximum=0.0;
        for(object in driver.carriedObjects()) {
          var body=frame.bodyState(harness.simulation.objectBody(object));
          var linear=Math.sqrt(body.linearVelocityX*body.linearVelocityX+body.linearVelocityY*body.linearVelocityY+body.linearVelocityZ*body.linearVelocityZ);
          var angular=Math.sqrt(body.angularVelocityX*body.angularVelocityX+body.angularVelocityY*body.angularVelocityY+body.angularVelocityZ*body.angularVelocityZ);
          maximum=Math.max(maximum,(linear+object.shape.boundingRadius()*angular)*1000);
        }
        frame.dispose();lastExecutionPointSpeed=Math.max(lastExecutionPointSpeed,maximum);
        if(maximum>pointLimit){driver.stopAndHold();throw "Guarded separation carried point-speed exceeded: "+maximum;}
      }
      if(iteration%100==0) {
        sys.io.File.saveContent(progressPath,haxe.Json.stringify({
          caseReference:profile.caseReference,withdrawal:withdrawal,edgeIndex:executor.edgeIndex(),edgeCount:edges.length,
          simulationSeconds:harness.simulation.simulationTime(),joints:driver.joints(),stopped:executor.stopped,failure:executor.failure}));
      }
      if(executor.stopped)throw executor.failure;
      if(executor.complete)return;
      tick();
    }
    throw "Electric route runner exhausted its tick budget";
  }
  function observedObjectFrame(object:SimObject):AssemblyFrame {
    var p=harness.simulation.objectPose(object);return {x:p.x,y:p.y,z:p.z,qx:p.qx,qy:p.qy,qz:p.qz,qw:p.qw};
  }
  function extractRegistered(profile:Dynamic,slot:Int):Void {
    var policy:Dynamic=profile.registration.extraction,object:SimObject=cast stock.get(slot);
    if(policy==null || !driver.carryingObject(object) || fixtureState.joint("mill/mill/vise/jaw")!=0 || fixtureState.joint("mill/door")!=doorStroke)
      throw "Registered extraction needs an open vise, open door and confirmed grasp";
    var seat=fixtureState.worldConnector("mill/mill/vise/body","seat"),load=fixtureState.worldConnector("mill/mill/vise/body","load");
    var x=AssemblyFrames.transformVector(seat,1,0,0),up=AssemblyFrames.transformVector(seat,0,0,1);
    var clearance:Float=policy.clearanceXmm;
    var delta:Array<Float> = [load.x-seat.x+x.x*clearance,load.y-seat.y+x.y*clearance,load.z-seat.z+x.z*clearance];
    var length=0.0;for(v in delta)length+=v*v;length=Math.sqrt(length);
    var samples=TendingGraspRegistration.escape(scene,harness.simulation,delta,policy.maximumStepMm),step=length/(samples.length-1);
    var slowCount=Std.int(Math.ceil(policy.slowSeparationDistanceMm/step));slowCount=Std.int(Math.max(1,Math.min(samples.length-1,slowCount)));
    var nominal:Array<Dynamic> = cast profile.outgoing,contacts:Array<String> = cast nominal[0].contactPolicy.expectedContacts;
    var slow=samples.slice(0,slowCount+1),workingProof=TendingGraspRegistration.checkPath(scene,harness.simulation,observedFixedObjects(),driver.carriedGrasps(),slow,object.handle.rawValue(),contacts);
    var guard={expectedContacts:contacts,approach:{maxTravelMm:slowCount*step+0.12,requireContactAtArrival:false},
      withdrawal:{maxTravelMm:slowCount*step+0.12},timeoutSeconds:policy.timeoutSeconds,
      forceLimitN:policy.forceLimitN,speedLimitMmPerSecond:policy.speedLimitMmPerSecond};
    var edges:Array<Dynamic> = [];
    for(i in 1...slow.length)edges.push({kind:"guardedJointEdge",from:slow[i-1],to:slow[i],contactPolicy:guard,separating:true,stockOwner:"tool",jawOpeningMm:17});
    driver.minimumMotionSeconds=1.875*step/policy.commandSpeedMmPerSecond;
    executeEdges(profile,true,edges,policy.totalPointSpeedLimitMmPerSecond);driver.minimumMotionSeconds=0;
    var peak=lastExecutionPointSpeed,relative=AssemblyFrames.compose(AssemblyFrames.inverse(wristFrame()),observedObjectFrame(object));
    var top=-1e100;
    for(id in fixtures.keys())if(StringTools.startsWith(id,"mill/mill/vise/")) {
      var fixture:SimObject=cast fixtures.get(id),frame=observedObjectFrame(fixture),vertices=fixture.shape.convexVertices();
      for(i in 0...Std.int(vertices.length/3)) {
        var p=AssemblyFrames.transformPoint(frame,vertices[i*3],vertices[i*3+1],vertices[i*3+2]);
        top=Math.max(top,p.x*up.x+p.y*up.y+p.z*up.z);
      }
    }
    if(top< -1e90)throw "Missing physical vise height for extraction";
    var rejoin=-1,vertices=object.shape.convexVertices();
    for(i in 0...nominal.length)if(rejoin<0) {
      var frame=AssemblyFrames.compose(TendingGraspRegistration.wristAt(scene,harness.simulation,cast nominal[i].to),relative),bottom=1e100;
      for(j in 0...Std.int(vertices.length/3)) {
        var p=AssemblyFrames.transformPoint(frame,vertices[j*3],vertices[j*3+1],vertices[j*3+2]);
        bottom=Math.min(bottom,p.x*up.x+p.y*up.y+p.z*up.z);
      }
      if(bottom>=top+policy.airRejoinClearanceMm*0.001)rejoin=i;
    }
    if(rejoin<0)throw "No reviewed withdrawal waypoint clears the observed vise";
    var path=[driver.joints()];for(i in slowCount+1...samples.length)path.push(samples[i]);
    path.push(cast nominal[rejoin].to);for(i in rejoin+1...nominal.length)path.push(cast nominal[i].to);
    var freeProof=TendingGraspRegistration.checkPath(scene,harness.simulation,observedFixedObjects(),driver.carriedGrasps(),path,-1,[]);
    extractionObservations.push({caseReference:profile.caseReference,deltaMm:delta,slowSteps:slowCount,policy:policy,
      guardedProof:workingProof,freeProof:freeProof,rejoinedOriginalOutgoingIndex:rejoin,peakGuardedPointSpeedMmPerSecond:peak,simulationOnly:true});
    edges=[];for(i in 1...path.length)edges.push({kind:"jointEdge",from:path[i-1],to:path[i],contactPolicy:null,separating:true,stockOwner:"tool",jawOpeningMm:17});
    executeEdges(profile,true,edges);
  }
  public function locate(profile:Dynamic,slot:Int,startIndex:Int=0):Void {
    var samples:Array<Array<Float>> = cast profile.locating.samples;
    if(startIndex<0 || startIndex>=samples.length-1)throw "Invalid seeded search entry";
    var seat=fixtureState.worldConnector("mill/mill/vise/body","seat"),axis=AssemblyFrames.transformVector(seat,1,0,0);
    contactSearch(profile,slot,profile.locating.limits,samples.slice(startIndex),axis,locatingObservations,samples);
  }
  /** Seeded tests and production share exactly the same final support-search action. */
  public function placeAtEntry(profile:Dynamic,slot:Int):Void {
    if(profile.placing==null)throw "Support placement requires its reviewed final approach";
    var object:SimObject=cast stock.get(slot),target=seatingTarget("outfeedTray",slot),axis=AssemblyFrames.transformVector(target,0,0,1);
    var at=harness.simulation.objectPose(object),vertices=object.shape.convexVertices(),clearance=1e100;
    var observed:AssemblyFrame={x:at.x,y:at.y,z:at.z,qx:at.qx,qy:at.qy,qz:at.qz,qw:at.qw};
    for(i in 0...Std.int(vertices.length/3)) {
      var world=AssemblyFrames.transformPoint(observed,vertices[i*3],vertices[i*3+1],vertices[i*3+2]);
      clearance=Math.min(clearance,((world.x-target.x*0.001)*axis.x+(world.y-target.y*0.001)*axis.y+(world.z-target.z*0.001)*axis.z)*1000);
    }
    if(vertices.length<12 || clearance<profile.placing.limits.minimumPresearchClearanceMm)
      throw "Measured stock lacks its required clearance before support search: "+clearance;
    contactSearch(profile,slot,profile.placing.limits,cast profile.placing.samples,axis,placingObservations,cast profile.placing.samples);
  }
  function contactSearch(profile:Dynamic,slot:Int,policy:Dynamic,samples:Array<Array<Float>>,axis:Dynamic,observations:Array<Dynamic>,returnSamples:Array<Array<Float>>):Void {
    var action=new TendingContactSearchAction(policy),object:SimObject=cast stock.get(slot);
    var expected:Array<String> = Reflect.hasField(policy,"expectedContacts") ? cast policy.expectedContacts : [policy.contact];
    if(!driver.carryingObject(object) || !TendingElectricRuntimeProfile.same(driver.joints(),samples[0],0.00001))
      throw "Contact search requires the confirmed grasp at its clearance endpoint";
    var start=wristFrame(),index=1,held=false,edgeStarted=false;
    var peakApproachClosing=0.0,peakHeldPoint=0.0,peakTotalPoint=0.0;
    driver.configure({kind:"guardedJointEdge",separating:false});
    // Quintic peak speed is 1.875 times average. Command rate and the
    // measured all-point bound are distinct: contact compliance can recoil.
    var commandSpeed:Float=policy.commandSpeedMmPerSecond;
    if(!Math.isFinite(commandSpeed) || commandSpeed<=0 || commandSpeed>policy.speedLimitMmPerSecond*0.25)
      throw "Contact search command lacks measured-speed margin";
    var stepTravel:Float=Reflect.hasField(policy,"sampleTravelMm") ? policy.sampleTravelMm : 0.05;
    if(!Math.isFinite(stepTravel) || stepTravel<=0)throw "Contact search requires a bounded sample distance";
    driver.minimumMotionSeconds=1.875*stepTravel/commandSpeed;
    var selectedHandle=harness.simulation.objectBody(object),selectedBody=selectedHandle.rawValue();
    var stockStart=harness.simulation.objectPose(object),radius=0.0,vertices=object.shape.convexVertices();
    for(i in 0...Std.int(vertices.length/3))radius=Math.max(radius,
      Math.sqrt(vertices[i*3]*vertices[i*3]+vertices[i*3+1]*vertices[i*3+1]+vertices[i*3+2]*vertices[i*3+2]));
    if(!(radius>0))throw "Contact search needs the physical stock hull's motion bound";
    for(iteration in 0...40000) {
      routeObservation={caseReference:profile.caseReference,phase:"contact-search",sampleIndex:index,holding:held};
      var observed=driver.feedback(),at=wristFrame();
      var dx=(at.x-start.x)*1000,dy=(at.y-start.y)*1000,dz=(at.z-start.z)*1000;
      var travel=-(dx*axis.x+dy*axis.y+dz*axis.z),transverse=Math.sqrt(Math.max(0,dx*dx+dy*dy+dz*dz-travel*travel));
      if(travel< -0.001 || transverse>0.12){driver.stopAndHold();throw "Contact search left its approved Cartesian corridor";}
      for(contact in harness.simulation.robotContacts(runtime))if(contact.active &&
        contact.otherKind==RobotContactOtherKind.Object && expected.indexOf(cast names.get(contact.otherObject))>=0) {
        driver.stopAndHold();throw "Robot/tool contacted a search fixture";
      }
      var selectedContacts:Array<Dynamic> = [];
      for(contact in harness.space.session.contacts())if(contact.active &&
        (contact.bodyA==selectedBody || contact.bodyB==selectedBody) &&
        expected.indexOf(cast bodyNames.get(contact.bodyA==selectedBody ? contact.bodyB : contact.bodyA))>=0) {
        if(!contact.wrenchValid){driver.stopAndHold();throw "Contact search has no measured wrench";}
        selectedContacts.push(contact);
      }
      var load=TendingContactSearchAction.contactLoad(selectedContacts,selectedBody,[axis.x,axis.y,axis.z]);
      var force=load.axialN,totalForce=load.resultantN,maximumContactForce=load.maximumContactN,sumContactMagnitudes=load.summedMagnitudesN;
      var selectedStopContact=selectedContacts.length>0;
      if(Math.max(totalForce,maximumContactForce)>policy.forceLimitN){
        observations.push({caseReference:profile.caseReference,side:sideOf(profile),slot:slot,time:harness.simulation.simulationTime(),
          forceN:force,totalForceN:totalForce,maximumContactForceN:maximumContactForce,summedContactMagnitudesN:sumContactMagnitudes,
          policy:policy,failure:"contact search resultant or individual force exceeded",simulationOnly:true});
        driver.stopAndHold();throw "Contact search resultant or individual force exceeded";
      }
      var reportedExpected=false;for(name in observed.contacts)if(expected.indexOf(name)>=0)reportedExpected=true;
      if(reportedExpected && !selectedStopContact) {
        driver.stopAndHold();throw "Search contact belongs to another payload or the tool";
      }
      var rest=true;for(j in scene.armJoints)if(Math.abs(harness.simulation.jointSample(0,j).get_velocity())>0.00005)rest=false;
      if(observed.speedMmPerSecond==null)throw "Contact search requires measured speed feedback";
      var frame=harness.space.session.capture(),body=frame.bodyState(selectedHandle);frame.dispose();
      var linear=Math.sqrt(body.linearVelocityX*body.linearVelocityX+body.linearVelocityY*body.linearVelocityY+body.linearVelocityZ*body.linearVelocityZ);
      var angular=Math.sqrt(body.angularVelocityX*body.angularVelocityX+body.angularVelocityY*body.angularVelocityY+body.angularVelocityZ*body.angularVelocityZ);
      // Bound every stock point, including rotation about a stationary flange.
      var pointSpeed=(linear+radius*angular)*1000;
      var closingSpeed=TendingContactSearchAction.closingPointSpeed(
        body.linearVelocityX*axis.x+body.linearVelocityY*axis.y+body.linearVelocityZ*axis.z,angular,radius)*1000;
      var speed=Math.max(closingSpeed,cast observed.speedMmPerSecond);
      peakTotalPoint=Math.max(peakTotalPoint,pointSpeed);
      if(held)peakHeldPoint=Math.max(peakHeldPoint,pointSpeed);else peakApproachClosing=Math.max(peakApproachClosing,speed);
      var stockTravel=-((body.pose.x-stockStart.x)*axis.x+(body.pose.y-stockStart.y)*axis.y+(body.pose.z-stockStart.z)*axis.z)*1000;
      if(Math.max(pointSpeed,cast observed.speedMmPerSecond)>0.001)rest=false;
      var gap:Null<Float> = null;
      if(Reflect.hasField(policy,"restGapLimitMm") && (held || force>=policy.contactThresholdN)) {
        if(expected.length!=1 || !fixtures.exists(expected[0]))throw "Touch-and-rest locating needs its physical stop";
        gap=TendingGraspRegistration.observedGapMm(harness.simulation,object,cast fixtures.get(expected[0]));
      }
      action.observe(0.01,Math.max(0,Math.max(travel,stockTravel)),observed.contacts,force,observed.penetrationMm,speed,rest,pointSpeed,gap);
      if(action.stopped){
        observations.push({caseReference:profile.caseReference,side:sideOf(profile),slot:slot,time:harness.simulation.simulationTime(),travelMm:travel,
          forceN:force,totalForceN:totalForce,maximumContactForceN:maximumContactForce,summedContactMagnitudesN:sumContactMagnitudes,penetrationMm:observed.penetrationMm,
          stockPointSpeedBoundMmPerSecond:pointSpeed,closingPointSpeedBoundMmPerSecond:closingSpeed,flangeSpeedMmPerSecond:observed.speedMmPerSecond,
          measuredClosingSpeedLimitMmPerSecond:policy.speedLimitMmPerSecond,
          totalPointSpeedLimitMmPerSecond:policy.totalPointSpeedLimitMmPerSecond,commandSpeedMmPerSecond:commandSpeed,
          observedStopGapMm:gap,failure:action.failure,simulationOnly:true});
        driver.stopAndHold();throw action.failure;
      }
      if(action.holding && !held){driver.holdAtContact();held=true;}
      if(action.complete) {
        var execution=runtime.snapshot();
        if(execution.trajectoryActive || execution.trajectoryQueueDepth!=0 || execution.safety!=RobotKitRuntimeConstants.RK_SAFETY_READY) {
          driver.stopAndHold();throw "Contact search stop did not discard its unexecuted trajectory";
        }
        driver.minimumMotionSeconds=0;stoppedSearches.set(profile.caseReference,{samples:returnSamples,limits:policy});
        observations.push({caseReference:profile.caseReference,side:sideOf(profile),slot:slot,time:harness.simulation.simulationTime(),travelMm:travel,
          forceN:force,totalForceN:totalForce,maximumContactForceN:maximumContactForce,summedContactMagnitudesN:sumContactMagnitudes,penetrationMm:observed.penetrationMm,stockPointSpeedBoundMmPerSecond:pointSpeed,
          observedStopGapMm:gap,stableSamples:policy.stableSamples,policy:policy,
          peakApproachClosingSpeedMmPerSecond:peakApproachClosing,peakHeldPointSpeedMmPerSecond:peakHeldPoint,
          peakTotalPointSpeedMmPerSecond:peakTotalPoint,
          joints:driver.joints(),pose:harness.simulation.objectPose(object),simulationOnly:true});return;
      }
      if(!held) {
        if(edgeStarted && observed.arrived){index++;edgeStarted=false;}
        if(index>=samples.length){action.finishSearch();driver.stopAndHold();throw action.failure;}
        if(!edgeStarted){driver.start(samples[index]);edgeStarted=true;}
      }
      if(iteration%100==0)sys.io.File.saveContent(progressPath,haxe.Json.stringify({phase:"contact-search",caseReference:profile.caseReference,side:sideOf(profile),slot:slot,
        simulationSeconds:harness.simulation.simulationTime(),travelMm:travel,forceN:force,holding:held}));
      tick();
    }
    driver.stopAndHold();throw "Contact search exhausted its bounded tick budget";
  }
  function observedFixedObjects():Array<{name:String,object:SimObject}> {
    var result:Array<{name:String,object:SimObject}> = [];
    for(id in fixtures.keys())result.push({name:id,object:cast fixtures.get(id)});
    for(object in stock)if(!driver.carryingObject(object))result.push({name:cast names.get(object.handle.rawValue()),object:object});
    return result;
  }
  public function registrationQueryControls():Dynamic
    return TendingGraspRegistration.controls(scene,harness.simulation,observedFixedObjects(),driver.carriedGrasps());
  function registerGrasp(profile:Dynamic,slot:Int):Void {
    var side=sideOf(profile),policy:Dynamic=profile.registration,object:SimObject=cast stock.get(slot),grip:SimulatedElectricGripper=cast grips.get(side);
    if(policy==null || driver.carryingObject(object) || Math.abs(fixtureState.joint("mill/mill/vise/jaw")-viseClosedPositionMm)>1e-9 ||
      Math.abs(fixtureState.joint("mill/door")-doorStroke)>1e-9 || scene.jawOpeningMm(side,grip.opening())<24.9)
      throw "Observed vise grasp registration needs a clamped part, open door and open jaws";
    var observed=harness.simulation.objectPose(object),target=seatingTarget("vise",slot);
    var dx=observed.x*1000-target.x,dy=observed.y*1000-target.y,dz=observed.z*1000-target.z;
    var angle=TendingSeatPolicy.orientationError([observed.qx,observed.qy,observed.qz,observed.qw],[target.qx,target.qy,target.qz,target.qw],TendingSeatOrientation.FullPose);
    if(Math.sqrt(dx*dx+dy*dy+dz*dz)>policy.positionToleranceMm || angle>policy.orientationToleranceRad)
      throw "Observed vise stock differs from its registered full-pose admission";
    var alignment=TendingGraspRegistration.plan(scene,harness.simulation,object,side,observedFixedObjects(),driver.carriedGrasps(),policy.maximumPointMotionMm);
    executeEdges(profile,false,[{kind:"jointEdge",from:alignment.from,to:alignment.to,contactPolicy:null,separating:false,stockOwner:"fixture",jawOpeningMm:25}]);
    registrationObservations.push({caseReference:profile.caseReference,phase:"open-jaw grasp registration",alignment:alignment});
    registeredStarts.set(profile.caseReference,true);
  }
  public function grasp(side:String,slot:Int,?registrationProfile:Dynamic):Void {
    if(registrationProfile!=null)registerGrasp(registrationProfile,slot);
    var grip:SimulatedElectricGripper=cast grips.get(side),object:SimObject=cast stock.get(slot);grip.command(true);var stable=0;
    for(_ in 0...1000) {
      tick();var contacts=harness.simulation.robotContacts(runtime);checkJawMotion(contacts,grip,object);
      if(_%100==0)sys.io.File.saveContent(progressPath,haxe.Json.stringify({phase:"grasp",side:side,slot:slot,
        simulationSeconds:harness.simulation.simulationTime(),commandComplete:grip.commandComplete(),
        openingMm:scene.jawOpeningMm(side,grip.opening()),objectPose:harness.simulation.objectPose(object)}));
      // Transient first contact is not a completed clamp: finish the bounded force ramp first.
      if(grip.commandComplete() && grip.grippedObjects(contacts).indexOf(object.handle.rawValue())>=0 && Math.abs(scene.jawOpeningMm(side,grip.opening())-17)<=0.2)stable++;else stable=0;
      if(stable>=10){
        graspObservations.push({side:side,slot:slot,time:harness.simulation.simulationTime(),
          commandComplete:grip.commandComplete(),openingMm:scene.jawOpeningMm(side,grip.opening()),wristFrame:wristFrame(),pose:harness.simulation.objectPose(object)});
        driver.expectPayload(side,object,grip,cast stockMassProperties.get(slot));return;
      }
    }
    driver.stopAndHold();throw "Bilateral grasp confirmation timeout";
  }
  public function release(side:String,slot:Int):Void {
    var grip:SimulatedElectricGripper=cast grips.get(side),object:SimObject=cast stock.get(slot);releasing=side;grip.command(false);
    for(_ in 0...1000) {
      tick();var contacts=harness.simulation.robotContacts(runtime);checkJawMotion(contacts,grip,object);
      if(grip.released(contacts,object.handle.rawValue())){driver.releasedPayload(side);releasing=null;return;}
    }
    driver.stopAndHold();throw "Release confirmation timeout";
  }
  function at(id:String,target:AssemblyFrame):Bool {
    var measured=harness.simulation.objectPose(cast fixtures.get(id));
    return Math.abs(measured.x*1000-target.x)+Math.abs(measured.y*1000-target.y)+Math.abs(measured.z*1000-target.z)<0.5;
  }
  function updatePlc():Void {
    var contacts=harness.simulation.robotContacts(runtime),held:Array<Int> = [];
    for(grip in grips)for(id in grip.grippedObjects(contacts))held.push(id);
    var present=false;for(object in stock)if(object.overlapsVolume(stockField,[0.025,0.025,0.0125]))present=true;
    var output=plc.update(new cnckit.controller.RobotInterfaceInput(request==1,request==2,request==3,request==4,request==5,
      zone.clear(held),false,at("mill/doorSlider",doorOpenFrame),at("mill/doorSlider",doorClosedFrame),
      at("mill/mill/vise/movingJaw",viseClosedFrame),at("mill/mill/vise/movingJaw",viseOpenFrame),present,!running,!running,completePulse,false),0.01);
    plcAlarm=output.alarm;lastOutput=output;
    if(plcAlarm!=0){driver.stopAndHold();throw 'Physical CNC PLC interlock alarm $plcAlarm';}
  }
  public function startCycle():Void {
    request=5;tick();var output:cnckit.controller.RobotInterfaceOutputs=cast lastOutput;
    if(!output.startCycle)throw "Physical PLC refused cycle start";
    running=true;request=0;tick();
  }
  public function cycleReady():Void {
    for(_ in 0...100)tick();running=false;completePulse=true;tick();completePulse=false;tick();
    var output:cnckit.controller.RobotInterfaceOutputs=cast lastOutput;
    if(!output.cycleComplete)throw "Physical PLC did not acknowledge cycle completion";
  }
  public function seatingTarget(station:String,slot:Int,datum:String="seat"):AssemblyFrame {
    var occurrence=station=="vise" ? "mill/mill/vise/body" : station+"/seat-r"+Std.int(slot/3)+"-c"+(slot%3)+"-x-1-y-1";
    var target=fixtureState.worldConnector(occurrence,"seat");
    if(station=="vise" && datum=="supported-load") {
      var loading=fixtureState.worldConnector(occurrence,"load"),up=AssemblyFrames.transformVector(target,0,0,1);
      var height=(loading.x-target.x)*up.x+(loading.y-target.y)*up.y+(loading.z-target.z)*up.z;
      target={x:loading.x-height*up.x,y:loading.y-height*up.y,z:loading.z-height*up.z,
        qx:target.qx,qy:target.qy,qz:target.qz,qw:target.qw};
    }
    return target;
  }
  public function checkPlaced(station:String,slot:Int,?policy:Dynamic):Void {
    var object:SimObject=cast stock.get(slot),datum=policy==null ? "seat" : Std.string(policy.datum);
    var target=seatingTarget(station,slot,datum);
    var orientation=TendingSeatOrientation.FullPose;
    if(policy!=null && Reflect.hasField(policy,"orientationConstraint"))orientation=TendingSeatPolicy.parseOrientation(policy.orientationConstraint);
    var requireSupport=true;
    if(policy!=null && Reflect.hasField(policy,"requireSupportLoad"))requireSupport=policy.requireSupportLoad==true;
    if(driver.carryingObject(object))throw "Support confirmation requires a confirmed release";
    var expected:Array<String> = [];
    if(station=="vise")expected=["mill/mill/vise/parallel0","mill/mill/vise/parallel1"];
    else for(x in [-1,1])for(y in [-1,1])expected.push(station+"/seat-r"+Std.int(slot/3)+"-c"+(slot%3)+"-x"+x+"-y"+y);
    if(policy!=null)expected=cast policy.supportContacts;
    var positionTolerance=policy==null ? 1.0 : (cast policy.positionToleranceMm:Float);
    var orientationTolerance=policy==null ? 0.001 : (cast policy.orientationToleranceRad:Float);
    var stableRequired=policy==null ? 10 : Std.int(policy.stableSamples);
    var ticks=policy==null ? 200 : Std.int(policy.timeoutSeconds/0.01);
    var minimumForce=policy==null ? 0.01 : (cast policy.minimumSupportForceN:Float);
    if(expected.length==0 || stableRequired<1 || ticks<stableRequired ||
      !Math.isFinite(positionTolerance) || positionTolerance<=0 || !Math.isFinite(orientationTolerance) || orientationTolerance<=0 ||
      !Math.isFinite(minimumForce) || minimumForce<=0)throw "Invalid post-release support policy";
    var allowed=expected.copy();
    if(station=="vise")for(member in ["fixedJaw","movingJaw","endStop"])allowed.push("mill/mill/vise/"+member);
    var normal=AssemblyFrames.transformPoint(target,0,0,1);
    var n=[normal.x-target.x,normal.y-target.y,normal.z-target.z];
    var bodyHandle=harness.simulation.objectBody(object),body=bodyHandle.rawValue(),stable=0;
    for(_ in 0...ticks) {
      tick();var frame=harness.space.session.capture(),state=frame.bodyState(bodyHandle);frame.dispose();
      var measured=state.pose;
      for(contact in harness.simulation.robotContacts(runtime))
        if(contact.active && contact.otherKind==RobotContactOtherKind.Object && contact.otherObject==object.handle.rawValue()) {
          driver.stopAndHold();throw "Seating confirmation found a workpiece still touching the robot/tool";
        }
      var dx=measured.x*1000-target.x,dy=measured.y*1000-target.y,dz=measured.z*1000-target.z;
      var error=Math.sqrt(dx*dx+dy*dy+dz*dz);
      var angle=TendingSeatPolicy.orientationError([measured.qx,measured.qy,measured.qz,measured.qw],
        [target.qx,target.qy,target.qz,target.qw],orientation);
      if(error>positionTolerance){driver.stopAndHold();throw 'Released/present stock is outside its fixture datum: $error mm';}
      var bearing:Array<String> = [];
      for(contact in harness.space.session.contacts())if(contact.active && (contact.bodyA==body || contact.bodyB==body)) {
        var name=bodyNames.get(contact.bodyA==body ? contact.bodyB : contact.bodyA);
        if(name==null)continue;
        if(allowed.indexOf(name)<0){driver.stopAndHold();throw "Unexpected workpiece/fixture seating contact: "+name;}
        var load=0.0;for(i in 0...3)load+=contact.force[i]*n[i];
        if(contact.wrenchValid && Math.abs(load)>minimumForce && expected.indexOf(name)>=0 && bearing.indexOf(name)<0)bearing.push(name);
      }
      var speed=Math.sqrt(state.linearVelocityX*state.linearVelocityX+state.linearVelocityY*state.linearVelocityY+state.linearVelocityZ*state.linearVelocityZ);
      var angularSpeed=Math.sqrt(state.angularVelocityX*state.angularVelocityX+state.angularVelocityY*state.angularVelocityY+state.angularVelocityZ*state.angularVelocityZ);
      var atRest=speed<=0.0001 && angularSpeed<=0.001;
      if(_%10==0)seatingTrace.push({station:station,slot:slot,orientationConstraint:orientation,time:harness.simulation.simulationTime(),
        supportContacts:bearing,positionErrorMm:error,orientationErrorRad:angle,
        speedMmPerSecond:speed*1000,angularSpeedRadPerSecond:angularSpeed,stableSamples:stable,pose:measured});
      if((!requireSupport || bearing.length==expected.length) && angle<=orientationTolerance && atRest)stable++;else stable=0;
      if(stable>=stableRequired) {
        seatObservations.push({station:station,slot:slot,datum:datum,orientationConstraint:orientation,supportLoadRequired:requireSupport,time:harness.simulation.simulationTime(),
          supportContacts:bearing,positionErrorMm:error,orientationErrorRad:angle,speedMmPerSecond:speed*1000,
          angularSpeedRadPerSecond:angularSpeed,stableSamples:stable,pose:measured});
        return;
      }
    }
    driver.stopAndHold();throw "Released stock did not satisfy its support, pose or rest constraints";
  }
  function checkJawMotion(contacts:Array<RobotContact>,active:SimulatedElectricGripper,object:SimObject):Void {
    for(contact in contacts)if(contact.active) {
      var intended=contact.otherKind==RobotContactOtherKind.Object && contact.otherObject==object.handle.rawValue() && active.jawContact(contact.linkIndex);
      for(grip in grips)if(contact.otherKind==RobotContactOtherKind.Object && grip.jawContact(contact.linkIndex) && grip.grippedObjects(contacts).indexOf(contact.otherObject)>=0)intended=true;
      if(!intended){driver.stopAndHold();throw "Unexpected contact during electric jaw motion";}
    }
  }
  /** Consume the latched controlled hold and measure the stop; never retry the failed action. */
  public function stopAndObserve():Dynamic {
    driver.stopAndHold();running=false;
    var stable=0,failure:Null<String> = null;
    try {
      for(_ in 0...300) {
        harness.step();var rest=true;
        for(j in scene.armJoints)if(Math.abs(harness.simulation.jointSample(0,j).get_velocity())>0.0001)rest=false;
        if(rest)stable++;else stable=0;if(stable>=10)break;
      }
    } catch(error:Dynamic)failure=Std.string(error);
    var contacts=harness.simulation.robotContacts(runtime),held:Array<Int> = [];
    for(grip in grips)for(object in grip.grippedObjects(contacts))held.push(object);
    return {latched:true,armAtRest:stable>=10,stopFailure:failure,grippedObjects:held,automaticRetry:false,virtualMachiningStopped:true};
  }
  public function dispose():Void {for(grip in grips)grip.dispose();zone.dispose();harness.dispose();driver.dispose();}
}
