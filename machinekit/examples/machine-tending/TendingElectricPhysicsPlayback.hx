import TendingSeatPolicy.TendingSeatOrientation;
import materia.project.SceneArtifact;
import robotkit.runtime.*;
import haxe.Int64;

/** Headless electric playback in the same SimKit/RobotKit world used by Materia. */
class TendingElectricPhysicsPlayback {
  static function digest(bytes:haxe.io.Bytes):String {
    var hash=haxe.crypto.Sha256.make(bytes),out=new StringBuf();
    for(i in 0...hash.length)out.add(StringTools.hex(hash.get(i),2).toLowerCase());return out.toString();
  }
  static function fileDigest(path:String):String {
    var process=sys.io.Process.run("sha256sum",[path]);var output=process.readStdout(),status=process.exitCode();process.close();
    if(status!=0 || output.length<64)throw "Cannot hash simulation input";return output.substr(0,64);
  }
  public static function main():Void {
    Sys.println("Electric physics playback: locating inputs");
    var repo=Sys.getCwd();
    while(!sys.FileSystem.exists(repo+"/machinekit/examples/machine-tending")) {
      var parent=haxe.io.Path.directory(StringTools.endsWith(repo,"/") ? repo.substr(0,repo.length-1) : repo);
      if(parent==repo || parent.length==0)throw "Run playback within the Materia checkout";repo=parent;
    }
    Sys.setCwd(repo);
    var clearanceFirst=Sys.args().indexOf("--clearance-first")>=0;
    var root="machinekit/examples/machine-tending/build/",review:Dynamic=haxe.Json.parse(sys.io.File.getContent(root+(clearanceFirst ? "electric-cycle-review-clearance-first.json" : "electric-cycle-review.json")));
    var profileRows:Array<Dynamic> = cast review.runtimeProfiles;
    if(review.modelSimulationReady!=true || profileRows==null || profileRows.length!=28)throw "Playback requires the passing full electric cycle review";
    Sys.println("Review loaded; reading geometry");
    var armBytes=sys.io.File.getBytes(root+"electric-simulation-inputs/arm.mtrg"),toolBytes=sys.io.File.getBytes(root+"electric-simulation-inputs/tool.mtrg");
    Sys.println("Geometry loaded; checking hashes");
    if(fileDigest(root+"electric-simulation-inputs/arm.mtrg")!=review.provenance.armSha256 || fileDigest(root+"electric-simulation-inputs/tool.mtrg")!=review.provenance.toolSha256)throw "Playback geometry differs from reviewed routes";
    var fixtureBytes=sys.io.File.getBytes(root+"electric-simulation-inputs/fixtures.mtrg");
    if(fileDigest(root+"electric-simulation-inputs/fixtures.mtrg")!=review.provenance.fixtureSha256)throw "Playback fixtures differ from reviewed geometry";
    var sources:Array<Dynamic> = cast review.sources;
    for(source in sources){Sys.println("Checking review source: "+source.path);if(fileDigest(source.path)!=source.sha256)throw "Reviewed process/route source changed";}
    // Read only placement metadata in Python; avoid materializing the multi-megabyte collision witnesses twice in the VM.
    var reader=sys.io.Process.run("python3",["-c",
      "import json,sys; r=json.load(open(sys.argv[1])); p=json.load(open(r['sourceReport'])); print(json.dumps({'path':r['sourceReport'],'sha':r['sourceReportSha256'],'armPlacement':p['armPlacement'],'millLoadPosition':p['millLoadPosition']}))",
      Std.string(sources[0].path)]);
    var proposalText=reader.readStdout(),readStatus=reader.exitCode();reader.close();
    if(readStatus!=0)throw "Cannot read reviewed placement metadata";
    var proposal:Dynamic=haxe.Json.parse(proposalText);
    if(fileDigest(proposal.path)!=proposal.sha)throw "Placement proposal changed";
    Sys.println("Input hashes verified; building the physical electric scene");Sys.stdout().flush();
    var scene=new TendingElectricSimScene(SceneArtifact.decode(armBytes),SceneArtifact.decode(toolBytes),review.provenance.toolSha256);
    var backend=Sys.args().indexOf("--deterministic")>=0 ? SimulationSpace.DETERMINISTIC : SimulationSpace.MUJOCO;
    var p:Dynamic=proposal.armPlacement;
    var placement:materia.assembly.AssemblyRecord.AssemblyFrame={x:p.x,y:p.y,z:p.z,qx:p.qx,qy:p.qy,qz:p.qz,qw:p.qw};
    var smoke=Sys.args().indexOf("--withdraw-smoke")>=0;
    var placementSmoke=Sys.args().indexOf("--placement-smoke")>=0;
    var yawClampCheck=Sys.args().indexOf("--seat-yaw-clamp-check")>=0;
    var settlingCheck=Sys.args().indexOf("--seat-settling-check")>=0 || yawClampCheck;
    var locatingOffsetCheck=Sys.args().indexOf("--locating-offset-check")>=0;
    var pickupSide:Null<String> =null;
    for(arg in Sys.args())if(StringTools.startsWith(arg,"--vise-pickup-smoke="))pickupSide=arg.substr("--vise-pickup-smoke=".length);
    if(pickupSide!=null && (!clearanceFirst || ["a","b"].indexOf(pickupSide)<0))throw "Vise pickup check requires a clearance-first gripper side";
    var traySide:Null<String> =null;
    for(arg in Sys.args())if(StringTools.startsWith(arg,"--tray-placement-smoke="))traySide=arg.substr("--tray-placement-smoke=".length);
    if(traySide!=null && (!clearanceFirst || ["a","b"].indexOf(traySide)<0))throw "Tray placement check requires a clearance-first gripper side";
    var locatingSide:Null<String> =null;
    for(arg in Sys.args())if(StringTools.startsWith(arg,"--locating-smoke="))locatingSide=arg.substr("--locating-smoke=".length);
    if(locatingOffsetCheck && locatingSide!="b")throw "The measured-grasp regression requires locating gripper B";
    if(locatingSide!=null && (!clearanceFirst || ["a","b"].indexOf(locatingSide)<0))throw "Locating check requires a clearance-first gripper side";
    var mode=smoke ? "withdrawal-smoke" : placementSmoke ? "placement-smoke" : settlingCheck ? "seat-settling-check" : "four-part-cycle";
    var reportFile=smoke ? "electric-physics-withdrawal-smoke.json" : placementSmoke ? "electric-physics-placement-smoke.json" : settlingCheck ? "electric-physics-seat-settling.json" : "electric-physics-playback.json";
    if(yawClampCheck){mode="seat-yaw-clamp-check";reportFile="electric-physics-seat-yaw-clamp.json";}
    if(locatingSide!=null){mode="seeded-locating-smoke/"+locatingSide;reportFile="electric-physics-locating-smoke-"+locatingSide+".json";}
    if(pickupSide!=null){mode="seeded-vise-pickup-with-companion/"+pickupSide;reportFile="electric-physics-vise-pickup-"+pickupSide+".json";}
    if(traySide!=null){mode="seeded-tray-placement/"+traySide;reportFile="electric-physics-tray-placement-"+traySide+".json";}
    if(locatingOffsetCheck){mode="seeded-locating-grasp-error/b";reportFile="electric-physics-locating-offset-b.json";}
    if(clearanceFirst)reportFile=StringTools.replace(reportFile,".json","-clearance-first.json");
    var seed:Array<Float> = cast review.park;
    if(smoke) {
      var selected=[for(profile in profileRows)if(profile.caseReference=="infeedTray-0-0/a")profile];
      if(selected.length!=1)throw "Withdrawal smoke needs its reviewed case";
      var edges:Array<Dynamic> = cast selected[0].incoming;seed=cast edges[edges.length-1].to;
    }
    if(locatingSide!=null) {
      var match=[for(profile in profileRows)if(profile.caseReference=="vise-load/"+locatingSide)profile];
      if(match.length!=1 || match[0].locating==null)throw "Locating check requires a reviewed search corridor";
      var samples:Array<Array<Float>> = cast match[0].locating.samples;
      if(locatingOffsetCheck && samples.length!=14)throw "Offset regression requires the reviewed 0.05 mm search samples";
      seed=samples[locatingOffsetCheck ? 9 : 0];
    }
    if(traySide!=null) {
      var reference="outfeedTray-0-"+(traySide=="a" ? "0" : "1")+"/"+traySide;
      var match=[for(profile in profileRows)if(profile.caseReference==reference)profile];
      if(match.length!=1 || match[0].placing==null)throw "Tray check requires a reviewed support search";
      var samples:Array<Array<Float>> = cast match[0].placing.samples;seed=samples[0];
    }
    if(pickupSide!=null) {
      var match=[for(profile in profileRows)if(profile.caseReference=="vise-seat/"+pickupSide)profile];
      if(match.length!=1 || match[0].registration==null)throw "Vise pickup needs reviewed grasp registration";
      seed=cast review.park;
    }
    var cell=new TendingElectricSimCell(scene,SceneArtifact.decode(fixtureBytes),seed,placement,cast proposal.millLoadPosition,backend,smoke ? "a" : pickupSide!=null ? (pickupSide=="a" ? "b" : "a") : traySide!=null ? traySide : locatingSide,locatingSide!=null);
    if(smoke)cell.progressPath=root+"electric-physics-withdrawal-progress.json";
    if(placementSmoke)cell.progressPath=root+"electric-physics-placement-progress.json";
    var progressOverride=Sys.getEnv("MATERIA_TENDING_PROGRESS_PATH");
    if(clearanceFirst)cell.progressPath=StringTools.replace(cell.progressPath,".json","-clearance-first.json");
    if(progressOverride!=null)cell.progressPath=progressOverride;
    var events:Array<Dynamic> = [],passed=false,failure:Null<String> = null,cuts=0,delivered=0;
    var profiles:Map<String,Dynamic> = [];var all:Array<Dynamic> = cast review.runtimeProfiles;
    for(profile in all)profiles.set(profile.caseReference,profile);
    var held:Map<String,Int> = [],part:Null<Int> = null,clampingPolicy:Dynamic=null;
    var activeOperation:Null<String> = null;var failureStack:Null<String> = null;
    function side(tool:String):String {
      if(tool==machinekit.robotics.DualArmGripperTool.TOOL_CHANNEL_A)return "a";
      if(tool==machinekit.robotics.DualArmGripperTool.TOOL_CHANNEL_B)return "b";
      throw "Unknown electric runtime tool channel";
    }
    function route(station:String,side:String,withdrawal:Bool,slot:Int):Void {
      var profile:Dynamic=profiles.get(station+"/"+side);
      if(profile==null)throw "Missing runtime route";
      cell.route(profile,withdrawal,slot);
    }
    try {
      // The imported absolute pose must settle under the active controller before mission admission.
      // This holds the original boot target; it never retries a failed route or changes its tolerance.
      var stable=0;
      for(_ in 0...200) {
        cell.tick();
        if(cell.harness.simulation.robotContacts(cell.runtime).filter(contact -> contact.active).length>0)throw "Contact during controller startup";
        if(TendingElectricRuntimeProfile.same(cell.driver.joints(),seed,0.00001))stable++;else stable=0;
        if(stable>=10)break;
      }
      if(stable<10)throw "Arm controller cannot settle at the reviewed start within 0.00001 rad";
      if(settlingCheck) {
        Sys.println("Physics seat check: free stock settling on the actual vise parallels");Sys.stdout().flush();
        var selected:Dynamic=profiles.get("vise-load/a");
        var perturbation:materia.assembly.AssemblyRecord.AssemblyFrame=yawClampCheck
          ? {x:0.1,y:0.0,z:0.5,qx:0.0,qy:0.0,qz:Math.sin(0.00125),qw:Math.cos(0.00125)}
          : {x:0.0,y:0.0,z:0.5,qx:Math.sin(0.0005),qy:0.0,qz:0.0,qw:Math.cos(0.0005)};
        var target=materia.assembly.AssemblyFrames.compose(cell.seatingTarget("vise",0,"supported-load"),perturbation);
        events.push({operation:"seed tilted free stock above vise",pose:target});
        cell.harness.stop();var object:nativekit.sim.SimObject=cast cell.stock.get(0);
        object.teleport(TendingElectricSimCell.pose(target));
        cell.checkPlaced("vise",0,selected.seating);
        events.push({operation:"free stock settling on vise supports",time:cell.harness.simulation.simulationTime()});
        if(yawClampCheck) {
          var freePose=cell.harness.simulation.objectPose(object),seat=cell.seatingTarget("vise",0,"supported-load");
          var freeAngle=TendingSeatPolicy.orientationError([freePose.qx,freePose.qy,freePose.qz,freePose.qw],
            [seat.qx,seat.qy,seat.qz,seat.qw],TendingSeatOrientation.FullPose);
          if(freeAngle<=0.002)throw "Yaw regression did not exercise an unaligned, level blank";
          events.push({operation:"supported stock still requires jaw alignment",orientationErrorRad:freeAngle});
          part=0;cell.door(true);cell.clamp(true);cell.checkPlaced("vise",0,selected.clamping);
          events.push({operation:"closed jaws establish the unchanged full-pose tolerance",time:cell.harness.simulation.simulationTime()});
          events.push({operation:"measured-stock return query positive/negative controls",
            checks:TendingReleasedStockReturnCheck.controls(scene,cell.harness.simulation,object)});
        }
        passed=true;
      } else if(pickupSide!=null) {
        var slot=pickupSide=="a" ? 0 : 1,profile:Dynamic=profiles.get("vise-seat/"+pickupSide);
        var setup=cell.seatingTarget("vise",slot),object:nativekit.sim.SimObject=cast cell.stock.get(slot);
        setup.x+=0.002;setup.z+=0.5;
        cell.harness.stop();object.teleport(TendingElectricSimCell.pose(setup));
        events.push({operation:"seed free stock offset from nominal, then physically settle and clamp; earlier transport excluded",side:pickupSide,pose:setup});
        cell.checkPlaced("vise",slot);part=slot;cell.door(true);cell.clamp(true);
        var load:Dynamic=profiles.get("vise-load/"+pickupSide);cell.checkPlaced("vise",slot,load.clamping);cell.door(false);
        var companionSide=pickupSide=="a" ? "b" : "a",companionSlot=pickupSide=="a" ? 1 : 2;
        var companionStation="infeedTray-0-"+companionSlot;
        cell.checkPlaced("infeedTray",companionSlot);route(companionStation,companionSide,false,companionSlot);
        cell.grasp(companionSide,companionSlot);held.set(companionSide,companionSlot);route(companionStation,companionSide,true,companionSlot);
        events.push({operation:"physically collected companion blank through its reviewed tray route",side:companionSide,slot:companionSlot});
        cell.door(true);route("vise-seat",pickupSide,false,slot);
        events.push({operation:"observed grasp collision positive/negative controls",checks:cell.registrationQueryControls()});
        cell.grasp(pickupSide,slot,profile);held.set(pickupSide,slot);cell.clamp(false);part=null;
        cell.route(profile,true,slot);passed=true;
      } else if(traySide!=null) {
        var slot=traySide=="a" ? 0 : 1,reference="outfeedTray-0-"+slot+"/"+traySide,profile:Dynamic=profiles.get(reference);
        var target=cell.seatingTarget("outfeedTray",slot),rx=-Math.sin(0.00183/2),ry=0.000009,rz=Math.sin(0.00056/2);
        var setup=materia.assembly.AssemblyFrames.compose(target,{x:0.0,y:0.0,z:profile.placing.limits.sampleTravelMm-0.82,
          qx:rx,qy:ry,qz:rz,qw:Math.sqrt(1-rx*rx-ry*ry-rz*rz)});
        var object:nativekit.sim.SimObject=cast cell.stock.get(slot);
        cell.harness.stop();object.teleport(TendingElectricSimCell.pose(setup));
        var under=materia.assembly.AssemblyFrames.compose(setup,{x:0.0,y:0.0,z:-0.5,qx:0.0,qy:0.0,qz:0.0,qw:1.0});
        var pedestal=cell.harness.space.session.createObject(nativekit.sim.MotionType.Static,nativekit.sim.SimShape.box(0.025,0.025,0.0005),TendingElectricSimCell.pose(under),0,null,SimulatedContactSurfaces.environmentPart());
        events.push({operation:"seeded low, tilted physical grasp at reviewed tray search entry; pickup/transport excluded",side:traySide,pose:setup});
        cell.grasp(traySide,slot);held.set(traySide,slot);
        var captured=cell.harness.simulation.objectPose(object),height=target.z+profile.placing.limits.sampleTravelMm-captured.z*1000;
        var angle=TendingSeatPolicy.orientationError([captured.qx,captured.qy,captured.qz,captured.qw],[target.qx,target.qy,target.qz,target.qw],TendingSeatOrientation.FullPose);
        if(height<0.5 || angle<0.001)throw "Tray regression lost its nonideal grasp";
        events.push({operation:"confirmed nonideal tray grasp",heightErrorMm:height,orientationErrorRad:angle});
        cell.harness.removeObject(pedestal);cell.harness.step();cell.driver.checkRetention();for(_ in 0...20)cell.tick();
        cell.placeAtEntry(profile,slot);cell.release(traySide,slot);held.remove(traySide);
        cell.checkPlaced("outfeedTray",slot,profile.seating);delivered=1;
        cell.route(profile,true,slot);passed=true;
      } else if(locatingSide!=null) {
        var slot=locatingSide=="a" ? 0 : 1,profile:Dynamic=profiles.get("vise-load/"+locatingSide);
        var load=cell.fixtureState.worldConnector("mill/mill/vise/body","load");
        var axis=materia.assembly.AssemblyFrames.transformVector(cell.fixtureState.worldConnector("mill/mill/vise/body","seat"),1,0,0);
        var clearance:Float=profile.locating.limits.clearanceMm;
        if(locatingOffsetCheck)clearance-=0.45; // Seed at reviewed search sample 9 to isolate contact response.
        var setup:materia.assembly.AssemblyRecord.AssemblyFrame={x:load.x+axis.x*clearance,
          y:load.y+axis.y*clearance,z:load.z+axis.z*clearance,
          qx:load.qx,qy:load.qy,qz:load.qz,qw:load.qw};
        if(locatingOffsetCheck) {
          // Envelope of the failed transported grasp: about 0.82 mm low,
          // 0.00183 rad roll and 0.00056 rad yaw, not an ideal centred grasp.
          var rx=-Math.sin(0.00183/2),ry=0.000009,rz=Math.sin(0.00056/2);
          setup=materia.assembly.AssemblyFrames.compose(setup,{x:0.0,y:0.0,z:-0.82,
            qx:rx,qy:ry,qz:rz,qw:Math.sqrt(1-rx*rx-ry*ry-rz*rz)});
        }
        var object:nativekit.sim.SimObject=cast cell.stock.get(slot);
        cell.harness.stop();object.teleport(TendingElectricSimCell.pose(setup));
        var under=materia.assembly.AssemblyFrames.compose(setup,{x:0.0,y:0.0,z:-0.5,qx:0.0,qy:0.0,qz:0.0,qw:1.0});
        var pedestal=cell.harness.space.session.createObject(nativekit.sim.MotionType.Static,nativekit.sim.SimShape.box(0.025,0.025,0.0005),TendingElectricSimCell.pose(under),0,null,SimulatedContactSurfaces.environmentPart());
        events.push({operation:"seeded test setup on temporary support; pickup and transport are outside this test",side:locatingSide,pose:setup});
        cell.grasp(locatingSide,slot);held.set(locatingSide,slot);
        if(locatingOffsetCheck) {
          var captured=cell.harness.simulation.objectPose(object);
          var angle=TendingSeatPolicy.orientationError([captured.qx,captured.qy,captured.qz,captured.qw],
            [load.qx,load.qy,load.qz,load.qw],TendingSeatOrientation.FullPose);
          var height=load.z-captured.z*1000;
          if(height<0.5 || angle<0.001)throw "Grasp-offset regression lost its height/tilt perturbation";
          events.push({operation:"confirmed nonideal physical grasp",heightErrorMm:height,orientationErrorRad:angle});
        }
        cell.harness.removeObject(pedestal);
        // Editing setup topology clears the hosted snapshot. Bootstrap a fresh
        // physical frame before the feedback driver reads any joint samples.
        cell.harness.step();cell.driver.checkRetention();
        for(_ in 0...20)cell.tick();
        if(locatingOffsetCheck)events.push({operation:"contact regression starts at reviewed search sample 9; earlier transport/free search excluded"});
        cell.locate(profile,slot,locatingOffsetCheck ? 9 : 0);cell.release(locatingSide,slot);held.remove(locatingSide);
        cell.checkPlaced("vise",slot,profile.seating);part=slot;
        cell.route(profile,true,slot);cell.clamp(true);cell.checkPlaced("vise",slot,profile.clamping);
        events.push({operation:"physical locating, release, support confirmation, withdrawal and clamp",time:cell.harness.simulation.simulationTime()});passed=true;
      } else if(smoke) {
        Sys.println("Physics withdrawal smoke: actual grasp, then certified withdrawal to park");Sys.stdout().flush();
        cell.grasp("a",0);held.set("a",0);route("infeedTray-0-0","a",true,0);
        events.push({operation:"physical grasp and withdrawal",time:cell.harness.simulation.simulationTime()});passed=true;
      } else {
      var plan=TendingCyclePlan.electricBearingBlocks(),placementComplete=false;
      for(operation in plan.operations()) {
        activeOperation=Std.string(operation);
        Sys.println("Physics operation: "+Std.string(operation));Sys.stdout().flush();
        switch(operation) {
          case Collect(slot,tool,_,_):
            var selected=side(tool),station='infeedTray-${plan.rowOf(slot)}-${plan.columnOf(slot)}';
            cell.checkPlaced("infeedTray",slot);route(station,selected,false,slot);cell.grasp(selected,slot);held.set(selected,slot);route(station,selected,true,slot);
          case OpenDoor: cell.door(true);
          case Unclamp: cell.clamp(false);
          case Load(tool,_):
            var selected=side(tool),slot:Int=cast held.get(selected);
            route("vise-load",selected,false,slot);var loadedProfile:Dynamic=profiles.get("vise-load/"+selected);
            if(clearanceFirst && loadedProfile.locating==null)throw "Clearance-first load needs a locating corridor";
            if(loadedProfile.locating!=null)cell.locate(loadedProfile,slot);
            cell.release(selected,slot);held.remove(selected);
            cell.checkPlaced("vise",slot,loadedProfile.seating);part=slot;clampingPolicy=loadedProfile.clamping;
          case LeaveVise(tool): route("vise-load",side(tool),true,cast part);
          case Clamp:
            cell.clamp(true);
            if(part==null || clampingPolicy==null)throw "Missing vise datum confirmation after clamp";
            cell.checkPlaced("vise",cast part,clampingPolicy);placementComplete=true;
          case RequestDoorClose: cell.door(false);
          case DoorClosed: if(cell.fixtureState.joint("mill/door")!=0)throw "Door closure not observed";
          case StartCycle: cell.startCycle();cuts++;
          case CycleReady: cell.cycleReady();
          case Unload(slot,tool):
            var selected=side(tool);route("vise-seat",selected,false,slot);var graspProfile:Dynamic=profiles.get("vise-seat/"+selected);cell.grasp(selected,slot,graspProfile.registration==null ? null : graspProfile);cell.clamp(false);held.set(selected,slot);part=null;
          case Retreat(tool): var selected=side(tool);route("vise-seat",selected,true,cast held.get(selected));
          case Deliver(slot,tool,_,_):
            var selected=side(tool),station='outfeedTray-${plan.rowOf(slot)}-${plan.columnOf(slot)}';
            route(station,selected,false,slot);cell.release(selected,slot);held.remove(selected);var deliveredProfile:Dynamic=profiles.get(station+"/"+selected);
            cell.checkPlaced("outfeedTray",slot,deliveredProfile.seating);delivered++;route(station,selected,true,slot);
        }
        events.push({operation:Std.string(operation),time:cell.harness.simulation.simulationTime(),heldA:held.get("a"),heldB:held.get("b"),visePart:part});
        if(placementSmoke && placementComplete){passed=true;break;}
      }
      if(!placementSmoke)passed=delivered==4 && cuts==4;
      }
    } catch(error:Dynamic) {failure=Std.string(error);failureStack=haxe.CallStack.toString(haxe.CallStack.exceptionStack(true));Sys.println(failureStack);cell.driver.stopAndHold();}
    var faultContacts=[for(contact in cell.harness.simulation.robotContacts(cell.runtime)) {
        link:(cast scene.blueprint.identity:RobotRuntimeIdentity).linkId(contact.linkIndex),otherKind:contact.otherKind,
        otherLink:contact.otherLink,otherName:cell.names.get(contact.otherObject),active:contact.active,distanceMm:contact.distance*1000,
        forceN:contact.force.norm()
      }];
    var faultWorldContacts=cell.harness.space.session.contacts();
    var faultJoints=cell.driver.joints();
    // Preserve the failure instant: the controlled stop below advances physics.
    var faultFrame=cell.harness.space.session.capture();
    var faultStockStates=[for(slot in cell.stock.keys()) {
      var state=faultFrame.bodyState(cell.harness.simulation.objectBody(cast cell.stock.get(slot)));
      {slot:slot,pose:state.pose,linearVelocity:[state.linearVelocityX,state.linearVelocityY,state.linearVelocityZ],
        angularVelocity:[state.angularVelocityX,state.angularVelocityY,state.angularVelocityZ]};
    }];
    faultFrame.dispose();
    var stopObservation:Dynamic=passed ? null : cell.stopAndObserve();
    var measured=cell.driver.joints();
    var report={mode:mode,passed:passed,failure:failure,failureStack:failureStack,physicsPlayback:backend==SimulationSpace.MUJOCO,backend:backend==SimulationSpace.MUJOCO ? "MuJoCo" : "deterministic test backend",
      programReady:false,hardwareReady:false,cuts:cuts,delivered:delivered,events:events,graspObservations:cell.graspObservations,seatObservations:cell.seatObservations,locatingObservations:cell.locatingObservations,placingObservations:cell.placingObservations,registrationObservations:cell.registrationObservations,extractionObservations:cell.extractionObservations,releasedStockReturnChecks:cell.releasedStockReturnChecks,seatingTrace:cell.seatingTrace,faultWorldContacts:faultWorldContacts,faultJoints:faultJoints,measuredJoints:measured,wristFrame:cell.wristFrame(),reviewedPark:review.park,
      jointTrackingToleranceRad:0.00001,gravityFeedforward:true,gravityDerivativeChecks:6,jawFrameChecks:scene.jawFrameChecks,
      stopObservation:stopObservation,stopped:cell.driver.stopped,heldA:held.get("a"),heldB:held.get("b"),visePart:part,
      activeOperation:activeOperation,routeObservation:cell.routeObservation,retentionFault:cell.driver.retentionFault,
      faultStockStates:faultStockStates,
      collisionPieces:scene.blueprint.linkCollisionShapes.length,collisionApproximation:scene.blueprint.collisionApproximation,provenance:review.provenance,
      executionHost:"headless SimKit/RobotKit harness; not MissionPlayer UI integration",
      fullCycleQualified:false,cncHandshakeBound:true,plcAlarm:cell.plcAlarm,
      fixtureTargets:{viseClosedPositionMm:cell.viseClosedPositionMm},
      supportContact:{timeConstantSeconds:SimulatedContactSurfaces.HARD_FIXTURE_TIME_CONSTANT,dampingRatio:1.0,
        frictionDimensions:0},
      worldBodies:[for(id in cell.bodyNames.keys()){id:id,name:cell.bodyNames.get(id)}],
      solver:cell.solverOptions,contactTimeConstantSeconds:SimulatedContactSurfaces.HARD_FIXTURE_TIME_CONSTANT,assumedFingerFrictionCoefficient:0.5,maximumJointCommandRateRadPerSecond:0.5,simulationGripperSettings:{perJawEffortCapN:70,jawSpeedMmPerSecond:5,minimumContactForceN:0.1,maximumContactPenetrationMm:0.1,qualified:false},
      stockMassProperties:[for(slot in cell.stockMassProperties.keys()){
        slot:slot,properties:cell.stockMassProperties.get(slot)}],
      stockPoses:[for(slot in cell.stock.keys()){slot:slot,pose:cell.harness.simulation.objectPose(cast cell.stock.get(slot))}],
      contacts:faultContacts,
      limitations:["Simulation mass and drive assumptions are unqualified", "CNC cutting is a process delay, not material removal", "Fixture motion is prescribed; vise clamping force is not modeled", "Hardware force/speed limits remain unqualified"]};
    sys.io.File.saveContent(root+reportFile,haxe.Json.stringify(report,null,"  "));
    cell.dispose();Sys.println(haxe.Json.stringify(report));
    if(!passed)Sys.exit(1);
  }
}
