import robotkit.collision.CollisionSceneBuilder;
import cadbridge.CellCollisionSnapshot;
import collisionkit.native.NativeCollisionWorld;
import cadbridge.AssemblySimulationBridge;
import cadbridge.AssemblyPhysicalPartView;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.Vector;
import machinekit.assembly.Diagnostics;
import machinekit.welding.WeldMetal;
import machinekit.welding.WeldSeam;
import machinekit.welding.Weldment.WeldmentSeams;
import machinekit.welding.WeldingMission;
import machinekit.welding.WeldingRecipe;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact.SceneArtifactData;
import materia.project.SceneArtifact.SceneArtifactMissionStep;
import materia.project.SceneArtifact.SceneArtifactWeld;
import materia.project.SceneContactRegistration.SceneContactWork;
import processkit.WeldStationCandidates;
import processkit.WeldStationPlanner;
import processkit.WeldStationPlanner.WeldStationCandidate;
import processkit.WeldStationPlanner.WeldStationPlan;
import processkit.WeldStationPlanner.WeldStationPlanFailure;
import processkit.WeldingPlanRunner;
import processkit.WeldingPlanRunner.WeldPlanning;
import processkit.WeldScenePlan;
import processkit.ProbeMotionPlanner;
import processkit.ProbeWire;
import processkit.ContactProbeRunner.ContactProbeRequest;
import machinekit.welding.WeldContactSelection;
import machinekit.welding.WeldProbeGeometry;
import machinekit.welding.WeldProbeGeometry.WeldProbeFace;
import robotkit.manipulation.Manipulator;
import robotkit.collision.CollisionClearance;
import robotkit.manipulation.JointRoute;
import robotkit.manipulation.ClearanceBodyData;
import robotkit.model.Frame;
import robotkit.mobile.Pose2;
import robotkit.navigation.FloorMap;
import robotkit.navigation.FloorMap.FloorBox;
import robotkit.navigation.Costmap2;
import robotkit.navigation.AStarPlanner;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;

/** CAD integration of the ProcessKit station policy and the execution motion checks. */
class MobileWeldStations {
  final cell:MobileWelderCell;
  final scene:SceneArtifactData;
  final arm:Manipulator;
  final ready:Array<Float>;
  final homeJoints:Array<Float>;
  final seams:Array<WeldSeam>;
  final probeFaces:Array<materia.project.SceneContactRegistration.SceneContactFace>;
  final probeGeometry:WeldProbeGeometry;
  final probeLink:robotkit.model.LinkId;
  final probeTip:Transform3;
  final probeWireDiameter:Float;
  final probeStickout:Float;
  final work:Transform3;
  final metal:String;
  final collision:CellCollisionSnapshot;
  final collisionScenes:Map<String, CollisionSceneBuilder> = new Map();
  final boxes:Array<FloorBox> = [];
  final planning:Map<String, WeldPlanning> = new Map();
  final clearances:Map<String, CollisionClearance> = new Map();
  final registrationChecks:Map<String, Null<String>> = new Map();
  final verifiedSchedules:Map<String, MobileStationSchedule> = new Map();
  var registrationJoints:Array<Float> = [];
  final candidates:Array<WeldStationCandidate>;
  final navigation:AStarPlanner;
  final start:Pose2;

  public function new(cell:MobileWelderCell, design:SceneArtifactData) {
    // Decode resolves the material library and validates the one current scene schema.
    var scene = materia.project.SceneArtifact.decode(materia.project.SceneArtifact.encode(design));
    this.cell = cell;
    this.scene = scene;
    var definition = scene.assemblyDefinition;
    if (definition == null) throw "Mobile station planning needs its assembly definition";
    var state = new AssemblyState(definition, scene.assemblyState);
    var physical = AssemblyPhysicalPartView.fromSceneArtifact(scene);
    var converted = AssemblySimulationBridge.toRobotModel(definition, physical, scene.assemblyState, null, null, scene.mobileBase);
    var flat = AssemblyDefinitionFlattener.flatten(definition);
    var tools = scene.robotTools;
    if (tools == null || tools.length != 1) throw "Mobile station planning needs one installed torch";
    var tool = tools[0];
    var contact = tool.contact;
    var mount = converted.partLinks.get(contact.occurrence);
    if (mount == null) throw "Mobile torch has no model carrier";
    var connectors:Array<AssemblyFrame> = [];
    for (occurrence in flat.occurrences) if (occurrence.id == contact.occurrence)
      for (part in flat.definitions) if (part.id == occurrence.definition)
        for (connector in part.connectors) if (connector.name == contact.connector) connectors.push(connector.frame);
    if (connectors.length != 1) throw "Mobile torch needs one CAD contact frame";
    var tcp = AssemblyFrames.compose(mount.offset, scaled(connectors[0], scene.metresPerUnit));
    var frame = converted.model.addFrame(new Frame("welding-contact", converted.model.links[mount.link]));
    frame.position = [tcp.x, tcp.y, tcp.z]; frame.rotation = [tcp.qx, tcp.qy, tcp.qz, tcp.qw];
    arm = new Manipulator(converted.model, converted.model.links[0].id, frame.id, null, null, null, converted.profile);
    var torch = tool.torch;
    if (torch == null) throw "Mobile station planning needs the CAD torch wire profile";
    probeLink = converted.model.links[mount.link].id;
    probeTip = new Transform3(new Vec3(tcp.x, tcp.y, tcp.z), new Quat(tcp.qx, tcp.qy, tcp.qz, tcp.qw));
    probeWireDiameter = torch.wireDiameterMm * 0.001;
    probeStickout = torch.stickoutMm * 0.001;
    // CadBridge bakes the scene's initial mechanical positions into the model frames.
    homeJoints = [for (joint in arm.group.jointIds) state.joint(joint)];
    ready = [for (_ in arm.group.jointIds) 0.0];
    var weldment = cell.weldment();
    seams = weldment.findIn(cell, cell.solvedPoses(state)).require();
    probeFaces = machinekit.welding.WeldProbeGeometry.findIn(cell, weldment, cell.solvedPoses(state), ["table", "clamp"]).contactFaces(scene.metresPerUnit);
    probeGeometry = WeldProbeGeometry.fromContactFaces(probeFaces, 1.0);
    work = transform(state.worldPose(weldment.reference), scene.metresPerUnit);
    metal = WeldMetal.carrierOf(cell, weldment);
    collision = CellCollisionSnapshot.fromAssembly("mobile-welder", definition, physical, state, converted, [metal], tool.members);
    for (bound in collision.fixedBounds()) {
      var min = bound.minimum, max = bound.maximum;
      boxes.push({id: bound.id, x: min[0] / 2 + max[0] / 2, y: min[1] / 2 + max[1] / 2,
        z: min[2] / 2 + max[2] / 2, halfX: (max[0] - min[0]) / 2, halfY: (max[1] - min[1]) / 2,
        halfZ: (max[2] - min[2]) / 2, yaw: 0});
    }
    var base = scene.mobileBase;
    if (base == null) throw "Mobile station planning needs its base";
    var origin = base.origin;
    if (origin == null) throw "Mobile station planning needs its floor origin";
    start = new Pose2(origin.x, origin.y, origin.yaw);
    var chassis = Transform3.fromPose2(start);
    var firstJoint = [for (joint in flat.joints) if (joint.id == arm.group.jointIds[0]) joint][0];
    var armMount = chassis.inverse().compose(transform(state.worldConnector(firstJoint.parent, firstJoint.parentConnector), scene.metresPerUnit));
    var points = [for (seam in seams) new Vec3(seam.start.x * scene.metresPerUnit, seam.start.y * scene.metresPerUnit, seam.start.z * scene.metresPerUnit)];
    for (seam in seams) points.push(new Vec3(seam.stop.x * scene.metresPerUnit, seam.stop.y * scene.metresPerUnit, seam.stop.z * scene.metresPerUnit));
    // Sampling parameters, not authored parking poses: access decides which proposals are usable.
    candidates = WeldStationCandidates.aroundWithHeadings(points, work.toPose2(), armMount.translation,
      0.45, 0.85, 5, 12, 4);
    var poses = [for (candidate in candidates) candidate.pose]; poses.push(start);
    var length = base.footprintLength, width = base.footprintWidth;
    if (length == null || width == null) throw "Mobile station planning needs its physical footprint";
    var radius = Math.sqrt(length * length + width * width) / 2;
    var map = new Costmap2(FloorMap.rasterize(boxes, poses, FloorMap.DEFAULT_RESOLUTION,
      FloorMap.navigationMargin(radius, 0.3), FloorMap.DEFAULT_CLEARANCE, "map"), radius, true, 0.3, 1.5);
    navigation = new AStarPlanner(map);
  }

  static function scaled(frame:AssemblyFrame, scale:Float):AssemblyFrame
    return {x: frame.x * scale, y: frame.y * scale, z: frame.z * scale, qx: frame.qx, qy: frame.qy, qz: frame.qz, qw: frame.qw};
  static function transform(frame:AssemblyFrame, scale:Float):Transform3
    return new Transform3(new Vec3(frame.x * scale, frame.y * scale, frame.z * scale), new Quat(frame.qx, frame.qy, frame.qz, frame.qw));

  function collisionAt(station:WeldStationCandidate):CollisionSceneBuilder {
    var cached = collisionScenes.get(station.id);
    if (cached != null) return cached;
    var scene = collision.at(Transform3.fromPose2(station.pose)).describe(arm, ready, CollisionClearance.MARGIN);
    var work:Array<String> = [];
    for (seam in seams) for (face in [seam.a, seam.b]) if (work.indexOf(face.member) < 0) work.push(face.member);
    for (face in probeFaces) if (work.indexOf(face.member) < 0) work.push(face.member);
    processkit.WeldCollisionPolicy.apply(scene, scene.memberObjects(work), processkit.WeldPathPlanner.AIR_MARGIN, CollisionClearance.CONTACT_MARGIN);
    collisionScenes.set(station.id, scene);
    return scene;
  }

  function context(station:WeldStationCandidate):WeldPlanning {
    var cached = planning.get(station.id);
    if (cached != null) return cached;
    var clearance = collisionAt(station).clearance(() -> new NativeCollisionWorld());
    clearances.set(station.id, clearance);
    var made = WeldingPlanRunner.planning(arm, 2.0, clearance);
    planning.set(station.id, made);
    return made;
  }

  function route(from:Pose2, to:Pose2):Null<Float> {
    if (from.x == to.x && from.y == to.y) return 0;
    try return navigation.plan(from, to).length catch (error:Dynamic) {
      if (Std.string(error).indexOf("AStarPlanner") < 0) throw error;
      return null;
    }
  }

  function screen(station:WeldStationCandidate, name:String):Null<String> {
    var cell = navigation.costmap.grid.worldToCell(station.pose);
    return cell == null || !navigation.costmap.isTraversable(cell.x, cell.y) ? "parking footprint is blocked" : null;
  }

  function contactWork():SceneContactWork {
    return {frame: cell.weldment().reference,
      nominal: {x: work.translation.x, y: work.translation.y, z: work.translation.z,
        qx: work.rotation.x, qy: work.rotation.y, qz: work.rotation.z, qw: work.rotation.w},
      translation: [0.02, 0.02, 0.0], rotation: [0.0, 0.0, 2 * Math.PI / 180],
      measurementError: 0.00001, contactOffset: processkit.tool.WeldArcModel.TOUCH_TOLERANCE, faces: probeFaces};
  }

  /** Prove a nominal 3-2-1 sequence with each selected checkpoint and the final ready return. */
  function registrationProblem(station:WeldStationCandidate):Null<String> {
    if (registrationChecks.exists(station.id)) return registrationChecks.get(station.id);
    var problem:Null<String> = null;
    Sys.println('Mobile station ${station.id} starts its registration preflight');
    try {
      var selected = context(station);
      // Registration adds the physical wire to its own copy; welding keeps the reusable station description.
      var scene = collision.at(Transform3.fromPose2(station.pose)).describe(arm, ready, CollisionClearance.MARGIN);
      var workMembers:Array<String> = [];
      for (seam in seams) for (face in [seam.a, seam.b]) if (workMembers.indexOf(face.member) < 0) workMembers.push(face.member);
      for (face in probeFaces) if (workMembers.indexOf(face.member) < 0) workMembers.push(face.member);
      processkit.WeldCollisionPolicy.apply(scene, scene.memberObjects(workMembers), processkit.WeldPathPlanner.AIR_MARGIN, CollisionClearance.CONTACT_MARGIN);
      var wire = ProbeWire.describe(scene, arm, probeLink, probeTip, probeWireDiameter, probeStickout,
        scene.toolObjects(), scene.memberObjects(workMembers));
      var clearance = scene.clearance(() -> new NativeCollisionWorld());
      var probes = new ProbeMotionPlanner(arm, selected.compiler, clearance, wire);
      registrationJoints = ready.copy();
      var selection = new WeldContactSelection(contactWork(), probes, () -> registrationJoints.copy());
      var rootWork = Transform3.fromPose2(station.pose).inverse().compose(work);
      var sequence = selection.sequence(rootWork);
      var request:Null<ContactProbeRequest> = sequence.start();
      var selectedStage = sequence.activeStage();
      if (selectedStage != null)
        Sys.println('Mobile station ${station.id} selected a ${selectedStage.probes.length}-contact registration stage');
      var contacts = 0;
      while (request != null) {
        contacts++;
        if (contacts > 6) throw "CAD registration plan exceeded its six contact points";
        var stage = sequence.activeStage();
        if (stage == null) throw "CAD registration plan lost its selected plane";
        Sys.println('Mobile station ${station.id} plans registration contact $contacts/6');
        var estimate = sequence.envelope.estimate();
        var rootNormal = estimate.rotation.rotate(stage.normal);
        var rootPlanePoint = estimate.transformPoint(stage.normal.scale(stage.offset));
        var denominator = rootNormal.dot(request.direction);
        if (Math.abs(denominator) < 1e-8) throw "Nominal contact ray is tangent to its selected CAD plane";
        var travel = rootNormal.dot(rootPlanePoint.sub(request.approach.translation)) / denominator;
        if (!Math.isFinite(travel) || travel < 0 || travel > request.distance + 1e-8)
          throw "Nominal contact lies outside the checked sensing corridor";
        var measured = request.approach.translation.add(request.direction.scale(travel));
        var workPoint = estimate.inverse().transformPoint(measured);
        var patch = false;
        for (face in probeGeometry.faces) if (face.target &&
            new Vec3(face.normal.x, face.normal.y, face.normal.z).sub(stage.normal).norm() < 1e-8 &&
            Math.abs(face.offset() - stage.offset) < 1e-8 &&
            face.contains(new Vector(workPoint.x, workPoint.y, workPoint.z), 0.000001)) patch = true;
        if (!patch) throw "Nominal contact ray misses the selected target CAD patch";
        if (request.approachJoints == null) throw "CAD registration plan has no checked probe posture";
        registrationJoints = request.approachJoints.copy();
        request = sequence.observe(measured);
        if (request == null && sequence.stageTransitionPending) {
          request = sequence.selectNextStage();
          var nextStage = sequence.activeStage();
          if (nextStage == null) throw "CAD registration plan lost its next stage entry";
          var stageCheckpoint = nextStage.entryJoints;
          var checkpoint = stageCheckpoint == null ? ready.copy() : stageCheckpoint;
          if (checkpoint.length != registrationJoints.length)
            throw "CAD registration stage checkpoint has the wrong joint count";
          var moved = false;
          for (joint in 0...checkpoint.length)
            if (Math.abs(registrationJoints[joint] - checkpoint[joint]) > 1e-12) moved = true;
          if (moved) {
            var restore = probes.approachJoints(checkpoint, registrationJoints);
            for (joint in 0...checkpoint.length) if (Math.abs(restore.endJoints[joint] - checkpoint[joint]) > 0.005)
              throw 'CAD registration plane transition misses checkpoint joint $joint';
          }
          registrationJoints = checkpoint.copy();
        }
        selectedStage = sequence.activeStage();
        if (request != null && selectedStage != null)
          Sys.println('Mobile station ${station.id} selected a ${selectedStage.probes.length}-contact registration stage');
      }
      var result = sequence.result;
      if (contacts != 6 || result == null || !result.accepted)
        throw result == null ? "CAD registration plan did not fit a complete work frame" :
          'CAD registration plan is not observable: ${result.reason}';
      var restore = probes.approachJoints(ready, registrationJoints);
      for (joint in 0...ready.length) if (Math.abs(restore.endJoints[joint] - ready[joint]) > 0.005)
        throw 'CAD registration return misses ready joint $joint';
      Sys.println('Mobile station ${station.id} proves 3-2-1 contact registration and checked return');
    } catch (error:Dynamic) {
      var summary = StringTools.trim(Std.string(error));
      var end = summary.indexOf(";");
      if (end >= 0) summary = summary.substr(0, end);
      if (summary.length > 180) summary = summary.substr(0, 177) + "...";
      problem = summary;
      Sys.println('Mobile station ${station.id} rejects contact registration: $problem');
    }
    registrationChecks.set(station.id, problem);
    return problem;
  }

  function verify(station:WeldStationCandidate, name:String):Null<String> {
    var registration = registrationProblem(station);
    if (registration != null) return 'contact registration: $registration';
    var found = [for (seam in seams) if (seam.name() == name) seam][0];
    var selected = context(station);
    var where = Transform3.fromPose2(station.pose).inverse().compose(work);
    var step = WeldingRecipe.passStep(found.legSize, {diameterMm: cell.robot.feeder.wireDiameterMm,
      depositionEfficiency: cell.robot.feeder.depositionEfficiency, maxSpeedMPerMin: cell.robot.feeder.maxSpeedMPerMin},
      [found], cell.weldment().reference, metal, scene.metresPerUnit);
    var weld = step.weld;
    if (weld == null) throw "Generated welding pass has no process definition";
    try {
      var q = ready.copy();
      for(pass in weld.passes){
        var selection=selected.checked(WeldScenePlan.pass(weld,pass,where),q);
        var last=selection.curves[selection.curves.length-1];q=last.q[last.q.length-1];
      }
      Sys.println('Mobile station ${station.id} proves $name with shared path selection');
      return null;
    }catch(error:Dynamic)return Std.string(error);
  }

  function generateSteps(station:WeldStationCandidate, group:Array<WeldSeam>):Array<SceneArtifactMissionStep> {
    var home = work.inverse().compose(Transform3.fromPose2(station.pose).compose(arm.forwardKinematics(ready)));
    var wire = home.rotation.rotate(new Vec3(0, 0, 1));
    return WeldingMission.generate(cell.weldment(), new WeldmentSeams(group, new Diagnostics()),
      {diameterMm: cell.robot.feeder.wireDiameterMm, depositionEfficiency: cell.robot.feeder.depositionEfficiency,
        maxSpeedMPerMin: cell.robot.feeder.maxSpeedMPerMin}, metal, scene.metresPerUnit,
      {position: new Vector(home.translation.x / scene.metresPerUnit, home.translation.y / scene.metresPerUnit,
        home.translation.z / scene.metresPerUnit), wire: new Vector(wire.x, wire.y, wire.z)}).require();
  }

  function scheduleKey(station:WeldStationCandidate, names:Array<String>):String {
    var result = station.id;
    for (name in names) result += ':${name.length}:$name';
    return result;
  }

  /** Prove the actual ordered weld runs and ready-posture return for one proposed station group. */
  function verifySchedule(plan:WeldStationPlan):Null<WeldStationPlanFailure> {
    for (index in 0...plan.stations.length) {
      var station = plan.stations[index];
      var names = plan.seams[index];
      var key = scheduleKey(station, names);
      if (verifiedSchedules.exists(key)) continue;
      var group = [for (seam in seams) if (names.indexOf(seam.name()) >= 0) seam];
      var steps:Array<SceneArtifactMissionStep>;
      try steps = generateSteps(station, group) catch (error:Dynamic)
        return new WeldStationPlanFailure(station.id, names,
          'CAD mission generation failed for its ordered seam group: ${Std.string(error)}');

      var selected = context(station);
      var where = Transform3.fromPose2(station.pose).inverse().compose(work);
      var q = ready.copy();
      for (step in steps) if (step.kind == "weld") {
        var weld = step.weld;
        if (weld == null) throw "Generated weld run has no process definition";
        var runNames:Array<String> = [];
        for (segment in weld.path) if (runNames.indexOf(segment.seam) < 0) runNames.push(segment.seam);
        try {
          for (pass in weld.passes) {
            var selection = selected.checked(WeldScenePlan.pass(weld, pass, where), q);
            var last = selection.curves[selection.curves.length - 1];
            q = last.q[last.q.length - 1];
          }
        } catch (error:Dynamic) {
          return new WeldStationPlanFailure(station.id, runNames,
            'Ordered weld run ${runNames.join(", ")} is not reachable from the preceding run: ${Std.string(error)}');
        }
      }

      var lower = [for (joint in 0...arm.group.count()) arm.group.limitsOf(joint).lower];
      var upper = [for (joint in 0...arm.group.count()) arm.group.limitsOf(joint).upper];
      try {
        var stow = JointRoute.plan(q, ready, lower, upper, (from, to) -> airMove(station, from, to));
        if (stow.length < 2) throw "no checked ready-posture stow";
      } catch (error:Dynamic) {
        return new WeldStationPlanFailure(station.id, names,
          'Station cannot stow after its ordered seam group: ${Std.string(error)}');
      }
      verifiedSchedules.set(key, new MobileStationSchedule(steps));
    }
    return null;
  }

  /** Validate the same exact-stop joint motion emitted to the application, including its time law. */
  function airMove(station:WeldStationCandidate, from:Array<Float>, to:Array<Float>):Bool {
    var clearance = clearances.get(station.id);
    if (clearance == null) throw 'Station ${station.id} has no collision context';
    if (clearance.violation(from) != null) return false;
    var moving = false;
    for (i in 0...from.length) if (Math.abs(from[i] - to[i]) > 1e-12) moving = true;
    if (!moving) return true;
    var selected = context(station);
    var program:Null<motionkit.robot.CompiledProgram> = null;
    try {
      program = selected.compiler.compile(new motionkit.program.MotionProgram([
        motionkit.program.MotionOp.MoveJ(motionkit.program.MoveTarget.JointTarget(to),
          new motionkit.MotionOptions(), motionkit.program.Blend.ExactStop)]), from, haxe.Int64.ofInt(1));
      for (block in program.blocks) for (trajectory in block.plans) {
        var proof = motionkit.robot.TrajectoryClearanceProof.run(clearance, trajectory,
          selected.compiler.maxVelocity, false);
        if (!proof.closed || proof.failure != null) { program.dispose(); return false; }
      }
      program.dispose(); return true;
    } catch (_:Dynamic) {
      if (program != null) program.dispose();
      return false;
    }
  }

  public function mission():Array<SceneArtifactMissionStep> {
    var chosen = WeldStationPlanner.verified([for (seam in seams) seam.name()], candidates, start,
      screen, verify, route, verifySchedule);
    var result:Array<SceneArtifactMissionStep> = [];
    for (index in 0...chosen.stations.length) {
      var station = chosen.stations[index];
      var group = [for (seam in seams) if (chosen.seams[index].indexOf(seam.name()) >= 0) seam];
      var schedule = verifiedSchedules.get(scheduleKey(station, [for (seam in group) seam.name()]));
      if (schedule == null) throw 'Station ${station.id} has no verified ordered weld schedule';
      result.push({kind: "goTo", pose: {x: station.pose.x, y: station.pose.y, yaw: station.pose.yaw}});
      var tools = scene.robotTools;
    if (tools == null || tools.length != 1) throw "Mobile station planning needs one installed torch";
      result.push({kind: "findWork", at: tools[0].contact, contactWork: contactWork()});
      for (step in schedule.steps) result.push(step);
      // The runtime replans this semantic stow from the measured post-weld arm
      // pose and current cell geometry; these nominal edges only prove station feasibility.
      result.push({kind: "stow", at: tools[0].contact});
    }
    Sys.println('Mobile weld mission: ${chosen.stations.length} stations, ${seams.length} CAD seams, ${chosen.travelCost} m route');
    return result;
  }
}

private class MobileStationSchedule {
  public final steps:Array<SceneArtifactMissionStep>;
  public function new(steps:Array<SceneArtifactMissionStep>) this.steps = steps.copy();
}
