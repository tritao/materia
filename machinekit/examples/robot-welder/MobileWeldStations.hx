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
import processkit.WeldStationCandidates;
import processkit.WeldStationPlanner;
import processkit.WeldStationPlanner.WeldStationCandidate;
import processkit.WeldingPlanRunner;
import processkit.WeldingPlanRunner.WeldPlanning;
import processkit.WeldScenePlan;
import robotkit.manipulation.Manipulator;
import robotkit.manipulation.ArmClearance;
import robotkit.manipulation.JointRoute;
import robotkit.manipulation.ArmClearance.ClearanceBodyData;
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
  final work:Transform3;
  final metal:String;
  final robotBodies:Array<ClearanceBodyData>;
  final external:Array<{name:String, vertices:Array<Float>}> = [];
  final boxes:Array<FloorBox> = [];
  final planning:Map<String, WeldPlanning> = new Map();
  final clearances:Map<String, ArmClearance> = new Map();
  final reversed:Map<String, Map<String, Bool>> = new Map();
  final candidates:Array<WeldStationCandidate>;
  final navigation:AStarPlanner;
  final start:Pose2;

  public function new(cell:MobileWelderCell, design:SceneArtifactData) {
    // Decode resolves the material library and validates the one current scene schema.
    var scene = materia.project.SceneArtifact.decode(materia.project.SceneArtifact.encode(design));
    this.cell = cell;
    this.scene = scene;
    var definition:materia.assembly.AssemblyDefinition = cast scene.assemblyDefinition;
    var state = new AssemblyState(definition, scene.assemblyState);
    var physical = AssemblyPhysicalPartView.fromSceneArtifact(scene);
    var converted = AssemblySimulationBridge.toRobotModel(definition, physical, scene.assemblyState, null, null, scene.mobileBase);
    var flat = AssemblyDefinitionFlattener.flatten(definition);
    var tools:Array<materia.project.SceneArtifact.SceneArtifactRobotTool> = cast scene.robotTools;
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
    // CadBridge bakes the scene's initial mechanical positions into the model frames.
    homeJoints = [for (joint in arm.group.jointIds) state.joint(joint)];
    ready = [for (_ in arm.group.jointIds) 0.0];
    var weldment = cell.weldment();
    seams = weldment.findIn(cell, cell.solvedPoses(state)).require();
    probeFaces = machinekit.welding.WeldProbeGeometry.findIn(cell, weldment, cell.solvedPoses(state), ["table", "clamp"]).contactFaces(scene.metresPerUnit);
    work = transform(state.worldPose(weldment.reference), scene.metresPerUnit);
    metal = WeldMetal.carrierOf(cell, weldment);
    var prefix = contact.occurrence.substr(0, contact.occurrence.lastIndexOf("/") + 1);
    robotBodies = [for (hull in converted.linkHulls) if (hull.part != metal)
      {name: hull.part, link: converted.model.links[hull.link].id, vertices: hull.vertices,
        tool: StringTools.startsWith(hull.part, prefix)}];
    for (occurrence in flat.occurrences) if (!converted.partLinks.exists(occurrence.id) && occurrence.id != metal) {
      var part = [for (body in physical.parts) if (body.id == occurrence.definition) body][0];
      var hull:Array<Float> = cast part.collisionHull;
      var pose = transform(state.worldPose(occurrence.id), scene.metresPerUnit);
      var world:Array<Float> = [];
      var min = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
      var max = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
      for (index in 0...Std.int(hull.length / 3)) {
        var point = pose.transformPoint(new Vec3(hull[index * 3] * scene.metresPerUnit,
          hull[index * 3 + 1] * scene.metresPerUnit, hull[index * 3 + 2] * scene.metresPerUnit));
        var values = point.toArray();
        for (axis in 0...3) { world.push(values[axis]); min[axis] = Math.min(min[axis], values[axis]); max[axis] = Math.max(max[axis], values[axis]); }
      }
      external.push({name: occurrence.id, vertices: world});
      boxes.push({id: occurrence.id, x: min[0] / 2 + max[0] / 2, y: min[1] / 2 + max[1] / 2,
        z: min[2] / 2 + max[2] / 2, halfX: (max[0] - min[0]) / 2, halfY: (max[1] - min[1]) / 2,
        halfZ: (max[2] - min[2]) / 2, yaw: 0});
    }
    var base:materia.project.SceneArtifact.SceneArtifactMobileBase = cast scene.mobileBase;
    var origin:materia.project.SceneArtifact.SceneArtifactFloorPose = cast base.origin;
    start = new Pose2(origin.x, origin.y, origin.yaw);
    var chassis = Transform3.fromPose2(start);
    var firstJoint = [for (joint in flat.joints) if (joint.id == arm.group.jointIds[0]) joint][0];
    var armMount = chassis.inverse().compose(transform(state.worldConnector(firstJoint.parent, firstJoint.parentConnector), scene.metresPerUnit));
    var points = [for (seam in seams) new Vec3(seam.start.x * scene.metresPerUnit, seam.start.y * scene.metresPerUnit, seam.start.z * scene.metresPerUnit)];
    for (seam in seams) points.push(new Vec3(seam.stop.x * scene.metresPerUnit, seam.stop.y * scene.metresPerUnit, seam.stop.z * scene.metresPerUnit));
    // Sampling parameters, not authored parking poses: access decides which proposals are usable.
    candidates = WeldStationCandidates.around(points, work.toPose2(), armMount.translation, 0.45, 0.65, 3, 12);
    var poses = [for (candidate in candidates) candidate.pose]; poses.push(start);
    var length:Float = cast base.footprintLength, width:Float = cast base.footprintWidth;
    var radius = Math.sqrt(length * length + width * width) / 2;
    var map = new Costmap2(FloorMap.rasterize(boxes, poses, FloorMap.DEFAULT_RESOLUTION,
      FloorMap.navigationMargin(radius, 0.3), FloorMap.DEFAULT_CLEARANCE, "map"), radius, true, 0.3, 1.5);
    navigation = new AStarPlanner(map);
  }

  static function scaled(frame:AssemblyFrame, scale:Float):AssemblyFrame
    return {x: frame.x * scale, y: frame.y * scale, z: frame.z * scale, qx: frame.qx, qy: frame.qy, qz: frame.qz, qw: frame.qw};
  static function transform(frame:AssemblyFrame, scale:Float):Transform3
    return new Transform3(new Vec3(frame.x * scale, frame.y * scale, frame.z * scale), new Quat(frame.qx, frame.qy, frame.qz, frame.qw));

  function context(station:WeldStationCandidate):WeldPlanning {
    var cached = planning.get(station.id);
    if (cached != null) return cached;
    var bodies = robotBodies.copy();
    var inverse = Transform3.fromPose2(station.pose).inverse();
    for (body in external) {
      var vertices:Array<Float> = [];
      for (index in 0...Std.int(body.vertices.length / 3)) {
        var point = inverse.transformPoint(new Vec3(body.vertices[index * 3], body.vertices[index * 3 + 1], body.vertices[index * 3 + 2]));
        vertices.push(point.x); vertices.push(point.y); vertices.push(point.z);
      }
      bodies.push({name: body.name, link: arm.baseLink, vertices: vertices, tool: false});
    }
    var clearance = new ArmClearance(arm, bodies, ready, processkit.WeldPathPlanner.AIR_MARGIN);
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
    var cell:robotkit.navigation.GridCell2 = cast navigation.costmap.grid.worldToCell(station.pose);
    return cell == null || !navigation.costmap.isTraversable(cell.x, cell.y) ? "parking footprint is blocked" : null;
  }

  function verify(station:WeldStationCandidate, name:String):Null<String> {
    var found = [for (seam in seams) if (seam.name() == name) seam][0];
    var selected = context(station);
    var where = Transform3.fromPose2(station.pose).inverse().compose(work);
    var reason = "";
    for (reverse in [false, true]) {
      var step = WeldingRecipe.passStep(found.legSize, {diameterMm: cell.robot.feeder.wireDiameterMm,
        depositionEfficiency: cell.robot.feeder.depositionEfficiency, maxSpeedMPerMin: cell.robot.feeder.maxSpeedMPerMin},
        [reverse ? found.reversed() : found], cell.weldment().reference, metal, scene.metresPerUnit);
      var weld:materia.project.SceneArtifact.SceneArtifactWeld = cast step.weld;
      try {
        var q = ready.copy();
        for (pass in weld.passes) q = selected.planner.plan(WeldScenePlan.pass(weld, pass, where), q).endJoints;
        var directions = reversed.get(station.id);
        if (directions == null) { directions = new Map(); reversed.set(station.id, directions); }
        directions.set(name, reverse);
        Sys.println('Mobile station ${station.id} proves $name ($reverse reverse)');
        return null;
      } catch (error:Dynamic) reason = Std.string(error);
    }
    return reason;
  }

  /** Validate the same exact-stop joint motion emitted to the application, including its time law. */
  function airMove(station:WeldStationCandidate, from:Array<Float>, to:Array<Float>):Bool {
    var clearance:ArmClearance = cast clearances.get(station.id);
    if (clearance.sweep(from, to) != null) return false;
    var moving = false;
    for (i in 0...from.length) if (Math.abs(from[i] - to[i]) > 1e-12) moving = true;
    if (!moving) return true;
    var selected = context(station);
    var program:Null<motionkit.robot.CompiledProgram> = null;
    try {
      program = selected.compiler.compile(new motionkit.program.MotionProgram([
        motionkit.program.MotionOp.MoveJ(motionkit.program.MoveTarget.JointTarget(to),
          new motionkit.MotionOptions(), motionkit.program.Blend.ExactStop)]), from, haxe.Int64.ofInt(1));
      var speed = 0.0;
      for (limit in selected.compiler.maxVelocity) speed = Math.max(speed, limit);
      for (block in program.blocks) for (trajectory in block.plans) {
        var samples = Std.int(Math.max(1.0, Math.ceil(trajectory.durationSeconds * speed / 0.02)));
        for (sample in 0...samples + 1)
          if (clearance.violation(trajectory.evaluate(trajectory.durationSeconds * sample / samples).positions) != null) {
            program.dispose(); return false;
          }
      }
      program.dispose(); return true;
    } catch (_:Dynamic) {
      if (program != null) program.dispose();
      return false;
    }
  }

  public function mission():Array<SceneArtifactMissionStep> {
    var chosen = WeldStationPlanner.verified([for (seam in seams) seam.name()], candidates, start, screen, verify, route);
    var result:Array<SceneArtifactMissionStep> = [];
    for (index in 0...chosen.stations.length) {
      var station = chosen.stations[index];
      var group = [for (seam in seams) if (chosen.seams[index].indexOf(seam.name()) >= 0) seam];
      var home = work.inverse().compose(Transform3.fromPose2(station.pose).compose(arm.forwardKinematics(ready)));
      var wire = home.rotation.rotate(new Vec3(0, 0, 1));
      var generated = WeldingMission.generate(cell.weldment(), new WeldmentSeams(group, new Diagnostics()),
        {diameterMm: cell.robot.feeder.wireDiameterMm, depositionEfficiency: cell.robot.feeder.depositionEfficiency,
          maxSpeedMPerMin: cell.robot.feeder.maxSpeedMPerMin}, metal, scene.metresPerUnit,
        {position: new Vector(home.translation.x / scene.metresPerUnit, home.translation.y / scene.metresPerUnit, home.translation.z / scene.metresPerUnit),
          wire: new Vector(wire.x, wire.y, wire.z)});
      var q = ready.copy();
      var selected = context(station);
      var where = Transform3.fromPose2(station.pose).inverse().compose(work);
      for (step in generated.require()) {
        var weld:materia.project.SceneArtifact.SceneArtifactWeld = cast step.weld;
        for (pass in weld.passes) q = selected.planner.plan(WeldScenePlan.pass(weld, pass, where), q).endJoints;
      }
      result.push({kind: "goTo", pose: {x: station.pose.x, y: station.pose.y, yaw: station.pose.yaw}});
      var tools:Array<materia.project.SceneArtifact.SceneArtifactRobotTool> = cast scene.robotTools;
      result.push({kind: "findWork", at: tools[0].contact, contactWork: {
        frame: cell.weldment().reference,
        nominal: {x: work.translation.x, y: work.translation.y, z: work.translation.z,
          qx: work.rotation.x, qy: work.rotation.y, qz: work.rotation.z, qw: work.rotation.w},
        translation: [0.02, 0.02, 0.0], rotation: [0.0, 0.0, 2 * Math.PI / 180],
        measurementError: 0.00001, contactOffset: processkit.tool.WeldArcModel.TOUCH_TOLERANCE, faces: probeFaces}});
      for (step in generated.steps) result.push(step);
      // Stow before base motion. Search in bounded joint space; every edge is compiler-checked.
      var lower = [for (joint in 0...arm.group.count()) arm.group.limitsOf(joint).lower];
      var upper = [for (joint in 0...arm.group.count()) arm.group.limitsOf(joint).upper];
      var stow = JointRoute.plan(q, ready, lower, upper, (from, to) -> airMove(station, from, to));
      if (stow.length < 2) throw "Mobile station has no checked ready-posture stow";
      // The runtime replans this semantic stow from the measured post-weld arm
      // pose and current cell geometry; these nominal edges only prove station feasibility.
      result.push({kind: "stow", at: tools[0].contact});
    }
    Sys.println('Mobile weld mission: ${chosen.stations.length} stations, ${seams.length} CAD seams, ${chosen.travelCost} m route');
    return result;
  }
}
