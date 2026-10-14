import haxe.io.Bytes;
import haxeon.wire.JsonWire;
import materia.assembly.AssemblyCodec;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyJointStop;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyDefinition.AssemblyStateRecord;
import materia.assembly.AssemblyDefinition.AssemblyHydraulicDrive;
import materia.assembly.AssemblyDefinitionFlattener;
import sys.io.File;
import materia.assembly.AssemblyBodies;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyFrames;
import materia.project.Appearance.Appearances;
import materia.project.MaterialLibrary;
import materia.project.MaterialDef;
import materia.project.SceneArtifact;
import materia.project.ScrewEntrySearchBounds;
import materia.units.LengthUnit;

class ProjectKitTests {
  static var assertions = 0;
  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw message;
  }
  static function near(actual:Float, expected:Float, message:String):Void
    check(Math.abs(actual - expected) < 1e-8, message);
  static function rejects(action:Void->Void, message:String):Void {
    var threw = false;
    try action() catch (_:Dynamic) threw = true;
    check(threw, message);
  }

  /** The message of the error `action` throws, or empty when it does not throw. */
  static function refusal(action:Void->Void):String {
    try action() catch (error:Dynamic) return Std.string(error);
    return "";
  }

  /** Force-search commissioning data is explicit, sensor-bound and stable before it reaches a mission. */
  static function screwEntrySearchProfile():Void {
    var profile:materia.project.SceneArtifact.SceneArtifactScrewEntrySearch = {
      sensor: "tool/force-torque", maximumAgeSeconds: 0.02, contactForceN: 5, contactThresholdN: 2,
      confirmationSamples: 2, virtualMass: [80, 80, 80, 1, 1, 1], damping: [4000, 4000, 4000, 20, 20, 20],
      maximumLinearSpeed: 0.004, maximumAngularSpeed: 0.0008, maximumStepSeconds: 0.01, approachSpeed: 0.002,
      maximumForceN: 25, maximumTorqueNm: 2, maximumTravelMeters: 0.045, maximumTravelRadians: 0.06,
      maximumRunSeconds: 60, regionHalfUMeters: 0.0021, regionHalfVMeters: 0.0021, spiralRadiusMeters: 0.0016,
      spiralPitchMeters: 0.0004, spiralSpeed: 0.002, spiralTrackingGainPerSecond: 10,
      spiralTrackingToleranceMeters: 0.00015, entryAdvanceMeters: 0.0003,
      stallSeconds: 5};
    var sensor:materia.project.SceneArtifact.SceneArtifactRobotSensor = {kind: "force_torque",
      id: "tool/force-torque", mount: {occurrence: "tool", connector: "tcp"}};
    var tool:materia.project.SceneArtifact.SceneArtifactRobotTool = {kind: "screwdriver", members: ["tool"],
      contact: {occurrence: "tool", connector: "tcp"}, channel: "tool/start",
      screwdriver: {bit: {kind: "hex", size: "5"}, bitMember: "tool", directionChannel: "tool/direction",
        programChannel: "tool/program", vacuumChannel: "tool/vacuum", feedJoint: "tool/feed",
        spindleJoint: "tool/spindle", strokeEncoder: "tool/stroke", maxTorque: 20,
        maxSpeed: 1, strokeMm: 20, springRate: 10}};
    var data:materia.project.SceneArtifact.SceneArtifactData = {metresPerUnit: 0.001,
      lengthUnit: "mm", parts: [], robotTools: [tool], robotSensors: [sensor]};
    function accepted(hasScrews:Bool, approachMeters:Float = 0.04):Bool {
      try {
        @:privateAccess SceneArtifact.validateScrewEntrySearch(profile, hasScrews, approachMeters, data,
          function(detail:String):Void throw detail);
        return true;
      } catch (_:Dynamic) return false;
    }
    var spiral = ScrewEntrySearchBounds.spiralLengthMeters(0.0016, 0.0004);
    check(Math.abs(spiral - Math.PI * 0.0016 * 0.0016 / 0.0004) < 0.0004,
      "an Archimedean spiral's length is close to the area it covers over its pitch");
    check(Math.abs(ScrewEntrySearchBounds.minimumRunSeconds(0.04, 0.0003, 0.002, 0.0016, 0.0004, 0.002) -
      (20.15 + spiral / 0.002)) < 1e-9, "the run bound covers the guarded descent and one full spiral");
    check(accepted(true), "a bounded force-search profile matches one mounted screwdriver sensor");
    check(!accepted(false), "force-search profile requires a screw step");
    check(!accepted(true, 0.045), "force-search travel covers the authored approach, entry advance and region");
    profile.maximumRunSeconds = 30;
    check(!accepted(true), "force-search time covers the guarded descent and one full spiral");
    profile.maximumRunSeconds = 32;
    check(accepted(true), "a run bound just above the descent and spiral is accepted");
    profile.maximumRunSeconds = 60;
    profile.maximumAngularSpeed = 0.002;
    check(!accepted(true), "angular speed over the run cannot exceed the commissioned angular travel bound");
    profile.maximumAngularSpeed = 0.0008;
    profile.approachSpeed = 0.005;
    check(!accepted(true), "the guarded descent cannot exceed the linear speed bound");
    profile.approachSpeed = 0.002;
    profile.spiralSpeed = 0.005;
    check(!accepted(true), "spiral speed must fit the linear speed bound");
    profile.spiralSpeed = 0.002;
    sensor.id = "other-sensor";
    check(!accepted(true), "force-search profile rejects a missing named sensor");
    sensor.id = profile.sensor;
    sensor.mount.occurrence = "other-tool";
    check(!accepted(true), "force-search profile rejects a sensor mounted off the screwdriver");
    sensor.mount.occurrence = "tool";
    profile.maximumForceN = 4;
    check(!accepted(true), "the regulated contact force stays below the hard force bound");
    profile.maximumForceN = 25;
    profile.contactThresholdN = 5;
    check(!accepted(true), "contact is confirmed below the force the regulation holds");
    profile.contactThresholdN = 2;
    profile.maximumStepSeconds = 0.1;
    check(!accepted(true), "force-search profile rejects an unstable admittance step");
    profile.maximumStepSeconds = 0.01;
    profile.spiralRadiusMeters = 0.002;
    check(!accepted(true), "the spiral and its tracking tolerance fit the registered region");
    profile.spiralRadiusMeters = 0.0016;
    profile.spiralPitchMeters = 0.002;
    check(!accepted(true), "the spiral pitch is below its radius");
    profile.spiralPitchMeters = 0.0004;
    check(accepted(true), "the restored profile is accepted again");
    var engagement:materia.project.SceneArtifact.SceneArtifactScrewEngagement = {
      mode: materia.project.SceneArtifact.SceneArtifactScrewEngagementMode.PhysicalThread,
      maximumAxialForceN: 25, threadStiffnessNPerM: 1.0e7};
    var mission:materia.project.SceneArtifact.SceneArtifactMission = {
      steps: [{kind: "screw"}], screwEntrySearch: profile, screwEngagement: engagement};
    var serialized = @:privateAccess SceneArtifact.encodeMission(mission);
    var decoded:materia.project.SceneArtifact.SceneArtifactMission = @:privateAccess
      SceneArtifact.decodeMission(haxe.Json.parse(serialized));
    check(decoded.screwEntrySearch != null && decoded.screwEntrySearch.sensor == profile.sensor &&
      decoded.screwEntrySearch.contactForceN == 5 && decoded.screwEntrySearch.spiralPitchMeters == 0.0004 &&
      decoded.screwEntrySearch.entryAdvanceMeters == profile.entryAdvanceMeters &&
      decoded.screwEntrySearch.stallSeconds == profile.stallSeconds &&
      decoded.screwEntrySearch.damping[2] == 4000 && decoded.screwEntrySearch.confirmationSamples == 2 &&
      decoded.screwEngagement != null && decoded.screwEngagement.mode == engagement.mode &&
      decoded.screwEngagement.maximumAxialForceN == engagement.maximumAxialForceN &&
      decoded.screwEngagement.threadStiffnessNPerM == 1.0e7,
      "mission force-search and physical-thread selections survive their independent JSON fields");
    var toolChanger:materia.project.SceneArtifact.SceneArtifactToolChanger = {
      master: {occurrence: "robot/master", connector: "tool"}, lockOutput: "changer.lock",
      lockedInput: "changer.locked", identityInputs: ["changer.id0", "changer.id1"],
      initialToolId: "gripper", maximumObservationAgeSeconds: 0.1,
      tools: [
        {id: "gripper", channel: "grip.close", occurrence: "robot/gripper", connector: "mount",
          dock: {occurrence: "dock/gripper", connector: "seat"}, identityCode: 1},
        {id: "screwdriver", channel: "driver.start", occurrence: "robot/driver", connector: "mount",
          dock: {occurrence: "dock/driver", connector: "seat"}, identityCode: 2}
      ]};
    var changerMission:materia.project.SceneArtifact.SceneArtifactMission = {
      steps: [{kind: "changeTool", targetToolId: "screwdriver"}], toolChanger: toolChanger};
    var changerJson = @:privateAccess SceneArtifact.encodeMission(changerMission);
    var changerDecoded:materia.project.SceneArtifact.SceneArtifactMission = @:privateAccess
      SceneArtifact.decodeMission(haxe.Json.parse(changerJson));
    check(changerDecoded.toolChanger != null && changerDecoded.toolChanger.initialToolId == "gripper" &&
      changerDecoded.toolChanger.identityInputs[1] == "changer.id1" &&
      changerDecoded.toolChanger.tools[1].identityCode == 2 &&
      changerDecoded.steps[0].targetToolId == "screwdriver",
      "tool changer CAD interfaces, measured identity and change step round trip independently");
    rejects(() -> @:privateAccess SceneArtifact.decodeMission(haxe.Json.parse(
      StringTools.replace(changerJson, "\"identityCode\":2", "\"identityCode\":2.5"))),
      "tool changer decoder rejects fractional physical identity codes");
    var noThreadForce:materia.project.SceneArtifact.SceneArtifactScrewEngagement = {
      mode: materia.project.SceneArtifact.SceneArtifactScrewEngagementMode.PhysicalThread};
    rejects(() -> @:privateAccess SceneArtifact.validateScrewEngagement(noThreadForce, true,
      function(detail:String):Void throw detail), "Physical thread mode requires an explicit force cap");
    engagement.maximumAxialForceN = 0;
    rejects(() -> @:privateAccess SceneArtifact.validateScrewEngagement(engagement, true,
      function(detail:String):Void throw detail), "Physical thread mode rejects a non-positive force cap");
    engagement.maximumAxialForceN = 25;
    engagement.threadStiffnessNPerM = null;
    rejects(() -> @:privateAccess SceneArtifact.validateScrewEngagement(engagement, true,
      function(detail:String):Void throw detail), "Physical thread mode requires an explicit thread stiffness");
    engagement.threadStiffnessNPerM = 1.0e7;
    rejects(() -> @:privateAccess SceneArtifact.validateScrewEngagement(engagement, false,
      function(detail:String):Void throw detail), "Physical thread mode requires a screw mission");
    var processWithForce:materia.project.SceneArtifact.SceneArtifactScrewEngagement = {
      mode: materia.project.SceneArtifact.SceneArtifactScrewEngagementMode.ProcessModel,
      maximumAxialForceN: 25};
    rejects(() -> @:privateAccess SceneArtifact.validateScrewEngagement(processWithForce, true,
      function(detail:String):Void throw detail), "Process-model mode cannot declare a physical thread force cap");
  }

  static function definition():AssemblyDefinition {
    var frame = AssemblyFrames.identity();
    return {schemaVersion: AssemblyDefinitionCodec.VERSION, id: "fixture", lengthUnit: "mm",
      definitions: [{id: "body", connectors: [{name: "pin", frame: frame}]}],
      occurrences: [{id: "base", definition: "body", initialPose: frame},
        {id: "arm", definition: "body", initialPose: frame}],
      joints: [{id: "hinge", type: AssemblyJointType.Revolute, role: AssemblyJointRole.Tree,
        parent: "base", parentConnector: "pin", child: "arm", childConnector: "pin",
        axis: {x: 0.0, y: 0.0, z: 1.0},
        limits: {lower: -1.0, upper: 1.0, velocity: null, effort: null}, defaultValue: 0.0}]};
  }

  static function assembly():Void {
    var frame = AssemblyFrames.identity();
    var model = definition();
    var decoded = AssemblyDefinitionCodec.decode(AssemblyDefinitionCodec.encode(model));
    check(decoded.joints[0].role == AssemblyJointRole.Tree, "tree role round trip");
    var response:AssemblyJointStop = {timeConstantSec: 0.001, dampingRatio: 1.0};
    model.joints[0].stop = response;
    var stopped = AssemblyDefinitionCodec.decode(AssemblyDefinitionCodec.encode(model));
    var stop = stopped.joints[0].stop;
    if (stop == null) throw "Saved assembly lost its mechanical end stop";
    near(stop.timeConstantSec, 0.001, "end-stop response time round trip");
    near(stop.dampingRatio, 1, "end-stop damping round trip");
    var flattenedStop = AssemblyDefinitionFlattener.flatten(stopped).joints[0].stop;
    check(flattenedStop != null && flattenedStop.timeConstantSec == stop.timeConstantSec,
      "flattening retains the authored mechanical end stop");
    for (value in [0.0, -1.0, Math.NaN]) {
      var invalid = definition();
      var badResponse:AssemblyJointStop = {timeConstantSec: value, dampingRatio: 1.0};
      invalid.joints[0].stop = badResponse;
      rejects(function() AssemblyDefinitionCodec.encode(invalid), "invalid mechanical stop response");
    }
    var undamped = definition();
    var noDamping:AssemblyJointStop = {timeConstantSec: 0.001, dampingRatio: 0.0};
    undamped.joints[0].stop = noDamping;
    rejects(function() AssemblyDefinitionCodec.encode(undamped), "mechanical stop needs positive damping");
    var fixedStop = definition(); fixedStop.joints[0].type = AssemblyJointType.Fixed;
    fixedStop.joints[0].stop = response;
    rejects(function() AssemblyDefinitionCodec.encode(fixedStop), "fixed joint cannot have moving end stops");
    model.joints[0].passive = true;
    var passive = AssemblyDefinitionCodec.decode(AssemblyDefinitionCodec.encode(model));
    check(passive.joints[0].passive == true, "passive ownership round trip");
    model.joints[0].driven = true;
    rejects(function() AssemblyDefinitionCodec.encode(model), "passive joint cannot be driven");
    model.joints[0].driven = false;
    model.joints[0].type = AssemblyJointType.Fixed;
    rejects(function() AssemblyDefinitionCodec.encode(model), "fixed joint cannot be passive");
    model = definition();
    model.joints[0].role = AssemblyJointRole.Closure;
    check(AssemblyDefinitionCodec.decode(AssemblyDefinitionCodec.encode(model)).joints[0].role ==
      AssemblyJointRole.Closure, "closure role round trip");
    model = definition();
    var state:AssemblyStateRecord = {schemaVersion: AssemblyDefinitionCodec.VERSION,
      definition: model.id, jointCoordinates: [{joint: "hinge", value: 0.25}],
      rootPoses: [{occurrence: "base", pose: AssemblyFrames.translation(1, 2, 3)}]};
    var restored = AssemblyDefinitionCodec.decodeState(model,
      AssemblyDefinitionCodec.encodeState(model, state));
    near(restored.jointCoordinates[0].value, 0.25, "state coordinate round trip");
    near(restored.rootPoses[0].pose.z, 3, "state root round trip");
    state.jointCoordinates[0].value = 2;
    rejects(function() AssemblyDefinitionCodec.encodeState(model, state),
      "state outside joint limits");
    model.schemaVersion = 99;
    rejects(function() AssemblyDefinitionCodec.encode(model), "bad definition version");
    model = definition(); model.lengthUnit = "yards";
    rejects(function() AssemblyDefinitionCodec.encode(model), "invalid definition unit");
    model = definition(); model.occurrences[1].id = "base";
    rejects(function() AssemblyDefinitionCodec.encode(model), "duplicate occurrence ID");
    model = definition(); model.joints[0].axis.z = 0;
    rejects(function() AssemblyDefinitionCodec.encode(model), "zero joint axis");
    model = definition(); model.joints[0].limits.lower = 2;
    rejects(function() AssemblyDefinitionCodec.encode(model), "inverted joint limits");
    model = definition();
    model.schemaVersion = AssemblyDefinitionCodec.VERSION;
    model.joints[0].type = AssemblyJointType.Prismatic;
    model.actuators = [{id: "air", joint: "hinge", maxEffort: 100.0, maxRate: 0.1,
      pneumatic: {bore: 25.0, rod: 10.0, stroke: 20.0, ratedSpeed: 100.0,
        pressurePa: 600000.0, channelA: "valve/coilA", normallyToA: true, extendSign: 1.0}}];
    rejects(function() AssemblyDefinitionCodec.encode(model),
      "a pneumatic block without a pneumatic drive kind is invalid");
  }

  static function homingDependencies():Void {
    var model = definition();
    model.joints[0].type = AssemblyJointType.Prismatic;
    model.occurrences.push({id: "tool", definition: "body", initialPose: AssemblyFrames.identity()});
    model.joints.push({id: "slide", type: AssemblyJointType.Prismatic, role: AssemblyJointRole.Tree,
      parent: "arm", parentConnector: "pin", child: "tool", childConnector: "pin", axis: {x: 1.0, y: 0.0, z: 0.0},
      limits: {lower: -1.0, upper: 1.0, velocity: null, effort: null}, defaultValue: 0.0});
    model.switches = [
      {id: "homeLift", joint: "hinge", part: "base", connector: "pin", trigger: "arm", triggerConnector: "pin",
        role: "home", side: -1, trip: -0.1, hysteresis: 0.01, repeatability: 0.001, seed: 1},
      {id: "homeSlide", joint: "slide", part: "arm", connector: "pin", trigger: "tool", triggerConnector: "pin",
        role: "home", side: -1, trip: -0.1, hysteresis: 0.01, repeatability: 0.001, seed: 1, homeAfter: ["hinge"]}];
    var restored = AssemblyDefinitionCodec.decode(AssemblyDefinitionCodec.encode(model));
    check(restored.switches[1].homeAfter[0] == "hinge", "home dependency survives assembly serialization");
    var scoped = materia.assembly.AssemblyDefinitionFlattener.copySwitch(restored.switches[1], id -> "cell/" + id);
    check(scoped.joint == "cell/slide" && scoped.homeAfter[0] == "cell/hinge", "included machines scope home dependencies with their joints");
    model.switches[0].homeAfter = ["slide"];
    rejects(() -> AssemblyDefinitionCodec.encode(model), "cyclic home dependencies are invalid design data");
    model.switches[0].homeAfter = ["missing"];
    rejects(() -> AssemblyDefinitionCodec.encode(model), "home prerequisites must have physical home switches");
    model.switches[0].homeAfter = new Array<String>();
    model.switches[1].homeAfter = ["hinge", "hinge"];
    rejects(() -> AssemblyDefinitionCodec.encode(model), "duplicate home prerequisites are invalid");
  }

  static function frames():Void {
    var turn = AssemblyFrames.axisMotion(AssemblyJointType.Revolute,
      {x: 0, y: 0, z: 1}, Math.PI / 2);
    near(turn.qz, Math.sin(Math.PI / 4), "quaternion z component");
    near(turn.qw, Math.cos(Math.PI / 4), "quaternion w component");
    near(turn.qx, 0, "quaternion x component");
    near(turn.qy, 0, "quaternion y component");
    var placed = AssemblyFrames.compose(AssemblyFrames.translation(2, 3, 4), turn);
    var point = AssemblyFrames.transformPoint(placed, 1, 0, 0);
    near(point.x, 2, "composed x");
    near(point.y, 4, "composed y");
    var identity = AssemblyFrames.compose(placed, AssemblyFrames.inverse(placed));
    near(identity.x, 0, "inverse x");
    near(identity.y, 0, "inverse y");
    near(identity.qw, 1, "inverse rotation");
  }

  static function hydraulicDefinition():AssemblyDefinition {
    var model = definition();
    model.id = "hydraulic-reference";
    model.joints[0].type = AssemblyJointType.Prismatic;
    model.joints[0].limits.lower = 0; model.joints[0].limits.upper = 100;
    model.actuators = [{id: "ram", joint: "hinge", drive: "hydraulic", maxEffort: 201061.92982974676,
      maxRate: 80, hydraulic: {bore: 160, rod: 100, stroke: 100, approachSpeed: 80,
        workingSpeed: 8, returnSpeed: 70, changePoint: 68.5, pressurePa: 10e6,
        channelA: "valve/coilA", channelB: "valve/coilB", extendSign: 1}}];
    return model;
  }

  static function rejectHydraulic(change:AssemblyHydraulicDrive->Void, message:String):Void {
    var model = hydraulicDefinition();
    var hydraulic = model.actuators[0].hydraulic;
    if (hydraulic == null) throw "Missing hydraulic fixture data";
    change(hydraulic);
    rejects(function() AssemblyDefinitionCodec.encode(model), message);
  }

  static function hydraulic():Void {
    var model = hydraulicDefinition();
    var encoded = AssemblyDefinitionCodec.encode(model);
    var restored = AssemblyDefinitionCodec.decode(encoded);
    var h = restored.actuators[0].hydraulic;
    if (h == null) throw "Hydraulic round trip lost its typed drive";
    check(restored.schemaVersion == AssemblyDefinitionCodec.VERSION && restored.actuators[0].drive == "hydraulic", "current hydraulic format");
    near(h.bore, 160, "hydraulic bore round trip"); near(h.rod, 100, "hydraulic rod round trip");
    near(h.stroke, 100, "hydraulic stroke round trip"); near(h.pressurePa, 10e6, "hydraulic pressure round trip");
    near(h.approachSpeed, 80, "hydraulic approach speed round trip");
    near(h.workingSpeed, 8, "hydraulic working speed round trip");
    near(h.returnSpeed, 70, "hydraulic return speed round trip");
    near(h.changePoint, 68.5, "hydraulic change point round trip");
    near(h.extendSign, 1, "hydraulic extension sign round trip");
    var copy = AssemblyDefinitionFlattener.copyActuator(restored.actuators[0], "press/ram", "press/hinge", "press/");
    var copied = copy.hydraulic; if (copied == null) throw "Hydraulic copy lost its drive";
    check(copied.channelA == "press/valve/coilA" && copied.channelB == "press/valve/coilB",
      "hydraulic channels follow nested member scope");
    copied.pressurePa = 1;
    near(h.pressurePa, 10e6, "hydraulic copies own their data");
    var nested:AssemblyDefinition = {schemaVersion: AssemblyDefinitionCodec.VERSION, id: "cell", lengthUnit: "mm",
      definitions: [], occurrences: [{id: "press", definition: model.id, assembly: model.id,
        initialPose: AssemblyFrames.identity()}], joints: [], assemblies: [{id: model.id,
        definitions: model.definitions, occurrences: model.occurrences, joints: model.joints,
        actuators: model.actuators}]};
    var flat = AssemblyDefinitionFlattener.flatten(AssemblyDefinitionCodec.decode(AssemblyDefinitionCodec.encode(nested)));
    check(flat.actuators[0].joint == "press/hinge", "nested hydraulic joint scope");
    var flatDrive = flat.actuators[0].hydraulic; if (flatDrive == null) throw "Flattening lost hydraulic data";
    check(flatDrive.channelA == "press/valve/coilA" && flatDrive.channelB == "press/valve/coilB",
      "nested hydraulic valve channels scope");
    var fixture = "fixtures/hydraulic-assembly.json";
    if (Sys.getEnv("PROJECTKIT_WRITE_FIXTURES") == "1") File.saveContent(fixture, encoded + "\n");
    check(AssemblyDefinitionCodec.encode(AssemblyDefinitionCodec.decode(File.getContent(fixture))) == encoded,
      "hydraulic fixture is generated by the current codec");
    for (version in [3, 4, 5, 6, 7, 8, 9, 10]) {
      var old = hydraulicDefinition(); old.schemaVersion = version;
      rejects(function() AssemblyDefinitionCodec.encode(old), "old assembly definition rejected");
    }
    rejectHydraulic(h -> h.bore = 100, "hydraulic rod consumes its bore");
    rejectHydraulic(h -> h.workingSpeed = 0, "zero hydraulic working speed");
    rejectHydraulic(h -> h.changePoint = 101, "hydraulic change point past stroke");
    rejectHydraulic(h -> h.changePoint = -1, "negative hydraulic change point");
    rejectHydraulic(h -> h.pressurePa = -1, "negative hydraulic pressure");
    rejectHydraulic(h -> h.returnSpeed = Math.NaN, "nonfinite hydraulic return speed");
    rejectHydraulic(h -> h.channelB = h.channelA, "hydraulic coils share a channel");
    rejectHydraulic(h -> h.extendSign = 0, "invalid hydraulic direction");
    var passiveRam = hydraulicDefinition(); passiveRam.joints[0].passive = true;
    rejects(function() AssemblyDefinitionCodec.encode(passiveRam), "passive ram cannot have a hydraulic actuator");
    var rotating = hydraulicDefinition(); rotating.joints[0].type = AssemblyJointType.Revolute;
    rejects(function() AssemblyDefinitionCodec.encode(rotating), "hydraulic joint cannot rotate");
    var wrongKind = hydraulicDefinition(); wrongKind.actuators[0].drive = "servo";
    rejects(function() AssemblyDefinitionCodec.encode(wrongKind), "hydraulic data cannot describe an electric drive");
    var untyped = hydraulicDefinition(); untyped.actuators[0].drive = null;
    rejects(function() AssemblyDefinitionCodec.encode(untyped), "hydraulic data needs a declared drive kind");
    var shortGuide = hydraulicDefinition(); shortGuide.joints[0].limits.upper = 99;
    rejects(function() AssemblyDefinitionCodec.encode(shortGuide), "hydraulic stroke differs from guide travel");
    var lowCeiling = hydraulicDefinition(); lowCeiling.actuators[0].maxRate = 79;
    rejects(function() AssemblyDefinitionCodec.encode(lowCeiling), "hydraulic phase speed exceeds ceiling");
  }

  static function sceneData():materia.project.SceneArtifact.SceneArtifactData {
    var vertices = Bytes.alloc(96), normals = Bytes.alloc(96), indices = Bytes.alloc(48);
    vertices.setDouble(24, 1); vertices.setDouble(56, 1); vertices.setDouble(88, 1);
    var corners = [0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3];
    for (i in 0...corners.length) indices.setInt32(i * 4, corners[i]);
    var data:materia.project.SceneArtifact.SceneArtifactData = {lengthUnit: "mm",
      metresPerUnit: 0.001, parts: [{id: "part", name: "Part", red: 0.2,
        green: 0.3, blue: 0.4, appearance: Appearances.machinedSteel(),
        vertexCount: 4, indexCount: 12, vertices: vertices, normals: normals,
        indices: indices, faceRanges: [], faceDescriptors: "[{\"index\":0}]",
        physicalMassKg: 2.4, physicalMassDeclared: true,
        physicalInertiaKgM2: [0.2, 0, 0, 0, 0.3, 0, 0, 0, 0.4]}], recipeDocument: "cube", recipeDiagnostics: ["ok"]};
    return data;
  }

  static function scene():Void {
    var data = sceneData();
    var encoded = SceneArtifact.encode(data);
    check(encoded.getInt32(4) == SceneArtifact.VERSION, "current scene version");
    var legacy = Bytes.alloc(encoded.length - 4);
    legacy.blit(0, encoded, 0, legacy.length);
    var rejectedVersion = SceneArtifact.VERSION - 1;
    legacy.setInt32(4, rejectedVersion);
    var oldRefusal = refusal(() -> SceneArtifact.decodeView(legacy));
    check(oldRefusal.indexOf('schema v$rejectedVersion is unsupported') >= 0 &&
      oldRefusal.indexOf('expected v${SceneArtifact.VERSION}') >= 0,
      "clean scene format break rejects the previous schema and names the current version");
    data.parts[0].id = "body";
    data.assemblyDefinition = definition();
    data.robotSensors = [{kind: "tool_contact", id: "arm/tool/tip-contact",
      mount: {occurrence: "base", connector: "pin"}}];
    var contactSensor = SceneArtifact.decode(SceneArtifact.encode(data)).robotSensors[0];
    check(contactSensor.kind == "tool_contact" && contactSensor.id == "arm/tool/tip-contact" &&
      contactSensor.mount.occurrence == "base" && contactSensor.rayCount == null && contactSensor.maxRange == null,
      "A mounted digital contact source round trips without LiDAR scan fields");
    data.robotSensors[0].updateRate = 10;
    rejects(() -> SceneArtifact.encode(data), "Digital contact sources cannot carry LiDAR scan settings");
    data.parts[0].id = "part"; data.assemblyDefinition = null; data.robotSensors = null;
    var thread:materia.project.ThreadRenderData = {faceName: "bore:cyl.side", faceIndex: 7,
      spec: {designation: "M6", majorDiameter: 6, pitch: 1, starts: 2,
        hand: materia.project.ThreadSpecData.ThreadHand.LeftHand, internal: true,
        toleranceClass: "6H", profile: materia.project.ThreadSpecData.ThreadProfile.Metric},
      origin: [4, 5, 6], xDirection: [1, 0, 0], direction: [0, 0, -1], length: 20,
      ringAtStart: false, ringAtEnd: true};
    data.parts[0].faceRanges = [{faceIndex: 7, firstIndex: 0, indexCount: 12}];
    data.parts[0].threads = [thread];
    var threaded = SceneArtifact.decodeView(SceneArtifact.encode(data)).parts[0].threads;
    check(threaded != null && threaded.length == 1 && threaded[0].faceName == thread.faceName
      && threaded[0].faceIndex == 7 && threaded[0].length == 20 && threaded[0].origin[2] == 6
      && threaded[0].direction[2] == -1 && threaded[0].ringAtEnd && !threaded[0].ringAtStart,
      "named thread interval and frame round trip");
    check(threaded[0].spec.starts == 2 && threaded[0].spec.pitch == 1 && threaded[0].spec.hand == thread.spec.hand,
      "nominal thread facts round trip without a kernel");
    thread.faceIndex = 8;
    rejects(function() SceneArtifact.encode(data), "thread must reference a base face range");
    thread.faceIndex = 7; thread.direction = [1, 0, 0];
    rejects(function() SceneArtifact.encode(data), "thread frame must have perpendicular axes");
    thread.direction = [0, 0, -1]; thread.origin[0] = Math.NaN;
    rejects(function() SceneArtifact.encode(data), "thread frame must be finite");
    thread.origin[0] = 4; thread.length = 0;
    rejects(function() SceneArtifact.encode(data), "thread display interval must be positive");
    thread.length = 20; thread.spec.pitch = 0;
    rejects(function() SceneArtifact.encode(data), "nominal thread pitch must be positive");
    thread.spec.pitch = 1;
    SceneScrewTests.namedFace(data, thread);
    data.parts[0].threads = new Array<materia.project.ThreadRenderData>(); data.parts[0].faceRanges.resize(0);
    data.project = {job: "fixture", dynamicParts: ["part"], motions: []};
    var bundled = SceneArtifact.decode(SceneArtifact.encode(data));
    check(bundled.project != null && bundled.project.job == "fixture" && bundled.project.dynamicParts[0] == "part",
      "published project job and dynamic parts round trip");
    data.project.dynamicParts = ["missing"];
    rejects(function() SceneArtifact.encode(data), "project settings reject unknown dynamic parts");
    data.project = null;
    var moving = SceneArtifact.decode(encoded);
    moving.parts[0].id = "body";
    moving.assemblyDefinition = definition();
    var publishedMotion:materia.project.SceneArtifact.SceneArtifactProject = {dynamicParts: ["arm"], motions: [{joint: "hinge", loop: true,
      keys: [{time: 0.0, position: 0.0}, {time: 1.0, position: 0.25}, {time: 2.0, position: 0.0}]}]};
    moving.project = publishedMotion;
    var moved = SceneArtifact.decode(SceneArtifact.encode(moving));
    var movedProject:materia.project.SceneArtifact.SceneArtifactProject = cast moved.project;
    check(movedProject.motions[0].joint == "hinge" && movedProject.motions[0].keys[1].position == 0.25,
      "published joint motion round trips");
    publishedMotion.motions[0].joint = "missing";
    rejects(function() SceneArtifact.encode(moving), "project settings reject unknown joints");
    publishedMotion.motions[0].joint = "hinge";
    publishedMotion.motions[0].keys[2].position = 0.3;
    rejects(function() SceneArtifact.encode(moving), "project settings reject discontinuous loops");
    var restored = SceneArtifact.decode(encoded);
    check(restored.parts[0].id == "part", "scene part round trip");
    check(restored.parts[0].physicalMassKg == 2.4 && restored.parts[0].physicalMassDeclared == true &&
      restored.parts[0].physicalInertiaKgM2[0] == 0.2 && restored.parts[0].physicalInertiaKgM2[8] == 0.4,
      "resolved physical mass and inertia round trip");
    var appearance = restored.parts[0].appearance;
    check(appearance != null && appearance.finish == "machined-steel", "scene appearance round trip");
    check(restored.recipeDocument == "cube", "scene source round trip");
    check(restored.parts[0].faceDescriptors == "[{\"index\":0}]", "scene face descriptors round trip");
    var diagnostics = restored.recipeDiagnostics;
    check(diagnostics != null && diagnostics[0] == "ok", "scene diagnostics round trip");
    var truncated = Bytes.alloc(10);
    truncated.blit(0, encoded, 0, 10);
    rejects(function() SceneArtifact.decode(truncated), "truncated scene");
    var previousVersion = Bytes.alloc(encoded.length); previousVersion.blit(0, encoded, 0, encoded.length);
    previousVersion.setInt32(4, SceneArtifact.VERSION - 1);
    rejects(function() SceneArtifact.decode(previousVersion), "previous scene version is rejected");
    var invalidScale = Bytes.alloc(encoded.length);
    invalidScale.blit(0, encoded, 0, encoded.length);
    invalidScale.setDouble(8, -1);
    rejects(function() SceneArtifact.decode(invalidScale), "invalid scene unit scale");
    var borrowed = SceneArtifact.decodeView(encoded);
    check(borrowed.parts[0].vertices.getDouble(24) == restored.parts[0].vertices.getDouble(24),
      "borrowed mesh matches copied decoding");
    check(borrowed.recipeDocument == restored.recipeDocument &&
      borrowed.parts[0].faceDescriptors == restored.parts[0].faceDescriptors, "borrowed metadata round trip");
    borrowed.parts[0].vertices.setDouble(24, 2);
    check(SceneArtifact.decode(encoded).parts[0].vertices.getDouble(24) == 2,
      "borrowed meshes retain and share source storage");
    check(restored.parts[0].vertices.getDouble(24) == 1, "ordinary decoding owns independent mesh buffers");
    borrowed.parts[0].vertices.setDouble(24, Math.NaN);
    rejects(() -> SceneArtifact.decodeView(encoded), "borrowed decoder rejects nonfinite vertices");
    borrowed.parts[0].vertices.setDouble(24, 1);
    borrowed.parts[0].indices.setInt32(0, -1);
    rejects(() -> SceneArtifact.decodeView(encoded), "borrowed decoder rejects invalid indices");
    borrowed.parts[0].indices.setInt32(0, 0);
    for (length in [0, 3, 7, 15, encoded.length - 1]) {
      var short = Bytes.view(encoded, 0, length);
      rejects(() -> SceneArtifact.decode(short), "copied decoder rejects truncation");
      rejects(() -> SceneArtifact.decodeView(short), "borrowed decoder rejects truncation");
    }
    var original = definition();
    var beforeValidation = AssemblyDefinitionCodec.encode(original);
    AssemblyDefinitionCodec.validate(original);
    AssemblyDefinitionCodec.validateState(original, {schemaVersion: AssemblyDefinitionCodec.VERSION,
      definition: original.id, jointCoordinates: [], rootPoses: []});
    check(AssemblyDefinitionCodec.encode(original) == beforeValidation, "validation leaves its input unchanged");
    check(materia.assembly.AssemblyDefinitionFlattener.flattenView(original) == original,
      "flat read-only view borrows the definition");
    var independent = materia.assembly.AssemblyDefinitionFlattener.flatten(original);
    independent.id = "edited";
    check(original.id == "fixture", "flatten still returns an independent editable copy");
    var nested:AssemblyDefinition = {
      schemaVersion: AssemblyDefinitionCodec.VERSION, id: "nested", lengthUnit: "mm",
      definitions: [], joints: [],
      occurrences: [{id: "cell", definition: "unit", assembly: "unit", initialPose: AssemblyFrames.identity()}],
      assemblies: [{id: "unit", definitions: original.definitions,
        occurrences: original.occurrences, joints: original.joints}]
    };
    var nestedBefore = AssemblyDefinitionCodec.encode(nested);
    var expanded = materia.assembly.AssemblyDefinitionFlattener.flattenView(nested);
    check(expanded != nested && expanded.occurrences[0].id == "cell/base", "nested views still expand members");
    check(AssemblyDefinitionCodec.encode(nested) == nestedBefore, "nested view leaves its source unchanged");
    nested.assemblies[0].joints[0].parent = "missing";
    rejects(() -> AssemblyDefinitionCodec.validate(nested), "nested views still reject missing joint members");

    data.parts.push(data.parts[0]);
    rejects(function() SceneArtifact.encode(data), "duplicate scene part ID");
    data.parts.pop(); data.parts[0].indices.setInt32(0, 99);
    rejects(function() SceneArtifact.encode(data), "out of range mesh index");
    data.parts[0].indices.setInt32(0, 0);

  }

  /** A mobile base names its robot's wheel joints and carries positive dimensions and limits. */
  /** Saved machine metadata stays typed, including nested output events and contact filters. */
  static function machineMetadata():Void {
    var vertices = Bytes.alloc(96), normals = Bytes.alloc(96), indices = Bytes.alloc(48);
    vertices.setDouble(24, 1); vertices.setDouble(56, 1); vertices.setDouble(88, 1);
    var corners = [0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3];
    for (i in 0...corners.length) indices.setInt32(i * 4, corners[i]);
    var surface:materia.project.SceneContactSurface = {friction: [0.3, 0, 0], frictionDimensions: 3,
      contactTimeConstant: 0.002, contactDampingRatio: 1,
      filter: materia.project.SceneContactFilter.PairsAndEnvironment};
    var contacts:Array<materia.project.SceneArtifact.SceneArtifactHullContact> = [{occurrence: "arm", surface: surface}];
    var motion:materia.project.SceneArtifact.SceneArtifactMachineMotion = {belts: [],
      contacts: contacts, program: {axes: ["hinge"], waypoints: [[0.2]], loop: true},
      virtualDevice: true};
    var data:materia.project.SceneArtifact.SceneArtifactData = {lengthUnit: "mm", metresPerUnit: 0.001,
      parts: [{id: "body", name: "Body", red: 0.2, green: 0.3, blue: 0.4,
        vertexCount: 4, indexCount: 12, vertices: vertices, normals: normals, indices: indices, faceRanges: []}],
      assemblyDefinition: definition(), machineMotion: motion};
    var restored = SceneArtifact.decode(SceneArtifact.encode(data)).machineMotion;
    if (restored == null) throw "Machine metadata disappeared";
    var restoredProgram = restored.program, restoredContacts = restored.contacts;
    if (restoredProgram == null || restoredContacts == null) throw "Machine metadata disappeared";
    near(restoredProgram.waypoints[0][0], 0.2, "typed axis waypoint round trip");
    check(restored.virtualDevice == true, "typed virtual axis device round trip");
    check(restoredContacts[0].surface.filter == materia.project.SceneContactFilter.PairsAndEnvironment,
      "typed hull contact filter round trip");
    near(restoredContacts[0].surface.contactTimeConstant, 0.002, "typed hull compliance round trip");
    motion.program = null; motion.virtualDevice = false;
    var digital:materia.project.SceneArtifact.SceneArtifactDigitalProgram = {durationSec: 2, loop: false, events: [
      {timeSec: 0.1, channel: "valve/A", value: true}, {timeSec: 1.5, channel: "valve/A", value: false}]};
    motion.digitalProgram = digital;
    var saved = SceneArtifact.decode(SceneArtifact.encode(data)).machineMotion;
    if (saved == null) throw "Digital program disappeared";
    var savedProgram = saved.digitalProgram;
    if (savedProgram == null) throw "Digital program disappeared";
    check(savedProgram.events[1].channel == "valve/A" && !savedProgram.events[1].value,
      "typed digital terminal output round trip");
    near(savedProgram.durationSec, 2, "typed digital duration round trip");
    contacts.push(contacts[0]);
    rejects(function() SceneArtifact.encode(data), "duplicate hull contact occurrence");
    contacts.pop(); contacts[0].occurrence = "absent";
    rejects(function() SceneArtifact.encode(data), "unknown hull contact occurrence");
    contacts[0].occurrence = "arm"; surface.frictionDimensions = 2;
    rejects(function() SceneArtifact.encode(data), "unsupported contact friction dimensions");
    surface.frictionDimensions = 3; surface.contactTimeConstant = -1;
    rejects(function() SceneArtifact.encode(data), "negative contact compliance");
    surface.contactTimeConstant = 0.002;
    digital.events[1].timeSec = 0.05;
    rejects(function() SceneArtifact.encode(data), "digital events out of order");
    digital.events[1].timeSec = 2;
    rejects(function() SceneArtifact.encode(data), "digital terminal event at program end");
    digital.events[1].timeSec = 0.1;
    rejects(function() SceneArtifact.encode(data), "digital output assigned twice at one time");
    digital.events[1].timeSec = 1.5;
    motion.program = {axes: ["hinge"], waypoints: [[0.2]], loop: false};
    rejects(function() SceneArtifact.encode(data), "axis and digital program execution conflict");
    motion.program = null; motion.digitalProgram = null;
    var pressAssembly = definition(), frame = AssemblyFrames.identity();
    pressAssembly.joints[0].type = AssemblyJointType.Prismatic;
    pressAssembly.joints[0].limits.upper = 300;
    for (id in ["depth", "X"]) {
      pressAssembly.occurrences.push({id: id, definition: "body", initialPose: frame});
      pressAssembly.joints.push({id: id, type: AssemblyJointType.Prismatic, role: AssemblyJointRole.Tree,
        parent: "base", parentConnector: "pin", child: id, childConnector: "pin",
        axis: {x: 0.0, y: 0.0, z: 1.0},
        limits: {lower: 0.0, upper: 300.0, velocity: null, effort: null}, defaultValue: 0.0});
    }
    pressAssembly.sensors = [for (id in ["top", "left", "right"])
      {id: id, kind: "joint_switch", joint: "hinge", windowLower: -0.1, windowUpper: 0.1, hysteresis: 0.001}];
    data.assemblyDefinition = pressAssembly;
    var press:materia.project.SceneArtifact.SceneArtifactPressBrake = {
      ramJoint: "hinge", depthAxis: "depth", gaugeX: "X", topSwitch: "top", fingerSwitches: ["left", "right"],
      steps: [{gaugeXmm: 150, gaugeRmm: 0, clampTravelMm: 78.5, depthTravelMm: 83, withdrawalMm: 20}],
      positionToleranceMm: 0.05, requestTimeoutSeconds: 20, releaseTravelMm: 0.5, releaseClosingForceN: 5000};
    motion.pressBrake = press;
    var pressSaved = SceneArtifact.decode(SceneArtifact.encode(data)).machineMotion;
    if (pressSaved == null || pressSaved.pressBrake == null) throw "Press controller disappeared";
    near(pressSaved.pressBrake.steps[0].depthTravelMm, 83, "typed press depth round trip");
    press.fingerSwitches[1] = "left";
    rejects(function() SceneArtifact.encode(data), "press needs independent registration switches");
    press.fingerSwitches[1] = "right"; press.depthAxis = "hinge";
    rejects(function() SceneArtifact.encode(data), "press cannot alias ram and depth ownership");
    press.depthAxis = "depth"; press.steps[0].depthTravelMm = 78;
    rejects(function() SceneArtifact.encode(data), "press depth must follow clamp travel");
    press.steps[0].depthTravelMm = 83; press.steps[0].gaugeRmm = 1;
    rejects(function() SceneArtifact.encode(data), "press cannot program an absent R axis");
    press.steps[0].gaugeRmm = 0; motion.digitalProgram = digital;
    rejects(function() SceneArtifact.encode(data), "press cannot share hydraulic output authority");
    motion.digitalProgram = null; press.controllerId = "absent";
    rejects(function() SceneArtifact.encode(data), "press cannot name an absent cabinet");
    press.controllerId = null; press.steps[0].gaugeXmm = 290;
    rejects(function() SceneArtifact.encode(data), "press withdrawal must fit bounded backgauge travel");
    press.steps[0].gaugeXmm = 150; press.controllerId = "press";
    data.controllers = [{id: "press", kind: "cnc", joints: ["hinge", "depth", "X"],
      sensors: ["top", "left", "right"], channels: []}];
    data.signals = [];
    var cabinetPress = SceneArtifact.decode(SceneArtifact.encode(data)).machineMotion;
    check(cabinetPress != null && cabinetPress.pressBrake != null && cabinetPress.pressBrake.controllerId == "press",
      "press controller ownership round trip");
    data.controllers[0].joints = ["hinge", "depth"];
    data.controllers.push({id: "robot", kind: "robot", joints: ["X"], sensors: [], channels: []});
    rejects(function() SceneArtifact.encode(data), "press cannot drive another cabinet's gauge axis");
  }

  static function mobileBase():Void {
    var vertices = Bytes.alloc(96), normals = Bytes.alloc(96), indices = Bytes.alloc(48);
    vertices.setDouble(24, 1); vertices.setDouble(56, 1); vertices.setDouble(88, 1);
    var corners = [0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3];
    for (i in 0...corners.length) indices.setInt32(i * 4, corners[i]);
    var frame = AssemblyFrames.identity();
    function wheel(id:String, child:String, type:AssemblyJointType):materia.assembly.AssemblyDefinition.KinematicJoint
      return {id: id, type: type, role: AssemblyJointRole.Tree, parent: "chassis", parentConnector: "pin",
        child: child, childConnector: "pin", axis: {x: 0.0, y: 1.0, z: 0.0},
        limits: {lower: null, upper: null, velocity: 10.0, effort: 1.0}, defaultValue: 0.0};
    var robot:AssemblyDefinition = {schemaVersion: AssemblyDefinitionCodec.VERSION, id: "robot",
      lengthUnit: "mm", definitions: [{id: "body", connectors: [{name: "pin", frame: frame}]}],
      occurrences: [for (id in ["chassis", "left", "right", "mast"]) {id: id, definition: "body", initialPose: frame}],
      joints: [wheel("wheel_l", "left", AssemblyJointType.Continuous), wheel("wheel_r", "right", AssemblyJointType.Continuous),
        wheel("lift", "mast", AssemblyJointType.Prismatic)]};
    var base:materia.project.SceneArtifact.SceneArtifactMobileBase = {leftWheel: "wheel_l", rightWheel: "wheel_r",
      wheelRadius: 0.075, trackWidth: 0.3, maxLinearSpeed: 0.8, maxAngularSpeed: 2.0,
      maxLinearAcceleration: 0.5, maxAngularAcceleration: 1.5, footprintLength: 0.6, footprintWidth: 0.44};
    var data:materia.project.SceneArtifact.SceneArtifactData = {lengthUnit: "mm", metresPerUnit: 0.001,
      parts: [{id: "body", name: "body", red: 0.5, green: 0.5, blue: 0.5, vertexCount: 4, indexCount: 12,
        vertices: vertices, normals: normals, indices: indices, faceRanges: []}],
      assemblyDefinition: robot, mobileBase: base};
    var restored = SceneArtifact.decode(SceneArtifact.encode(data)).mobileBase;
    if (restored == null) throw "mobile base round trip lost the base";
    check(restored.leftWheel == "wheel_l" && restored.rightWheel == "wheel_r" && restored.wheelRadius == 0.075 &&
      restored.trackWidth == 0.3 && restored.maxAngularAcceleration == 1.5 && restored.footprintWidth == 0.44,
      "mobile base round trip");
    robot.occurrences.push({id: "battery", definition: "body", initialPose: frame});
    robot.occurrences.push({id: "charger", definition: "body", initialPose: frame});
    base.energy = {initialChargeFraction: 1.0, batteryOccurrence: "battery", nominalVolts: 48, capacityWh: 25000,
      fullVoltage: 54.6, emptyVoltage: 40, cutoffVoltage: 42, internalResistanceOhms: 0.02,
      maxDischargeCurrentA: 250, maxChargePowerW: 5000, chargeEfficiency: 0.95,
      inverterContinuousWatts: 10000,
      inverterEfficiency: 0.92, armSupplyEfficiency: 0.95, driveSupplyEfficiency: 0.95,
      computerSupplyEfficiency: 0.95, armDriveEfficiency: 0.72, driveMotorEfficiency: 0.68,
      driveGearEfficiency: 0.9, computerLoadWatts: 120, chargerOccurrence: "charger",
      chargerPowerWatts: 3000, dockPose: {x: -3.35, y: 0, yaw: Math.PI / 2},
      dockPositionToleranceMeters: 0.015, dockHeadingToleranceRadians: 0.02};
    var energyRestored = SceneArtifact.decode(SceneArtifact.encode(data)).mobileBase;
    if (energyRestored == null || energyRestored.energy == null) throw "mobile battery profile round trip lost data";
    check(energyRestored.energy.batteryOccurrence == "battery" &&
      energyRestored.energy.chargerOccurrence == "charger" && energyRestored.energy.capacityWh == 25000 &&
      energyRestored.energy.inverterContinuousWatts == 10000 && energyRestored.energy.inverterEfficiency == 0.92 &&
      energyRestored.energy.dockPose.yaw == Math.PI / 2 &&
      energyRestored.energy.dockPositionToleranceMeters == 0.015 &&
      energyRestored.energy.dockHeadingToleranceRadians == 0.02 &&
      energyRestored.energy.initialChargeFraction == 1,
      "mobile battery, conversion and CAD dock profile round trip");
    base.energy.batteryOccurrence = "missing";
    rejects(function() SceneArtifact.encode(data), "mobile energy profile with no battery occurrence");
    base.energy.batteryOccurrence = "battery";
    base.energy.dockPositionToleranceMeters = 0;
    rejects(function() SceneArtifact.encode(data), "mobile energy profile with no physical dock alignment tolerance");
    base.energy.dockPositionToleranceMeters = 0.015;
    base.energy.dockHeadingToleranceRadians = Math.PI / 2;
    rejects(function() SceneArtifact.encode(data), "mobile energy profile with unbounded dock heading tolerance");
    base.energy.dockHeadingToleranceRadians = 0.02;
    data.mobileBase = base;
    base.rightWheel = "lift";
    rejects(function() SceneArtifact.encode(data), "mobile base wheel on a sliding joint");
    base.rightWheel = "wheel_l";
    rejects(function() SceneArtifact.encode(data), "mobile base with one wheel twice");
    base.rightWheel = "missing";
    rejects(function() SceneArtifact.encode(data), "mobile base wheel outside the assembly");
    base.rightWheel = "wheel_r"; base.trackWidth = 0;
    rejects(function() SceneArtifact.encode(data), "mobile base without a track");
    base.trackWidth = 0.3; base.footprintWidth = null;
    rejects(function() SceneArtifact.encode(data), "mobile base footprint with one side");
    base.footprintWidth = 0.44;
    // A robot in a scene of its own: its subtree, where it stands, and where it drives.
    base.robot = "missing";
    rejects(function() SceneArtifact.encode(data), "mobile base robot subtree without occurrences");
    base.robot = null; base.origin = {x: 1.0, y: -0.5, yaw: 0.25};
    var jointStep:materia.project.SceneArtifact.SceneArtifactMissionStep = {
      kind: "moveJoints", at: {occurrence: "mast", connector: "pin"}, joints: [{joint: "lift", position: 0.02}]};
    data.mission = {steps: [jointStep]};
    var jointBack = SceneArtifact.decode(SceneArtifact.encode(data)).mission;
    check(jointBack != null && jointBack.steps[0].joints[0].position == 0.02 &&
      jointBack.steps[0].joints[0].joint == "lift", "absolute joint mission round trip in metres");
    jointStep.configuration={shoulder:"front",elbow:"up",wrist:"no-flip",turns:[0,0,0,0,0,2]};
    var pinnedMission=SceneArtifact.decode(SceneArtifact.encode(data)).mission;
    if(pinnedMission==null || pinnedMission.steps[0].configuration==null)throw "Mission configuration was lost";
    var restoredPin:materia.project.SceneArtifact.SceneArtifactArmConfiguration=cast pinnedMission.steps[0].configuration;
    var restoredTurns:Array<Int> =cast restoredPin.turns;
    check(restoredTurns[5]==2,"mission configuration and physical turns round trip");
    var authoredPin:materia.project.SceneArtifact.SceneArtifactArmConfiguration=cast jointStep.configuration;
    authoredPin.turns=[0];
    rejects(function() SceneArtifact.encode(data),"mission rejects malformed configuration turns");
    authoredPin.turns=null;authoredPin.shoulder="left";
    rejects(function() SceneArtifact.encode(data),"mission rejects an unknown shoulder configuration");
    jointStep.configuration=null;
    jointStep.joints.push({joint: "lift", position: 0.03});
    rejects(function() SceneArtifact.encode(data), "joint mission repeats a joint");
    jointStep.joints.pop(); jointStep.joints[0].joint = "missing";
    rejects(function() SceneArtifact.encode(data), "joint mission names an unknown joint");
    jointStep.joints[0].joint = "lift"; jointStep.joints[0].position = Math.NaN;
    rejects(function() SceneArtifact.encode(data), "joint mission targets a nonfinite position");
    jointStep.joints[0].position = 0.02;
    robot.joints[2].limits.upper = 10.0;
    rejects(function() SceneArtifact.encode(data), "joint mission respects millimetre mechanical limits");
    robot.joints[2].limits.upper = null;
    var mission:materia.project.SceneArtifact.SceneArtifactMission = {loop: true, steps: [
      {kind: "goTo", pose: {x: 2.0, y: 1.0, yaw: Math.PI}}, {kind: "goTo", pose: {x: 0.0, y: 0.0, yaw: 0.0}}]};
    data.mission = mission;
    var decoded = SceneArtifact.decode(SceneArtifact.encode(data));
    var placed = decoded.mobileBase, restoredMission = decoded.mission;
    if (placed == null || placed.origin == null || restoredMission == null) throw "mobile base mission round trip lost data";
    var firstPose:materia.project.SceneArtifact.SceneArtifactFloorPose = cast restoredMission.steps[0].pose;
    var origin:materia.project.SceneArtifact.SceneArtifactFloorPose = cast placed.origin;
    check(origin.yaw == 0.25 && restoredMission.steps.length == 2 && restoredMission.steps[0].kind == "goTo" &&
      firstPose.yaw == Math.PI && restoredMission.loop == true, "mobile base origin and mission round trip");
    var secondPose:materia.project.SceneArtifact.SceneArtifactFloorPose = cast mission.steps[1].pose;
    secondPose.y = Math.NaN;
    rejects(function() SceneArtifact.encode(data), "mission pose that is not finite");
    secondPose.y = 0.0; mission.steps[1].kind = "fly";
    rejects(function() SceneArtifact.encode(data), "mission step of an unknown kind");
    mission.steps[1].kind = "goTo";
    // Picking and placing name connectors of occurrences, and the tool that holds.
    mission.steps.push({kind: "pick", at: {occurrence: "mast", connector: "pin"},
      yaw: 0.3, toolYaw: Math.PI / 2});
    rejects(function() SceneArtifact.encode(data), "picking mission without a suction tool");
    data.robotTools = [{kind: "suction", members: ["chassis"], contact: {occurrence: "chassis", connector: "pin"}, channel: "tool/cup.enable",
      sensor: "tool/sensor.pressureSignal"}];
    var picked = SceneArtifact.decode(SceneArtifact.encode(data));
    var picking = picked.mission, tools = picked.robotTools;
    if (picking == null || tools == null || picking.steps[2].at == null) throw "picking mission round trip lost data";
    var pickAt:materia.project.SceneArtifact.SceneArtifactPlace = cast picking.steps[2].at;
    check(pickAt.occurrence == "mast" && picking.steps[2].yaw == 0.3 && picking.steps[2].toolYaw == Math.PI / 2 &&
      tools.length == 1 && tools[0].channel == "tool/cup.enable" &&
      tools[0].contact.connector == "pin" && tools[0].sensor == "tool/sensor.pressureSignal", "picking mission and tool round trip");
    data.robotTools.push({kind: "suction", members: ["mast"], contact: {occurrence: "mast", connector: "pin"}, channel: "tool/cup2.enable"});
    mission.steps[2].tool = "tool/cup2.enable";
    var selectedTool = SceneArtifact.decode(SceneArtifact.encode(data)).mission;
    check(selectedTool != null && selectedTool.steps[2].tool == "tool/cup2.enable",
      "handling mission can select one of multiple physical tools");
    mission.steps[2].tool = null;
    rejects(function() SceneArtifact.encode(data), "multi-tool handling step must select a tool");
    mission.steps[2].tool = "tool/cup.enable"; data.robotTools.pop();
    mission.steps[2].toolYaw = Math.NaN;
    rejects(function() SceneArtifact.encode(data), "handling mission has a finite tool yaw");
    mission.steps[2].toolYaw = Math.PI / 2;
    mission.steps[1].toolYaw = 0.0;
    rejects(function() SceneArtifact.encode(data), "non-handling mission cannot set a tool yaw");
    mission.steps[1].toolYaw = null;
    mission.steps.push({kind: "place", at: {occurrence: "mast", connector: "seat"}});
    rejects(function() SceneArtifact.encode(data), "placing on a connector the occurrence lacks");
    mission.steps.pop();
    data.robotTools[0].contact.connector = "nowhere";
    rejects(function() SceneArtifact.encode(data), "suction tool touching with a missing connector");
    data.robotTools[0].contact.connector = "pin";
    data.robotTools.push({kind: "suction", members: ["mast"], contact: {occurrence: "mast", connector: "pin"}, channel: "tool/cup.enable"});
    rejects(function() SceneArtifact.encode(data), "two tools on one channel");
    data.robotTools.pop(); mission.steps.pop(); data.robotTools = null;
    // Sensors name the connector they scan from.
    data.robotSensors = [{kind: "lidar", id: "chassis/scanner", mount: {occurrence: "chassis", connector: "pin"},
      rayCount: 64, maxRange: 6.0, updateRate: 10.0}];
    var sensed = SceneArtifact.decode(SceneArtifact.encode(data)).robotSensors;
    if (sensed == null || sensed.length != 1) throw "robot sensor round trip lost data";
    check(sensed[0].id == "chassis/scanner" && sensed[0].mount.connector == "pin" && sensed[0].rayCount == 64 &&
      sensed[0].maxRange == 6.0 && sensed[0].updateRate == 10.0, "robot sensor round trip");
    data.robotSensors[0].mount.connector = "nowhere";
    rejects(function() SceneArtifact.encode(data), "sensor mounted on a missing connector");
    data.robotSensors[0].mount.connector = "pin"; data.robotSensors[0].rayCount = 361;
    rejects(function() SceneArtifact.encode(data), "sensor with more rays than a runtime reports");
    data.robotSensors[0].rayCount = 64; data.robotSensors.push({kind: "lidar", id: "chassis/scanner",
      mount: {occurrence: "chassis", connector: "pin"}, rayCount: 64, maxRange: 6.0, updateRate: 10.0});
    rejects(function() SceneArtifact.encode(data), "two sensors with one id");
    data.robotSensors.pop();
    // A robot zone's field is the union of its boxes, each a centre and three positive half extents.
    robot.sensors = [{id: "chassis/zone", kind: "robot_zone", occurrence: "chassis", connector: "pin",
      zoneBoxes: [0, 0, 0, 10, 20, 30, 50, 0, 5, 5, 5, 5]}];
    var zoneDefinition = SceneArtifact.decode(SceneArtifact.encode(data)).assemblyDefinition;
    var zoneSensors = zoneDefinition == null ? null : zoneDefinition.sensors;
    var zoneBoxes = zoneSensors == null || zoneSensors.length != 1 ? null : zoneSensors[0].zoneBoxes;
    check(zoneBoxes != null && zoneBoxes.length == 12 && zoneBoxes[6] == 50 && zoneBoxes[11] == 5,
      "robot zone round trip keeps every box");
    var zone = robot.sensors[0];
    zone.zoneBoxes = [0, 0, 0, 10, 20];
    rejects(function() SceneArtifact.encode(data), "robot zone box of fewer than six numbers");
    zone.zoneBoxes = [0, 0, 0, 10, -20, 30];
    rejects(function() SceneArtifact.encode(data), "robot zone box with a negative half extent");
    zone.zoneBoxes = null;
    rejects(function() SceneArtifact.encode(data), "robot zone without boxes");
    robot.sensors = [{id: "chassis/presence", kind: "presence", occurrence: "chassis",
      connector: "pin", range: 0.02, zoneBoxes: [0, 0, 0, 1, 1, 1]}];
    rejects(function() SceneArtifact.encode(data), "presence sensor with zone boxes");
    robot.sensors = [{id: "chassis/presence", kind: "presence", occurrence: "chassis",
      connector: "pin", range: 0.02}];
    data.controllers = [{id: "robotController", kind: "robot", joints: ["wheel_l", "wheel_r", "lift"],
      sensors: ["chassis/presence"], channels: []}];
    data.signals = [];
    data.mission = {controllerId: "robotController", steps: [{kind: "waitFor",
      waitFor: {name: "chassis/presence", value: true, timeout: 1.0}}]};
    var localSensorMission = SceneArtifact.decode(SceneArtifact.encode(data)).mission;
    var localSensorWaitName = "";
    if (localSensorMission != null && localSensorMission.steps.length > 0) {
      var localSensorWait = localSensorMission.steps[0].waitFor;
      if (localSensorWait != null) localSensorWaitName = localSensorWait.name;
    }
    check(localSensorWaitName == "chassis/presence",
      "robot mission can wait on a locally owned field sensor");
    var localInputMission = data.mission;
    if (localInputMission == null || localInputMission.steps.length == 0 ||
        localInputMission.steps[0].waitFor == null)
      throw "local sensor wait fixture was not retained";
    var localInputWait = localInputMission.steps[0].waitFor;
    localInputWait.name = "unowned/presence";
    var unownedInput = refusal(function() SceneArtifact.encode(data));
    check(unownedInput.indexOf("reads no input signal or sensor") >= 0,
      "robot mission cannot wait on an input owned by no signal or sensor");
    data.mission = mission; data.controllers = null; data.signals = null; robot.sensors = null;
    data.mobileBase = null;
    rejects(function() SceneArtifact.encode(data), "driving mission without a mobile base");
    data.mobileBase = base; data.mission = null; data.assemblyDefinition = null;
    rejects(function() SceneArtifact.encode(data), "mobile base without its robot");
  }

  static function toolChangerMission():Void {
    var data = sceneData();
    data.parts[0].id = "body";
    var assembly = definition();
    var frame = AssemblyFrames.identity();
    for (id in ["master", "toolA", "toolB", "dockA", "dockB"])
      assembly.occurrences.push({id: id, definition: "body", initialPose: frame});
    assembly.sensors = [
      {id: "changer/locked", kind: "presence", occurrence: "master", connector: "pin", range: 0.01},
      {id: "changer/id0", kind: "presence", occurrence: "master", connector: "pin", range: 0.01},
      {id: "changer/id1", kind: "presence", occurrence: "master", connector: "pin", range: 0.01}
    ];
    data.assemblyDefinition = assembly;
    data.robotTools = [
      {kind: "suction", members: ["toolA"], contact: {occurrence: "toolA", connector: "pin"}, channel: "grip-a"},
      {kind: "suction", members: ["toolB"], contact: {occurrence: "toolB", connector: "pin"}, channel: "grip-b"}
    ];
    data.controllers = [{id: "robotController", kind: "robot", joints: ["hinge"],
      sensors: ["changer/locked", "changer/id0", "changer/id1"], channels: ["changer.lock"]}];
    data.signals = [{name: "changer.lock", from: {controller: "robotController", port: "lock"},
      to: {controller: "robotController", port: "lock-command"}}];
    var changer:materia.project.SceneArtifact.SceneArtifactToolChanger = {
      master: {occurrence: "master", connector: "pin"}, lockOutput: "changer.lock",
      lockedInput: "changer/locked", identityInputs: ["changer/id0", "changer/id1"],
      initialToolId: "gripper", maximumObservationAgeSeconds: 0.1,
      tools: [
        {id: "gripper", channel: "grip-a", occurrence: "toolA", connector: "pin",
          dock: {occurrence: "dockA", connector: "pin"}, identityCode: 1},
        {id: "cup", channel: "grip-b", occurrence: "toolB", connector: "pin",
          dock: {occurrence: "dockB", connector: "pin"}, identityCode: 2}
      ]};
    var mission:materia.project.SceneArtifact.SceneArtifactMission = {controllerId: "robotController",
      toolChanger: changer, steps: [
        {kind: "pick", at: {occurrence: "arm", connector: "pin"}},
        {kind: "changeTool", targetToolId: "cup", timeoutSeconds: 30},
        {kind: "pick", at: {occurrence: "arm", connector: "pin"}}
      ]};
    data.mission = mission;
    var restored = SceneArtifact.decode(SceneArtifact.encode(data)).mission;
    check(restored != null && restored.toolChanger != null && restored.steps.length == 3 &&
      restored.steps[1].kind == "changeTool" && restored.steps[1].targetToolId == "cup",
      "CAD tool changer validates and round trips an explicit handling-tool exchange");

    mission.steps[2].tool = "grip-a";
    rejects(function() SceneArtifact.encode(data), "handling step cannot select a parked physical tool channel");
    mission.steps[2].tool = null;
    mission.steps[1].targetToolId = "gripper";
    rejects(function() SceneArtifact.encode(data), "tool changer cannot exchange to its already-mounted identity");
    mission.steps[1].targetToolId = "cup";
    changer.tools[1].identityCode = 1;
    rejects(function() SceneArtifact.encode(data), "tool changer identity codes are unique");
    changer.tools[1].identityCode = 2;
    data.signals = [];
    rejects(function() SceneArtifact.encode(data), "tool changer lock command must be wired from the robot controller");
  }

  /** A torch tool carries its welder with it, and names only what the assembly has. */
  static function torchTool():Void {
    var vertices = Bytes.alloc(96), normals = Bytes.alloc(96), indices = Bytes.alloc(48);
    vertices.setDouble(24, 1); vertices.setDouble(56, 1); vertices.setDouble(88, 1);
    var corners = [0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3];
    for (i in 0...corners.length) indices.setInt32(i * 4, corners[i]);
    var frame = AssemblyFrames.identity();
    var cell:AssemblyDefinition = {schemaVersion: AssemblyDefinitionCodec.VERSION, id: "cell", lengthUnit: "mm",
      definitions: [{id: "body", connectors: [{name: "tcp", frame: frame}]}],
      occurrences: [for (id in ["torch", "plate", "upright"]) {id: id, definition: "body", initialPose: frame}], joints: []};
    function welder():materia.project.SceneArtifact.SceneArtifactTorch
      return {wireSpeedChannel: "tool/torch.wire_speed", voltageChannel: "tool/torch.voltage", groundedWork: ["plate", "upright"],
        maxCurrentA: 350.0, efficiency: 0.88, wireDiameterMm: 1.2, stickoutMm: 15.0, maxWireSpeedMPerMin: 20.0, depositionEfficiency: 0.95};
    function torch():materia.project.SceneArtifact.SceneArtifactRobotTool
      return {kind: "torch", members: ["torch"], contact: {occurrence: "torch", connector: "tcp"}, channel: "tool/torch.arc",
        sensor: "tool/torch.weld", torch: welder()};
    var data:materia.project.SceneArtifact.SceneArtifactData = {lengthUnit: "mm", metresPerUnit: 0.001,
      parts: [{id: "body", name: "body", red: 0.5, green: 0.5, blue: 0.5, vertexCount: 4, indexCount: 12,
        vertices: vertices, normals: normals, indices: indices, faceRanges: []}],
      assemblyDefinition: cell, robotTools: [torch()]};
    var restored = SceneArtifact.decode(SceneArtifact.encode(data)).robotTools;
    if (restored == null || restored.length != 1 || restored[0].torch == null) throw "torch tool round trip lost the tool";
    var back:materia.project.SceneArtifact.SceneArtifactTorch = cast restored[0].torch;
    check(restored[0].kind == "torch" && restored[0].channel == "tool/torch.arc" && restored[0].sensor == "tool/torch.weld" &&
      restored[0].contact.connector == "tcp" && back.wireSpeedChannel == "tool/torch.wire_speed" &&
      back.voltageChannel == "tool/torch.voltage" && back.groundedWork.join(",") == "plate,upright" && back.maxCurrentA == 350.0 &&
      back.efficiency == 0.88 && back.wireDiameterMm == 1.2 && back.stickoutMm == 15.0, "torch tool round trip");
    // A suction tool beside it keeps working, and the torch's channels may not collide with it.
    data.robotTools = [torch(), {kind: "suction", members: ["plate"], contact: {occurrence: "plate", connector: "tcp"}, channel: "cup.enable"}];
    var both = SceneArtifact.decode(SceneArtifact.encode(data)).robotTools;
    check(both != null && both.length == 2, "torch beside a suction tool");
    data.robotTools = [torch()];
    function bad(change:(materia.project.SceneArtifact.SceneArtifactRobotTool, materia.project.SceneArtifact.SceneArtifactTorch) -> Void,
        message:String):Void {
      var tool = torch(), welding = welder();
      tool.torch = welding;
      change(tool, welding);
      data.robotTools = [tool];
      rejects(function() SceneArtifact.encode(data), message);
    }
    bad((tool, w) -> tool.torch = null, "torch without its welder");
    bad((tool, w) -> tool.sensor = null, "torch without its weld sensor");
    bad((tool, w) -> tool.contact.connector = "nowhere", "torch whose wire tip is no connector");
    bad((tool, w) -> w.groundedWork = [], "torch with no grounded work");
    bad((tool, w) -> w.groundedWork = ["plate", "ghost"], "torch grounded on an occurrence the assembly lacks");
    bad((tool, w) -> w.groundedWork = ["plate", "plate"], "torch grounded on one occurrence twice");
    bad((tool, w) -> w.voltageChannel = "tool/torch.arc", "torch whose voltage shares the arc channel");
    bad((tool, w) -> w.wireSpeedChannel = "", "torch with an unnamed wire speed channel");
    bad((tool, w) -> w.maxCurrentA = 0, "torch with no supply rating");
    bad((tool, w) -> w.efficiency = 1.2, "torch with a supply that gives more than it takes");
    bad((tool, w) -> w.wireDiameterMm = Math.NaN, "torch with a wire that is not a number");
    bad((tool, w) -> w.stickoutMm = 0, "torch with no stickout");
    bad((tool, w) -> w.depositionEfficiency = 1.5, "torch whose wire deposits more than it melts");
    bad((tool, w) -> w.maxWireSpeedMPerMin = 0, "torch whose feeder cannot feed");
    bad((tool, w) -> tool.kind = "plasma", "tool of an unknown kind");
    data.robotTools = [torch(), torch()];
    data.robotTools[1].channel = "tool/other.arc";
    data.robotTools[1].torch = {wireSpeedChannel: "w2", voltageChannel: "v2", groundedWork: ["plate"], maxCurrentA: 350.0,
      efficiency: 0.88, wireDiameterMm: 1.2, stickoutMm: 15.0, maxWireSpeedMPerMin: 20.0, depositionEfficiency: 0.95};
    rejects(function() SceneArtifact.encode(data), "a second torch");
    data.robotTools = [{kind: "suction", members: ["plate"], contact: {occurrence: "plate", connector: "tcp"}, channel: "cup.enable", torch: welder()}];
    rejects(function() SceneArtifact.encode(data), "a suction tool with a welder");
    ContactWorkTests.run(data, torch());
    weldMission(data, torch);
  }

  /**
   * A `weld` step is a path of line segments, each naming the seam of two occurrences it lies along, relative to the
   * workpiece's reference member; it carries its process, and needs the robot's torch.
   */
  static function weldMission(data:materia.project.SceneArtifact.SceneArtifactData,
      torch:Void -> materia.project.SceneArtifact.SceneArtifactRobotTool):Void {
    // Wires pointing down into the corner from the open side: 30 degrees off the plate's normal toward the upright (or the
    // second upright, whose face looks along -X).
    var intoY = [0.96593, 0.0, 0.0, -0.25882];
    var intoX = [0.0, 0.96593, 0.0, 0.25882];
    function segment(seam:String, faces:Array<materia.project.SceneArtifact.SceneArtifactWeldFace>, from:Array<Float>, to:Array<Float>, normals:Array<Array<Float>>, rotation:Array<Float>):materia.project.SceneArtifact.SceneArtifactWeldSegment
      return {kind: "line", seam: seam, faces: faces, joint: "fillet", start: {position: from, rotation: rotation}, stop: {position: to, rotation: rotation},
        normals: normals};
    function line():materia.project.SceneArtifact.SceneArtifactWeldSegment
      return segment("plate:f3|upright:f7", [{member: "plate", face: "f3"}, {member: "upright", face: "f7"}], [0.0, 0.0, 0.01], [0.18, 0.0, 0.01], [[0.0, 0.0, 1.0], [0.0, -1.0, 0.0]], intoY);
    function second():materia.project.SceneArtifact.SceneArtifactWeldSegment
      return segment("plate:f3|plate:f9", [{member: "plate", face: "f3"}, {member: "plate", face: "f9"}], [0.18, 0.0, 0.01], [0.18, 0.06, 0.01], [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0]], intoX);
    function seam():materia.project.SceneArtifact.SceneArtifactWeld
      return {frame: "plate", metal: "plate", path: [line()], legSize: 0.005,
        passes: [{name: "single", offset: [0.0, 0.0], interpassDwell: 0.0, process: {wireSpeed: 8.0, voltage: 24.0, travelSpeed: 0.01, approach: 0.05, startDwell: 0.2, craterDwell: 0.3, burnback: 0.1}}]};
    function mission(weld:materia.project.SceneArtifact.SceneArtifactWeld):materia.project.SceneArtifact.SceneArtifactMission
      return {steps: [{kind: "weld", weld: weld}]};
    data.robotTools = [torch()];
    data.mission = mission(seam());
    var back = SceneArtifact.decode(SceneArtifact.encode(data)).mission;
    if (back == null || back.steps.length != 1 || back.steps[0].weld == null) throw "weld mission round trip lost the weld";
    var weld:materia.project.SceneArtifact.SceneArtifactWeld = cast back.steps[0].weld;
    check(weld.frame == "plate" && weld.path.length == 1 && weld.path[0].seam == "plate:f3|upright:f7" && weld.path[0].joint == "fillet" &&
      weld.path[0].faces[0].member == "plate" && weld.path[0].faces[1].face == "f7" &&
      weld.path[0].kind == "line" && weld.metal == "plate" && weld.path[0].stop.position[0] == 0.18 &&
      weld.path[0].normals[1][1] == -1.0 && weld.legSize == 0.005 && weld.passes[0].process.wireSpeed == 8.0 && weld.passes[0].process.travelSpeed == 0.01 &&
      weld.passes[0].process.startDwell == 0.2 && weld.passes[0].process.craterDwell == 0.3 &&
      weld.passes[0].process.burnback == 0.1, "weld mission round trip including both dwells");
    data.mission = {steps: [{kind: "weld", weld: seam()}, {kind: "stow"}]};
    var stowBack = SceneArtifact.decode(SceneArtifact.encode(data)).mission;
    check(stowBack != null && stowBack.steps.length == 2 && stowBack.steps[1].kind == "stow",
      "welding stow mission round trip");
    // A chained seam is one weld of several segments, each with its own faces.
    var chain = seam();
    chain.path = [line(), second()];
    data.mission = mission(chain);
    var chainBack = SceneArtifact.decode(SceneArtifact.encode(data)).mission;
    if (chainBack == null) throw "chained weld round trip lost the mission";
    var chained:materia.project.SceneArtifact.SceneArtifactWeld = cast chainBack.steps[0].weld;
    check(chained.path.length == 2 && chained.path[1].normals[1][0] == -1.0 && chained.path[1].stop.position[1] == 0.06,
      "a chained weld keeps its segments");
    // A weld without a path is rejected.
    var legacy = haxe.Json.parse('{"seam":"plate:f3|upright:f7","joint":"fillet","metal":"plate","start":{"position":[0,0,0.01],"rotation":[0.96593,0,0,-0.25882]},' +
      '"stop":{"position":[0.18,0,0.01],"rotation":[0.96593,0,0,-0.25882]},"normals":[[0,0,1],[0,-1,0]],"legSize":0.005,' +
      '"process":{"wireSpeed":8,"voltage":24,"travelSpeed":0.01,"approach":0.05,"startDwell":0.2,"craterDwell":0.3,"burnback":0.1}}');
    rejects(function() { @:privateAccess SceneArtifact.decodeMission({steps: [{kind: "weld", weld: legacy}]}); },
      "a weld without a path");
    function first(w:materia.project.SceneArtifact.SceneArtifactWeld):materia.project.SceneArtifact.SceneArtifactWeldSegment return w.path[0];
    function bad(change:materia.project.SceneArtifact.SceneArtifactWeld -> Void, message:String):Void {
      var weld = seam();
      change(weld);
      data.mission = mission(weld);
      rejects(function() SceneArtifact.encode(data), message);
    }
    bad(function(w) { var g = first(w); g.faces = [{member: "plate", face: "f3"}]; }, "weld of a seam with one face");
    bad(function(w) { var g = first(w); g.faces[0].member = "ghost"; }, "weld of a seam on an occurrence the assembly lacks");
    bad(function(w) { var g = first(w); g.joint = "butt"; }, "weld of a joint that is not deposited");
    bad(function(w) { var g = first(w); g.kind = "arc"; }, "weld of a segment that is not a line");
    bad(w -> w.metal = "ghost", "weld metal on an occurrence the assembly lacks");
    bad(w -> w.frame = "ghost", "weld placed by an occurrence the assembly lacks");
    bad(w -> w.path = [], "weld of no path");
    bad(function(w) { var g = first(w); g.stop.position = [0.0, 0.0, 0.01]; }, "weld of a seam with no length");
    bad(function(w) { var g = first(w); g.start.rotation = [0.0, 0.0, 0.0, 2.0]; }, "weld whose rotation is not a unit quaternion");
    bad(function(w) { var g = first(w); g.normals = [[0.0, 0.0, 1.0]]; }, "weld with one face normal");
    bad(function(w) { var g = first(w); g.normals[0] = [0.0, 0.0, 2.0]; }, "weld with a face normal that is not a unit vector");
    bad(function(w) { var g = first(w); g.normals = [[0.0, 0.0, 1.0], [0.0, 0.0, -1.0]]; }, "weld between parallel faces");
    bad(function(w) { var g = first(w); g.normals = [[0.0, 0.0, 1.0], [0.0, 0.0, 1.0]]; }, "weld between the same face twice");
    bad(function(w) { var g = first(w); g.start.rotation = [0.0, 0.0, 0.0, 1.0]; g.stop.rotation = [0.0, 0.0, 0.0, 1.0]; }, "weld whose wire points out of the corner");
    bad(w -> w.path = [line(), segment("plate:f3|plate:f9", [{member: "plate", face: "f3"}, {member: "plate", face: "f9"}], [0.5, 0.0, 0.01], [0.5, 0.06, 0.01], [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0]], intoX)],
      "a path whose segments do not join");
    bad(w -> w.legSize = 0.0, "weld with no leg");
    bad(w -> w.passes = [], "weld with no passes");
    bad(function(w) { var pass = w.passes[0]; pass.offset = [0.0]; }, "pass with one face offset");
    bad(function(w) { var pass = w.passes[0]; pass.offset = [-0.001, 0.0]; }, "pass inside a face");
    bad(function(w) { var pass = w.passes[0]; pass.interpassDwell = Math.NaN; }, "nonfinite interpass dwell");
    bad(function(w) { var pass = w.passes[0]; pass.weave = {pattern: "unknown", amplitude: 0.001, cyclesPerMetre: 100, edgeDwell: 0.05}; }, "unknown weave pattern");
    bad(function(w) { var pass = w.passes[0]; pass.weave = {pattern: "sine", amplitude: 0.001, cyclesPerMetre: 100, edgeDwell: 1.0}; }, "edge dwells consume the weave period");
    var multipass = seam();
    multipass.passes.push({name: "cap", offset: [0.002, 0.003], interpassDwell: 0.5,
      process: multipass.passes[0].process, weave: {pattern: "triangle", amplitude: 0.001, cyclesPerMetre: 100, edgeDwell: 0.05}});
    data.mission = mission(multipass);
    var passesBack = SceneArtifact.decode(SceneArtifact.encode(data)).mission;
    if (passesBack == null || passesBack.steps[0].weld == null) throw "multi-pass weld lost";
    var restored:materia.project.SceneArtifact.SceneArtifactWeld = cast passesBack.steps[0].weld;
    var cap = restored.passes[1];
    check(restored.passes.length == 2 && cap.name == "cap" && cap.offset[1] == 0.003 && cap.interpassDwell == 0.5 &&
      cap.weave != null && cap.weave.pattern == "triangle" && cap.weave.cyclesPerMetre == 100, "pass schedule round trip");

    bad(function(w) { var process = w.passes[0].process; process.wireSpeed = 0.0; }, "weld with no wire speed");
    bad(function(w) { var process = w.passes[0].process; process.wireSpeed = 25.0; }, "weld that asks the feeder for more wire than it can feed");
    bad(function(w) { var process = w.passes[0].process; process.travelSpeed = Math.NaN; }, "weld that travels at no number");
    bad(function(w) { var process = w.passes[0].process; process.approach = 0.0; }, "weld with no approach");
    bad(function(w) { var process = w.passes[0].process; process.craterDwell = -1.0; }, "weld with a negative crater dwell");
    bad(function(w) { var process = w.passes[0].process; process.burnback = 0.0; }, "weld with no burnback");
    data.mission = mission(seam());
    // The robot's arm carries one tool: a mission cannot both pick and weld.
    data.mission = {steps: [{kind: "weld", weld: seam()}, {kind: "pick", at: {occurrence: "plate", connector: "tcp"}}]};
    data.robotTools = [torch(), {kind: "suction", members: ["upright"], contact: {occurrence: "upright", connector: "tcp"}, channel: "cup.enable"}];
    rejects(function() SceneArtifact.encode(data), "a mission that picks and welds");
    data.mission = mission(seam());
    data.robotTools = null;
    rejects(function() SceneArtifact.encode(data), "weld mission without a torch");
    data.mission = null;
  }

  /** A machining job travels with its machine and names only what the scene has. */
  static function machining():Void {
    var vertices = Bytes.alloc(96), normals = Bytes.alloc(96), indices = Bytes.alloc(48);
    vertices.setDouble(24, 1); vertices.setDouble(56, 1); vertices.setDouble(88, 1);
    var corners = [0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3];
    for (i in 0...corners.length) indices.setInt32(i * 4, corners[i]);
    function part(id:String):materia.project.SceneArtifact.SceneArtifactPart
      return {id: id, name: id, red: 0.5, green: 0.5, blue: 0.5, vertexCount: 4, indexCount: 12,
        vertices: vertices, normals: normals, indices: indices, faceRanges: []};
    var frame = AssemblyFrames.identity();
    function slide(id:String, parent:String, child:String):materia.assembly.AssemblyDefinition.KinematicJoint
      return {id: id, type: AssemblyJointType.Prismatic, role: AssemblyJointRole.Tree,
        parent: parent, parentConnector: "pin", child: child, childConnector: "pin",
        axis: {x: 0.0, y: 0.0, z: 1.0},
        limits: {lower: -1.0, upper: 1.0, velocity: null, effort: null}, defaultValue: 0.0};
    var machine:AssemblyDefinition = {schemaVersion: AssemblyDefinitionCodec.VERSION, id: "machine",
      lengthUnit: "mm", definitions: [{id: "body", connectors: [{name: "pin", frame: frame}]}],
      occurrences: [for (id in ["bed", "gantry", "carriage", "spindle"]) {id: id, definition: "body", initialPose: frame}],
      joints: [slide("y", "bed", "gantry"), slide("x", "gantry", "carriage"), slide("z", "carriage", "spindle")]};
    var job:materia.project.SceneArtifact.SceneArtifactMachining = {program: "G0 X1\nM2\n",
      axes: ["x", "y", "z"], spindle: "spindle", channelScope: "cabinet/process", workOffset: [0.1, 0.1, -0.05],
      tools: [{number: 1, length: 0.03, profile: [[0.0, 0.0, 0.0, 0.0, 0.003, 0.0], [0.0, 0.0, 0.003, 0.0, 0.003, 0.02]]}],
		stocks: ["bed"], sacrificial: ["gantry"], toolPart: "spindle", loadedTool: 1, target: "finished", loop: true,
      controller: {stepTickHz: 40000}};
    var data:materia.project.SceneArtifact.SceneArtifactData = {lengthUnit: "mm", metresPerUnit: 0.001,
      parts: [part("body"), part("finished")], assemblyDefinition: machine, machining: job};
    var restored = SceneArtifact.decode(SceneArtifact.encode(data)).machining;
    if (restored == null) throw "machining job round trip lost the job";
    check(restored.program == job.program && restored.axes.join(",") == "x,y,z" &&
      restored.channelScope == "cabinet/process" &&
		restored.spindle == "spindle" && restored.toolPart == "spindle" && restored.loadedTool == 1 && restored.stocks.length == 1 && restored.stocks[0] == "bed" && restored.sacrificial[0] == "gantry" &&
      restored.target == "finished" && restored.loop == true, "machining job round trip");
    check(restored.tools.length == 1 && restored.tools[0].number == 1 && restored.tools[0].length == 0.03 &&
      restored.tools[0].profile[1][5] == 0.02, "machining tool table round trip");
    var wiring = restored.controller;
    check(wiring != null && wiring.stepTickHz == 40000, "machining controller round trip");
    check(haxe.Json.stringify(restored.controller).indexOf("microsteps") < 0,
      "machining jobs do not save driver microstepping");
    job.controller = {stepTickHz: 0};
    rejects(function() SceneArtifact.encode(data), "machining controller with no step rate");
    Reflect.deleteField(job, "controller");
    var bare = SceneArtifact.decode(SceneArtifact.encode(data)).machining;
    check(bare != null && bare.controller == null, "a job names no controller unless it is given one");
    job.controller = {stepTickHz: 40000};
    job.spindle = "missing";
    rejects(function() SceneArtifact.encode(data), "machining spindle outside the assembly");
    job.spindle = "spindle"; job.axes = ["x", "x", "z"];
    rejects(function() SceneArtifact.encode(data), "repeated machining axis");
    job.axes = ["x", "y", "z"]; job.tools.push(job.tools[0]);
    rejects(function() SceneArtifact.encode(data), "duplicate machining tool number");
    job.tools.pop(); job.loadedTool = 2;
    rejects(function() SceneArtifact.encode(data), "machining job starting with a tool it lacks");
    job.loadedTool = 1; job.target = "absent";
    rejects(function() SceneArtifact.encode(data), "machining target outside the scene");
    job.target = null; data.assemblyDefinition = null;
    rejects(function() SceneArtifact.encode(data), "machining job without its machine");
  }


  /** A print job travels with its printer: its axes, extruder and heaters name only what the assembly has. */
  static function printing():Void {
    var vertices = Bytes.alloc(96), normals = Bytes.alloc(96), indices = Bytes.alloc(48);
    vertices.setDouble(24, 1); vertices.setDouble(56, 1); vertices.setDouble(88, 1);
    var corners = [0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3];
    for (i in 0...corners.length) indices.setInt32(i * 4, corners[i]);
    function part(id:String):materia.project.SceneArtifact.SceneArtifactPart
      return {id: id, name: id, red: 0.5, green: 0.5, blue: 0.5, vertexCount: 4, indexCount: 12,
        vertices: vertices, normals: normals, indices: indices, faceRanges: []};
    var frame = AssemblyFrames.identity();
    function joint(id:String, type:AssemblyJointType, parent:String, child:String):materia.assembly.AssemblyDefinition.KinematicJoint
      return {id: id, type: type, role: AssemblyJointRole.Tree, parent: parent, parentConnector: "pin", child: child,
        childConnector: "pin", axis: {x: 0.0, y: 0.0, z: 1.0},
        limits: {lower: -1.0, upper: 1.0, velocity: null, effort: null}, defaultValue: 0.0};
    var printer:AssemblyDefinition = {schemaVersion: AssemblyDefinitionCodec.VERSION, id: "printer", lengthUnit: "mm",
      definitions: [{id: "body", connectors: [{name: "pin", frame: frame}]}],
      occurrences: [for (id in ["bed", "gantry", "carriage", "block", "gear", "sensor", "bedSensor"]) {id: id, definition: "body", initialPose: frame}],
      joints: [joint("y", AssemblyJointType.Prismatic, "bed", "gantry"), joint("x", AssemblyJointType.Prismatic, "gantry", "carriage"),
        joint("z", AssemblyJointType.Prismatic, "carriage", "block"), joint("e", AssemblyJointType.Continuous, "block", "gear")]};
    function heater(name:String, slot:Int, mount:String, ?fan:String):materia.project.SceneArtifact.SceneArtifactHeater {
      var made:materia.project.SceneArtifact.SceneArtifactHeater = {name: name, sensor: name, slot: slot, mount: mount,
        target: 'heater.$name.target', feedsExtruder: name == "hotend", r0Ohm: 100000.0, t0Celsius: 25.0, beta: 3950.0,
        pullupOhm: 4700.0, vRef: 3.3, kp: 0.06, ki: 0.004, kd: 0.0, maxDuty: 1.0, maxTemperature: 275.0, bandCelsius: 3.0,
        pidRangeCelsius: 10.0, gainWindowSeconds: 28.0, gainMinCelsius: 2.0, hysteresisCelsius: 15.0, hysteresisWindowSeconds: 10.0,
        heatCapacity: 13.5, power: 40.0, loss: 0.12, fanLoss: 0.05, ambient: 22.0};
      if (fan != null) made.fan = fan;
      return made;
    }
    var job:materia.project.SceneArtifact.SceneArtifactPrinting = {source: "G21\nG90\nM83\nG1 X10 Y10 E0.5 F1200\n",
      axes: ["x", "y", "z"], extruder: "e", filamentDiameter: 1.75e-3, gearRadius: 3.65e-3, maxFlow: 15e-9,
      pressureAdvance: 0.04, pressureAdvanceWindow: 0.04, meltLag: 0.04, rampLength: 0.0005, minExtrusionTemperature: 170.0,
      heaters: [heater("hotend", 0, "sensor", "fan.part"), heater("bed", 1, "bedSensor")], controller: {stepTickHz: 40000}};
    var data:materia.project.SceneArtifact.SceneArtifactData = {lengthUnit: "mm", metresPerUnit: 0.001,
      parts: [part("body")], assemblyDefinition: printer, printing: job};
    var restored = SceneArtifact.decode(SceneArtifact.encode(data)).printing;
    if (restored == null) throw "print job round trip lost the job";
    check(restored.source == job.source && restored.axes.join(",") == "x,y,z" && restored.extruder == "e" &&
      restored.filamentDiameter == 1.75e-3 && restored.gearRadius == 3.65e-3 && restored.maxFlow == 15e-9 &&
      restored.pressureAdvance == 0.04 && restored.meltLag == 0.04 && restored.minExtrusionTemperature == 170.0 && restored.controller.stepTickHz == 40000,
      "print job round trip");
    check(restored.heaters.length == 2 && restored.heaters[0].fan == "fan.part" && restored.heaters[1].fan == null &&
      restored.heaters[0].feedsExtruder && !restored.heaters[1].feedsExtruder && restored.heaters[1].slot == 1 &&
      restored.heaters[0].heatCapacity == 13.5 && restored.heaters[0].mount == "sensor", "print job heaters round trip");
    check(restored.shaping == null, "a job without shapers has none");
    job.shaping = [{axis: "x", kind: "mzv", frequency: 52.5, damping: 0.1}, {axis: "y", kind: "ei", frequency: 40.0, damping: 0.05}];
    var shaped = SceneArtifact.decode(SceneArtifact.encode(data)).printing;
    check(shaped != null && shaped.shaping != null && shaped.shaping.length == 2 && shaped.shaping[0].axis == "x" && shaped.shaping[0].kind == "mzv" &&
      shaped.shaping[0].frequency == 52.5 && shaped.shaping[1].damping == 0.05, "input shapers round trip");
    job.shaping = [{axis: "e", kind: "mzv", frequency: 50.0, damping: 0.1}];
    rejects(function() SceneArtifact.encode(data), "a shaper on a joint that is no print axis");
    job.shaping = [{axis: "x", kind: "zv", frequency: 50.0, damping: 0.1}, {axis: "x", kind: "zv", frequency: 50.0, damping: 0.1}];
    rejects(function() SceneArtifact.encode(data), "an axis shaped twice");
    job.shaping = [{axis: "x", kind: "wiggle", frequency: 50.0, damping: 0.1}];
    rejects(function() SceneArtifact.encode(data), "a shaper of unknown kind");
    job.shaping = [{axis: "x", kind: "zv", frequency: 0.0, damping: 0.1}];
    rejects(function() SceneArtifact.encode(data), "a shaper with no frequency");
    job.shaping = [{axis: "x", kind: "zv", frequency: 50.0, damping: 1.0}];
    rejects(function() SceneArtifact.encode(data), "a shaper with a damping ratio of one");
    job.shaping = null;
    // The job's own rules.
    job.axes = ["x", "y", "e"];
    rejects(function() SceneArtifact.encode(data), "a print axis that does not slide");
    job.axes = ["x", "x", "z"];
    rejects(function() SceneArtifact.encode(data), "a repeated print axis");
    job.axes = ["x", "y", "z"]; job.extruder = "x";
    rejects(function() SceneArtifact.encode(data), "an extruder that is an axis");
    job.extruder = "e"; job.heaters[1] = heater("bed", 0, "bedSensor");
    rejects(function() SceneArtifact.encode(data), "two heaters on one sensor slot");
    job.heaters[1] = heater("bed", 1, "nowhere");
    rejects(function() SceneArtifact.encode(data), "a heater mounted on no occurrence");
    job.heaters[1] = heater("bed", 1, "bedSensor", "fan.part");
    rejects(function() SceneArtifact.encode(data), "two heaters sharing a channel");
    job.heaters[1] = heater("bed", 1, "bedSensor"); job.heaters[0].kp = Math.NaN;
    rejects(function() SceneArtifact.encode(data), "a heater with a tuning that is not a number");
    job.heaters[0].kp = 0.06; job.gearRadius = 0.0;
    rejects(function() SceneArtifact.encode(data), "an extruder gear with no radius");
    job.gearRadius = 3.65e-3; job.pressureAdvance = 2.0;
    rejects(function() SceneArtifact.encode(data), "a pressure advance of seconds");
    job.pressureAdvance = 0.04; job.heaters = [];
    rejects(function() SceneArtifact.encode(data), "a print job with no heaters");
    job.heaters = [heater("hotend", 0, "sensor", "fan.part")];
    data.machining = {program: "M2\n", axes: ["x", "y", "z"], spindle: "gear", channelScope: "", workOffset: [0.0, 0.0, 0.0], tools: [], stocks: [], controller: {stepTickHz: 40000}};
    rejects(function() SceneArtifact.encode(data), "machining and printing together");
    data.machining = null; data.assemblyDefinition = null;
    rejects(function() SceneArtifact.encode(data), "a print job without its printer");
  }

  static function bodies():Void {
    var frame = AssemblyFrames.identity();
    function joint(id:String, type:AssemblyJointType, parent:String, child:String,
        ?role:AssemblyJointRole):materia.assembly.AssemblyDefinition.KinematicJoint
      return {id: id, type: type, role: role == null ? AssemblyJointRole.Tree : role, parent: parent,
        parentConnector: "pin", child: child, childConnector: "pin", axis: {x: 0.0, y: 0.0, z: 1.0},
        limits: {lower: null, upper: null, velocity: null, effort: null}, defaultValue: 0.0};
    var chain:AssemblyDefinition = {schemaVersion: AssemblyDefinitionCodec.VERSION, id: "chain", lengthUnit: "mm",
      definitions: [{id: "body", connectors: [{name: "pin", frame: frame}]}],
      occurrences: [for (id in ["tip", "base", "plate", "arm", "spare"])
        {id: id, definition: "body", initialPose: frame}],
      // Listed out of order on purpose: a body's root need not be its first occurrence.
      joints: [joint("tip-seat", AssemblyJointType.Fixed, "arm", "tip"),
        joint("plate-seat", AssemblyJointType.Fixed, "base", "plate"),
        joint("hinge", AssemblyJointType.Revolute, "plate", "arm"),
        joint("loop", AssemblyJointType.Revolute, "tip", "base", AssemblyJointRole.Closure)]};
    var found = AssemblyBodies.of(chain);
    check(found.length == 3, "fixed joints merge occurrences into bodies");
    var byId = new Map<String, materia.assembly.AssemblyBodies.AssemblyBody>();
    for (body in found) byId.set(body.id, body);
    function named(id:String):materia.assembly.AssemblyBodies.AssemblyBody {
      var body = byId.get(id);
      if (body == null) throw 'missing body "$id"';
      return body;
    }
    check(named("base").occurrences.join(",") == "base,plate" && named("base").parent == null,
      "a body lists its root first and has no parent when nothing carries it");
    check(named("arm").occurrences.join(",") == "arm,tip" && named("arm").parent == "base" &&
      named("arm").joint == "hinge" && named("arm").jointType == AssemblyJointType.Revolute,
      "a moving joint separates bodies and names the parent");
    check(named("spare").occurrences.join(",") == "spare" && named("spare").parent == null,
      "an unjoined occurrence is a body of its own");
    check(found[0].id == "arm", "bodies follow the order their first occurrence appears");
    check(AssemblyBodies.displayName("upperArm") == "Upper arm", "camel case words");
    check(AssemblyBodies.displayName("joint1") == "Joint 1", "trailing digits");
    check(AssemblyBodies.displayName("joint12") == "Joint 12", "multi-digit numbers stay together");
    check(AssemblyBodies.displayName("tool/suctionCup") == "Tool › Suction cup", "path segments");
    check(AssemblyBodies.displayName("rack-frame_2") == "Rack frame 2", "separators");
    check(AssemblyBodies.displayName("ARM") == "Arm", "capitals in a row stay one word");
  }

  static function main():Void {
    assertions += DrawingProjectionTests.run();
    SceneScrewTests.run();
    SceneScrewFeederTests.run();
    SceneBlowFeedTests.run();
    near(LengthUnit.metresPerUnit("mm"), 0.001, "millimetres");
    near(LengthUnit.metresPerUnit("in"), 0.0254, "inches");
    check(LengthUnit.fromScale(0.01) == "cm", "scale to centimetres");
    rejects(function() LengthUnit.metresPerUnit("feet"), "unsupported unit");
    assembly(); hydraulic();
    homingDependencies(); frames(); scene(); screwEntrySearchProfile(); machineMetadata(); machining(); printing();
    mobileBase(); toolChangerMission(); torchTool(); bodies();
    check(MaterialLibrary.require("steel-c45").physical.density == 7850, "steel density");
    check(MaterialLibrary.fromSpec("steel C45") == "steel-c45", "material lookup");
    rejects(function() MaterialLibrary.require("unknown"), "unknown material");
    var custom:MaterialDef = {id: "custom-bronze", name: "Custom bronze",
      visual: {baseColor: [0.5, 0.3, 0.2], metallic: 0.7, roughness: 0.4},
      physical: {density: 8700.0, spec: "custom bronze"}};
    MaterialLibrary.validateCustom([custom]);
    check(true, "valid custom material");
    rejects(function() MaterialLibrary.validateCustom([custom, custom]),
      "duplicate custom material");
    var sheetGrades = MaterialLibrary.sheetMaterials();
    check(sheetGrades.length == 2, "specific sheet grades have development defaults");
    check(MaterialLibrary.require("painted").mechanical == null, "visual steel does not infer a structural grade");
    var steel = MaterialLibrary.require("mild-steel");
    MaterialLibrary.validateSheetProperties(steel);
    var mechanical = steel.mechanical; if (mechanical == null) throw "missing steel properties";
    near(mechanical.uts, 400, "plan material A UTS in MPa");
    near(mechanical.youngs, 205000, "plan material A modulus in MPa");
    var aluminium = MaterialLibrary.require("aluminium-5052-h32");
    var aluminiumMechanical = aluminium.mechanical; if (aluminiumMechanical == null) throw "missing aluminium properties";
    near(aluminiumMechanical.yield, 193, "plan material B yield in MPa");
    var encoded = JsonWire.encode(sheetGrades);
    var decoded:Array<MaterialDef> = JsonWire.decode(encoded);
    check(decoded.length == 2, "typed material array round trip");
    var decodedSheet = decoded[0].sheet; if (decodedSheet == null) throw "missing decoded sheet defaults";
    near(decodedSheet.kFactorTable[1].k, 0.38, "typed sheet defaults round trip");
    check(decodedSheet.grain, "rolling-direction requirement round trip");
    custom.mechanical = {uts: 200, yield: 300, youngs: 70000};
    rejects(function() MaterialLibrary.validateCustom([custom]), "yield above UTS rejected");
    custom.mechanical = null;
    custom.sheet = {kFactorTable: [{ratio: 1, k: 0.4}, {ratio: 1, k: 0.4}],
      defaultRadiusRatio: 1, minRadiusRatio: 0.5, grain: false};
    rejects(function() MaterialLibrary.validateCustom([custom]), "unordered or duplicate K rows rejected");
    var customSheet = custom.sheet; if (customSheet == null) throw "missing custom sheet";
    customSheet.kFactorTable = [{ratio: 0, k: 0.6}];
    rejects(function() MaterialLibrary.validateCustom([custom]), "K outside ANSI range rejected");
    customSheet.kFactorTable = [{ratio: 0, k: 0.4}];
    customSheet.minRadiusRatio = 2;
    rejects(function() MaterialLibrary.validateCustom([custom]), "default radius below minimum rejected");
    check(Appearances.same(Appearances.neutral(), null), "default appearance");
    check(Appearances.preset("rubber") != null, "appearance preset");
    Sys.println('ProjectKit tests passed ($assertions assertions)');
  }
}
