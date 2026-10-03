import haxe.io.Bytes;
import materia.assembly.AssemblyCodec;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyDefinition.AssemblyStateRecord;
import materia.assembly.AssemblyBodies;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyFrames;
import materia.project.Appearance.Appearances;
import materia.project.MaterialLibrary;
import materia.project.MaterialDef;
import materia.project.SceneArtifact;
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

  static function scene():Void {
    var vertices = Bytes.alloc(96), normals = Bytes.alloc(96), indices = Bytes.alloc(48);
    vertices.setDouble(24, 1); vertices.setDouble(56, 1); vertices.setDouble(88, 1);
    var corners = [0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3];
    for (i in 0...corners.length) indices.setInt32(i * 4, corners[i]);
    var data:materia.project.SceneArtifact.SceneArtifactData = {lengthUnit: "mm",
      metresPerUnit: 0.001, parts: [{id: "part", name: "Part", red: 0.2,
        green: 0.3, blue: 0.4, appearance: Appearances.machinedSteel(),
        vertexCount: 4, indexCount: 12, vertices: vertices, normals: normals,
        indices: indices, faceRanges: [], faceDescriptors: "[{\"index\":0}]"}], recipeDocument: "cube", recipeDiagnostics: ["ok"]};
    var encoded = SceneArtifact.encode(data);
    check(encoded.getInt32(4) == SceneArtifact.VERSION, "current scene version");
    var restored = SceneArtifact.decode(encoded);
    check(restored.parts[0].id == "part", "scene part round trip");
    var appearance = restored.parts[0].appearance;
    check(appearance != null && appearance.finish == "machined-steel", "scene appearance round trip");
    check(restored.recipeDocument == "cube", "scene source round trip");
    check(restored.parts[0].faceDescriptors == "[{\"index\":0}]", "scene face descriptors round trip");
    var diagnostics = restored.recipeDiagnostics;
    check(diagnostics != null && diagnostics[0] == "ok", "scene diagnostics round trip");
    var truncated = Bytes.alloc(10);
    truncated.blit(0, encoded, 0, 10);
    rejects(function() SceneArtifact.decode(truncated), "truncated scene");
    var badVersion = Bytes.alloc(encoded.length); badVersion.blit(0, encoded, 0, encoded.length);
    badVersion.setInt32(4, 100);
    rejects(function() SceneArtifact.decode(badVersion), "unsupported scene version");
    var invalidScale = Bytes.alloc(encoded.length);
    invalidScale.blit(0, encoded, 0, encoded.length);
    invalidScale.setDouble(8, -1);
    rejects(function() SceneArtifact.decode(invalidScale), "invalid scene unit scale");
    data.parts.push(data.parts[0]);
    rejects(function() SceneArtifact.encode(data), "duplicate scene part ID");
    data.parts.pop(); data.parts[0].indices.setInt32(0, 99);
    rejects(function() SceneArtifact.encode(data), "out of range mesh index");
    data.parts[0].indices.setInt32(0, 0);

  }

  /** A mobile base names its robot's wheel joints and carries positive dimensions and limits. */
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
    mission.steps.push({kind: "pick", at: {occurrence: "mast", connector: "pin"}});
    rejects(function() SceneArtifact.encode(data), "picking mission without a suction tool");
    data.robotTools = [{kind: "suction", contact: {occurrence: "chassis", connector: "pin"}, channel: "tool/cup.enable",
      sensor: "tool/sensor.pressureSignal"}];
    var picked = SceneArtifact.decode(SceneArtifact.encode(data));
    var picking = picked.mission, tools = picked.robotTools;
    if (picking == null || tools == null || picking.steps[2].at == null) throw "picking mission round trip lost data";
    var pickAt:materia.project.SceneArtifact.SceneArtifactPlace = cast picking.steps[2].at;
    check(pickAt.occurrence == "mast" && tools.length == 1 && tools[0].channel == "tool/cup.enable" &&
      tools[0].contact.connector == "pin" && tools[0].sensor == "tool/sensor.pressureSignal", "picking mission and tool round trip");
    mission.steps.push({kind: "place", at: {occurrence: "mast", connector: "seat"}});
    rejects(function() SceneArtifact.encode(data), "placing on a connector the occurrence lacks");
    mission.steps.pop();
    data.robotTools[0].contact.connector = "nowhere";
    rejects(function() SceneArtifact.encode(data), "suction tool touching with a missing connector");
    data.robotTools[0].contact.connector = "pin";
    data.robotTools.push({kind: "suction", contact: {occurrence: "mast", connector: "pin"}, channel: "tool/cup.enable"});
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
    data.mobileBase = null;
    rejects(function() SceneArtifact.encode(data), "driving mission without a mobile base");
    data.mobileBase = base; data.mission = null; data.assemblyDefinition = null;
    rejects(function() SceneArtifact.encode(data), "mobile base without its robot");
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
      axes: ["x", "y", "z"], spindle: "spindle", workOffset: [0.1, 0.1, -0.05],
      tools: [{number: 1, length: 0.03, profile: [[0.0, 0.0, 0.0, 0.0, 0.003, 0.0], [0.0, 0.0, 0.003, 0.0, 0.003, 0.02]]}],
      stock: "bed", sacrificial: ["gantry"], toolPart: "spindle", loadedTool: 1, target: "finished", loop: true,
      controller: {stepTickHz: 40000}};
    var data:materia.project.SceneArtifact.SceneArtifactData = {lengthUnit: "mm", metresPerUnit: 0.001,
      parts: [part("body"), part("finished")], assemblyDefinition: machine, machining: job};
    var restored = SceneArtifact.decode(SceneArtifact.encode(data)).machining;
    if (restored == null) throw "machining job round trip lost the job";
    check(restored.program == job.program && restored.axes.join(",") == "x,y,z" &&
      restored.spindle == "spindle" && restored.toolPart == "spindle" && restored.loadedTool == 1 && restored.stock == "bed" && restored.sacrificial[0] == "gantry" &&
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
    near(LengthUnit.metresPerUnit("mm"), 0.001, "millimetres");
    near(LengthUnit.metresPerUnit("in"), 0.0254, "inches");
    check(LengthUnit.fromScale(0.01) == "cm", "scale to centimetres");
    rejects(function() LengthUnit.metresPerUnit("feet"), "unsupported unit");
    assembly(); frames(); scene(); machining(); mobileBase(); bodies();
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
    check(Appearances.same(Appearances.neutral(), null), "default appearance");
    check(Appearances.preset("rubber") != null, "appearance preset");
    Sys.println('ProjectKit tests passed ($assertions assertions)');
  }
}
