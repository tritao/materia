import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactData;
import materia.project.SceneArtifact.SceneArtifactRobotTool;
import materia.project.SceneContactRegistration;
import materia.project.SceneContactRegistration.SceneContactWork;
import materia.project.SceneContactRegistration.SceneContactFace;
import materia.assembly.AssemblyFrames;

class ContactWorkTests {
  static var checks = 0;
  static function check(ok:Bool, message:String):Void { checks++; if (!ok) throw message; }
  static function rejects(action:Void->Void, message:String):Void {
    var rejected = false;
    try action() catch (_:Dynamic) rejected = true;
    check(rejected, message);
  }
  static function square(axis:Int, size:Float):Array<Array<Array<Float>>> {
    var points = [[-size, -size], [size, -size], [size, size], [-size, size]];
    var vertices = [for (point in points) axis == 0 ? [0.0, point[0], point[1]] :
      axis == 1 ? [point[0], 0.0, point[1]] : [point[0], point[1], 0.0]];
    return [for (i in 0...4) [vertices[i], vertices[(i + 1) % 4]]];
  }
  static function job():SceneContactWork {
    return {frame: "plate", nominal: AssemblyFrames.translation(1.7, -0.2, 0.6),
      translation: [0.02, 0.02, 0.0], rotation: [0.0, 0.0, 2 * Math.PI / 180],
      measurementError: 0.00001, contactOffset: 0.0005,
      faces: [for (axis in 0...3) {member: "plate", face: 'face$axis', target: true,
        normal: [for (i in 0...3) i == axis ? 1.0 : 0.0], centre: [0.0, 0.0, 0.0],
        chords: axis == 2 ? square(axis, 0.05).concat(square(axis, 0.01)) : square(axis, 0.05)}]};
  }
  public static function run(data:SceneArtifactData, torch:SceneArtifactRobotTool):Void {
    data.robotTools = [torch];
    var work = job();
    data.mission = {steps: [{kind: "findWork", at: torch.contact, contactWork: work}]};
    var bytes = SceneArtifact.encode(data);
    var restored = SceneArtifact.decode(bytes).mission;
    if (restored == null || restored.steps[0].contactWork == null) throw "Contact mission lost its job";
    var back:SceneContactWork = cast restored.steps[0].contactWork;
    check(restored.steps[0].kind == "findWork" && back.frame == "plate" && back.nominal.x == 1.7 && back.nominal.qw == 1,
      "Contact mission preserves nominal assembly/work frame independently of scene millimetres");
    check(back.translation[0] == 0.02 && back.rotation[2] == 2 * Math.PI / 180 &&
      back.measurementError == 0.00001 && back.contactOffset == 0.0005, "Parking and contact calibration round trip in metres/radians");
    check(back.faces.length == 3 && back.faces[2].chords.length == 8 && back.faces[2].chords[4][0][0] == -0.01,
      "CAD contour edges preserve the hole and target normals");
    bytes.setInt32(4, 16);
    rejects(() -> SceneArtifact.decode(bytes), "Previous scene schema is rejected without migration");
    function bad(change:SceneContactWork->Void, message:String):Void {
      var candidate = job(); change(candidate);
      data.mission = {steps: [{kind: "findWork", at: torch.contact, contactWork: candidate}]};
      rejects(() -> SceneArtifact.encode(data), message);
    }
    bad(w -> w.frame = "missing", "Unknown registration frame is rejected");
    bad(w -> w.nominal.qw = 2, "Non-unit nominal frame rotation is rejected");
    bad(w -> w.translation[0] = -0.02, "Negative parking uncertainty is rejected");
    bad(w -> w.rotation[2] = Math.PI / 2, "Unsupported angular intervals are rejected");
    bad(w -> w.measurementError = 0, "Zero observation tolerance is rejected");
    bad(w -> w.contactOffset = 0.003, "Unbounded contact calibration offset is rejected");
    bad(w -> { var face:SceneContactFace = w.faces[0]; face.member = "missing"; }, "Unknown face occurrence is rejected");
    bad(w -> { var face:SceneContactFace = w.faces[1]; var first:SceneContactFace = w.faces[0]; face.face = first.face; }, "Repeated CAD face identity is rejected");
    bad(w -> { var face:SceneContactFace = w.faces[0]; face.normal[0] = 2; }, "Non-unit CAD plane normal is rejected");
    bad(w -> { var face:SceneContactFace = w.faces[0]; var edge:Array<Array<Float>> = face.chords[0]; var point:Array<Float> = edge[0]; point[0] = 0.001; }, "Boundary vertices off the CAD plane are rejected");
    bad(w -> { var face:SceneContactFace = w.faces[0]; face.chords.pop(); }, "Open CAD contour is rejected");
    bad(w -> { var face:SceneContactFace = w.faces[0]; face.target = false; }, "Unobservable target plane basis is rejected");
    data.mission = {steps: [{kind: "findWork", at: {occurrence: "plate", connector: "tcp"}, contactWork: job()}]};
    rejects(() -> SceneArtifact.encode(data), "Contact probing needs the actual torch connector");
    var malformed:Dynamic = haxe.Json.parse(haxe.Json.stringify(job()));
    var wrongNumbers:Array<Dynamic> = ["0.02", 0.02, 0.0];
    Reflect.setField(malformed, "translation", wrongNumbers);
    rejects(() -> SceneContactRegistration.decode(malformed), "Numeric strings are rejected by the saved contact decoder");
    data.mission = null;
    Sys.println('Contact work saved format: $checks assertions passed');
  }
}
