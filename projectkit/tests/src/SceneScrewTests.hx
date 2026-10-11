import materia.project.SceneScrewDefinition;
import materia.project.SceneScrewDefinition.SceneScrew;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinitionCodec;
import materia.project.SceneArtifact.SceneArtifactData;
import materia.project.ThreadRenderData;

/** Saved screw missions keep identity and qualified units at the boundary. */
class SceneScrewTests {
  static var checks = 0;
  static function check(ok:Bool, message:String):Void {
    checks++;
    if (!ok) throw message;
  }
  static function job():SceneScrew {
    return {fastening:"work/screw1:bore", screw:"screw1", frame:"work", threadFace:"bore:cyl.side",
      presenter:"nest", presentSensor:"nest/present", socket:AssemblyFrames.identity(),
      pickup:AssemblyFrames.translation(0,0,0.1), entry:AssemblyFrames.translation(0,0,-0.01),
      seat:AssemblyFrames.identity(), hole:AssemblyFrames.identity(), leadMm:1, hand:1, slot:1, approach:0.04,
      program:{runDownSpeed:20, finalSpeed:2, targetTorque:10, minimumAngle:0.2, maximumAngle:1,
        runDownTorqueLimit:2, finalTorqueLimit:12, maximumRunDownAngle:80, minimumStroke:2, maximumStroke:18},
      simulation:{leadInMm:0.5, maximumAxisAngle:0.1, prevailingTorque:0.1, stiffnessNmPerRad:20, stripTorque:15}};
  }
  static function bad(change:SceneScrew->Void, diagnostic:String):Void {
    var candidate = job(); change(candidate);
    var error = "";
    try SceneScrewDefinition.validate(candidate,["screw1","work","nest"]) catch (value:Dynamic) error = Std.string(value);
    check(error.indexOf(diagnostic) >= 0, 'Expected "$diagnostic", got "$error"');
  }
  public static function namedFace(data:SceneArtifactData, thread:ThreadRenderData):Void {
    var flat:AssemblyDefinition = {schemaVersion:AssemblyDefinitionCodec.VERSION,id:"screw-test",lengthUnit:"mm",
      definitions:[], joints:[], occurrences:[for (id in ["screw1","work","nest"])
        {id:id,definition:"part",initialPose:AssemblyFrames.identity()}],
      sensors:[{id:"nest/present",kind:"presence",occurrence:"nest",connector:"sense",range:8}]};
    var candidate = job();
    candidate.hole = {x:0.004,y:0.005,z:0.006,qx:1,qy:0,qz:0,qw:0};
    candidate.seat = AssemblyFrames.compose(candidate.hole,AssemblyFrames.translation(0,0,0.01));
    candidate.entry = candidate.hole;
    candidate.leadMm = 2; candidate.hand = -1;
    SceneScrewDefinition.validateScene(candidate,flat,data);
    check(true,"Named thread resolves its own millimetre origin, hand and multi-start lead");
    function refused(diagnostic:String):Void {
      var error = "";
      try SceneScrewDefinition.validateScene(candidate,flat,data) catch (value:Dynamic) error = Std.string(value);
      check(error.indexOf(diagnostic) >= 0,'Expected "$diagnostic", got "$error"');
    }
    candidate.threadFace = "other:cyl.side";
    refused("has no internal thread");
    candidate.threadFace = thread.faceName; candidate.hand = 1;
    refused("lead or hand");
    candidate.hand = -1;
    thread.origin[0] = 4.1;
    refused("hole frame disagrees");
    thread.origin[0] = 4;
    candidate.presentSensor = "elsewhere/present";
    refused("missing or ambiguous");
    candidate.presentSensor = "nest/present";
    data.parts[0].threads = [thread,thread];
    refused("named thread is ambiguous");
    data.parts[0].threads = [thread];
    Sys.println("Scene screw named faces: 6 checks passed");
  }
  public static function run():Void {
    var source = job();
    var restored = SceneScrewDefinition.decode(haxe.Json.parse(haxe.Json.stringify(source)));
    SceneScrewDefinition.validate(restored,["screw1","work","nest"]);
    check(restored.screw == "screw1" && restored.threadFace == "bore:cyl.side", "Named screw and face survive decoding");
    check(SceneScrewDefinition.runDownMm(restored) == 10 && restored.approach == 0.04,
      "Run-down converts metre frames to mm without changing approach units");
    check(restored.program.targetTorque == 10 && restored.simulation.stiffnessNmPerRad == 20,
      "Qualified program and simulation response remain independent");
    bad(s -> s.screw = "missing", 'occurrence "missing"');
    bad(s -> s.threadFace = "", "topological face names");
    bad(s -> s.screw = "work", "own occurrence");
    bad(s -> s.entry.qw = 2, "unit rotation");
    bad(s -> s.entry.x = 0.001, "named thread axis");
    bad(s -> s.entry.z = 0.001, "behind the thread entry");
    bad(s -> s.seat.qx = 1, "unit rotation");
    bad(s -> s.hand = 0, "thread hand");
    bad(s -> s.leadMm = 0, "lead and approach");
    bad(s -> s.program.minimumStroke = 18, "no room to follow");
    bad(s -> s.program.minimumAngle = 2, "inconsistent windows");
    bad(s -> s.simulation.prevailingTorque = -1, "prevailing torque");
    for (value in [1.5,2147483648.0]) {
      var raw:Dynamic = haxe.Json.parse(haxe.Json.stringify(job()));
      Reflect.setField(raw,"slot",value);
      var refused = false;
      try { var ignored = SceneScrewDefinition.decode(raw); } catch (_:Dynamic) refused = true;
      check(refused,"Fractional and overflowing slots are rejected before integer conversion");
    }
    Sys.println('Scene screw definitions: $checks checks passed');
  }
}
