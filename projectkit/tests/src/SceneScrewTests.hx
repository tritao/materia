import materia.project.SceneScrewDefinition;
import materia.project.SceneScrewDefinition.SceneScrew;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinitionCodec;
import materia.project.SceneArtifact.SceneArtifactData;
import materia.project.ThreadRenderData;
import materia.project.SceneScrewInvocation;
import materia.project.SceneScrewInvocation.ScrewInvocation;
import materia.project.SceneWireJson;
import materia.project.SceneArtifact.SceneArtifactMissionStep;

/** Saved screw missions keep identity and qualified units at the boundary. */
class SceneScrewTests {
  static var checks = 0;
  static function check(ok:Bool, message:String):Void {
    checks++;
    if (!ok) throw message;
  }
  static function job():SceneScrew {
    return {delivery:Presented,fastening:"work/screw1:bore", screw:"screw1", frame:"work", threadFace:"bore:cyl.side",
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
    flat.occurrences.push({id:"spare",definition:"part",initialPose:AssemblyFrames.identity()});
    flat.occurrences.push({id:"reserve",definition:"nest",initialPose:AssemblyFrames.identity()});
    flat.occurrences.push({id:"bin",definition:"bin",initialPose:AssemblyFrames.identity()});
    var sensors = flat.sensors;
    if (sensors == null) throw "Recovery fixture lost its presence sensors";
    sensors.push({id:"reserve/present",kind:"presence",occurrence:"reserve",connector:"sense",range:8});
    candidate.recovery = {replacement:{screw:"spare",presenter:"reserve",presentSensor:"reserve/present",pickup:AssemblyFrames.identity()},
      rejectMember:"bin",reject:AssemblyFrames.translation(0,0,0.1)};
    SceneScrewDefinition.validateScene(candidate,flat,data);
    check(true,"The replacement resolves its exact component and scoped presence sensor");
    flat.occurrences[3].definition = "similar-part";
    refused("exact screw component definition");
    flat.occurrences[3].definition = "part";
    sensors[1].occurrence = "nest";
    refused("replacement presenter sensor");
    sensors[1].occurrence = "reserve";
    sensors.push(sensors[1]);
    refused("replacement presenter sensor");
    sensors.pop();
    flat.occurrences.pop();
    refused('recovery occurrence "bin"');
    Sys.println("Scene screw recovery identity: 5 checks passed");
  }
  public static function run():Void {
    var source = job();
    var restored = SceneScrewDefinition.decode(haxeon.wire.JsonWire.encode(source));
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
      var encoded = haxeon.wire.JsonWire.encode(job());
      var malformed = StringTools.replace(encoded,'["14:slot",1]','["14:slot",'+haxe.Json.stringify(value)+']');
      if (malformed == encoded) throw "Wire slot mutation did not find its explicit field ID";
      var refused = false;
      try { var ignored = SceneScrewDefinition.decode(malformed); } catch (_:Dynamic) refused = true;
      check(refused,'Fractional and overflowing slots are rejected before integer conversion: $value');
    }
    var recovering = job();
    recovering.recovery = {replacement:{screw:"spare",presenter:"reserve",presentSensor:"reserve/present",pickup:AssemblyFrames.translation(0,0,0.1)},
      rejectMember:"bin",reject:AssemblyFrames.translation(0,0,0.1)};
    var restoredRecovery = SceneScrewDefinition.decode(haxeon.wire.JsonWire.encode(recovering));
    SceneScrewDefinition.validate(restoredRecovery,["screw1","work","nest","spare","reserve","bin"]);
    var replacement = SceneScrewDefinition.replacement(restoredRecovery);
    check(replacement.screw == "spare" && replacement.fastening == recovering.fastening && replacement.threadFace == recovering.threadFace,
      "A replacement changes only supply identity while preserving the exact connection");
    var same = recovering.recovery;
    if (same == null) throw "Recovery fixture lost its supply";
    same.replacement.screw = "screw1";
    var rejected = false;
    try SceneScrewDefinition.validate(recovering,["screw1","work","nest","reserve","bin"]) catch (_:Dynamic) rejected = true;
    check(rejected,"A replacement cannot reuse the original occurrence");
    same.replacement.screw = "spare"; same.reject.qw = 2;
    rejected = false;
    try SceneScrewDefinition.validate(recovering,["screw1","work","nest","spare","reserve","bin"]) catch (_:Dynamic) rejected = true;
    check(rejected,"Reject frames require a unit rotation");
    Sys.println('Scene screw definitions: $checks checks passed');
    runInvocations();
  }
  static function runInvocations():Void {
    checks = 0;
    var qualified = job();
    qualified.recovery = {replacement:{screw:"spare",presenter:"reserve",presentSensor:"reserve/present",pickup:qualified.pickup},
      rejectMember:"bin",reject:qualified.pickup};
    var staged:Array<SceneArtifactMissionStep> = [{kind:"screw",screw:Snug(qualified,2)},
      {kind:"screw",screw:Final(qualified.fastening)}];
    SceneScrewInvocation.validate(staged);
    var invocation:ScrewInvocation = Snug(qualified,2);
    var restored:ScrewInvocation = haxeon.wire.JsonWire.decode(haxeon.wire.JsonWire.encode(invocation));
    check(switch restored {case Snug(job,slot):slot == 2 && job.slot == 1 && job.recovery != null; case _:false;},
      "Wire IDs retain distinct snug/final slots and one replacement definition");
    check(SceneScrewInvocation.job(staged[1].screw,staged) == qualified,
      "Final resolves the exact canonical snug job without another geometry or supply copy");
    function refused(steps:Array<SceneArtifactMissionStep>):Void {
      var failed = false; try SceneScrewInvocation.validate(steps) catch (_:Dynamic) failed = true;
      check(failed,"Invalid stage or supply ownership must be rejected");
    }
    refused([staged[1]]);
    refused([staged[0]]);
    refused([staged[0],staged[1],staged[1]]);
    refused([staged[1],staged[0]]);
    refused([{kind:"screw",screw:Snug(qualified,1)},staged[1]]);
    refused([{kind:"screw",screw:Single(qualified)},staged[0],staged[1]]);
    var other = job(); other.fastening = "second-connection";
    refused([{kind:"screw",screw:Single(qualified)},{kind:"screw",screw:Single(other)}]);
    var unqualified = job();
    refused([{kind:"screw",screw:Snug(unqualified,2)},{kind:"screw",screw:Final(unqualified.fastening)}]);
    refused([null]);
    var unnamed = job(); unnamed.fastening = "";
    refused([{kind:"screw",screw:Single(unnamed)}]);
    var missingSupply = job(); missingSupply.screw = "";
    refused([{kind:"screw",screw:Single(missingSupply)}]);
    Sys.println('Scene screw invocation: $checks checks passed');
    runWireNumbers(qualified);
  }
  static function runWireNumbers(qualified:SceneScrew):Void {
    checks = 0;
    var encoded = haxeon.wire.JsonWire.encode(qualified);
    var malformed = StringTools.replace(encoded,'["14:slot",1]','["14:slot","1"]');
    var failed = false;
    try { var ignored = SceneScrewDefinition.decode(malformed); } catch (_:Dynamic) failed = true;
    check(malformed != encoded && failed,"Numeric strings cannot stand in for a qualified integer slot");
    var badId = StringTools.replace(encoded,'"1:runDownSpeed"','"4294967297:runDownSpeed"');
    failed = false;
    try { var ignored = SceneScrewDefinition.decode(badId); } catch (_:Dynamic) failed = true;
    check(badId != encoded && failed,"Nested program field IDs cannot wrap into a valid field through annotated-string parsing");
    var invocation:ScrewInvocation = Snug(qualified,2);
    var staged = haxeon.wire.JsonWire.encode(invocation);
    for (value in ["2147483648","-2147483649","1.5",'"2"']) {
      // Replace the final argument without depending on field order inside the job.
      var end = staged.lastIndexOf(",2]]]");
      if (end < 0) throw "Wire snug mutation did not find its final numeric argument";
      var corrupted = staged.substr(0,end)+','+value+staged.substr(end+2);
      failed = false;
      try { var ignored = SceneScrewInvocation.decode(corrupted); } catch (_:Dynamic) failed = true;
      check(failed,'Snug slot refuses non-integer JSON values: $value');
    }
    var restored = SceneScrewInvocation.decode(staged);
    check(switch restored {case Snug(_,slot):slot == 2; case _:false;},"Current valid snug slot remains decodable");
    var text = '{"coordinate":2147483648,"minimum":-2147483648,"label":"2147483648 \\\"quoted\\\""}';
    var preserved = SceneWireJson.preserveNumbers(text);
    var raw:Dynamic = haxe.Json.parse(preserved);
    check(Reflect.field(raw,"coordinate") == 2147483648.0 && Reflect.field(raw,"minimum") == -2147483648.0,
      "Large Float coordinates retain their value while the signed Int32 boundary remains valid");
    check(Reflect.field(raw,"label") == '2147483648 "quoted"',"Numeric preservation leaves quoted and escaped text unchanged");
    var large = job(), largePickup = large.pickup;
    if (largePickup == null) throw "Fixture lost its presented pickup";
    largePickup.x = 2147483648.0;
    var mission: materia.project.SceneArtifact.SceneArtifactMission = {steps:[{kind:"screw",screw:Single(large)}]};
    var encodedMission = @:privateAccess materia.project.SceneArtifact.encodeMission(mission);
    var decodedMission = @:privateAccess materia.project.SceneArtifact.decodeMission(haxe.Json.parse(SceneWireJson.preserveNumbers(encodedMission)));
    var decodedJob = SceneScrewInvocation.job(decodedMission.steps[0].screw,decodedMission.steps);
    if (decodedJob == null) throw "Decoded mission lost its fastening";
    var decodedPickup = decodedJob.pickup;
    if (decodedPickup == null) throw "Decoded presenter lost its pickup frame";
    check(decodedPickup.x == largePickup.x,
      "Mission envelope encoding preserves large Float coordinates through its intermediate JSON parse");
    Sys.println('Scene wire numbers: $checks checks passed');
  }
}
