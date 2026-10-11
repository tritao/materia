import materia.project.SceneBlowFeed;
import materia.project.SceneBlowFeed.SceneBlowFeeds;
import materia.project.SceneScrewDefinition;
import materia.project.SceneScrewInvocation.ScrewInvocation;
import materia.assembly.AssemblyFrames;

/** Explicit acquisition modes and one receiver packet survive the current wire format. */
class SceneBlowFeedTests {
  static var checks = 0;
  static function check(on:Bool,label:String):Void {if (!on) throw label; checks++;}
  static function rejects(run:Void -> Void,label:String):Void {var failed = false; try run() catch (_:Dynamic) failed = true; check(failed,label);}
  public static function run():Void {
    checks = 0;
    var job = @:privateAccess SceneScrewTests.job();
    job.delivery = BlowFeed("blow"); job.presenter = null; job.presentSensor = null; job.pickup = null;
    SceneScrewDefinition.validate(job,["screw1","work"]); check(true,"Blow mode needs no remote presenter");
    job.pickup = AssemblyFrames.identity(); rejects(() -> SceneScrewDefinition.validate(job,["screw1","work"]),"Blow mode rejects a fake pickup"); job.pickup = null;
    var path:SceneBlowFeed = {id:"blow",receiver:"nose",socketConnector:"receipt",receiptSensor:"nose/present",inventory:["screw1"],
      bufferCapacity:2,trackDelay:0.1,deliveryDelay:0.4,deliveryTimeout:3,airPulse:0.2,airPressureBar:4,tube:"tube",
      externalTubeLengthMm:2500,minimumBendRadiusMm:200,receivingAngularSpeed:0.01,receivingFeedSpeed:0.001};
    var flat:materia.assembly.AssemblyDefinition = {schemaVersion:materia.assembly.AssemblyDefinitionCodec.VERSION,
      id:"blow-test",lengthUnit:"mm",definitions:[{id:"same",connectors:[{name:"receipt",frame:AssemblyFrames.identity()}]}],joints:[],
      occurrences:[for (id in ["screw1","work","nose","tube"]) {id:id,definition:"same",initialPose:AssemblyFrames.identity()}],
      sensors:[{id:"nose/present",kind:"presence",occurrence:"nose",connector:"receipt",range:10}]};
    var scene:materia.project.SceneArtifact.SceneArtifactData = {metresPerUnit:0.001,parts:[],assemblyDefinition:flat,
      mission:{steps:[{kind:"screw",screw:Single(job)}],blowFeeds:[path]}};
    SceneBlowFeeds.validate([path],scene); check(true,"Named tube/receiver, actual sensor and owned inventory qualify");
    var source = @:privateAccess materia.project.SceneArtifact.encodeMission(scene.mission);
    var decoded = @:privateAccess materia.project.SceneArtifact.decodeMission(haxe.Json.parse(source));
    var paths = decoded.blowFeeds;
    check(paths != null && paths.length == 1 && paths[0].inventory[0] == "screw1" && paths[0].airPulse == 0.2,"Mission wire retains inventory and air timing");
    var again = materia.project.SceneScrewInvocation.job(decoded.steps[0].screw,decoded.steps);
    check(again != null && again.presenter == null && again.pickup == null,"Saved blow delivery never grows presenter fields");
    rejects(() -> SceneBlowFeeds.validate(null,scene),"Blow job cannot lose its delivery path");
    function invalid(change:Void->Void,restore:Void->Void):Void {
      change(); rejects(() -> SceneBlowFeeds.validate([path],scene),"Invalid blow-feed commissioning or ownership rejected"); restore();
    }
    invalid(() -> path.receiptSensor = "missing",() -> path.receiptSensor = "nose/present");
    invalid(() -> path.socketConnector = "missing",() -> path.socketConnector = "receipt");
    invalid(() -> path.airPulse = 3,() -> path.airPulse = 0.2);
    invalid(() -> path.airPressureBar = 0,() -> path.airPressureBar = 4);
    invalid(() -> path.externalTubeLengthMm = 100,() -> path.externalTubeLengthMm = 2500);
    invalid(() -> path.inventory.push("screw1"),() -> path.inventory.pop());
    var envelope:Dynamic = haxe.Json.parse(haxeon.wire.JsonWire.encode([path]));
    var entries:Array<Dynamic> = cast Reflect.field(envelope,"value"), fields:Array<Dynamic> = cast entries[0];
    var capacity:Array<Dynamic> = null;
    for (pair in fields) {var field:Array<Dynamic> = cast pair; var label:String = cast field[0]; if (label.indexOf(":bufferCapacity") >= 0) capacity = field;}
    if (capacity == null) throw "Blow feed lost capacity wire ID";
    capacity[1] = "2"; rejects(() -> {var value = SceneBlowFeeds.decode(envelope);},"Wire capacity cannot be a numeric string");
    capacity[1] = 4294967298.0; rejects(() -> {var value = SceneBlowFeeds.decode(envelope);},"Wire capacity cannot wrap Int32");
    trace('Scene blow-feed wire tests passed ($checks checks)');
  }
}
