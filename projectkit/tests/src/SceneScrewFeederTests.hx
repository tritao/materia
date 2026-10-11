import materia.project.SceneScrewInvocation.ScrewInvocation;
import materia.project.SceneScrewFeeder;
import materia.project.SceneScrewFeeder.SceneScrewFeeders;
import materia.project.SceneArtifact.SceneArtifactMission;
import materia.assembly.AssemblyFrames;

/** Wire identity and numeric fields must survive the mission envelope without coercion. */
class SceneScrewFeederTests {
  static var checks = 0;
  static function check(on:Bool,message:String):Void {checks++; if (!on) throw message;}
  static function refuses(source:Dynamic):Void {
    var failure = false;
    try {var result = SceneScrewFeeders.decode(source);} catch (_:Dynamic) failure = true;
    check(failure,"Feeder wire must reject an invalid integer");
  }
  public static function run():Void {
    checks = 0;
    var feeder:SceneScrewFeeder = {id:"bowl",presenter:"nest",presentSensor:"nest/present",inventory:["a","b"],
      deposit:AssemblyFrames.translation(0,0,0.03),bufferCapacity:2,trackDelay:0.1,deliveryDelay:0.2,deliveryTimeout:2};
    var mission:SceneArtifactMission = {steps:[],screwFeeders:[feeder]};
    var source = @:privateAccess materia.project.SceneArtifact.encodeMission(mission);
    var decoded = @:privateAccess materia.project.SceneArtifact.decodeMission(haxe.Json.parse(source));
    var paths = decoded.screwFeeders;
    check(paths != null && paths.length == 1 && paths[0].inventory.join(",") == "a,b","Mission wire retains inventory identity/order");
    check(paths != null && paths[0].deposit.z == 0.03 && paths[0].bufferCapacity == 2,"Mission wire retains deposit units and buffering");
    var envelope:Dynamic = haxe.Json.parse(haxeon.wire.JsonWire.encode([feeder]));
    var value:Array<Dynamic> = cast Reflect.field(envelope,"value");
    var fields:Array<Dynamic> = cast value[0];
    var capacity:Array<Dynamic> = null;
    for (pair in fields) {
      var tuple:Array<Dynamic> = cast pair;
      var label:String = cast tuple[0];
      if (label.indexOf(":bufferCapacity") >= 0) capacity = tuple;
    }
    if (capacity == null) throw "Generated feeder schema lost its capacity ID";
    capacity[1] = "2"; refuses(envelope);
    capacity[1] = 4294967298.0; refuses(envelope);
    capacity[1] = 2.5; refuses(envelope);
    capacity[1] = 2;
    feeder.deposit.x = 4294967296.0;
    var large = SceneScrewFeeders.decode(haxe.Json.parse(materia.project.SceneWireJson.preserveNumbers(haxeon.wire.JsonWire.encode([feeder]))));
    check(large[0].deposit.x == feeder.deposit.x,"Feeder Float coordinates cannot wrap through Int32 parsing");
    var job = @:privateAccess SceneScrewTests.job();
    var feed:SceneScrewFeeder = {id:"bowl",presenter:"nest",presentSensor:"nest/present",inventory:["screw1"],
      deposit:AssemblyFrames.translation(0,0,0.1),bufferCapacity:2,trackDelay:0.1,deliveryDelay:0.2,deliveryTimeout:2};
    var flat:materia.assembly.AssemblyDefinition = {schemaVersion:materia.assembly.AssemblyDefinitionCodec.VERSION,
      id:"feeder-test",lengthUnit:"mm",definitions:[],joints:[],
      occurrences:[for (id in ["screw1","work","nest","stock"]) {id:id,definition:"same",initialPose:AssemblyFrames.identity()}],
      sensors:[{id:"nest/present",kind:"presence",occurrence:"nest",connector:"sense",range:8}]};
    var scene:materia.project.SceneArtifact.SceneArtifactData = {metresPerUnit:0.001,parts:[],assemblyDefinition:flat,
      mission:{steps:[{kind:"screw",screw:Single(job)}],screwFeeders:[feed]}};
    SceneScrewFeeders.validate([feed],scene); check(true,"Feeder packet resolves its named free supply and sensor");
    function invalid(change:Void->Void, restore:Void->Void):Void {
      change(); var failed = false;
      try SceneScrewFeeders.validate([feed],scene) catch (_:Dynamic) failed = true;
      restore(); check(failed,"Invalid feeder ownership or commissioning must be rejected");
    }
    invalid(() -> feed.deposit.x = 0.1,() -> feed.deposit.x = 0);
    invalid(() -> feed.bufferCapacity = 0,() -> feed.bufferCapacity = 2);
    invalid(() -> feed.deliveryTimeout = 0.1,() -> feed.deliveryTimeout = 2);
    invalid(() -> feed.deposit.qw = 2,() -> feed.deposit.qw = 1);
    invalid(() -> feed.presentSensor = "missing",() -> feed.presentSensor = "nest/present");
    invalid(() -> feed.inventory = ["screw1","screw1"],() -> feed.inventory = ["screw1"]);
    invalid(() -> feed.inventory = ["work"],() -> feed.inventory = ["screw1"]);
    invalid(() -> {feed.inventory = ["screw1","stock"]; flat.occurrences[3].definition = "wrong";},
      () -> {feed.inventory = ["screw1"]; flat.occurrences[3].definition = "same";});
    Sys.println('Scene screw feeder wire tests passed ($checks checks)');
  }
}
