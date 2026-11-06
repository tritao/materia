package tests;

import robotkit.model.*;
import toolpathkit.motion.*;

/** Master sampling is independent of the motor's servo loop and uses the measured spindle coordinate. */
class SpindleEncoderTests {
  static var checks = 0;
  static function check(ok:Bool, label:String):Void {
    checks++;
    if (!ok) throw label;
  }
  public static function run():Void {
    checks = 0;
    var model = new RobotModel("belt-spindle");
    var base = model.addLink(new Link("base"));
    for (id in ["B", "motor", "C"]) model.addJoint(new Joint(id, Continuous, base, model.addLink(new Link(id + "-shaft"))));
    model.addCoupling(new JointCoupling("belt", "B", "motor", 3, 0));
    var cartridge = model.addEncoder(Encoder.perRevolution("cartridge", "B", Absolute, 524288));
    var motor = model.addEncoder(Encoder.perRevolution("motor-feedback", "motor", Absolute, 16777216));
    var table = model.addEncoder(Encoder.perRevolution("table-feedback", "C", Absolute, 8388608));
    var motorDrive = model.addActuator(new Actuator("motor-drive", 10, 1000, SimpleTransmission("motor", 1, 0)));
    motorDrive.encoder = motor.id;
    motorDrive.positionLoopRate = 8000;
    motorDrive.processVelocity = new ProcessVelocityDrive("spindle.speed", "spindle.direction", 2 * Math.PI / 60);
    var tableDrive = model.addActuator(new Actuator("table-drive", 10, 100, SimpleTransmission("C", 1, 0)));
    tableDrive.encoder = table.id;
    tableDrive.positionLoopRate = 2000;
    var binding = new MachineBinding("axes", "X", "Y", "Z", 0.02, new ToolpathChannels("", "spindle"));
    var channels = GearBinding.install(binding, model, 4000, ["B" => 123, "C" => 456], ["C"]);
    var master = binding.gearEncoders.get("B"), follower = binding.gearEncoders.get("C");
    check(master != null && master.quantum == cartridge.resolution() && master.latencyNs == 123 && master.loopHz == 4000,
      "Cartridge master uses its own encoder and explicit controller clock without an actuator on B");
    check(follower != null && follower.quantum == table.resolution() && follower.latencyNs == 456 && follower.loopHz == 2000,
      "Table follower keeps its feedback and servo rate ceiling");
    check(channels.length == 1 && !binding.gearEncoders.exists("motor"), "Only the independently planned table is engaged");
    model.addEncoder(Encoder.perRevolution("cartridge-alias", "B", Absolute, 524288));
    GearBinding.install(binding, model, 4000, ["B" => 0, "C" => 0], ["C"]);
    var alias = binding.gearEncoders.get("B");
    check(alias != null && alias.quantum == cartridge.resolution(), "Equivalent exported encoder aliases agree");
    model.addEncoder(Encoder.perRevolution("conflicting-cartridge", "B", Absolute, 4096));
    var rejected = false;
    try GearBinding.install(binding, model, 4000, ["B" => 0, "C" => 0], ["C"]) catch (_:Dynamic) rejected = true;
    check(rejected, "Conflicting master encoder resolutions need explicit selection");
    model.encoders.pop();
    tableDrive.encoder = motor.id;
    rejected = false;
    try GearBinding.install(binding, model, 4000, ["B" => 0, "C" => 0], ["C"]) catch (_:Dynamic) rejected = true;
    check(rejected, "Follower feedback cannot silently use an encoder on another joint");
    Sys.println('Spindle encoder: $checks checks passed');
  }
}
