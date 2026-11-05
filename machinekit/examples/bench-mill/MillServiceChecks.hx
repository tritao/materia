import machinekit.assembly.AssemblyIntegrity;
import machinekit.assembly.MachineAssembly;
import machinekit.design.DesignRequirement;

/**
 * The service removals of the small mill concept: the ones the machine declares (`MillXYStage`, `MillZAxis`,
 * `MillFramePreview.attachSpindle`) sweep clear against the whole mill, and the procedures that do not work are stated as findings
 * with their blockers, so the finding is kept when the mill changes. One sweep of everything takes minutes (every removal is sampled
 * every 2 mm along its path against every member).
 */
class MillServiceChecks {
  static function check(condition:Bool, message:String):Void {
    if (!condition) throw 'Mill service: $message';
  }

  static function ids(mill:MachineAssembly, prefix:String):Array<String> return [for (m in mill.components()) if (StringTools.startsWith(m.id, prefix)) m.id];

  static function removal(mill:MachineAssembly, id:String, movers:Array<String>, direction:Array<Float>, distance:Float, off:Array<String>):Void
    mill.addDesignRequirement(ServiceRemoval(id, movers, [{direction: direction, distance: distance}], off, []));

  public static function main():Void {
    var reference = new BenchMill(false, false), frame = MillProject.smallFrame();
    var mill = MillFramePreview.concept(reference, frame);
    var declared:Array<String> = [];
    for (r in mill.designRequirements()) switch r {
      case ServiceRemoval(id, _, _, _, _): declared.push(id);
      case _:
    }
    for (id in ["xyStage/yMotorWithdrawal", "xyStage/xMotorWithdrawal", "xyStage/xScrewWithdrawal", "xyStage/yScrewWithdrawal", "zAxis/zMotorWithdrawal",
      "spindle/motorWithdrawal", "spindle/cartridgeWithdrawal"]) check(declared.indexOf(id) >= 0, 'the mill declares $id, has: $declared');

    // The procedures that do not work as built, added to a copy of the mill. Each is the order a service technician would try; the sweep
    // names the member in the way and the distance travelled.
    var blocked = MillFramePreview.concept(reference, frame);
    var motorY = ["xyStage/motorY", "xyStage/motorShaftY", "xyStage/motorCoverY", "xyStage/motorMountY", "xyStage/floatingBearingY", "xyStage/floatingRingY",
      "xyStage/nutBracketY", "xyStage/couplingY"].concat(ids(blocked, "xyStage/motorYBolt")).concat(ids(blocked, "xyStage/nutYBolt"));
    // Out of the back with the nut on it: the 48 mm nut flange meets the Z axis' lower plate, which has only a 24 mm bore for the Y shaft.
    removal(blocked, "yScrewRear", ["xyStage/screwY", "xyStage/nutY"], [0.0, 1.0, 0.0], 700, motorY);
    // Up out of the column: the nut's grease fitting meets the Z adapter plate across the front of the channel.
    removal(blocked, "zScrewUp", ["zAxis/screw", "zAxis/nut", "zAxis/coupling", "zAxis/motorShaft", "zAxis/floatingRing", "zAxis/motorMount"], [0.0, 0.0, 1.0], 950,
      ["zAxis/motor", "zAxis/floatingBearing"].concat(ids(blocked, "zAxis/motorBolt")).concat(ids(blocked, "zAxis/floatingBolt")));
    // Up out of the head: the head's top plate is in the way of the cartridge flange.
    removal(blocked, "spindleUp", ["spindle/spindle", "spindle/rotor", "spindle/springs", "spindle/pullStud", "spindle/holder", "spindle/drawbar", "spindle/gripper",
      "spindle/spindlePulley"], [0.0, 0.0, 1.0], 380, ["spindle/tool", "spindle/belt", "spindle/motorCarrier", "spindle/motor", "spindle/motorShaft", "spindle/motorPulley",
      "spindle/beltGuard", "spindle/reactionFrame", "spindle/cylinder", "spindle/pistonRod", "spindle/piston"].concat(ids(blocked, "spindle/motorNut"))
      .concat(ids(blocked, "spindle/motorBolt")).concat(ids(blocked, "spindle/guardBolt")).concat(ids(blocked, "spindle/carrierBolt")));

    var started = Sys.time();
    var clear = AssemblyIntegrity.service(mill);
    Sys.println('declared removals: ${clear.errors().length} findings in ${Math.round(Sys.time() - started)} s');
    for (item in clear.errors()) Sys.println("  " + item.code + ": " + item.message);
    check(clear.errors().length == 0, "the declared removals are clear: " + [for (item in clear.errors()) item.message].join("; "));

    started = Sys.time();
    var found = AssemblyIntegrity.service(blocked);
    Sys.println('blocked procedures: ${found.errors().length} findings in ${Math.round(Sys.time() - started)} s');
    for (item in found.errors()) Sys.println("  " + item.code + ": " + item.message);
    function blockedBy(code:String, member:String):Bool {
      for (item in found.errors()) if (item.code == "integrity.service." + code && item.message.indexOf("blocked by " + member) >= 0) return true;
      return false;
    }
    check(blockedBy("yScrewRear", "zAxis/floatingMount"), "the Y screw with its nut cannot leave by the rear: the nut meets the Z axis' lower plate");
    check(blockedBy("zScrewUp", "zAxis/fixedMount"), "the Z nut's grease fitting meets the Z adapter plate on the way up");
    check(blockedBy("spindleUp", "zAxis/head"), "the spindle cartridge cannot leave upwards: the head is over it");
    Sys.println("mill service: declared removals clear; the Y screw by the rear, the Z screw upwards and the cartridge upwards are blocked, as stated");
  }
}
