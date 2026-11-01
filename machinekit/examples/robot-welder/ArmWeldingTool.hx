import machinekit.robotics.ArmTool;
import machinekit.assembly.MachineAssembly;
import machinekit.robotics.EndEffector;
import machinekit.robotics.EndEffectorPlate;
import machinekit.robotics.RobotFlange;
import machinekit.welding.WeldingTorch;
import machinekit.welding.WeldingTorchNeck;
import machinekit.welding.WeldingTorchNozzle;

/** Welding tool for the arm's ISO 9409-1 style tool flange: an adapter plate and a MIG torch
 * with a breakaway mount. Its `tcp` working frame is the wire tip at nominal stickout, +Z along
 * the wire. The torch takes weld current, gas, wire and its trigger from a feeder, so the arm
 * publishes the `toolTcp` connector and the torch's four inlets as `torchPower`, `torchGas`,
 * `torchWire` and `torchControl`.
 */
class ArmWeldingTool implements ArmTool {
  /** The torch's inlets, as the tool and the arm name them. */
  static final INLETS:Array<{tool:String, arm:String}> = [
    {tool: "power", arm: "torchPower"},
    {tool: "gas", arm: "torchGas"},
    {tool: "wire", arm: "torchWire"},
    {tool: "control", arm: "torchControl"}
  ];

  public final bendDegrees:Float;

  public function new(bendDegrees:Float = 45) {
    this.bendDegrees = bendDegrees;
  }

  public function build(flange:RobotFlange):EndEffector {
    var result = new EndEffector();
    result.addComponent("plate", new EndEffectorPlate(flange));
    var torch = new WeldingTorch(bendDegrees);
    var neck = new WeldingTorchNeck(bendDegrees);
    var nozzle = new WeldingTorchNozzle(bendDegrees);
    neck.includedInPurchase(torch.bom.partNumber);
    nozzle.includedInPurchase(torch.bom.partNumber);
    result.addComponent("torch", torch);
    result.addComponent("neck", neck);
    result.addComponent("nozzle", nozzle);
    result.mount("plate", "robot");
    result.addMate("torch-mate", "fixed", "plate", "tool", "torch", "robot");
    result.addMate("neck-mate", "fixed", "torch", "neck", "neck", "base");
    result.addMate("nozzle-mate", "fixed", "neck", "nozzle", "nozzle", "base");
    for (inlet in INLETS) result.exposePort(inlet.tool, "torch", inlet.tool);
    result.workingFrame("tcp", "torch", "tcp", true);
    return result;
  }

  public function expose(arm:MachineAssembly):Void {
    arm.exposeConnector("toolTcp", "tool/torch", "tcp");
    for (inlet in INLETS) arm.exposePort(inlet.arm, "tool/torch", inlet.tool);
  }
}
