import machinekit.power.BatteryPack;
import machinekit.power.Inverter;
import machinekit.power.IsolatedDcConverter;
import machinekit.welding.WeldingPowerSource;
import machinekit.welding.GasCylinder;
import machinekit.welding.WireFeeder;
import materia.assembly.AssemblyFrames;
import machinekit.component.Solids;
import MobileBase.MobileBaseLayout;

/** The same torch arm on a payload platform, with its welding services carried onboard. */
class MobileWeldingRobot extends MobileBase {
  public static inline var EQUIPMENT_GAP:Float = 40;
  public final source:WeldingPowerSource;
  public final inverter:Inverter;
  public final cylinder:GasCylinder;
  public final feeder:WireFeeder;
  public final computerSupply:IsolatedDcConverter;

  public function new(?pack:BatteryPack, ?powerSource:WeldingPowerSource, ?powerInverter:Inverter) {
    var storage = pack == null ? new BatteryPack(48, 25000, 1000, 600, 180, 230, 4) : pack;
    var supply = powerSource == null ? new WeldingPowerSource() : powerSource;
    var conversion = powerInverter == null ? new Inverter(48, 230, 10000, 600, 400, 300, 45) : powerInverter;
    if (storage.nominalVolts != conversion.inputVolts || storage.outlets < 4)
      throw "The mobile welding pack must match the inverter and supply four branches";
    var weldingArm = new RobotArm(false, new ArmWeldingTool());
    weldingArm.replaceComponent("powerSupply", new IsolatedDcConverter(storage.nominalVolts, 48, 30, 6));
    super(weldingArm, storage.nominalVolts, payloadLayout(storage, supply, conversion));
    source = supply;
    inverter = conversion;
    cylinder = new GasCylinder();
    feeder = new WireFeeder(150, 240, 180);
    computerSupply = new IsolatedDcConverter(storage.nominalVolts, 24, 10, 1);

    var equipmentX = -storage.length / 2 - MobileBase.WHEEL_DIAMETER / 2 - EQUIPMENT_GAP;
    var top = DECK_Z + DECK_THICKNESS;
    addComponent("source", source, AssemblyFrames.translation(equipmentX, -inverter.width / 2 - EQUIPMENT_GAP, top));
    addComponent("inverter", inverter, AssemblyFrames.translation(equipmentX, source.depth / 2 + EQUIPMENT_GAP, top));
    addComponent("cylinder", cylinder, AssemblyFrames.translation(equipmentX - source.width / 2 - EQUIPMENT_GAP -
      GasCylinder.DIAMETER / 2, 0, top));
    // All carried equipment is attached to the deck in the robot's mate tree.
    for (name in ["source", "inverter", "cylinder"]) {
      var poses = solvedPoses();
      var local = AssemblyFrames.compose(AssemblyFrames.inverse(poses.get("deck")),
        AssemblyFrames.compose(poses.get(name), memberConnectorFrame(name, "base")));
      addMemberConnector("deck", name, local);
      addMate('$name-mount', "fixed", "deck", name, name, "base");
    }
    addComponent("computerSupply", computerSupply);
    addMemberConnector("basePlate", "computerSupply", Solids.axial(200, 160, BASE_THICKNESS));
    addMate("computer-supply-mount", "fixed", "basePlate", "computerSupply", "computerSupply", "mount");
    addComponent("feeder", feeder);
    addMemberConnector("arm/upperArm", "feederSeat", AssemblyFrames.alongY(40, 0, 160, 1, 0, 0));
    addMate("feeder-mount", "fixed", "arm/upperArm", "feederSeat", "feeder", "mount");

    connectPorts("pack-inverter", "battery", "power2", "inverter", "dc");
    connectPorts("pack-arm", "battery", "power3", "arm/powerSupply", "dc");
    connectPorts("pack-computer", "battery", "power4", "computerSupply", "dc");
    connectPorts("welder-mains", "inverter", "mains", "source", "mains");
    connectPorts("gas-hose", "cylinder", "gas", "source", "gas");
    connectPorts("weld-cable", "source", "weldPositive", "feeder", "power");
    connectPorts("feeder-gas", "source", "gasOut", "feeder", "gas");
    connectPorts("feeder-control", "source", "feederControl", "feeder", "control");
    connectPorts("torch-power", "feeder", "torchPower", "arm/tool/torch", "power");
    connectPorts("torch-gas", "feeder", "torchGas", "arm/tool/torch", "gas");
    connectPorts("torch-wire", "feeder", "torchWire", "arm/tool/torch", "wire");
    connectPorts("torch-control", "feeder", "torchControl", "arm/tool/torch", "control");
    exposePort("workLead", "source", "weldNegative");
    exposePort("control", "source", "control");
    exposePort("computerPower", "computerSupply", "power1");
  }

  /** Rear equipment plus the forward arm bay determine the plate, rather than a fixed part name. */
  public static function payloadLayout(pack:BatteryPack, source:WeldingPowerSource, inverter:Inverter):MobileBaseLayout {
    var gap = EQUIPMENT_GAP;
    var equipmentX = -pack.length / 2 - MobileBase.WHEEL_DIAMETER / 2 - gap;
    var halfLength = Math.max(pack.length + MobileBase.WHEEL_DIAMETER / 2 + 2 * gap,
      -equipmentX + source.width / 2 + gap + GasCylinder.DIAMETER + gap);
    halfLength = Math.max(halfLength, -equipmentX + inverter.length / 2 + gap);
    var plateWidth = Math.max(MobileBase.WIDTH, Math.max(pack.width + 2 * gap,
      source.depth + inverter.width + 3 * gap));
    return {length: 2 * halfLength, width: plateWidth, payloadX: halfLength - 300,
      batteryX: equipmentX, battery: pack,
      driveSupply: new IsolatedDcConverter(pack.nominalVolts, 48, 30, 2)};
  }
}
