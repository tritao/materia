import hobber.*;
import cadkit.modeling.AssemblyState;
import gearkit.*;
import machinekit.assembly.*;
import machinekit.assembly.AxisBuilder;
import machinekit.component.*;
import machinekit.control.BissMaster;
import machinekit.design.DesignRequirement;
import machinekit.motion.*;
import machinekit.hobbing.*;
import machinekit.power.*;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyDefinition.AssemblyJointType;

class HobberElectricalRack extends HobberUnit {

  public function new(skeleton:HobberSkeleton, tableSpec:RotaryAxisSpec) {
    super("electrical", skeleton);
    var supply = new ThreePhaseRectifier(
      new AcMains(ThreePhase, new AcRmsVoltage(400), 50),
      new DcCurrent(40),
      6,
      240,
      120,
      100,
      15000
    );
    // The rack is a free-standing floor unit with its own base plate on the floor, not part of the machine's structure (it was
    // mated to the bed's lifting seat 650 mm away); the wiring to the machine is by ports, not by a mate.
    addComponent("electricalPanel", new HobberElectricalPanel(), layout.electricalPanelPose());
    fixedPart("powerSupply", supply, layout.electricalDevicePose(0, 130), "electricalPanel");
    exposePort("mains", "powerSupply", "mains");
    // The 48 V bus for the A actuator, from the same mains: it takes back what the actuator regenerates in a brake chopper and resistor.
    fixedPart("supply48", new ThreePhaseDcSupply(new AcMains(ThreePhase, new AcRmsVoltage(400), 50), 48, 20, 1, 125, 125, 120, 300),
      layout.electricalDevicePose(-105, HobberElectricalPanel.ELECTRICAL_HEIGHT - 120), "electricalPanel");
    exposePort("mains48", "supply48", "mains");
    // The three sealed scales are read by a BiSS-C master module of the controller.
    fixedPart("bissMaster", new BissMaster(4), layout.electricalDevicePose(110, HobberElectricalPanel.ELECTRICAL_HEIGHT - 120), "electricalPanel");
    var ids=["Xdrive","Zdrive","Ydrive","table-drive","hob-drive"];
    for (i in 0...ids.length) {
      var id=ids[i];
      fixedPart(id+"-driver",new MotorDriver("GENERIC-400V-SERVO-AMP",new PhaseRmsCurrent(id=="table-drive"?tableSpec.number("peakCurrent"):20)),
        layout.electricalDevicePose(-140+(i%3)*140,300+Std.int(i/3)*250),"electricalPanel");
      connectPorts(id+"-power","powerSupply","power"+(i+1),id+"-driver","power");
    }

  }

}
