import RobotArm.ArmTable;
import RobotArm.ArmBlock;
import machinekit.component.BomItem;
import machinekit.component.Solids;
import machinekit.welding.GasCylinder;
import machinekit.welding.WeldingEquipment;
import machinekit.welding.WeldingEquipment.WeldingEquipmentData;
import machinekit.welding.WeldingPowerSource;
import machinekit.welding.Weldment;
import machinekit.welding.WireFeeder;
import machinekit.welding.WorkClamp;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

import machinekit.assembly.MachineAssembly;
import machinekit.welding.WeldMetal;

/** The external track supplies the travel for a 2.6 m CAD-derived fillet. */
class TrackWelder extends machinekit.gantry.LinearTrack {
	public final arm:RobotArm;
	public final feeder = new WireFeeder(150, 240, 180);
	public final source = new WeldingPowerSource();
	public final cylinder = new GasCylinder();
	public final clamp = new WorkClamp();
	public function new() {
		super(3000);
		arm = new RobotArm(false, new ArmWeldingTool());
		includeArm("arm", arm);
		var top = RobotArm.TABLE_TOP + mountZero.z;
		addComponent("table", new ArmTable(3500, 500, top), AssemblyFrames.translation(1500, RobotArm.TABLE_CENTRE_Y, 0));
		addMemberConnector("table", "workSeat", Solids.axial(0, 0, top));
		var work = new MachineAssembly();
		work.addComponent("basePlate", new ArmBlock(3000, 180, 10, "steel", "Long base plate"));
		work.addComponent("upright", new ArmBlock(2600, 8, 80, "steel", "Long upright"));
		work.addMate("upright-mate", "fixed", "basePlate", "top", "upright", "base");
		work.addComponent("weldMetal", new WeldMetal());
		work.addMate("metal-mate", "fixed", "basePlate", "base", "weldMetal", "base");
		include("work", work);
		addMate("work-mate", "fixed", "table", "workSeat", "work/basePlate", "base");
		addMemberConnector("work/basePlate", "clampSeat", Solids.axial(-1400, 75, 10));
		addComponent("clamp", clamp);
		addMate("clamp-mate", "fixed", "work/basePlate", "clampSeat", "clamp", "contact");
		addComponent("source", source, AssemblyFrames.translation(-800, 100, 0));
		addComponent("feeder", feeder, AssemblyFrames.translation(-700, 300, 0));
		addComponent("cylinder", cylinder, AssemblyFrames.translation(-800, 500, 0));
		function line(partNumber:String, description:String):BomItem
			return {partNumber: partNumber, description: description, quantity: 1, material: null};
		connectPorts("gas-hose", "cylinder", "gas", "source", "gas", line("HOSE-GAS-6M", "Shielding gas hose, 6 m"));
		connectPorts("weld-cable", "source", "weldPositive", "feeder", "power", line("CABLE-WELD-35-6M", "Weld cable 35 mm², 6 m"));
		connectPorts("feeder-gas", "source", "gasOut", "feeder", "gas", line("HOSE-GAS-6M", "Shielding gas hose, 6 m"));
		connectPorts("feeder-control", "source", "feederControl", "feeder", "control",
			line("CABLE-CONTROL-12-6M", "Welder control cable, 12 core, 6 m"));
		connectPorts("work-lead", "source", "weldNegative", "clamp", "lead", line("CABLE-WORK-35-6M", "Work lead 35 mm², 6 m"));
		connectPorts("torch-power", "feeder", "torchPower", "arm/tool/torch", "power");
		connectPorts("torch-gas", "feeder", "torchGas", "arm/tool/torch", "gas");
		connectPorts("torch-wire", "feeder", "torchWire", "arm/tool/torch", "wire");
		connectPorts("torch-control", "feeder", "torchControl", "arm/tool/torch", "control");
		addBomItem(line("HOSEPACK-TRACK-6M", "Torch hose pack, 6 m: power, gas, wire liner and control"));
		// The power source takes mains power and the gantry controller.
		exposePort("mains", "source", "mains");
		exposePort("control", "source", "control");
	}
	public function weldment():Weldment return new Weldment("work/basePlate", ["work/basePlate", "work/upright"]).join("work/basePlate", "work/upright", 5, 1);
	public function equipment():WeldingEquipmentData return WeldingEquipment.of(this, [weldment()]);
}
