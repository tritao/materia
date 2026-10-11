import machinekit.assembly.MachineAssembly;
import machinekit.screwdriving.ArmScrewdrivingTool;
import machinekit.screwdriving.ScrewdriverBit;
import machinekit.screwdriving.TighteningController;
import machinekit.screwdriving.ScrewPresenter;
import machinekit.screwdriving.FlangeBearingMount;
import machinekit.standard.DeepGrooveBearing;
import materia.assembly.AssemblyFrames;
import machinekit.component.Solids;
import machinekit.pneumatic.VacuumGenerator;
import RobotArm.ArmTable;
import RobotArm.ArmBlock;
/** Equipment cell for the SD3 flange workpiece; screws remain seated until the driving step exists. */
class ScrewdrivingCell extends MachineAssembly {
	public final work:FlangeBearingMount;
	public final tool:ArmScrewdrivingTool;
	public final arm:RobotArm;
	public final controller:TighteningController;
	public final presenter:ScrewPresenter;
	public function new(?bit:ScrewdriverBit, workX:Float = 0, ?bearing:DeepGrooveBearing) {
		super();
		work = new FlangeBearingMount(bearing == null ? DeepGrooveBearing.metric("608") : bearing);
		tool = new ArmScrewdrivingTool(bit == null ? new ScrewdriverBit(work.flange.screw.screwDrive()) : bit);
		arm = new RobotArm(false, tool); controller = new TighteningController();
		presenter = new ScrewPresenter(work.flange.screw);
		include("arm", arm);
		addComponent("table", new ArmTable(700, 400, RobotArm.TABLE_TOP), AssemblyFrames.translation(0, -520, 0));
		addComponent("controller", controller, AssemblyFrames.translation(-800, 200, 0));
		addComponent("vacuum", new VacuumGenerator(60, null, null, machinekit.screwdriving.ScrewdrivingInterfaces.vacuum()),
			AssemblyFrames.translation(-800, 350, 0));
		include("work", work);
		// The table constrains the tapped plate, not any screw or housing inferred from radius.
		addMemberConnector("table", "workSeat", Solids.axial(workX, 20, RobotArm.TABLE_TOP));
		// Included members are snapshots, so add the cell's base connector explicitly.
		addMemberConnector("work/plate", "base", Solids.axial(0, 0, 0));
		addMate("work-mount", "fixed", "table", "workSeat", "work/plate", "base");
		var stop = (work.flange.housing.face + 20) / 2 + 10;
		for (side in [-1, 1]) {
			var name = side < 0 ? "fixtureLeft" : "fixtureRight";
			addComponent(name, new ArmBlock(20, work.flange.housing.face + 20, 15, "steel", "Fixture"));
			addMemberConnector("table", name, Solids.axial(workX + side * stop, 20, RobotArm.TABLE_TOP));
			addMate(name + "-mount", "fixed", "table", name, name, "base");
		}
		include("presenter", presenter, AssemblyFrames.translation(180, -450, RobotArm.TABLE_TOP));
		// One cable carries distinct electrical and signal services.
		connectPorts("tool-power", "controller", "toolPower", "arm/tool/driver/body", "power");
		connectPorts("tool-cable", "controller", "tool", "arm/tool/driver/body", "control");
		connectPorts("vacuum-hose", "vacuum", "vacuum", "arm/tool/driver/body", "vacuum");
		exposePort("mains", "controller", "mains"); exposePort("control", "controller", "control");
		exposePort("air", "vacuum", "air"); exposePort("presenterControl", "presenter/escapement", "control");
	}
	public function readyPose():Array<Float> return [0.0, 0.25, Math.PI / 2 - 1.4, 0.0, Math.PI / 2 - 0.25 + Math.PI / 2 - 1.4, 0.0];
}
