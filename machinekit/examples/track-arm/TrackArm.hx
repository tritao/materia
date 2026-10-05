import RobotArm.ArmTable;
import RobotArm.ArmBlock;
import machinekit.gantry.LinearTrack;
import materia.assembly.AssemblyFrames;

/** The arm works at each end of a long table; the external track carries it between stations. */
class TrackArm extends LinearTrack {
	public final arm:RobotArm;
	public final tableTop:Float;
	public function new() {
		super(3000);
		arm = new RobotArm(false);
		includeArm("arm", arm);
		tableTop = RobotArm.TABLE_TOP + mountZero.z;
		addComponent("table", new ArmTable(3500, 500, tableTop), AssemblyFrames.translation(1500, RobotArm.TABLE_CENTRE_Y, 0));
		var pad = new ArmBlock(RobotArm.WORKPIECE_WIDTH + 20, RobotArm.WORKPIECE_WIDTH + 20,
			RobotArm.PAD_HEIGHT, "rubber", "track station");
		for (station in [0, 1]) addComponent(station == 0 ? "padPick" : "padPlace", pad,
			AssemblyFrames.translation(station * axis.upper + RobotArm.PICK_X, RobotArm.WORK_Y, tableTop));
		addComponent("workpiece", new ArmBlock(RobotArm.WORKPIECE_WIDTH, RobotArm.WORKPIECE_WIDTH,
			RobotArm.WORKPIECE_HEIGHT, "birch plywood", "track workpiece"),
			AssemblyFrames.translation(RobotArm.PICK_X, RobotArm.WORK_Y, tableTop + RobotArm.PAD_HEIGHT));
	}
}
