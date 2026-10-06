import machinekit.milling.MillPanel;
import machinekit.robotics.CobotArm;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** The fabricated pedestal shared by the reach model and the assembled tending cell. */
class TendingCellRiser extends MillPanel {
	public static inline var WIDTH:Float = 120;
	public static inline var DEPTH:Float = 120;

	public function new(height:Float) super(WIDTH, DEPTH, height, "steel");

	/** World frame of the arm's floor connector at its selected subassembly placement. */
	public static function armFloorFrame(arm:CobotArm, placement:AssemblyFrame):AssemblyFrame {
		var poses = arm.solvedPoses();
		var baseFlange = poses.get("baseFlange");
		if (baseFlange == null) throw "Cobot arm has no solved base flange";
		return AssemblyFrames.compose(placement,
			AssemblyFrames.compose(baseFlange, arm.memberConnectorFrame("baseFlange", "floor")));
	}

	/** Riser-local seat frame that leaves the arm subassembly at its selected placement. */
	public static function armSeatFrame(arm:CobotArm, riserPose:AssemblyFrame,
		placement:AssemblyFrame):AssemblyFrame
		return AssemblyFrames.compose(AssemblyFrames.inverse(riserPose), armFloorFrame(arm, placement));
}
