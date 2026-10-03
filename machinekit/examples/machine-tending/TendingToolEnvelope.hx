import machinekit.robotics.ArmTool;
import machinekit.robotics.RobotFlange;
import machinekit.robotics.EndEffector;
import machinekit.robotics.EndEffectorPlate;
import machinekit.assembly.MachineAssembly;
import machinekit.milling.MillPanel;
import materia.assembly.AssemblyFrames;

/** Assumed future gripper envelope for the reach study, including opening travel.
 * This is a static body and TCP; operational fingers and pneumatics belong to MT8.
 */
class TendingToolEnvelope implements ArmTool {
	public final width:Float = 80;
	public final depth:Float = 50;
	public final length:Float = 150;
	public final openingTravel:Float = 16;
	public final fingerWidth:Float = 8;
	public final fingerDepth:Float = 10;
	public final fingerLength:Float = 50;
	public static inline var TCP_PART:String = "tool/bodyEnvelope";
	public function new() {}
	public function build(flange:RobotFlange):EndEffector {
		var tool = new EndEffector();
		tool.addComponent("plate", new EndEffectorPlate(flange));
		var bodyLength = length - fingerLength;
		tool.addComponent("bodyEnvelope", new MillPanel(width + openingTravel, depth, bodyLength, "plastic"));
		tool.addMemberConnector("bodyEnvelope", "gripperMount", flange.pinAlignedFrame(0));
		tool.addMemberConnector("bodyEnvelope", "tcp", AssemblyFrames.translation(0, 0, length));
		tool.mount("plate", "robot");
		tool.addMate("gripper-mount", "fixed", "plate", "tool", "bodyEnvelope", "gripperMount");
		for (side in [-1, 1]) {
			var id = side < 0 ? "leftFingerEnvelope" : "rightFingerEnvelope";
			var sweptWidth = fingerWidth + openingTravel / 2;
			tool.addComponent(id, new MillPanel(sweptWidth, fingerDepth, fingerLength, "plastic"));
			tool.addMemberConnector("bodyEnvelope", id, AssemblyFrames.translation(
				side * (BenchMill.STOCK_WIDTH / 2 + sweptWidth / 2), 0, bodyLength));
			tool.addMemberConnector(id, "mountAt", AssemblyFrames.identity());
			tool.addMate('$id-mount', "fixed", "bodyEnvelope", id, id, "mountAt");
		}
		tool.workingFrame("tcp", "bodyEnvelope", "tcp", true);
		return tool;
	}
	public function expose(arm:MachineAssembly):Void arm.exposeConnector("toolTcp", TCP_PART, "tcp");
}
