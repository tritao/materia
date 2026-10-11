import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import machinekit.assembly.AssemblyPreview;
import machinekit.robot.RobotScene;
import materia.project.SceneArtifact;
import haxe.io.Bytes;
/** Equipment preview only; the tightening process and mission are later steps. */
class ScrewdrivingPreview {
	public static function state(cell:ScrewdrivingCell, assemblyId:String = "screwdriving-cell"):AssemblyState {
		var model = new AssemblyModel("mm"); cell.addTo(model, "");
		var state = new AssemblyState(model.definition(assemblyId));
		var q = cell.readyPose();
		for (i in 0...6) state.setJoint('arm/j${i + 1}', q[i]);
		return state;
	}
	public static function cell():Bytes {
		var cell = new ScrewdrivingCell();
		ScrewdrivingChecks.check(cell);
		var scene = AssemblyPreview.scene(cell, "screwdriving-cell");
		scene.assemblyState = state(cell).record();
		scene.robotTools = RobotScene.robotTools(cell.arm.tool, "arm/tool");
		return SceneArtifact.encode(scene);
	}
}
function main():Void {
	var scene = SceneArtifact.decode(ScrewdrivingPreview.cell());
	if (scene.robotTools == null) throw "Screwdriving cell lost its tool";
	trace("Screwdriving equipment cell passed");
}
