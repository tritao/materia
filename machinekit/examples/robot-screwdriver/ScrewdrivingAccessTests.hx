import machinekit.assembly.PosedParts;
import materia.assembly.AssemblyFrames;

/** Access must reject missing reach and actual cell obstruction, while restoring design coordinates. */
class ScrewdrivingAccessTests {
	public static function run():Void {
		var design = ScrewdrivingAssemblyPreview.design();
		var tools = [for (tool in design.scene.robotTools) if (tool.screwdriver != null) tool];
		if (tools.length != 1) throw "Access test lost its screwdriver";
		var tool = tools[0], state = design.state, capacity = tool.screwdriver;
		if (capacity == null) throw "Access test lost its feed capability";
		var fastening = design.found.require()[0], selected = [for (job in design.jobs) if (job.fastening == fastening.name()) job][0];
		var recovery = selected.recovery;
		if (recovery == null) throw "Access test lost its reserve";
		var before = [for (i in 1...7) state.joint('arm/j$i')], beforeFeed = state.joint(capacity.feedJoint);
		PosedParts.scope(parts -> {
			var access = new ScrewdrivingAccess(design.cell,state,design.jobs,design.staging,parts,
				design.scene.metresPerUnit,tool);
			var x = recovery.replacement.pickup.x;
			recovery.replacement.pickup.x = x+10;
			var unreachable = access.problem(fastening);
			recovery.replacement.pickup.x = x;
			if (unreachable == null || unreachable.indexOf("replacement pickup") < 0 || unreachable.indexOf("unreachable") < 0)
				throw 'Unreachable reserve has no named access diagnostic: $unreachable';
			var controller = state.worldPose("controller");
			var pick = AssemblyFrames.compose(state.worldPose(selected.presenter),
				{x:selected.pickup.x/design.scene.metresPerUnit,y:selected.pickup.y/design.scene.metresPerUnit,
				 z:selected.pickup.z/design.scene.metresPerUnit,qx:selected.pickup.qx,qy:selected.pickup.qy,qz:selected.pickup.qz,qw:selected.pickup.qw});
			state.setRootPose("controller",AssemblyFrames.translation(pick.x,pick.y,pick.z));
			var blocked = access.problem(fastening);
			state.setRootPose("controller",controller);
			if (blocked == null || blocked.indexOf("controller") < 0 || blocked.indexOf("hits") < 0)
				throw 'Cell obstruction has no named access diagnostic: $blocked';
			for (i in 1...7) if (state.joint('arm/j$i') != before[i-1]) throw "Access changed its saved arm pose";
			if (state.joint(capacity.feedJoint) != beforeFeed) throw "Access changed its saved feed coordinate";
		});
		trace("Screwdriving access: unreachable reserve, actual cell obstruction and coordinate restoration passed");
	}
}
