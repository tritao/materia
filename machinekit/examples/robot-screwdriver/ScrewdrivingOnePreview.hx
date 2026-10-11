import cadkit.modeling.AssemblyState;
import machinekit.assembly.AssemblyPreview;
import machinekit.robot.RobotScene;
import machinekit.screwdriving.ScrewdrivingScene;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactData;
import haxe.io.Bytes;

/** One explicitly qualified M6 cycle, with the identified CAD screw staged in the presenter. */
class ScrewdrivingOnePreview {
	public static function scene():SceneArtifactData {
		var cell = new ScrewdrivingCell(new machinekit.screwdriving.ScrewdriverBit(HexSocket(5),45,6,0.05),
			0,machinekit.standard.DeepGrooveBearing.metric("6003"));
		var qualification = cell.work.join("flange/screw1",null,{minimum:0.2,maximum:1.0});
		ScrewdrivingChecks.check(cell);
		var found = [for (item in cell.work.fastenings().require()) if (item.screw.member == "flange/screw1") item];
		if (found.length != 1) throw "Drive-one cell needs its named screw1 fastening";
		var scene = AssemblyPreview.scene(cell,"screwdriving-one");
		scene.assemblyState = ScrewdrivingPreview.state(cell,"screwdriving-one").record();
		scene.robotTools = RobotScene.robotTools(cell.arm.tool,"arm/tool");
		// These values qualify this simulation example, not the published torque table.
		var job = ScrewdrivingScene.fromFastening(cell.work,found[0],"work/",scene,
			"presenter/nest","presenter/screw","presenter/part-present",1,
			{runDownSpeed:20,finalSpeed:2,runDownTorqueLimit:1,finalTorqueLimit:19.5,maximumRunDownAngle:180,
				minimumStroke:2,maximumStroke:18},
			{leadInMm:0.5,maximumAxisAngle:0.1,prevailingTorque:0.2,stiffnessNmPerRad:40,stripTorque:22});
		var definition = scene.assemblyDefinition;
		if (definition == null) throw "Drive-one scene lost its assembly";
		var flat = AssemblyDefinitionFlattener.flatten(definition);
		var state = new AssemblyState(flat,scene.assemblyState);
		// Start above the presenter rather than sweeping the parked tool through
		// the workpiece on its first Cartesian move.
		var pick = state.worldPose("presenter/screw");
		var initialTarget = materia.assembly.AssemblyFrames.compose(pick,cell.presenter.screw.driveFrame());
		var initialDirection = materia.assembly.AssemblyFrames.transformVector(initialTarget,0,0,1);
		var initialJoints = [for (i in 1...7) 'arm/j$i'];
		var initialLimits = [for (spec in cell.arm.specs) {lower:spec.lower,upper:spec.upper}];
		var initial = machinekit.robot.ToolAxisReach.solve(state,initialJoints,initialLimits,
			{x:initialTarget.x-initialDirection.x*40,y:initialTarget.y-initialDirection.y*40,z:initialTarget.z-initialDirection.z*40},
			initialDirection,cell.readyPose(),"arm/tool/driver/carrier","tcp");
		if (initial == null) throw "Drive-one presenter approach has no initial arm pose";
		for (i in 0...6) state.setJoint(initialJoints[i],initial[i]);
		var initialTcp = state.worldConnector("arm/tool/driver/carrier","tcp");
		var initialX = materia.assembly.AssemblyFrames.transformVector(initialTcp,1,0,0);
		var desiredX = materia.assembly.AssemblyFrames.transformVector(initialTarget,1,0,0);
		var crossX = {x:initialX.y*desiredX.z-initialX.z*desiredX.y,
			y:initialX.z*desiredX.x-initialX.x*desiredX.z,z:initialX.x*desiredX.y-initialX.y*desiredX.x};
		var roll = Math.atan2(initialDirection.x*crossX.x+initialDirection.y*crossX.y+initialDirection.z*crossX.z,
			initialX.x*desiredX.x+initialX.y*desiredX.y+initialX.z*desiredX.z);
		state.setJoint(initialJoints[5],initial[5]+0.001);
		var rotatedX = materia.assembly.AssemblyFrames.transformVector(state.worldConnector("arm/tool/driver/carrier","tcp"),1,0,0);
		var sense = initialDirection.x*(initialX.y*rotatedX.z-initialX.z*rotatedX.y)
			+initialDirection.y*(initialX.z*rotatedX.x-initialX.x*rotatedX.z)
			+initialDirection.z*(initialX.x*rotatedX.y-initialX.y*rotatedX.x);
		state.setJoint(initialJoints[5],initial[5]+(sense > 0 ? roll : -roll));
		scene.assemblyState = state.record();
		var pickupBody = state.worldPose("presenter/screw");
		// Derive the job from the seated CAD first; the execution scene then frees
		// that same occurrence and removes the presenter's fixed display copy.
		flat.joints = [for (edge in flat.joints) if (edge.parent != job.screw && edge.child != job.screw &&
			edge.parent != "presenter/screw" && edge.child != "presenter/screw") edge];
		flat.occurrences = [for (item in flat.occurrences) if (item.id != "presenter/screw") item];
		for (item in flat.occurrences) if (item.id == job.screw) item.initialPose = pickupBody;
		var mates = flat.mates;
		if (mates != null) flat.mates = [for (mate in mates) if (mate.first != job.screw && mate.second != job.screw &&
			mate.first != "presenter/screw" && mate.second != "presenter/screw") mate];
		var exports = flat.exposedConnectors;
		if (exports != null) flat.exposedConnectors = [for (item in exports) if (item.occurrence != "presenter/screw") item];
		scene.assemblyDefinition = flat;
		var previous = scene.assemblyState;
		if (previous == null) throw "Drive-one scene lost its starting state";
		var execution = new AssemblyState(flat);
		for (coordinate in previous.jointCoordinates) execution.setJoint(coordinate.joint,coordinate.value);
		for (root in previous.rootPoses) if (root.occurrence != "presenter/screw" && root.occurrence != job.screw)
			execution.setRootPose(root.occurrence,root.pose);
		scene.assemblyState = execution.record();
		scene.mission = {steps:[{kind:"screw",screw:job}]};
		var controllers = scene.controllers;
		if (controllers != null && controllers.length > 0) {
			var owners = [for (controller in controllers) if (controller.kind == "robot" && controller.joints.indexOf("arm/j1") >= 0) controller];
			if (owners.length != 1) throw "Drive-one cell needs its declared arm controller";
			scene.mission.controllerId = owners[0].id;
		}
		return scene;
	}
	public static function one():Bytes return SceneArtifact.encode(scene());
}
function main():Void {
	var restored = SceneArtifact.decode(ScrewdrivingOnePreview.one());
	if (restored.mission == null || restored.mission.steps[0].screw == null) throw "Drive-one scene lost its fastening";
	trace("Screwdriving one-screw CAD mission passed");
}
