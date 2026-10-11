import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.project.SceneArtifact.SceneArtifactData;

/** Author initial alignment and stage identified bodies from their actual presenter templates. */
typedef ScrewStaging = {var body:String; var template:String; var like:String;}

class ScrewdrivingExecution {
	public static function align(cell:ScrewdrivingCell, state:AssemblyState, presented:String):Void {
		// Start above the presenter rather than sweeping the parked tool through
		// the workpiece on its first Cartesian move.
		var pick = state.worldPose(presented);
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
	}

	public static function stage(scene:SceneArtifactData, supplies:Array<ScrewStaging>):Void {
		var definition = scene.assemblyDefinition;
		if (definition == null) throw "Screw execution needs its CAD assembly";
		var flat = AssemblyDefinitionFlattener.flatten(definition), state = new AssemblyState(flat,scene.assemblyState);
		var removed = [for (supply in supplies) supply.template], freed = [for (supply in supplies) supply.body];
		for (supply in supplies) {
			var bodies = [for (item in flat.occurrences) if (item.id == supply.body) item];
			var originals = [for (item in flat.occurrences) if (item.id == supply.like) item];
			if (originals.length != 1) throw 'Supply "${supply.body}" has no exact source component';
			var pose = state.worldPose(supply.template);
			if (bodies.length == 0) flat.occurrences.push({id:supply.body,definition:originals[0].definition,initialPose:pose});
			else if (bodies.length == 1) bodies[0].initialPose = pose;
			else throw 'Supply "${supply.body}" has ambiguous body identity';
		}
		flat.joints = [for (joint in flat.joints) if (removed.indexOf(joint.parent) < 0 && removed.indexOf(joint.child) < 0 && freed.indexOf(joint.parent) < 0 && freed.indexOf(joint.child) < 0) joint];
		flat.occurrences = [for (item in flat.occurrences) if (removed.indexOf(item.id) < 0) item];
		var mates = flat.mates;
		if (mates != null) flat.mates = [for (mate in mates) if (removed.indexOf(mate.first) < 0 && removed.indexOf(mate.second) < 0 && freed.indexOf(mate.first) < 0 && freed.indexOf(mate.second) < 0) mate];
		var exposed = flat.exposedConnectors;
		if (exposed != null) flat.exposedConnectors = [for (item in exposed) if (removed.indexOf(item.occurrence) < 0) item];
		var previous = scene.assemblyState;
		if (previous == null) throw "Screw execution lost its initial state";
		var execution = new AssemblyState(flat);
		for (coordinate in previous.jointCoordinates) execution.setJoint(coordinate.joint,coordinate.value);
		for (root in previous.rootPoses) if (removed.indexOf(root.occurrence) < 0 && freed.indexOf(root.occurrence) < 0)
			execution.setRootPose(root.occurrence,root.pose);
		scene.assemblyDefinition = flat; scene.assemblyState = execution.record();
	}
}
