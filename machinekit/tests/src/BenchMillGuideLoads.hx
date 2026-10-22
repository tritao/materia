import cadbridge.AssemblyPhysicalPartView;
import cadbridge.AssemblySimulationBridge;
import cadkit.modeling.AssemblyState;
import machinekit.motion.GuideSupport;
import machinekit.motion.GuideSupport.GuideSupportWrench;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact.SceneArtifactData;

/** Reference guide admission at the drive acceleration limits and travel corners.
 * Cutting load is an assumed envelope of 500 N per world axis, with a static
 * safety factor of two. Equal block stiffness is assumed; this is not a life calculation. */
class BenchMillGuideLoads {
	static inline var CUTTING_FORCE:Float = 500;
	static inline var SAFETY_FACTOR:Float = 2;

	public static function run(mill:BenchMill, scene:SceneArtifactData, prefix:String):Void {
		var assemblyDefinition = scene.assemblyDefinition;
		if (assemblyDefinition == null) throw "Guide study requires an assembly definition";
		var physicalView = AssemblyPhysicalPartView.fromSceneArtifact(scene);
		var model = AssemblySimulationBridge.toRobotModel(assemblyDefinition, physicalView, scene.assemblyState).model;
		var physicalParts:Map<String, cadbridge.AssemblySimulationBridge.AssemblyPhysicalPart> = [];
		for (part in physicalView.parts) physicalParts.set(part.id, part);
		var definitions:Map<String, String> = [];
		for (occurrence in assemblyDefinition.occurrences) definitions.set(occurrence.id, occurrence.definition);
		var lengthScale = scene.metresPerUnit;
		var volumeScale = lengthScale * lengthScale * lengthScale;
		var state = new AssemblyState(assemblyDefinition, scene.assemblyState);
		var parents = mill.mateParents();
		var studies = [
			{axis: "x", root: "blockXFront", rail: "railXFront", blocks: ["blockXFront", "blockXFrontRear", "blockXBack", "blockXBackRear"]},
			{axis: "y", root: "blockYLeft", rail: "railYLeft", blocks: ["blockYLeft", "blockYLeftRear", "blockYRight", "blockYRightRear"]},
			{axis: "z", root: "blockZLeft", rail: "railZLeft", blocks: ["blockZLeft", "blockZLeftTop", "blockZRight", "blockZRightTop"]}
		];
		var accelerations:Array<{x:Float, y:Float, z:Float}> = [];
		for (i in 0...studies.length) {
			var study = studies[i], spec = mill.specs[i];
			var limit = model.coupledLimits(prefix + study.axis).requireAcceleration();
			if (!Math.isFinite(limit) || !(limit > 0)) throw '${study.axis} guide study requires an acceleration limit';
			var original = state.joint(prefix + study.axis);
			state.setJoint(prefix + study.axis, spec.lower); state.forwardKinematics();
			var firstPose = state.worldPose(prefix + study.root);
			var first = {x: firstPose.x, y: firstPose.y, z: firstPose.z};
			state.setJoint(prefix + study.axis, spec.lower + 1); state.forwardKinematics();
			var second = state.worldPose(prefix + study.root);
			var dx = second.x - first.x, dy = second.y - first.y, dz = second.z - first.z;
			var distance = Math.sqrt(dx * dx + dy * dy + dz * dz);
			if (!Math.isFinite(distance) || !(distance > 0)) throw '${study.axis} guide study has no measured motion direction';
			accelerations.push({x: dx * limit / distance, y: dy * limit / distance, z: dz * limit / distance});
			state.setJoint(prefix + study.axis, original);
		}
		var stockTop = 0.0;
		for (entry in mill.components()) if (entry.id == "stock") {
			var solid = entry.component.geometry();
			stockTop = solid.shape.bounds().get_max().get_z(); solid.close();
		}
		if (!(stockTop > 0)) throw "Guide study requires the actual stock top";
		var worst:Map<String, Float> = [];
		var overloadRejected = false;
		for (x in [mill.specs[0].lower, mill.specs[0].upper])
		for (y in [mill.specs[1].lower, mill.specs[1].upper])
		for (z in [mill.specs[2].lower, mill.specs[2].upper]) {
			state.setJoint(prefix + "x", x); state.setJoint(prefix + "y", y); state.setJoint(prefix + "z", z); state.forwardKinematics();
			for (study in studies) {
				var inverse = AssemblyFrames.inverse(state.worldPose(prefix + study.rail));
				var centres:Array<{x:Float, y:Float, z:Float}> = [];
				for (id in study.blocks) {
					var cx = 0.0, cy = 0.0, cz = 0.0;
					for (i in 1...5) {
						var mount = state.worldConnector(prefix + id, 'mount$i');
						cx += mount.x / 4; cy += mount.y / 4; cz += mount.z / 4;
					}
					centres.push(AssemblyFrames.transformPoint(inverse, cx, cy, cz));
				}
				var centre = {x: 0.0, y: 0.0, z: 0.0};
				var minX = centres[0].x, maxX = minX, minZ = centres[0].z, maxZ = minZ;
				for (point in centres) {
					centre.x += point.x / 4; centre.y += point.y / 4; centre.z += point.z / 4;
					minX = Math.min(minX, point.x); maxX = Math.max(maxX, point.x);
					minZ = Math.min(minZ, point.z); maxZ = Math.max(maxZ, point.z);
				}
				var screw = state.worldPose(prefix + "screw" + study.axis.toUpperCase());
				var drive = AssemblyFrames.transformPoint(inverse, screw.x, screw.y, screw.z);
				var support = new GuideSupport((maxX - minX) * lengthScale, (maxZ - minZ) * lengthScale,
					(drive.x - centre.x) * lengthScale, (drive.y - centre.y) * lengthScale);
				var masses:Array<{mass:Float, x:Float, y:Float, z:Float, mask:Int}> = [];
				for (entry in mill.components()) if (descends(entry.id, study.root, parents)) {
					var definition = definitions.get(prefix + entry.id);
					if (definition == null) throw 'Guide study has no occurrence for "${entry.id}"';
					var physical = physicalParts.get(definition);
					if (physical == null) throw 'Guide study has no physical part for "$definition"';
					var local = physical.centerOfMass;
					var world = AssemblyFrames.transformPoint(state.worldPose(prefix + entry.id), local[0], local[1], local[2]);
					var point = AssemblyFrames.transformPoint(inverse, world.x, world.y, world.z);
					var mask = 0;
					for (i in 0...studies.length) if (descends(entry.id, studies[i].root, parents)) mask |= 1 << i;
					masses.push({mass: physical.volume * physical.density * volumeScale, x: (point.x - centre.x) * lengthScale,
						y: (point.y - centre.y) * lengthScale, z: (point.z - centre.z) * lengthScale, mask: mask});
				}
				var toolFrame = state.worldConnector(prefix + "tool", "tip");
				var cutting:{x:Float, y:Float, z:Float} = study.axis == "z" ? {x: toolFrame.x, y: toolFrame.y, z: toolFrame.z} :
					AssemblyFrames.transformPoint(state.worldPose(prefix + "stock"), 0, 0, stockTop);
				var tip = AssemblyFrames.transformPoint(inverse, cutting.x, cutting.y, cutting.z);
				for (accelerationCase in 0...8) for (cuttingCase in 0...8) {
					var wrench:GuideSupportWrench = {fx: 0, fy: 0, fz: 0, mx: 0, my: 0, mz: 0};
					for (mass in masses) {
						var ax = 0.0, ay = 0.0, az = 0.0;
						for (i in 0...accelerations.length) if ((mass.mask & (1 << i)) != 0) {
							var sign = (accelerationCase & (1 << i)) == 0 ? -1.0 : 1.0;
							ax += sign * accelerations[i].x; ay += sign * accelerations[i].y; az += sign * accelerations[i].z;
						}
						var force = AssemblyFrames.transformVector(inverse, -mass.mass * ax, -mass.mass * ay,
							mass.mass * (-robotkit.model.DriveLoads.GRAVITY - az));
						add(wrench, force.x, force.y, force.z, mass.x, mass.y, mass.z);
					}
					var force = AssemblyFrames.transformVector(inverse,
						(cuttingCase & 1) == 0 ? -CUTTING_FORCE : CUTTING_FORCE,
						(cuttingCase & 2) == 0 ? -CUTTING_FORCE : CUTTING_FORCE,
						(cuttingCase & 4) == 0 ? -CUTTING_FORCE : CUTTING_FORCE);
					add(wrench, force.x, force.y, force.z, (tip.x - centre.x) * lengthScale,
						(tip.y - centre.y) * lengthScale, (tip.z - centre.z) * lengthScale);
					var used = support.requireCapacity(wrench, machinekit.motion.LinearRailSystem.rating(mill.guideBlock.spec).staticLoad, SAFETY_FACTOR);
					var previous = worst.get(study.axis);
					worst.set(study.axis, previous == null ? used : Math.max(previous, used));
					if (!overloadRejected) {
						try { var unused = support.requireCapacity(wrench, 1, SAFETY_FACTOR); }
						catch (error:Dynamic) overloadRejected = Std.string(error).indexOf("static capacity exceeded") >= 0;
					}
				}
			}
		}
		if (!overloadRejected) throw "Guide admission accepted a one-newton bearing under the actual assembled load";
		for (study in studies) trace('Mill ${study.axis} guide static utilization: ${worst.get(study.axis)} (assumed capacity, cutting envelope and safety factor)');
	}

	static function descends(id:String, root:String, parents:Map<String, String>):Bool {
		var seen:Array<String> = [];
		while (true) {
			if (id == root) return true;
			if (seen.indexOf(id) >= 0) throw "Guide study found a cyclic mate tree";
			seen.push(id);
			var parent = parents.get(id);
			if (parent == null) return false;
			id = parent;
		}
	}

	static function add(wrench:GuideSupportWrench, fx:Float, fy:Float, fz:Float, x:Float, y:Float, z:Float):Void {
		wrench.fx += fx; wrench.fy += fy; wrench.fz += fz;
		wrench.mx += y * fz - z * fy; wrench.my += z * fx - x * fz; wrench.mz += x * fy - y * fx;
	}
}
