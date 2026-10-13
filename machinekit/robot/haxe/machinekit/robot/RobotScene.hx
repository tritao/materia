package machinekit.robot;

import machinekit.assembly.MachineAssembly;
import machinekit.robotics.EndEffector;
import machinekit.welding.WeldingEquipment.WeldingEquipmentData;
import materia.project.SceneArtifact.SceneArtifactRobotSensor;
import materia.project.SceneArtifact.SceneArtifactRobotTool;

/**
 * The robot records of a scene, read from the facets of a machine's parts: the tools its end
 * effector works and the sensors it carries. MachineKit says what the parts are; this says how a
 * robot runtime sees them.
 */
class RobotScene {
	/**
	 * A robot's tools as its end effector `tool` declares them, its members' occurrences under
	 * `prefix`: each suction cup's contact, worked by the effector's vacuum control channel and
	 * reporting on its pressure sensor when it has one.
	 */
	public static function robotTools(tool:EndEffector, prefix:String, ?welding:WeldingEquipmentData):Array<SceneArtifactRobotTool> {
		var controls = EndEffectorControls.derive(tool, prefix);
		var members = [for (member in tool.components()) prefix + "/" + member.id];
		if (controls.arcs.length > 0) return torchTools(controls, prefix, welding, members);
		if (controls.grippers.length > 0) {
			if (controls.suctions.length != 0)
				throw "A robot end effector cannot mix physical grippers and suction contacts";
			return [for (grip in controls.grippers) {
				var channel = controls.gripperChannel(grip.member);
				if (channel == null) throw 'Parallel gripper "${grip.member}" has no close control';
				var entry:SceneArtifactRobotTool = {kind: "gripper", members: members.copy(), contact: {occurrence: prefix + "/" + grip.member,
					connector: grip.connector}, channel: channel, sensor: grip.sensor,
					gripper: {padMembers: [for (member in grip.padMembers) prefix + "/" + member],
						envelopeMembers: [for (member in grip.envelopeMembers) prefix + "/" + member], minimumPartWidthMm: grip.minimumPartWidthMm,
						nominalPartWidthMm: grip.nominalPartWidthMm, maximumPartWidthMm: grip.maximumPartWidthMm,
						fingerLengthMm: grip.fingerLengthMm,
						fingerPadFrictionCoefficient: grip.fingerPadFrictionCoefficient}};
				entry;
			}];
		}
		var channel = controls.vacuumChannel();
		if (channel == null) throw "The suction tool has no vacuum control";
		return [for (suction in controls.suctions) {
			var entry:SceneArtifactRobotTool = {kind: "suction", members: members.copy(),
				contact: {occurrence: prefix + "/" + suction.member, connector: suction.connector}, channel: channel};
			if (controls.vacuumSensor != null) entry.sensor = controls.vacuumSensor;
			entry;
		}];
	}

	/**
	 * Each arc torch as a `torch` tool: its wire tip as the contact, its three channels and weld sensor, and the
	 * welder behind it (`welding`: the supply's limits, the wire, and the work the circuit returns through, found by
	 * `WeldingEquipment.of`).
	 */
	static function torchTools(controls:EndEffectorControls, prefix:String, welding:Null<WeldingEquipmentData>, members:Array<String>):Array<SceneArtifactRobotTool> {
		if (welding == null) throw "A torch needs the welding equipment of its cell: pass WeldingEquipment.of(...)";
		return [for (arc in controls.arcs) {
			kind: "torch", members: members.copy(),
			contact: {occurrence: prefix + "/" + arc.member, connector: arc.tcpConnector},
			channel: arc.channel,
			sensor: arc.sensor,
			torch: {wireSpeedChannel: arc.wireSpeedChannel, voltageChannel: arc.voltageChannel,
				groundedWork: welding.groundedWork.copy(), maxCurrentA: welding.maxCurrentA, efficiency: welding.efficiency,
				wireDiameterMm: welding.wireDiameterMm, stickoutMm: arc.stickoutMm,
				maxWireSpeedMPerMin: welding.maxWireSpeedMPerMin, depositionEfficiency: welding.depositionEfficiency}
		}];
	}

	/**
	 * The sensors `robot` declares, its members' occurrences under `prefix`: each planar scanner,
	 * mounted at its scan connector and named after its occurrence.
	 */
	public static function robotSensors(robot:MachineAssembly, prefix:String):Array<SceneArtifactRobotSensor> {
		var result:Array<SceneArtifactRobotSensor> = [];
		for (entry in robot.components()) {
			var scanner = machinekit.sensing.PlanarScannerFacet.of(entry.component);
			if (scanner != null) result.push({kind: "lidar", id: prefix + entry.id,
				mount: {occurrence: prefix + entry.id, connector: scanner.scanConnector},
				rayCount: scanner.rayCount, maxRange: scanner.maxRangeMeters, updateRate: scanner.rateHz});
		}
		return result;
	}

}
