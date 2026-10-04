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
		if (controls.arcs.length > 0) return torchTools(controls, prefix, welding);
		var channel = controls.vacuumChannel();
		if (channel == null) throw "The suction tool has no vacuum control";
		return [for (suction in controls.suctions) {
			var entry:SceneArtifactRobotTool = {kind: "suction",
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
	static function torchTools(controls:EndEffectorControls, prefix:String, welding:Null<WeldingEquipmentData>):Array<SceneArtifactRobotTool> {
		if (welding == null) throw "A torch needs the welding equipment of its cell: pass WeldingEquipment.of(...)";
		return [for (arc in controls.arcs) {
			kind: "torch",
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
