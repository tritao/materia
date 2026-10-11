package materia.project;

import haxeon.wire.JsonWire;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.project.SceneArtifact.SceneArtifactData;

/** One commissioned simulated material path; poses are metres in the named nest frame. */
@:wire typedef SceneScrewFeeder = {
	@:id(1) var id:String;
	@:id(2) var presenter:String;
	@:id(3) var presentSensor:String;
	@:id(4) var inventory:Array<String>;
	@:id(5) var deposit:AssemblyFrame;
	@:id(6) var bufferCapacity:Int;
	@:id(7) var trackDelay:Float;
	@:id(8) var deliveryDelay:Float;
	@:id(9) var deliveryTimeout:Float;
}

/** Strict current-format material paths, with physical sensor and identified-body ownership. */
class SceneScrewFeeders {
	public static function decode(value:Dynamic):Array<SceneScrewFeeder> {
		var source = SceneWireJson.preserveNumbers(haxe.Json.stringify(value));
		var result:Array<SceneScrewFeeder> = JsonWire.decode(source);
		var raw:Dynamic = Reflect.field(haxe.Json.parse(source),"value");
		var canonical:Dynamic = Reflect.field(haxe.Json.parse(JsonWire.encode(result)),"value");
		var inputs:Array<Dynamic> = cast raw, outputs:Array<Dynamic> = cast canonical;
		for (i in 0...result.length) SceneWireJson.integerFields(inputs[i],outputs[i],["bufferCapacity"]);
		return result;
	}
	public static function validate(feeders:Array<SceneScrewFeeder>, scene:SceneArtifactData):Void {
		if (feeders == null) return;
		var definition = scene.assemblyDefinition;
		if (definition == null || feeders.length == 0 || feeders.length > 32) throw "Screw feeders need an assembly and bounded material paths";
		var flat = AssemblyDefinitionFlattener.flattenView(definition);
		var ids:Map<String,Bool> = [], bodies:Map<String,Bool> = [], nests:Map<String,Bool> = [];
		for (feeder in feeders) {
			if (feeder == null || feeder.id == null || feeder.id.length == 0 || ids.exists(feeder.id)
				|| nests.exists(feeder.presenter)) throw "Screw feeders need distinct identities and pickup nests";
			ids.set(feeder.id,true); nests.set(feeder.presenter,true);
			if ([for (item in flat.occurrences) if (item.id == feeder.presenter) item].length != 1)
				throw 'Screw feeder "${feeder.id}" lost its named presenter';
			var sensors = flat.sensors;
			if (sensors == null || [for (sensor in sensors) if (sensor.id == feeder.presentSensor && sensor.kind == "presence" && sensor.occurrence == feeder.presenter) sensor].length != 1)
				throw 'Screw feeder "${feeder.id}" needs a physical presence sensor';
			if (feeder.inventory == null || feeder.inventory.length == 0 || feeder.inventory.length > 256
				|| feeder.bufferCapacity < 1 || feeder.bufferCapacity > 256) throw "Screw feeder needs bounded identified inventory and buffering";
			var mission = scene.mission;
			if (mission == null) throw "Screw feeder needs its authored fastening mission";
			var jobs:Array<materia.project.SceneScrewDefinition.SceneScrew> = [];
			for (step in mission.steps) if (step.kind == "screw") {
				var job = SceneScrewInvocation.job(step.screw,mission.steps);
				if (job != null && job.presenter == feeder.presenter) jobs.push(job);
			}
			if (jobs.length == 0) throw 'Screw feeder "${feeder.id}" supplies no named fastening';
			for (job in jobs) if (job.presentSensor != feeder.presentSensor || feeder.inventory.indexOf(job.screw) < 0)
				throw 'Screw feeder "${feeder.id}" does not own the fastening supply "${job.screw}"';
			var expected = [for (item in flat.occurrences) if (item.id == jobs[0].screw) item.definition];
			if (expected.length != 1) throw "Screw feeder lost its commissioned screw definition";
			for (body in feeder.inventory) {
				if (body == feeder.presenter || bodies.exists(body) || [for (item in flat.occurrences) if (item.id == body) item].length != 1
					|| [for (joint in flat.joints) if (joint.parent == body || joint.child == body) joint].length != 0)
					throw 'Screw feeder "${feeder.id}" inventory "$body" must be a distinct free occurrence';
				var component = [for (item in flat.occurrences) if (item.id == body) item.definition];
				if (component[0] != expected[0]) throw 'Screw feeder "${feeder.id}" inventory "$body" does not match its commissioned recipe';
				bodies.set(body,true);
			}
			for (delay in [feeder.trackDelay,feeder.deliveryDelay,feeder.deliveryTimeout])
				if (!Math.isFinite(delay) || delay <= 0) throw "Screw feeder needs positive finite commissioned timing";
			if (feeder.deliveryTimeout <= feeder.deliveryDelay) throw "Screw feeder timeout must exceed delivery delay";
			var pose = feeder.deposit;
			if (pose == null) throw "Screw feeder needs its physical deposit frame";
			for (value in [pose.x,pose.y,pose.z,pose.qx,pose.qy,pose.qz,pose.qw])
				if (!Math.isFinite(value)) throw "Screw feeder deposit frame is not finite";
			if (Math.abs(pose.qx*pose.qx+pose.qy*pose.qy+pose.qz*pose.qz+pose.qw*pose.qw-1) > 1e-8)
				throw "Screw feeder deposit frame needs a unit quaternion";
			for (job in jobs) {
				var pickup = job.pickup;
				if (job.socket == null || pickup == null) throw "Screw feeder job lost its CAD socket/pickup frames";
				var socket = materia.assembly.AssemblyFrames.compose(pose,job.socket);
				if (Math.abs(socket.x-pickup.x) > 1e-8 || Math.abs(socket.y-pickup.y) > 1e-8 || Math.abs(socket.z-pickup.z) > 1e-8
					|| Math.abs(Math.abs(socket.qx*pickup.qx+socket.qy*pickup.qy+socket.qz*pickup.qz+socket.qw*pickup.qw)-1) > 1e-8)
					throw 'Screw feeder "${feeder.id}" deposit does not match fastening "${job.fastening}" pickup calibration';
			}
		}
	}
}
