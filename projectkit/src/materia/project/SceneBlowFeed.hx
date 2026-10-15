package materia.project;

import haxeon.wire.JsonWire;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.project.SceneArtifact.SceneArtifactData;
import materia.project.SceneArtifact.SceneArtifactMission;
import materia.project.SceneScrewDefinition.SceneScrew;

/** One commissioned tube/receiver path; seconds, bar and millimetres are explicit. */
@:wire typedef SceneBlowFeed = {
	@:id(1) var id:String;
	@:id(2) var receiver:String;
	@:id(3) var socketConnector:String;
	@:id(4) var receiptSensor:String;
	@:id(5) var inventory:Array<String>;
	@:id(6) var bufferCapacity:Int;
	@:id(7) var trackDelay:Float;
	@:id(8) var deliveryDelay:Float;
	@:id(9) var deliveryTimeout:Float;
	@:id(10) var airPulse:Float;
	@:id(11) var airPressureBar:Float;
	@:id(12) var tube:String;
	@:id(13) var externalTubeLengthMm:Float;
	@:id(14) var minimumBendRadiusMm:Float;
	@:id(15) var receivingAngularSpeed:Float;
	@:id(16) var receivingFeedSpeed:Float;
}
class SceneBlowFeeds {
	public static function decode(value:Dynamic):Array<SceneBlowFeed> {
		var source = SceneWireJson.preserveNumbers(haxe.Json.stringify(value));
		var result:Array<SceneBlowFeed> = JsonWire.decode(source);
		var inputs:Array<Dynamic> = cast Reflect.field(haxe.Json.parse(source),"value");
		var outputs:Array<Dynamic> = cast Reflect.field(haxe.Json.parse(JsonWire.encode(result)),"value");
		for (i in 0...result.length) SceneWireJson.integerFields(inputs[i],outputs[i],["bufferCapacity"]);
		return result;
	}
	public static function validate(paths:Array<SceneBlowFeed>, mission:SceneArtifactMission, scene:SceneArtifactData):Void {
		if (mission == null) return;
		var jobs = [for (step in mission.steps) if (step.kind == "screw") SceneScrewInvocation.job(step.screw,mission.steps)];
		var ids:Map<String,Bool> = [], bodies:Map<String,Bool> = [], receivers:Map<String,Bool> = [];
		if (mission.screwFeeders != null) for (feeder in mission.screwFeeders) for (id in feeder.inventory) bodies.set(id,true);
		if (paths != null) {
			if (scene.assemblyDefinition == null || paths.length == 0 || paths.length > 32) throw "Blow feed needs bounded paths and actual assembly data";
			var flat = AssemblyDefinitionFlattener.flattenView(scene.assemblyDefinition);
			for (path in paths) {
				if (path == null || path.id == null || path.id.length == 0 || ids.exists(path.id) || receivers.exists(path.receiver))
					throw "Blow-feed paths need distinct identities and receivers";
				ids.set(path.id,true); receivers.set(path.receiver,true);
				for (member in [path.receiver,path.tube]) if ([for (item in flat.occurrences) if (item.id == member) item].length != 1)
					throw 'Blow-feed "${path.id}" lost its CAD occurrence "$member"';
				var receiver = [for (item in flat.occurrences) if (item.id == path.receiver) item][0];
				var component = [for (item in flat.definitions) if (item.id == receiver.definition) item][0];
				if ([for (connector in component.connectors) if (connector.name == path.socketConnector) connector].length != 1)
					throw 'Blow-feed "${path.id}" lost its named receiving socket';
				if (flat.sensors == null || [for (sensor in flat.sensors) if (sensor.id == path.receiptSensor && sensor.kind == "presence" && sensor.occurrence == path.receiver) sensor].length != 1)
					throw 'Blow-feed "${path.id}" needs actual receiving presence';
				if (path.inventory == null || path.inventory.length == 0 || path.inventory.length > 256 || path.bufferCapacity < 1 || path.bufferCapacity > 256)
					throw "Blow feed needs bounded identified inventory and buffering";
				for (value in [path.trackDelay,path.deliveryDelay,path.deliveryTimeout,path.airPulse,path.airPressureBar,path.externalTubeLengthMm,path.minimumBendRadiusMm,path.receivingAngularSpeed,path.receivingFeedSpeed])
					if (!Math.isFinite(value) || value <= 0) throw "Blow-feed commissioned timing, air and routing inputs must be positive and finite";
				if (path.deliveryTimeout <= path.deliveryDelay || path.deliveryTimeout <= path.airPulse || path.externalTubeLengthMm <= 2*Math.PI*path.minimumBendRadiusMm)
					throw "Blow-feed receipt timeout or reserved tube bend/slack is inconsistent";
				var actual:Array<SceneScrew> = [];
				for (job in jobs) if (job != null) switch job.delivery {
					case BlowFeed(id): if (id == path.id) actual.push(job);
					case Presented: {}
				}
				if (actual.length == 0) throw 'Blow-feed "${path.id}" supplies no named fastening';
				var expected = [for (item in flat.occurrences) if (item.id == actual[0].screw) item.definition];
				if (expected.length != 1) throw "Blow feed lost its commissioned screw recipe";
				for (job in actual) if (path.inventory.indexOf(job.screw) < 0) throw 'Blow-feed "${path.id}" does not own "${job.screw}"';
				for (body in path.inventory) {
					var found = [for (item in flat.occurrences) if (item.id == body) item];
					if (bodies.exists(body) || found.length != 1 || found[0].definition != expected[0] || [for (joint in flat.joints) if (joint.parent == body || joint.child == body) joint].length != 0)
						throw 'Blow-feed "${path.id}" supply "$body" must be a distinct free instance of the actual commissioned screw';
					bodies.set(body,true);
				}
			}
		}
		for (job in jobs) if (job != null) switch job.delivery {
			case BlowFeed(id): if (!ids.exists(id)) throw 'Fastening "${job.fastening}" lost blow-feed path "$id"';
			case Presented: {}
		}
	}
}
