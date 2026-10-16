package materia.project;

import materia.project.SceneScrewDefinition.SceneScrew;
import materia.project.SceneArtifact.SceneArtifactMissionStep;

/** A connection is qualified once; its final pass refers to the preceding snug job. */
@:wire enum ScrewInvocation {
	@:id(1) Single(job:SceneScrew);
	@:id(2) Snug(job:SceneScrew, slot:Int);
	@:id(3) Final(fastening:String);
	/** Single pass whose exhausted NOK stops the controller program before downstream handling. */
	@:id(4) Required(job:SceneScrew);
}

/** Resolve stage identity without duplicating geometry, qualification or replacement supply. */
class SceneScrewInvocation {
	public static function job(value:Null<ScrewInvocation>, steps:Array<SceneArtifactMissionStep>):Null<SceneScrew> {
		if (value == null) return null;
		return switch value {
			case Single(job), Required(job), Snug(job,_): job;
			case Final(name):
				var found:Null<SceneScrew> = null;
				for (step in steps) if (step.screw != null) switch step.screw {
					case Snug(job,_) if (job.fastening == name):
						if (found != null) throw 'Final fastening "$name" has more than one snug definition';
						found = job;
					case _: {}
				}
				if (found == null) throw 'Final fastening "$name" has no snug definition';
				found;
		};
	}
	public static function decode(source:String):ScrewInvocation {
		var preserved = SceneWireJson.preserveNumbers(source);
		var value:ScrewInvocation = haxeon.wire.JsonWire.decode(preserved);
		var raw:Dynamic = Reflect.field(haxe.Json.parse(preserved),"value");
		var canonical:Dynamic = Reflect.field(haxe.Json.parse(haxeon.wire.JsonWire.encode(value)),"value");
		var arguments = SceneWireJson.enumArguments(raw,canonical);
		switch value {
			case Single(job), Required(job), Snug(job,_): SceneScrewDefinition.validateWireIntegers(arguments[0],job);
			case _: {}
		}
		return value;
	}
	/** All snug passes precede final; every connection and physical supply has exactly one owner. */
	public static function validate(steps:Array<SceneArtifactMissionStep>):Void {
		var names:Map<String,Bool> = [], supplies:Map<String,String> = [];
		var snug:Map<String,Bool> = [], finalPass:Map<String,Bool> = [];
		var finalStarted = false;
		for (index in 0...steps.length) {
			var step = steps[index];
			if (step == null) throw 'Scene artifact mission step $index is empty';
			if (step.kind != "screw") continue;
			var value = step.screw;
			if (value == null) throw "Screw mission step has no invocation";
			switch value {
				case Final(name):
					if (!snug.exists(name) || finalPass.exists(name)) throw 'Final fastening "$name" needs one preceding unmatched snug pass';
					finalPass.set(name,true); finalStarted = true;
				case Single(job), Required(job), Snug(job,_):
					if (job == null) throw "Screw invocation has no qualified CAD job";
					if (job.fastening == null || job.fastening.length == 0) throw "Screw invocation has no fastening identity";
					if (names.exists(job.fastening)) throw 'Fastening "${job.fastening}" has more than one qualified job';
					names.set(job.fastening,true);
					switch value {
						case Snug(_,slot):
							if (finalStarted) throw 'Snug fastening "${job.fastening}" follows a final pass';
							if (slot <= 0 || slot == job.slot) throw 'Fastening "${job.fastening}" needs distinct positive snug and final slots';
							if (job.recovery == null) throw 'Staged fastening "${job.fastening}" needs an explicit replacement and reject policy';
							snug.set(job.fastening,true);
						case _: {}
					}
					var bodies = [job.screw], recovery = job.recovery;
					if (recovery != null) {
						if (recovery.replacement == null) throw 'Fastening "${job.fastening}" has no replacement supply';
						bodies.push(recovery.replacement.screw);
					}
					for (body in bodies) {
						if (body == null || body.length == 0) throw 'Fastening "${job.fastening}" has no screw occurrence identity';
						if (supplies.exists(body)) throw 'Screw occurrence "$body" supplies more than one attempt or fastening';
						supplies.set(body,job.fastening);
					}
			}
		}
		for (name in snug.keys()) if (!finalPass.exists(name)) throw 'Snug fastening "$name" has no final pass';
	}
}
