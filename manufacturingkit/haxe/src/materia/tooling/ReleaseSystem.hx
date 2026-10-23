package materia.tooling;

import materia.casting.CastingInput;
import materia.casting.CastingInput.CastingTextInput;
import materia.casting.CastingInput.CastingInputs;
import materia.casting.CastingFeature.CastingFeatureCodec;
import materia.casting.CastingRecordCodec;

@:wire enum EPSReleaseSafety {
	@:id(1) DeclaredSafe;
	@:id(2) Unsafe;
	@:id(3) Unresolved(reason:String);
}

enum abstract EPSCoveringKind(String) to String {
	var Tape = "tape";
	var Film = "film";
}

typedef ReleaseSealer = {
	var reference:CastingTextInput;
	var coats:Int;
	var drySeconds:CastingInput;
	var cureSeconds:CastingInput;
}

typedef ReleaseAgent = {
	var reference:CastingTextInput;
	var coats:Int;
	var drySeconds:CastingInput;
	var buffSeconds:CastingInput;
	var recoatSeconds:CastingInput;
	var cureSeconds:CastingInput;
}

typedef ReleaseFilm = {
	var reference:CastingTextInput;
	var coats:Int;
	var drySeconds:CastingInput;
}

/** One declared process, shared by every geometrically wetted tooling face.
 * Product selection and manufacturer times may remain unresolved shop inputs.
 * A declaration of EPS safety never substitutes for a physical coupon trial.
 */
typedef ReleaseSystem = {
	var sealer:ReleaseSealer;
	var agent:ReleaseAgent;
	var pva:Null<ReleaseFilm>;
	var epsCovering:{kind:EPSCoveringKind,reference:CastingTextInput};
	var epsSafety:EPSReleaseSafety;
}

class ReleaseSystems {
	/** Examples declare coat counts; actual products and their datasheet times
	 * remain shop inputs. There are no library defaults for these choices.
	 */
	public static function unresolved(sealerCoats:Int,agentCoats:Int,pvaCoats:Null<Int>,
			kind:EPSCoveringKind):ReleaseSystem {
		var seal="Select an exposed-board sealer and supply its product timing data";
		var agent="Select a mold-release wax and supply its product timing data";
		var film="Select PVA release film and supply its product drying data";
		var pva:Null<ReleaseFilm>=null;
		if (pvaCoats!=null) pva={reference:CastingTextInput.Unresolved(film),coats:cast pvaCoats,
			drySeconds:CastingInput.Unresolved(film)};
		return copy({
			sealer:{reference:CastingTextInput.Unresolved(seal),coats:sealerCoats,
				drySeconds:CastingInput.Unresolved(seal),cureSeconds:CastingInput.Unresolved(seal)},
			agent:{reference:CastingTextInput.Unresolved(agent),coats:agentCoats,
				drySeconds:CastingInput.Unresolved(agent),buffSeconds:CastingInput.Unresolved(agent),
				recoatSeconds:CastingInput.Unresolved(agent),cureSeconds:CastingInput.Unresolved(agent)},
			pva:pva,epsCovering:{kind:kind,reference:CastingTextInput.Unresolved("Select the declared EPS covering product")},
			epsSafety:EPSReleaseSafety.Unresolved("Release-agent EPS safety is not yet declared")
		});
	}

	public static function safety(raw:Dynamic):EPSReleaseSafety return switch CastingRecordCodec.string(raw,"kind") {
		case "declared-safe":
			CastingFeatureCodec.fields(raw,["kind"]);EPSReleaseSafety.DeclaredSafe;
		case "unsafe":
			CastingFeatureCodec.fields(raw,["kind"]);EPSReleaseSafety.Unsafe;
		case "unresolved":
			var reason=switch CastingInputs.text(raw) {
				case Known(_):throw "Expected unresolved EPS safety";
				case Unresolved(value):value;
			};
			EPSReleaseSafety.Unresolved(reason);
		default:throw "Unknown EPS release safety";
	};

	public static function safetyRecord(value:EPSReleaseSafety):Dynamic return switch value {
		case DeclaredSafe:{kind:"declared-safe"};
		case Unsafe:{kind:"unsafe"};
		case Unresolved(reason):{kind:"unresolved",reason:reason};
	};

	public static function covering(token:String):EPSCoveringKind return switch token {
		case "tape":Tape;
		case "film":Film;
		default:throw "Unknown EPS covering kind: "+token;
	};

	public static function safetyValues():Array<EPSReleaseSafety>
		return [EPSReleaseSafety.DeclaredSafe,EPSReleaseSafety.Unsafe,EPSReleaseSafety.Unresolved("fixture safety input")];

	public static function coveringValues():Array<EPSCoveringKind>
		return [Tape,Film];

	static function coats(raw:Dynamic):Int {
		var count=CastingRecordCodec.number(raw,"coats");
		if (count!=Math.floor(count) || count<1 || count>100) throw "Invalid release coat count";
		return Std.int(count);
	}

	static function seconds(raw:Dynamic,key:String):CastingInput {
		var value=CastingInputs.number(Reflect.field(raw,key));
		switch value {
			case Known(number):if (number<0) throw "Negative release process time: "+key;
			case Unresolved(_):
		}
		return value;
	}

	/** Raw records only. Typed copies explicitly pass through record(). */
	public static function snapshot(raw:Dynamic):ReleaseSystem {
		CastingFeatureCodec.fields(raw,["sealer","agent","pva","epsCovering","epsSafety"]);
		var sealer:Dynamic=Reflect.field(raw,"sealer"),agent:Dynamic=Reflect.field(raw,"agent");
		CastingFeatureCodec.fields(sealer,["reference","coats","drySeconds","cureSeconds"]);
		CastingFeatureCodec.fields(agent,["reference","coats","drySeconds","buffSeconds","recoatSeconds","cureSeconds"]);
		var pvaRaw:Dynamic=Reflect.field(raw,"pva"),pva:Null<ReleaseFilm>=null;
		if (pvaRaw!=null) {
			CastingFeatureCodec.fields(pvaRaw,["reference","coats","drySeconds"]);
			pva={reference:CastingInputs.text(Reflect.field(pvaRaw,"reference")),coats:coats(pvaRaw),
				drySeconds:seconds(pvaRaw,"drySeconds")};
		}
		var eps:Dynamic=Reflect.field(raw,"epsCovering");
		CastingFeatureCodec.fields(eps,["kind","reference"]);
		return {
			sealer:{reference:CastingInputs.text(Reflect.field(sealer,"reference")),coats:coats(sealer),
				drySeconds:seconds(sealer,"drySeconds"),cureSeconds:seconds(sealer,"cureSeconds")},
			agent:{reference:CastingInputs.text(Reflect.field(agent,"reference")),coats:coats(agent),
				drySeconds:seconds(agent,"drySeconds"),buffSeconds:seconds(agent,"buffSeconds"),
				recoatSeconds:seconds(agent,"recoatSeconds"),cureSeconds:seconds(agent,"cureSeconds")},
			pva:pva,epsCovering:{kind:covering(CastingRecordCodec.string(eps,"kind")),
				reference:CastingInputs.text(Reflect.field(eps,"reference"))},
			epsSafety:safety(Reflect.field(raw,"epsSafety"))
		};
	}

	static function filmRecord(film:ReleaseFilm):Dynamic return {
		reference:CastingInputs.textRecord(film.reference),coats:film.coats,
		drySeconds:CastingInputs.record(film.drySeconds)
	};

	public static function record(system:ReleaseSystem):Dynamic return {
		sealer:{reference:CastingInputs.textRecord(system.sealer.reference),coats:system.sealer.coats,
			drySeconds:CastingInputs.record(system.sealer.drySeconds),cureSeconds:CastingInputs.record(system.sealer.cureSeconds)},
		agent:{reference:CastingInputs.textRecord(system.agent.reference),coats:system.agent.coats,
			drySeconds:CastingInputs.record(system.agent.drySeconds),buffSeconds:CastingInputs.record(system.agent.buffSeconds),
			recoatSeconds:CastingInputs.record(system.agent.recoatSeconds),cureSeconds:CastingInputs.record(system.agent.cureSeconds)},
		pva:system.pva==null ? null : filmRecord(cast system.pva),
		epsCovering:{kind:(system.epsCovering.kind:String),reference:CastingInputs.textRecord(system.epsCovering.reference)},
		epsSafety:safetyRecord(system.epsSafety)
	};

	public static function copy(system:ReleaseSystem):ReleaseSystem return snapshot(record(system));

	/** Missing inputs are gathered from typed states, never field-name reflection. */
	public static function missing(system:ReleaseSystem,needsSealer:Bool=true,needsEPS:Bool=true,needsAgent:Bool=true):Array<{reason:String,blocks:Array<String>}> {
		var result:Array<{reason:String,blocks:Array<String>}>=[];
		function add(reason:String,block:String):Void {
			var existing=result.filter(item -> item.reason==reason);
			if (existing.length==0) result.push({reason:reason,blocks:[block]});
			else if (existing[0].blocks.indexOf(block)<0) existing[0].blocks.push(block);
		}
		function text(value:CastingTextInput,block:String):Void switch value {
			case Known(_):
			case Unresolved(reason):add(reason,block);
		}
		function time(value:CastingInput,block:String):Void switch value {
			case Known(_):
			case Unresolved(reason):add(reason,block);
		}
		if (needsSealer) {
			text(system.sealer.reference,"board sealing");
			time(system.sealer.drySeconds,"sealer recoating");
			time(system.sealer.cureSeconds,"release after sealer cure");
		}
		if (needsAgent) {
			text(system.agent.reference,"release coating");
			time(system.agent.drySeconds,"release-agent drying");
			time(system.agent.buffSeconds,"release-agent buffing");
			time(system.agent.recoatSeconds,"release-agent recoating");
			time(system.agent.cureSeconds,"assembly after release cure");
			if (system.pva!=null) {
				var film:ReleaseFilm=cast system.pva;
				text(film.reference,"PVA coating");
				time(film.drySeconds,"assembly after PVA drying");
			}
		}
		if (needsEPS) text(system.epsCovering.reference,"EPS covering");
		result.sort((a,b) -> Reflect.compare(a.reason,b.reason));
		return result;
	}
}
