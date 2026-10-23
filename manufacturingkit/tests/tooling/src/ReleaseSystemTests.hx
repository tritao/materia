import materia.casting.CastingInput;
import materia.casting.CastingInput.CastingTextInput;
import materia.tooling.ReleaseSystem;
import materia.tooling.ReleaseSystem.ReleaseSystems;

class ReleaseSystemTests {
	static var assertions=0;
	static function check(value:Bool,message:String):Void {
		assertions++;
		if (!value) throw message;
	}
	static function rejects(fn:Void->Void,expected:String):Void {
		var message="";
		try fn() catch (error:Dynamic) message=Std.string(error);
		check(message.indexOf(expected)>=0,'Expected "$expected", got "$message"');
	}
	static function fixture():ReleaseSystem return {
		sealer:{reference:CastingTextInput.Known("fixture sealer"),coats:2,
			drySeconds:CastingInput.Known(30),cureSeconds:CastingInput.Known(60)},
		agent:{reference:CastingTextInput.Known("fixture wax"),coats:5,
			drySeconds:CastingInput.Known(20),buffSeconds:CastingInput.Known(10),
			recoatSeconds:CastingInput.Known(30),cureSeconds:CastingInput.Known(60)},
		pva:{reference:CastingTextInput.Known("fixture PVA"),coats:1,drySeconds:CastingInput.Known(60)},
		epsCovering:{kind:EPSCoveringKind.Tape,reference:CastingTextInput.Known("fixture tape")},
		epsSafety:EPSReleaseSafety.DeclaredSafe
	};

	public static function run():Void {
		var source=fixture(),copy=ReleaseSystems.copy(source);
		check(copy.sealer.coats==2 && copy.agent.coats==5 && copy.pva!=null && copy.pva.coats==1,
			"The source declares the complete coat counts without library hardware defaults");
		check(ReleaseSystems.missing(copy).length==0,"Known product/timing inputs have no missing-input records");
		check(!Reflect.hasField(ReleaseSystems.record(copy),"couponEvidence"),
			"Chemical coupon evidence keeps its single existing fabrication-spec source");
		copy.agent.coats=3;
		check(source.agent.coats==5,"Typed process copies do not alias mutable choices");
		copy.pva=null;
		check(ReleaseSystems.copy(copy).pva==null,"A declared process may omit the optional PVA film");
		var reason="Select sealer product and supply its datasheet";
		copy.sealer.reference=CastingTextInput.Unresolved(reason);
		copy.sealer.drySeconds=CastingInput.Unresolved(reason);
		copy.sealer.cureSeconds=CastingInput.Unresolved(reason);
		var missing=ReleaseSystems.missing(copy);
		check(missing.length==1 && missing[0].reason==reason && missing[0].blocks.length==3,
			"One missing shop input reports every process step that it blocks");
		for (value in ReleaseSystems.safetyValues()) {
			check(haxe.Json.stringify(ReleaseSystems.safetyRecord(ReleaseSystems.safety(ReleaseSystems.safetyRecord(value))))
				==haxe.Json.stringify(ReleaseSystems.safetyRecord(value)),"Every EPS safety state round-trips through its checked decoder");
			source.epsSafety=value;
			check(haxe.Json.stringify(ReleaseSystems.safetyRecord(ReleaseSystems.copy(source).epsSafety))
				==haxe.Json.stringify(ReleaseSystems.safetyRecord(value)),"Every EPS safety state survives the complete raw codec");
		}
		for (value in ReleaseSystems.coveringValues()) {
			check(ReleaseSystems.covering((value:String))==value,"Every EPS covering vocabulary value round-trips through its checked decoder");
			source.epsCovering.kind=value;
			check(ReleaseSystems.copy(source).epsCovering.kind==value,"Every EPS covering token survives the complete raw codec");
		}
		rejects(() -> ReleaseSystems.safety({kind:"probably-safe"}),"Unknown EPS release safety");
		rejects(() -> ReleaseSystems.covering("paint"),"Unknown EPS covering kind");
		var raw:Dynamic=ReleaseSystems.record(source);
		raw.agent.coats=0;
		rejects(() -> ReleaseSystems.snapshot(raw),"Invalid release coat count");
		raw=haxe.Json.parse(StringTools.replace(haxe.Json.stringify(ReleaseSystems.record(source)),
			'"coats":5','"coats":1.5'));
		rejects(() -> ReleaseSystems.snapshot(raw),"Invalid release coat count");
		raw=ReleaseSystems.record(source);
		raw.agent.drySeconds={kind:"known",value:-1};
		rejects(() -> ReleaseSystems.snapshot(raw),"Negative release process time: drySeconds");
		raw=ReleaseSystems.record(source);
		raw.epsSafety={kind:"yes"};
		rejects(() -> ReleaseSystems.snapshot(raw),"Unknown EPS release safety");
		raw=ReleaseSystems.record(source);
		raw.agent.reference={kind:"known",value:""};
		rejects(() -> ReleaseSystems.snapshot(raw),"Empty known casting input");
		raw=ReleaseSystems.record(source);
		raw.unplanned="extra";
		rejects(() -> ReleaseSystems.snapshot(raw),"Unexpected casting record fields");
		Sys.println('ReleaseSystemTests: $assertions assertions passed');
	}
	public static function main():Void run();
}
