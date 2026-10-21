import materia.casting.FoundationJoint;
import materia.casting.FoundationJoint.FoundationJoints as J;

class FoundationJointTests {
	static var assertions=0;
	static function check(ok:Bool,message:String):Void {assertions++;if (!ok) throw message;}
	static function rejects(fn:Void->Void,expected:String):Void {
		var message="";try fn() catch (error:Dynamic) message=Std.string(error);
		check(message.indexOf(expected)>=0,'Expected "$expected", got "$message"');
	}
	static function example():FoundationJoint return {
		id:"column-base",lowerBody:"base/casting",upperBody:"column/casting",route:FoundationJointRoute.Grouted,gapMm:2,gapToleranceMm:0.1,
		seats:[{id:"front",lower:{member:"base/seat",feature:"seat"},upper:{member:"column/seat",feature:"seat"}}],
		jacks:[{id:"front",screw:"jack",bearing:{member:"column/seat",feature:"seat"},receiver:{member:"block",feature:"fixing"}}],
		anchors:[{id:"front",screw:"anchor",target:{member:"base/seat",feature:"fixing"},
			through:[{member:"column/seat",feature:"fixing"}]}],cureBeforeTorque:true
	};
	public static function run():Void {
		var source=example(),saved=haxe.Json.stringify(J.snapshot(source));
		check(haxe.Json.stringify(J.snapshot(haxe.Json.parse(saved)))==saved,"joint records round trip without physical evidence placeholders");
		var copy=J.snapshot(source);copy.seats[0].lower.member="changed";
		check(source.seats[0].lower.member=="base/seat","joint snapshots own nested references");
		var mapped=J.remap(source,s -> "machine/"+s);
		check(mapped.id==source.id,"joint definition IDs are not instance paths");
		check(mapped.lowerBody=="machine/base/casting" && mapped.upperBody=="machine/column/casting","both body paths are remapped");
		check(mapped.seats[0].lower.member=="machine/base/seat" && mapped.seats[0].upper.member=="machine/column/seat","seat member paths are remapped");
		check(mapped.jacks[0].screw=="machine/jack" && mapped.jacks[0].receiver.member=="machine/block"
			&& mapped.jacks[0].bearing.member=="machine/column/seat","jack, receiver and bearing paths are remapped");
		check(mapped.anchors[0].screw=="machine/anchor" && mapped.anchors[0].target.member=="machine/base/seat"
			&& mapped.anchors[0].through[0].member=="machine/column/seat","anchor and every hole path are remapped");
		var ordered=example(),other=example();
		other.jacks[0].id="rear";other.jacks[0].screw="rear-jack";
		ordered.jacks.push(other.jacks[0]);
		var canonical=haxe.Json.stringify(J.snapshot(ordered));ordered.jacks.reverse();
		check(haxe.Json.stringify(J.snapshot(ordered))==canonical,"jack array order does not change the source encoding");
		var value:Dynamic=haxe.Json.parse(saved);Reflect.setField(value,"gapMm",null);
		rejects(() -> J.snapshot(value),"gapMm");
		source=example();source.gapMm=0;rejects(() -> J.snapshot(source),"positive gap");
		source=example();source.gapToleranceMm=2;rejects(() -> J.snapshot(source),"positive gap");
		source=example();source.gapToleranceMm=-0.1;rejects(() -> J.snapshot(source),"positive gap");
		source=example();source.upperBody=source.lowerBody;rejects(() -> J.snapshot(source),"distinct casting bodies");
		source=example();source.cureBeforeTorque=false;rejects(() -> J.snapshot(source),"cure before final torque");
		value=haxe.Json.parse(saved);Reflect.setField(value,"cureBeforeTorque","true");
		rejects(() -> J.snapshot(value),"cure before final torque");
		source=example();source.jacks=[];rejects(() -> J.snapshot(source),"requires seats, jacking screws and anchors");
		source=example();source.anchors[0].through=[];rejects(() -> J.snapshot(source),"source clearance holes");
		source=example();source.seats.push(source.seats[0]);rejects(() -> J.snapshot(source),"Duplicate foundation joint seat");
		source=example();source.anchors[0].through.push(source.anchors[0].target);
		rejects(() -> J.snapshot(source),"Duplicate foundation joint anchor hole");
		source=example();source.jacks[0].receiver.member="../block";rejects(() -> J.snapshot(source),"reference path");
		value=haxe.Json.parse(saved);Reflect.setField(value,"route","bonded-ish");rejects(() -> J.snapshot(value),"Unknown foundation joint route");
		value=haxe.Json.parse(saved);Reflect.setField(value,"jackingReference","free-text");rejects(() -> J.snapshot(value),"Unexpected casting record fields");
		source=example();source.route=FoundationJointRoute.Replicated;
		check(J.snapshot(source).route==FoundationJointRoute.Replicated,"replication and grouting are explicit source routes");
		Sys.println('FoundationJointTests: $assertions assertions passed');
	}
}
