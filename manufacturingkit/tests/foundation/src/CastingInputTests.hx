import materia.casting.CastingInput;
import materia.casting.CastingInput.CastingInputs as I;

class CastingInputTests {
	static var assertions=0;
	static function check(ok:Bool,message:String):Void {
		assertions++;
		if (!ok) throw message;
	}
	static function rejects(raw:Dynamic):Void {
		var error="";
		try I.number(raw) catch (e:Dynamic) error=Std.string(e);
		check(error.length>0,"Only explicit known/unresolved input records decode");
	}
	public static function run():Void {
		var supplied:CastingInput=Known(0.02);
		var zero:CastingInput=Known(0.0);
		var instrument:CastingInput.CastingTextInput=CastingTextInput.Known("instrument-1");
		check(I.require(I.number(I.record(supplied)))==0.02,"Known values survive the raw boundary");
		var unresolved:CastingInput=Unresolved("RM0 placement allocation");
		var missing=I.number(I.record(unresolved));
		check(I.missing([zero,missing,missing]).join(",")=="RM0 placement allocation",
			"Missing-input lists retain reasons and deduplicate the same input");
		check(!I.isKnown(missing) && I.isKnown(zero),"An explicit zero is supplied");
		check(I.requireText(I.text(I.textRecord(instrument)))=="instrument-1","Text evidence uses the same input states");
		rejects(null);
		rejects(0.02);
		rejects({kind:"future",value:1});
		rejects({kind:"unresolved",reason:" "});
		rejects({kind:"known",value:1,reason:"extra"});
		var error="";
		try I.require(missing) catch (e:Dynamic) error=Std.string(e);
		check(error.indexOf("RM0 placement allocation")>=0,"Unresolved arithmetic names its missing input");
		Sys.println('CastingInputTests: $assertions assertions passed');
	}
}
