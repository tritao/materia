import materia.casting.CastingRegion;
import materia.casting.CastingRegion.CastingRegionCodec as C;
import materia.casting.CastingFeature.CastingGeometrySolid;
import materia.assembly.AssemblyFrames as F;

class CastingRegionTests {
	static var assertions=0;
	static function check(value:Bool,message:String):Void {assertions++;if (!value) throw message;}
	static function rejects(fn:Void->Void,expected:String):Void {
		var message="";try fn() catch (error:Dynamic) message=Std.string(error);
		check(message.indexOf(expected)>=0,"Expected "+expected+", got "+message);
	}
	public static function run():Void {
		var a=CastingRegion.Placed({solid:CastingGeometrySolid.Cut(materia.casting.CastingSolid.Box(20,30,40)),pose:F.identity()});
		var b=CastingRegion.Placed({solid:CastingGeometrySolid.Cut(materia.casting.CastingSolid.Cylinder(10,40)),pose:F.translation(1,2,3)});
		var one=CastingRegion.Union([a,b]),two=CastingRegion.Union([b,a]);
		check(haxe.Json.stringify(C.record(one))==haxe.Json.stringify(C.record(two)),"union encoding is independent of operand order");
		check(haxe.Json.stringify(C.record(CastingRegion.Intersection([a,b])))
			==haxe.Json.stringify(C.record(CastingRegion.Intersection([b,a]))),"intersection encoding is independent of operand order");
		var difference=CastingRegion.Difference(a,[b]);
		var restored=C.snapshot(haxe.Json.parse(haxe.Json.stringify(C.record(difference))));
		check(haxe.Json.stringify(C.record(restored))==haxe.Json.stringify(C.record(difference)),
			"boolean expression round-trips through strict neutral records");
		var draft=CastingRegion.Drafted("former",materia.casting.CastingProfiles.rectangle(-20,-15,40,30),F.identity(),60,2);
		check(switch C.snapshot(C.record(draft)) {case Drafted(id,_,_,depth,degrees):id=="former" && depth==60 && degrees==2;default:false;},
			"drafted expression keeps source identity, opening, depth and angle");
		var raw:Dynamic=C.record(a);Reflect.setField(raw,"bounds",{});
		rejects(() -> C.snapshot(raw),"Unexpected casting record fields");
		rejects(() -> C.snapshot({kind:"bounds",box:{}}),"Unknown casting region kind");
		rejects(() -> C.snapshot({kind:"union",regions:new Array<Dynamic>()}),"Invalid casting region operand count");
		rejects(() -> C.snapshot(CastingRegion.Drafted("bad-former",materia.casting.CastingProfiles.roundedRectangle(-20,-15,40,30,1),F.identity(),60,2)),
			"Casting feature bad-former");
		var children:Array<CastingRegion> =[],cycle=CastingRegion.Union(children);children.push(cycle);
		rejects(() -> C.snapshot(cycle),"Casting region expression exceeds limit");
		check(switch C.snapshot(CastingRegion.Difference(a,[])) {case Difference(_,cuts):cuts.length==0;default:false;},"difference may have no cuts without inventing a solid");
		Sys.println("CastingRegionTests: "+assertions+" assertions passed");
	}
}
