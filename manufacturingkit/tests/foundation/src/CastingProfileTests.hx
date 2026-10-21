import materia.casting.CastingFeature;
import materia.casting.CastingProfile;
import materia.casting.CastingProfileGeometry as G;
import materia.casting.CastingProfiles as P;
import materia.casting.CastingFeatureCodec;
import materia.casting.CastingPatterns;

class CastingProfileTests {
	static var assertions=0;
	static function check(value:Bool,message:String):Void {
		assertions++;
		if (!value) throw message;
	}
	static function rejects(fn:Void->Void,expected:String):Void {
		var message="";
		try fn() catch (error:Dynamic) message=Std.string(error);
		check(message.indexOf(expected)>=0,"Expected "+expected+", got "+message);
	}
	public static function run():Void {
		var profile=P.roundedRectangle(-20,-15,40,30,4);
		G.validate(profile);
		check(Math.abs(G.area(profile)-(1200-(4-Math.PI)*16))<1e-9,"rounded profile has analytic line/arc area");
		check(profile.filter(segment -> segment.kind==Arc).length==4,"rounded corners remain four exact arcs");
		check(Math.abs(G.area(P.circle(3,-2,5))-25*Math.PI)<1e-9,"two exact half arcs enclose a circle");
		var filleted=P.roundedPolygon([{x:-20,y:-15},{x:20,y:-15},{x:20,y:15},{x:-20,y:15}],4);
		check(Math.abs(G.area(filleted)-G.area(profile))<1e-9,"parametric corner fillets and explicit arcs describe the same area");
		var circle=P.roundedPolygon([{x:0,y:0},{x:20,y:0},{x:20,y:20},{x:0,y:20}],10);
		check(circle.length==4 && Math.abs(G.area(circle)-100*Math.PI)<1e-9,
			"meeting fillets retain exact arcs without zero-length lines");
		var reentrant=P.roundedPolygon([{x:-20,y:-15},{x:20,y:-15},{x:20,y:0},{x:0,y:0},{x:0,y:15},{x:-20,y:15}],2);
		check(reentrant.filter(segment -> segment.sweep<0).length==1,"re-entrant corner has the correct negative arc sweep");
		check(Math.abs(G.area(reentrant)-(900-(4-Math.PI)*4))<1e-9,"concave/convex fillets have analytic signed area");
		rejects(() -> P.roundedPolygon([{x:0,y:0},{x:40,y:0},{x:40,y:30},{x:0,y:30}],16),"Corner radius does not fit polygon");
		var copy=CastingProfileCodec.snapshot(profile);
		copy[0].start.x+=1;
		check(copy[0].start.x!=profile[0].start.x,"profile codec owns its points");
		var broken=CastingProfileCodec.snapshot(profile);
		// Keep the loop endpoints intact; only the arc description is inconsistent.
		broken[1].sweep+=0.1;
		rejects(() -> G.validate(broken),"Invalid profile arc");
		var raw:Array<Dynamic> = cast haxe.Json.parse(haxe.Json.stringify(profile));
		raw[0].center={x:0,y:0};
		rejects(() -> CastingProfileCodec.snapshot(raw),"Invalid profile line");
		raw=cast haxe.Json.parse(haxe.Json.stringify(profile));
		raw[0].extra=1;
		rejects(() -> CastingProfileCodec.snapshot(raw),"Unexpected casting record fields");
		var cut=CastingPatterns.box("profile-port",ServiceOpening,
			[CastingPatterns.face("body.z-min"),CastingPatterns.face("body.z-max")],40,30,20);
		cut.geometry.solid=Prism(profile,20);
		var encoded=CastingFeatureCodec.encode([cut]),decoded=CastingFeatureCodec.decode(encoded)[0];
		check(CastingFeatureCodec.encode([decoded])==encoded,"profile feature has one strict canonical encoding");
		check(switch decoded.geometry.solid {case Prism(contour,height):height==20 && contour.length==8;default:false;},
			"profile and extrusion height survive the feature codec");
		check(!CastingGeometrySolids.isBox(decoded.geometry.solid),"a rounded prism never becomes its bounding box");
		rejects(() -> CastingGeometrySolids.box(decoded.geometry.solid,decoded.id),"Unsupported non-box solid: profile-port");
		var connected=CastingPatterns.box("box-port",Clearance,
			[CastingPatterns.face("body.z-min"),CastingPatterns.feature("profile-port")],20,20,10);
		var cavities=materia.casting.CastingCavities.derive(materia.casting.CastingEnvelope.build(80,60,20),[connected,decoded]);
		check(switch cavities.filter(region -> region.featureId==connected.id)[0].geometry {
			case SolidIntersection(_,_,_):true;default:false;
		},"box-to-profile openings retain the exact shared expression");
		var previous:Dynamic=haxe.Json.parse(encoded);previous.version=3;
		rejects(() -> CastingFeatureCodec.decode(haxe.Json.stringify(previous)),"Unsupported casting feature schema");
		cut.geometry.solid=Prism(profile,0);
		rejects(() -> CastingFeatureCodec.encode([cut]),"height");
		cut.geometry.solid=Prism(profile,20);cut.kind=InsertRecess;
		rejects(() -> CastingFeatureCodec.encode([cut]),"Insert recess requires source insert geometry");
		Sys.println('CastingProfileTests: $assertions assertions passed');
	}
}
