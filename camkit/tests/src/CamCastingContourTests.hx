import camkit.CamCastingContours;
import camkit.CamJob;
import materia.casting.CastingProfiles;
import toolpathkit.path.Point3;
import toolpathkit.tool.Tool;

/** Neutral contours reach CAM without panel-shaped placeholder operations. */
class CamCastingContourTests {
	static var assertions=0;
	static function check(value:Bool,message:String):Void {
		assertions++;
		if (!value) throw message;
	}
	static function near(actual:Float,expected:Float,message:String):Void
		check(Math.abs(actual-expected)<1e-9,message+": "+actual+" vs "+expected);
	static function rejects(action:Void->Void,expected:String):Void {
		var message="";
		try action() catch (error:Dynamic) message=Std.string(error);
		check(message.indexOf(expected)>=0,'Expected "'+expected+'", got "'+message+'"');
	}

	public static function run():Int {
		assertions=0;
		var profile=CastingProfiles.roundedRectangle(-7,3,40,25,5);
		var original=haxe.Json.stringify(profile),tolerance=0.00001;
		var converted=CamCastingContours.fromProfile(profile,tolerance,true,-20,-10,80,0.004,0.0123);
		var exact=converted.exactBoundary;
		check(exact!=null && exact.length==profile.length,"conversion retains each exact source line/arc");
		if (exact==null) throw "Missing exact neutral CAM boundary";
		var area=0.0,arcs=0;
		var lowX=Math.POSITIVE_INFINITY,highX=Math.NEGATIVE_INFINITY;
		var lowY=Math.POSITIVE_INFINITY,highY=Math.NEGATIVE_INFINITY;
		for (edge in exact) {
			area+=edge.area();
			if (!edge.isLine()) {
				arcs++;
				near(edge.radius,0.005,"source radius is scaled once, without cutter compensation");
				near(edge.sweep,-Math.PI/2,"reflection reverses each exact quarter arc");
			}
			for (point in edge.extrema()) {
				lowX=Math.min(lowX,point.x);highX=Math.max(highX,point.x);
				lowY=Math.min(lowY,point.y);highY=Math.max(highY,point.y);
			}
		}
		check(arcs==4,"all four neutral rounded corners remain arcs");
		near(area,-(40*25-(4-Math.PI)*25)*1e-6,"exact analytic area survives scaling and reflection");
		near(lowX,0.031,"reflected and translated minimum X");
		near(highX,0.071,"reflected and translated maximum X");
		near(lowY,0.017,"translated minimum Y");
		near(highY,0.042,"translated maximum Y");
		near(converted.z,0.0123,"a tooling section keeps its declared work-plane height");
		check(converted.signedArea<0 && converted.chordTolerance==tolerance,
			"the chordal contour retains reflection and its declared tolerance");
		check(haxe.Json.stringify(profile)==original,"CAM conversion leaves neutral finished geometry intact");

		var circle=CastingProfiles.circle(30,40,10);
		var fine=CamCastingContours.fromProfile(circle,tolerance);
		var coarse=CamCastingContours.fromProfile(circle,0.0001);
		check(fine.vertices.length>coarse.vertices.length,"tighter chord tolerance refines only the CAM representation");
		for (index in 0...fine.vertices.length) {
			var a=fine.vertices[index],b=fine.vertices[(index+1)%fine.vertices.length];
			near(Math.sqrt(Math.pow(a.x-0.03,2)+Math.pow(a.y-0.04,2)),0.01,
				"each sampled circle vertex lies on the analytic source boundary");
			var x=(a.x+b.x)/2-0.03,y=(a.y+b.y)/2-0.04;
			check(0.01-Math.sqrt(x*x+y*y)<=tolerance+1e-12,
				"every chord respects the declared sagitta tolerance");
		}
		var tool=new Tool(9,0,0.006);
		var program=new CamJob(0.005,10000,new Point3(0.05,0.05,0.02),0,tolerance)
			.profile(fine,tool,-0.002,0.005,"outside",0.002,0.001,"neutral-prism").finish();
		var compensatedArcs=0;
		for (operation in program.ops) switch operation {
			case Move(Cut,Arc(_,radius,_,_),_,_,span):
				compensatedArcs++;
				near(radius,0.013,"CamJob applies the cutter radius to the unchanged source circle");
				check(span.featureRef=="neutral-prism","CAM motion retains its source feature identity");
			default:
		}
		check(compensatedArcs>0,"neutral source curves produce real compensated arc motion");
		rejects(() -> CamCastingContours.fromProfile(circle,0),"positive chord tolerance");
		rejects(() -> CamCastingContours.fromProfile(circle,tolerance,false,0,0,0,0,Math.NaN),"finite work plane");
		var damaged=CastingProfiles.circle(30,40,10);
		damaged[0].end.x+=0.2;
		rejects(() -> CamCastingContours.fromProfile(damaged,tolerance,false,0,0,0,0,0,"former-pocket"),
			"former-pocket: Invalid profile arc");
		return assertions;
	}
}
