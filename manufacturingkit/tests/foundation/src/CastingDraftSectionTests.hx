import materia.casting.CastingProfileOffsets as O;
import materia.casting.CastingProfiles as P;
import materia.casting.CastingProfileGeometry as G;

class CastingDraftSectionTests {
	static var assertions=0;
	static function check(value:Bool,message:String):Void {
		assertions++;
		if (!value) throw message;
	}
	static function near(actual:Float,expected:Float,message:String):Void
		check(Math.abs(actual-expected)<1e-7,message+": "+actual+" vs "+expected);
	static function rejects(fn:Void->Void,expected:String):Void {
		var message="";
		try fn() catch (error:Dynamic) message=Std.string(error);
		check(message.indexOf(expected)>=0,"Expected "+expected+", got "+message);
	}
	public static function main():Void run();
	public static function run():Void {
		var opening=P.rectangle(-20,-15,40,30);
		near(G.area(O.inward(opening,2)),36*26,"rectangular walls move by their normal distance");
		near(G.area(opening),1200,"opening dimensions stay authored");
		near(G.area(O.inward(P.circle(3,-2,10),2)),64*Math.PI,"circular draft retains exact half arcs");
		var round=P.roundedRectangle(-20,-15,40,30,4),inner=O.inward(round,2);
		near(G.area(inner),36*26-(4-Math.PI)*4,"rounded draft offsets both lines and corner radii");
		check(inner.length==round.length,"rounded section keeps one-to-one wall correspondence");
		var concave=P.polygon([{x:0,y:0},{x:30,y:0},{x:30,y:10},{x:10,y:10},{x:10,y:30},{x:0,y:30}]);
		near(G.area(O.inward(concave,1)),28*28-20*20,"reentrant line walls intersect analytically");
		var tangent=P.obround(0,0,40,20);
		near(G.area(O.inward(tangent,2)),20*16+64*Math.PI,"obround retains tangent line and arc joins");
		var unchanged=O.inward(round,0);unchanged[0].start.x+=1;
		check(unchanged[0].start.x!=round[0].start.x,"zero draft returns detached records");
		rejects(() -> O.inward(round,4),"arc collapses");
		rejects(() -> O.inward(opening,16),"edge collapses");
		rejects(() -> O.inward(opening,-1),"Invalid draft section offset");
		var body:Array<materia.casting.CastingPrism> =[{id:"body",profile:[{x:-60.0,y:-50.0},{x:60.0,y:-50.0},
			{x:60.0,y:50.0},{x:-60.0,y:50.0}],height:80,pose:materia.assembly.AssemblyFrames.identity()}];
		var feature=materia.casting.CastingPatterns.box("drafted-pocket",materia.casting.CastingFeature.CastingFeatureKind.Lightening,
			[materia.casting.CastingPatterns.face("body.z-max")],40,30,61,0,0,20);
		feature.draft.degrees=2;
		var sections=materia.casting.CastingDraftSections.derive(body,feature),offset=60*Math.tan(2*Math.PI/180);
		near(sections.depth,60,"draft depth ends at body opening, independent of arbitrary cut overshoot");
		near(G.area(sections.opening),1200,"authored dimensions belong to opening");
		near(G.area(sections.inner),(40-2*offset)*(30-2*offset),"inner dimensions follow angle and actual depth");
		near(sections.pose.z,20,"inner section has source-coordinate placement");
		feature.geometry.pose.z=-1;feature.draft.axis={x:0,y:0,z:-1};
		feature.openings=[materia.casting.CastingPatterns.face("body.z-min")];
		sections=materia.casting.CastingDraftSections.derive(body,feature);
		near(sections.depth,60,"reverse withdrawal uses actual opening plane");
		near(sections.pose.z,60,"reverse withdrawal starts at inner section");
		near(materia.assembly.AssemblyFrames.transformVector(sections.pose,0,0,1).z,-1,"reverse section frame points toward opening");
		feature=materia.casting.CastingPatterns.box("side-former",materia.casting.CastingFeature.CastingFeatureKind.ServiceOpening,
			[materia.casting.CastingPatterns.face("body.x-max")],61,30,30,30.5,0,25);
		feature.draft={degrees:2,axis:{x:1,y:0,z:0}};
		sections=materia.casting.CastingDraftSections.derive(body,feature);
		near(sections.depth,60,"side withdrawal respects source-coordinate axis");
		near(G.area(sections.opening),900,"side former uses transverse dimensions");
		check(materia.casting.CastingDraftSections.unsupportedReason(body,feature)==null,
			"a realized side draft is available independently of the selected mold template");
		feature.draft.degrees=30;
		check(materia.casting.CastingDraftSections.unsupportedReason(body,feature)==null,
			"a collapsing section is a design error rather than a missing geometry capability");
		rejects(() -> materia.casting.CastingDraftSections.derive(body,feature),"edge collapses");
		feature.draft.degrees=2;
		feature.draft.axis={x:0.6,y:0,z:0.8};
		check(materia.casting.CastingDraftSections.unsupportedReason(body,feature).indexOf("oblique")>=0,
			"an unrealized oblique draft has an explicit capability reason");
		feature.draft.axis={x:0,y:0,z:1};
		check(materia.casting.CastingDraftSections.unsupportedReason(body,feature).indexOf("declared planar withdrawal opening")>=0,
			"an unavailable withdrawal plane stays explicitly unsupported");
		rejects(() -> materia.casting.CastingDraftSections.derive(body,feature),"Casting feature side-former");
		Sys.println("CastingDraftSectionTests: "+assertions+" assertions passed");
	}
}
