import materia.casting.CastingFeature;
import materia.casting.CastingFeatureCodec;
import materia.casting.CastingPatterns as P;
import materia.casting.CastingProfiles;
import materia.casting.CastingCavities;
import materia.casting.CastingEnvelope;
import machinekit.structural.CastingCavityCad;
import machinekit.structural.CastingProfileCad;

class FeatureGeometryTests {
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
	static function port(profile:Array<materia.casting.CastingProfile.CastingSegment>):CastingFeature {
		var f=P.box("profile-port",ServiceOpening,[P.face("body.z-min"),P.face("body.z-max")],40,30,30);
		f.geometry.solid=Prism(profile,30);
		return CastingFeatureCodec.decode(CastingFeatureCodec.encode([f]))[0];
	}
	static function fillGeometry():Void {
		var local=cadkit.modeling.Part.prism([
			new cadkit.modeling.Vector(0,0),new cadkit.modeling.Vector(40,0),new cadkit.modeling.Vector(40,20),
			new cadkit.modeling.Vector(10,40),new cadkit.modeling.Vector(0,40)
		],0,30);
		var pose=materia.assembly.AssemblyFrames.fromRotationMatrix(0,0,0,[1,0,0,0,0,-1,0,1,0]);
		var wedge=machinekit.assembly.PosedParts.moved(local,pose);local.close();
		var geometry=new machinekit.tooling.CastingFillGeometry("sloped-ceiling",wedge);
		var heights=geometry.criticalHeights();
		check(heights.length==3 && Math.abs(heights[0])<1e-6 && Math.abs(heights[1]-20)<1e-6
			&& Math.abs(heights[2]-40)<1e-6,"sloped ceiling has exact critical vertex levels");
		for (height in [0.0,10.0,20.0,30.0,40.0]) {
			var below=geometry.below(height);
			var expected=height<=20 ? height*1200 : 24000+30*(40*(height-20)-0.75*(height-20)*(height-20));
			check(Math.abs(below.volume()-expected)<1e-5,"sloped-ceiling volume agrees with the analytic level integral: "+height);
			below.close();
		}
		var opening=geometry.section(40);
		check(Math.abs(opening.shape.area()-300)<1e-6,"native fill section retains the true open area");opening.close();
		var air=geometry.airComponents(20);
		check(air.length==1 && Math.abs(air[0].volume()-15000)<1e-5,"sloped-ceiling air is one exact solid at the critical level");
		for (part in air) part.close();
		var atTop=geometry.airComponents(40);
		check(atTop.length==0,"the fill-plane cap cannot survive as a zero-volume air component");
		geometry.close();wedge.close();
		rejects(() -> geometry.section(20),"Closed fill geometry: sloped-ceiling");
		var cylinder=cadkit.modeling.Part.cylinderSpan(5,0,20);
		var horizontal=machinekit.assembly.PosedParts.moved(cylinder,pose);cylinder.close();
		geometry=new machinekit.tooling.CastingFillGeometry("round-section",horizontal);horizontal.close();
		heights=geometry.criticalHeights();
		check(heights.filter(height -> Math.abs(height+5)<1e-6).length==1
			&& heights.filter(height -> Math.abs(height-5)<1e-6).length==1,
			"critical levels include circle extrema between the seam vertices");
		var half=geometry.below(0);
		check(Math.abs(half.volume()-250*Math.PI)<1e-6,"round clipping retains exact half-cylinder volume after caller closes the input");
		half.close();geometry.close();
		for (edgeContact in [false,true]) {
			var a=cadkit.modeling.Part.box(10,10,10),localB=cadkit.modeling.Part.box(10,10,10);
			var b=localB.translated(new cadkit.modeling.Vector(10,edgeContact ? 10 : 0,0));localB.close();
			var joined=cadkit.modeling.Part.fuseAll([a,b]);a.close();b.close();
			geometry=new machinekit.tooling.CastingFillGeometry("contact",joined);joined.close();
			air=geometry.airComponents(0);
			check(air.length==(edgeContact ? 2 : 1),"positive-area faces connect air, edge contact does not: "+edgeContact);
			for (part in air) part.close();
			geometry.close();
		}

	}

	public static function main():Void {
		var body=CastingEnvelope.build(80,60,30);
		var stepped=port(CastingProfiles.polygon([
			{x:-20,y:-15},{x:20,y:-15},{x:20,y:0},{x:0,y:0},{x:0,y:15},{x:-20,y:15}
		]));
		var cavity=CastingCavityCad.part(CastingCavities.derive(body,[stepped])[0]);
		check(Math.abs(cavity.volume()-27000)<1e-6,"stepped prism agrees with two analytic rectangles");
		cavity.close();
		var rounded=port(CastingProfiles.roundedRectangle(-20,-15,40,30,4));
		var finished=CastingCavities.derive(body,[rounded])[0];
		check(!CastingCavities.supportsBoxes(finished),"rounded cavity remains non-box for orthogonal consumers");
		cavity=CastingCavityCad.part(finished);
		check(Math.abs(cavity.volume()-(1200-(4-Math.PI)*16)*30)<1e-5,"arc prism agrees with analytic circular corner area");
		cavity.close();
		var parametric=P.box("parametric-radius",ServiceOpening,[P.face("body.z-min"),P.face("body.z-max")],40,30,30);
		parametric.cornerRadius=4;
		var roundA=CastingCavityCad.part(CastingCavities.derive(body,[parametric])[0]);
		var roundB=CastingCavityCad.part(finished);
		var removed=roundA.subtractAll([roundB]),added=roundB.subtractAll([roundA]);
		check(removed.volume()+added.volume()<1e-6,"source cornerRadius and explicit line/arc profile have zero symmetric difference");
		removed.close();added.close();roundA.close();roundB.close();
		var connected=P.box("box-port",Clearance,[P.face("body.z-min"),P.feature("profile-port")],20,20,20);
		var linked=CastingCavities.derive(body,[connected,rounded]).filter(region -> region.featureId==connected.id)[0];
		cavity=CastingCavityCad.part(linked);
		check(Math.abs(cavity.volume()-8000)<1e-6,"box-to-profile openings use exact source geometry instead of a box projection");
		cavity.close();
		var failed=port(CastingProfiles.rectangle(20,-15,30,30));
		rejects(() -> CastingCavityCad.part(CastingCavities.derive(body,[failed])[0]),
			"Undeclared casting breakout: profile-port body.x-max");
		failed.openings.push(P.face("body.x-max"));
		cavity=CastingCavityCad.part(CastingCavities.derive(body,[failed])[0]);
		check(Math.abs(cavity.volume()-18000)<1e-6,"declared crossing derives exact cut/body intersection without a fixed margin");
		cavity.close();
		failed.openings.push(P.face("body.x-min"));
		rejects(() -> CastingCavityCad.part(CastingCavities.derive(body,[failed])[0]),
			"Unreached declared opening: profile-port body.x-min");
		var panel=machinekit.tooling.PanelCad.contourPrism(CastingProfiles.circle(0,0,5),2,12);
		var source=CastingProfileCad.prism(CastingProfiles.circle(0,0,5),2,12);
		var missing=source.subtractAll([panel]),excess=panel.subtractAll([source]);
		check(missing.volume()+excess.volume()<1e-7,"source and tooling use the same exact native profile realization");
		missing.close();excess.close();source.close();panel.close();
		fillGeometry();
		FillGeometryNativeTests.run();
		Sys.println('FeatureGeometryTests: $assertions assertions passed');
	}
}
