import materia.casting.CastingFeature.CastingDimension;
import materia.casting.CastingFeature;
import materia.casting.CastingPatterns as P;
import materia.casting.CastingEnvelope;
import materia.casting.CastingCavities;
import materia.casting.BoxAlgebra;
import materia.casting.CastingFeatureCodec;

class FoundationModelTests {
	static var assertions = 0;

	static function check(value:Bool, message:String):Void {
		assertions++;
		if (!value)
			throw message;
	}

	static function rejects(fn:Void->Void, text:String):Void {
		var message = "";
		try
			fn()
		catch (e:Dynamic)
			message = Std.string(e);
		check(message.indexOf(text) >= 0, "Expected " + text + ", got " + message);
	}

	static function volume(p:materia.casting.CastingPrism):Float
		return materia.casting.CastingPolygons.area(p.profile) * p.height;

	public static function main():Void {
		var body = CastingEnvelope.build(100, 80, 100);
		var cut = P.box("pocket", Lightening, [P.face("body.z-min")], 40, 30, 25, 0, 0, 0, null, [CastingDimension.Width, CastingDimension.Depth, CastingDimension.Height]);
		var encoded = CastingFeatureCodec.encode([cut]);
		check(CastingFeatureCodec.encode(CastingFeatureCodec.decode(encoded)) == encoded, "typed feature codec roundtrip");
		var derived = CastingCavities.derive(body, [cut])[0];
		check(BoxAlgebra.volume(CastingCavities.boxes(derived)) == 30000, "exact box cavity");
		cut.geometry.height = 26;
		cut.geometry.pose.z = -1;
		check(volume(CastingCavities.derive(body, [cut])[0].pieces[0]) == 30000, "cut margin does not change finished cavity");
		cut.openings = [];
		rejects(() -> CastingCavities.derive(body, [cut]), "Undeclared casting breakout: pocket");
		cut.openings = [P.face("body.x-min")];
		rejects(() -> CastingCavities.derive(body, [cut]), "Unreached declared opening: pocket");
		var flare = CastingEnvelope.build(100, 80, 120, 0, 160, 60, 20);
		var footFront = P.box("foot-front", ServiceOpening, [P.face("body.y-min")], 20, 20, 10, 0, -40, 25);
		rejects(() -> CastingCavities.derive(flare, [footFront]), "Unreached declared opening: foot-front body.y-min");
		footFront.openings = [P.face("foot.y-min")];
		check(CastingCavities.derive(flare, [footFront]).length == 1,
			"coplanar foot and body faces retain distinct opening identities");
		var access = P.box("access", ServiceOpening, [], 60, 30, 30, 65, 0, 25);
		access.openings = CastingCavities.reachedOpenings(flare, access.geometry);
		check(access.openings.filter(o -> o.target == "foot.side-2").length == 1, "sloped foot opening is named exactly");
		var wedge = CastingCavities.derive(flare, [access])[0];
		check(wedge.pieces.length == 1, "foot intersection is one exact prism");
		var expected = (35.0 + (80 - (55 - 20) * 30 / 40) - 35) / 2 * 30 * 30;
		// At z=25 the sloped boundary is x=76.25; at z=55 it is x=53.75.
		expected = ((76.25 - 35) + (53.75 - 35)) / 2 * 30 * 30;
		check(Math.abs(volume(wedge.pieces[0]) - expected) < 1e-8, "sloped-foot cut has analytic wedge volume");
		rejects(() -> CastingCavities.boxes(wedge), "Unsupported non-box finished cavity: access");
		var cells = [
			{
				x: 0.0,
				y: 0.0,
				z: 0.0,
				width: 100.0,
				depth: 60.0,
				height: 20.0
			}
		];
		check(Math.abs(BoxAlgebra.minimumIy(cells, 0, 100) - 60 * 20 * 20 * 20 / 12) < 1e-8, "solid beam section inertia");
		var removed = BoxAlgebra.remove(cells, {
			x: 40,
			y: 0,
			z: 0,
			width: 20,
			depth: 30,
			height: 20
		});
		check(BoxAlgebra.volume(removed) == 108000, "exact retained material volume");
		check(Math.abs(BoxAlgebra.minimumIy(removed, 0, 100) - 30 * 20 * 20 * 20 / 12) < 1e-8, "minimum section through cutout");
		check(BoxAlgebra.volume(BoxAlgebra.compact(removed)) == BoxAlgebra.volume(removed), "compaction preserves exact material");
        var whole = {x:0.0,y:0.0,z:0.0,width:20.0,depth:30.0,height:40.0};
        var almostWhole = {x:0.0,y:0.0,z:0.0,width:20.0-5e-14,depth:30.0,height:40.0};
        check(BoxAlgebra.difference(whole,almostWhole).length==0,
            "coincident-plane roundoff does not become a degenerate collision sliver");
        var clearance = BoxAlgebra.difference(whole,{x:0.0,y:0.0,z:0.0,width:20.0-1e-5,depth:30.0,height:40.0});
        check(clearance.length==1 && Math.abs(clearance[0].width-1e-5)<1e-12,
            "small intentional material thickness survives plane tolerance");
        var nearPlanes = BoxAlgebra.compact([
            {x:0.0,y:0.0,z:0.0,width:20.0,depth:10.0,height:10.0},
            {x:20.0-5e-14,y:10.0,z:0.0,width:20.0,depth:10.0,height:10.0},
            {x:0.0,y:20.0,z:0.0,width:40.0,depth:10.0,height:10.0}
        ]);
        for(piece in nearPlanes)check(piece.width>1e-9 && piece.depth>1e-9 && piece.height>1e-9,
            "compaction does not reintroduce coincident-plane slivers");
        check(Math.abs(BoxAlgebra.volume(nearPlanes)-8000)<1e-8,
            "near-coincident partition planes preserve union volume");
		var parent = P.box("bay", Lightening, [], 100, 60, 20, 0, 0, 10, null, [CastingDimension.Width, CastingDimension.Depth]);
		var fragments = BoxAlgebra.retain([parent], [
			{
				x: -10,
				y: -30,
				z: 10,
				width: 20,
				depth: 60,
				height: 20
			}
		]);
		check(fragments.length == 2 && fragments[0].derivedFrom == "bay" && fragments[1].derivedFrom == "bay", "retained support preserves parent identity");
		var repeated = BoxAlgebra.retain([parent], [
			{
				x: -10,
				y: -30,
				z: 10,
				width: 20,
				depth: 60,
				height: 20
			}
		]);
		check(fragments[0].id == repeated[0].id && fragments[1].id == repeated[1].id, "fragment IDs depend on geometry");
		var supports:Array<materia.casting.Box3> = [{x: -10, y: -30, z: 10, width: 20, depth: 60, height: 20}];
		var unrelated = P.box("unrelated", Lightening, [], 10, 10, 10, 200, 0, 10);
		var withUnrelated = BoxAlgebra.retain([unrelated, parent], supports).filter(feature -> feature.derivedFrom == parent.id);
		check(withUnrelated.length == 2 && withUnrelated[0].id == fragments[0].id && withUnrelated[1].id == fragments[1].id,
			"unrelated feature edits and order preserve fragment IDs");
		var wider = CastingFeatureCodec.snapshot([parent])[0];
		wider.geometry.width += 2;
		var changed = BoxAlgebra.retain([wider], supports);
		check(changed.length == 2 && changed[0].id != fragments[0].id && changed[1].id != fragments[1].id,
			"geometry edits change fragment IDs");
		var parentAccess = P.box("access", ServiceOpening, [P.feature("bay")], 10, 10, 10, -40, 0, 10);
		var linked = fragments.concat([parentAccess]);
		check(CastingCavities.derive(CastingEnvelope.build(200, 100, 60), linked).length == 3,
			"declared feature openings resolve retained children through their authored parent");
		parentAccess.geometry.pose.x = 0;
		rejects(() -> CastingCavities.derive(CastingEnvelope.build(200, 100, 60), fragments.concat([parentAccess])), "Unreached feature opening: access bay");
		var touchParent = P.box("touch-parent", Lightening, [], 10, 10, 10, 0, 0, 10);
		var touchAccess = P.box("touch-access", ServiceOpening, [P.feature(touchParent.id)], 10, 10, 10, 10, 10, 10);
		rejects(() -> CastingCavities.derive(body, [touchParent, touchAccess]), "Unreached feature opening: touch-access touch-parent");
		touchAccess.geometry.pose.y = 0;
		check(CastingCavities.derive(body, [touchParent, touchAccess]).length == 2,
			"feature openings require positive area instead of edge-only contact");
		var tilted = P.box("almost-axis", Lightening, [], 10, 10, 10, 0, 0, 10);
		var angle = 1e-5, cosine = Math.cos(angle), sine = Math.sin(angle);
		tilted.geometry.pose = materia.assembly.AssemblyFrames.fromRotationMatrix(0, 0, 10,
			[cosine, -sine, 0, sine, cosine, 0, 0, 0, 1]);
		rejects(() -> CastingCavities.derive(body, [tilted]), "Unsupported casting feature: almost-axis");
		var drafted = P.box("drafted-intent", Lightening, [], 10, 10, 10, 0, 0, 10);
		drafted.draft.degrees = 2;
		rejects(() -> CastingCavities.derive(body, [drafted]), "Unsupported drafted or rounded casting feature: drafted-intent");
		var legacy:Dynamic = haxe.Json.parse(CastingFeatureCodec.encode([parent]));
		legacy.version = 1;
		rejects(() -> CastingFeatureCodec.decode(haxe.Json.stringify(legacy)), "Unsupported casting feature schema");
		var flared = CastingEnvelope.build(100, 80, 120, 0, 160, 60, 20);
		var crossing = P.box("sloped-foot-crossing", ServiceOpening, [P.face("foot.side-2")], 60, 30, 30, 65, 0, 25);
		rejects(() -> BoxAlgebra.material(flared, [crossing]), "Unsupported non-box finished cavity: sloped-foot-crossing");
		var holes = materia.casting.InsertHoleGeometry.through(6, 20);
		holes[0].thread = {
			designation: "M7",
			majorDiameter: 7,
			pitch: 1,
			starts: 1,
			hand: Right,
			internal: true,
			toleranceClass: "6H",
			profile: Metric
		};
		holes[0].tapDrill = 6;
		var copied = materia.casting.InsertHole.InsertHoleCodec.snapshot(holes);
		check(copied[0].thread != null && copied[0].thread.designation == "M7", "thread data has portable typed tokens");
		copied[0].diameter = 5;
		check(holes[0].diameter == 6, "hole snapshots are detached");
		holes[0].limits.minWall = 0;
		rejects(() -> materia.casting.InsertHole.InsertHoleCodec.snapshot(holes), "Invalid insert hole limit: minWall");
		var near = 20.000000000000057;
		var sections:Array<materia.casting.Box3> = [
			{x: 0, y: 0, z: 0, width: 40, depth: 10, height: 10},
			{x: 0, y: 0, z: 10, width: 20, depth: 10, height: 10},
			{x: near, y: 0, z: 10, width: 40 - near, depth: 10, height: 10}
		];
		check(Math.abs(BoxAlgebra.minimumIy(sections, 0, 40) - 10 * 20 * 20 * 20 / 12) < 1e-6,
			"coincident clipping planes do not invent a weak section one ULP wide");
		sections[2].x = 20.000001;
		sections[2].width = 40 - sections[2].x;
		check(Math.abs(BoxAlgebra.minimumIy(sections, 0, 40) - 10 * 10 * 10 * 10 / 12) < 1e-6,
			"a resolved narrow gap still controls the minimum section stiffness");
		Sys.println("FoundationModelTests: " + assertions + " assertions passed");
	}
}
