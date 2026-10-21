import materia.casting.CastingFeature.CastingDimension;
import materia.casting.CastingFeature;
import materia.casting.CastingPatterns as P;
import materia.casting.CastingEnvelope;
import materia.casting.CastingCavities;
import materia.casting.BoxAlgebra;
import materia.casting.CastingFeatureCodec;
import materia.casting.CastingSolid.CastingSolidCodec;
import materia.casting.CastingSolid;
import materia.casting.InsertSpec.InsertBody;
import materia.casting.InsertSpec.InsertBodyCodec;
import materia.casting.InsertOrigin;
import materia.casting.InsertOrigin.InsertOriginCodec;

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
		FoundationRequirementTests.main();
		PourPlanningTests.run();
		InterfaceStockTests.run();
		InsertManufacturingTests.run();
		insertBodies();
		threadRelief();
		solidFeatures();
		insertOrigin();
		var body = CastingEnvelope.build(100, 80, 100);
		var cut = P.box("pocket", Lightening, [P.face("body.z-min")], 40, 30, 25, 0, 0, 0, null, [CastingDimension.Width, CastingDimension.Depth, CastingDimension.Height]);
		var encoded = CastingFeatureCodec.encode([cut]);
		check(CastingFeatureCodec.encode(CastingFeatureCodec.decode(encoded)) == encoded, "typed feature codec roundtrip");
		var derived = CastingCavities.derive(body, [cut])[0];
		check(BoxAlgebra.volume(CastingCavities.boxes(derived)) == 30000, "exact box cavity");
		cut.geometry.solid = materia.casting.CastingFeature.CastingGeometrySolid.Cut(materia.casting.CastingSolid.Box(40, 30, 26));
		cut.geometry.pose.z = -1;
		check(volume(CastingCavities.prisms(CastingCavities.derive(body, [cut])[0])[0]) == 30000, "cut margin does not change finished cavity");
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
		var access = P.box("access", ServiceOpening, [P.face("foot.side-2")], 60, 30, 30, 65, 0, 25);
		check(access.openings.filter(o -> o.target == "foot.side-2").length == 1, "sloped foot opening is named exactly");
		var wedge = CastingCavities.derive(flare, [access])[0];
		check(materia.casting.CastingCavities.prisms(wedge).length == 1, "foot intersection is one exact prism");
		var expected = (35.0 + (80 - (55 - 20) * 30 / 40) - 35) / 2 * 30 * 30;
		// At z=25 the sloped boundary is x=76.25; at z=55 it is x=53.75.
		expected = ((76.25 - 35) + (53.75 - 35)) / 2 * 30 * 30;
		check(Math.abs(volume(materia.casting.CastingCavities.prisms(wedge)[0]) - expected) < 1e-8, "sloped-foot cut has analytic wedge volume");
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
		wider.geometry.solid = materia.casting.CastingFeature.CastingGeometrySolid.Cut(materia.casting.CastingSolid.Box(102, 60, 20));
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
		var draftIntent = CastingCavities.derive(body, [drafted])[0];
		check(!CastingCavities.supportsBoxes(draftIntent), "draft intent decodes without approximating its finished geometry");
		rejects(() -> CastingCavities.boxes(draftIntent), "Unsupported non-box finished cavity: drafted-intent");
		var legacy:Dynamic = haxe.Json.parse(CastingFeatureCodec.encode([parent]));
		legacy.version = 1;
		rejects(() -> CastingFeatureCodec.decode(haxe.Json.stringify(legacy)), "Unsupported casting feature schema");
		var flared = CastingEnvelope.build(100, 80, 120, 0, 160, 60, 20);
		var crossing = P.box("sloped-foot-crossing", ServiceOpening, [P.face("foot.side-2")], 60, 30, 30, 65, 0, 25);
		rejects(() -> BoxAlgebra.material(flared, [crossing]), "Unsupported non-box finished cavity: sloped-foot-crossing");
		var upperPocket = P.box("upper-pocket", Lightening, [], 20, 20, 20, 0, 0, 80);
		rejects(() -> BoxAlgebra.material(flared, [upperPocket]), "Unsupported non-box casting material residue after features: upper-pocket");
		var holes = TestInsertHoles.through(6, 20);
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
		holes[0].threadDepth = holes[0].depth;
		holes[0].clearanceDiameter = 7.5;
		holes[0].limits.threaded = {minEngagement: 7, minTipGap: 1};
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

	static function insertBodies():Void {
		var box = CastingSolidCodec.snapshot({kind: "box", width: 20, depth: 30, height: 40});
		var cylinder = CastingSolidCodec.snapshot({kind: "cylinder", diameter: 8, height: 40});
		check(switch box {case Box(w, d, h): w == 20 && d == 30 && h == 40; default: false;}, "primitive box dimensions");
		check(switch cylinder {case Cylinder(d, h): d == 8 && h == 40; default: false;}, "primitive cylinder dimensions");
		var cut = materia.casting.CastingPatterns.box("shape-domain", materia.casting.CastingFeature.CastingFeatureKind.Lightening, [], 8, 8, 40, 0, 0, 0);
		cut.geometry.solid = materia.casting.CastingFeature.CastingGeometrySolid.Insert(Primitive(cylinder));
		rejects(() -> CastingFeatureCodec.snapshot([cut]), "Design cut cannot use insert retention geometry");
		cut.kind = materia.casting.CastingFeature.CastingFeatureKind.InsertRecess;
		cut.owner = "source-insert";
		cut.geometry.solid = materia.casting.CastingFeature.CastingGeometrySolid.Cut(cylinder);
		rejects(() -> CastingFeatureCodec.snapshot([cut]), "Insert recess requires source insert geometry");
		var formed = CrimpedSocket(Cylinder(8, 25), {length: 15, width: 11, thickness: 2.2, wave: 1.4, blend: 0.5});
		for (body in [Primitive(box), Primitive(cylinder), formed]) {
			var encoded = InsertBodyCodec.encode(body);
			check(InsertBodyCodec.encode(InsertBodyCodec.decode(encoded)) == encoded, "strict insert exterior round trip");
			check(InsertBodyCodec.encode(InsertBodyCodec.snapshot(body)) == encoded, "typed insert exterior snapshot");
			var spec:materia.casting.InsertSpec = {body: body, holes: [], origin: Fabricated(materia.casting.InsertManufacturing.InsertManufacturingCodec.draft(
				materia.casting.InsertMaterial.InsertMaterials.specification(materia.casting.InsertMaterial.C45Bright))), interfaces: []};
			var copied = materia.casting.InsertSpec.InsertSpecCodec.snapshot(materia.casting.InsertSpec.InsertSpecCodec.record(spec));
			check(copied.holes.length == 0 && InsertBodyCodec.encode(copied.body) == encoded, "one insert contract without invented holes");
		}
		var copied = InsertBodyCodec.snapshot(formed);
		switch formed {case CrimpedSocket(_, tail): tail.width = 12; default:}
		check(switch copied {case CrimpedSocket(_, tail): tail.width == 11; default: false;}, "snapshot detaches retention dimensions");
		rejects(() -> CastingSolidCodec.snapshot({kind: "sphere", diameter: 8}), "Unsupported casting solid kind");
		rejects(() -> CastingSolidCodec.snapshot({kind: "cylinder", diameter: 0, height: 40}), "finite and positive");
		rejects(() -> CastingSolidCodec.record(Cylinder(Math.NaN, 40)), "finite and positive");
		rejects(() -> CastingSolidCodec.record(Box(20, -30, 40)), "finite and positive");
		rejects(() -> CastingSolidCodec.snapshot({kind: "cylinder", diameter: 8, height: 40, tail: null}), "Unexpected casting record fields");
		rejects(() -> InsertBodyCodec.record(CrimpedSocket(Box(8, 8, 25), {length:15, width:11, thickness:2.2, wave:1.4, blend:0.5})), "mouth must be cylindrical");
		rejects(() -> InsertBodyCodec.record(CrimpedSocket(Cylinder(8, 25), {length:15, width:7, thickness:2.2, wave:1.4, blend:0.5})), "Inconsistent insert crimped tail");
		rejects(() -> CastingSolidCodec.snapshot({kind: "box", width: 20, depth: 30}), "Unexpected casting record fields");
		rejects(() -> CastingSolidCodec.snapshot({kind: "cylinder", diameter: "8", height: 40}), "Invalid casting number");
		rejects(() -> materia.casting.InsertSpec.InsertSpecCodec.snapshot({body: InsertBodyCodec.record(Primitive(box)), holes: null, origin: {kind:"fabricated"}, interfaces: new Array<materia.casting.InterfaceRequirement>()}), "Invalid insert hole list");
	}

	static function threadRelief():Void {
		var hole = TestInsertHoles.through(5, 25)[0];
		hole.extent = Blind;
		hole.thread = {designation: "M6", majorDiameter: 6, pitch: 1, starts: 1, hand: Right,
			internal: true, toleranceClass: "6H", profile: Metric};
		hole.tapDrill = 5;
		hole.threadDepth = 6;
		hole.clearanceDiameter = 6.6;
		hole.relief = {diameter: 6.2, start: 6};
		hole.limits.threaded = {minEngagement: 6, minTipGap: 1};
		var copy = materia.casting.InsertHole.InsertHoleCodec.snapshot([hole])[0];
		check(copy.threadDepth == 6 && copy.depth == 25 && copy.relief != null && copy.relief.start == 6 && copy.relief.diameter == 6.2,
			"short thread and deeper clearance bore remain distinct");
		hole.relief.start = 5;
		rejects(() -> materia.casting.InsertHole.InsertHoleCodec.snapshot([hole]), "Invalid insert thread relief");
		hole.relief.start = 6;
		hole.relief.diameter = 6;
		rejects(() -> materia.casting.InsertHole.InsertHoleCodec.snapshot([hole]), "Invalid insert thread relief");
		hole.relief.diameter = 6.2;
		hole.extent = Through;
		rejects(() -> materia.casting.InsertHole.InsertHoleCodec.snapshot([hole]), "Invalid insert thread relief");
	}

	static function solidFeatures():Void {
		var feature = P.box("round-port", ServiceOpening, [P.face("body.z-max")], 1, 1, 1, 0, 0, 80);
		feature.geometry.solid = materia.casting.CastingFeature.CastingGeometrySolid.Cut(materia.casting.CastingSolid.Cylinder(8, 40));
		var encoded = CastingFeatureCodec.encode([feature]);
		check(CastingFeatureCodec.encode(CastingFeatureCodec.decode(encoded)) == encoded, "solid feature schema roundtrip");
		var cavity = CastingCavities.derive(CastingEnvelope.build(100, 80, 100), [feature])[0];
		check(switch cavity.geometry {
			case SolidIntersection(body, cut, _): body.length == 1 && body[0].height == 100
				&& materia.casting.CastingFeature.CastingGeometrySolids.height(cut.geometry.solid) == 40 && cut.geometry.pose.z == 80;
			default: false;
		}, "non-box derivation retains the exact cut/body expression without clipping the design cut");
		rejects(() -> CastingCavities.boxes(cavity), "Unsupported non-box finished cavity: round-port");
		var previous:Dynamic = haxe.Json.parse(encoded);
		previous.version = 2;
		rejects(() -> CastingFeatureCodec.decode(haxe.Json.stringify(previous)), "Unsupported casting feature schema");
	}

	static function insertOrigin():Void {
		var body = Primitive(Box(20, 30, 40));
		var holes:Array<materia.casting.InsertHole> = [];
		var product:materia.casting.InsertOrigin.CatalogueInsert = {manufacturer:"test supplier", reference:"fixture-product",
			revision:"drawing-1", holePolicyId:"fixture-policy", document:"fixture-drawing", geometryBasis:NominalStudy,
			geometryFingerprint:null, evidence:[]};
		var origin = Catalogue(product);
		var nominal = InsertOriginCodec.snapshot(origin, body, holes);
		check(switch nominal {case Catalogue(p): p.geometryBasis == NominalStudy; default:false;}, "catalogue origin is explicitly nominal");
		check(switch InsertOriginCodec.snapshot(Fabricated(materia.casting.InsertManufacturing.InsertManufacturingCodec.draft(
			materia.casting.InsertMaterial.InsertMaterials.specification(materia.casting.InsertMaterial.C45Bright))), body, holes) {case Fabricated(_):true; default:false;}, "fabricated origin is explicit");
		rejects(() -> InsertOriginCodec.snapshot(null, body, holes), "Missing insert origin");
		product.evidence = [{reference:"fixture-measurements", sha256:haxe.crypto.Sha256.encode("synthetic test evidence")}];
		rejects(() -> InsertOriginCodec.snapshot(origin, body, holes), "Nominal insert geometry cannot claim verified geometry evidence");
		product.geometryBasis = Measured;
		product.geometryFingerprint = InsertOriginCodec.geometryFingerprint(product, body, holes);
		var measured = InsertOriginCodec.snapshot(origin, body, holes);
		check(switch measured {case Catalogue(p):p.evidence.length == 1; default:false;}, "evidence binds the complete insert");
		product.revision = "drawing-2";
		rejects(() -> InsertOriginCodec.snapshot(origin,body,holes),"bound to different geometry");
		product.revision = "drawing-1";
		rejects(() -> InsertOriginCodec.snapshot(origin, Primitive(Box(22,30,40)), holes), "bound to different geometry");
		holes = TestInsertHoles.through(5,40);
		rejects(() -> InsertOriginCodec.snapshot(origin, body, holes), "bound to different geometry");
	}
}
