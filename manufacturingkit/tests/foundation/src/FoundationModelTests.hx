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
		var cut = P.box("pocket", Lightening, [P.face("body.z-min")], 40, 30, 25, 0, 0, 0, null, ["width", "depth", "height"]);
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
		var parent = P.box("bay", Lightening, [], 100, 60, 20, 0, 0, 10, null, ["width", "depth"]);
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
		Sys.println("FoundationModelTests: " + assertions + " assertions passed");
	}
}
