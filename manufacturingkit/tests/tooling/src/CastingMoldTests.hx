import materia.tooling.ToolingDocument;
import materia.tooling.ToolingCodec;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingMoldPlanner;
import materia.tooling.casting.CastingMoldCodec;
import materia.tooling.casting.CastingMoldOptionsCodec;
import materia.tooling.casting.BoxVolumes;
import materia.tooling.casting.MoldVolumeCodec;

class CastingMoldTests {
	static var assertions = 0;

	static function check(ok:Bool, label:String):Void {
		assertions++;
		if (!ok)
			throw label;
	}

	static function rejects(fn:Void->Void, label:String):Void {
		var failed = false;
		try
			fn()
		catch (e:Dynamic)
			failed = true;
		check(failed, label);
	}

	static function options(d:ToolingDocument):CastingMoldOptions
		return {
			groutedJoints: [],
			precision: [
				for (i in d.input.inserts)
					{
						insertId: i.id,
						route: Replicated,
						allowance: null,
						masterReference: null,
						masterFlatnessMm: null,
						releaseSpec: null,
						damGeometry: [],
						holeProtectionReference: null,
						pourGroup: null,
						evidence: []
					}
			],
			compaction: {
				method: Undeclared,
				bedInterfaceReference: null,
				frequencyHz: null,
				amplitudeMm: null,
				durationSeconds: null,
				evidence: []
			},
			anchorage: [
				for (i in d.input.inserts)
					{
						insertId: i.id,
						kind: Pending,
						geometry: [],
						pulloutN: null,
						shearN: null,
						evidence: []
					}
			],
			requiresFlip: true,
			liftingEvidence: []
		};

	public static function run(d:ToolingDocument):Void {
		d = ToolingCodec.snapshot(d);
		d.recipe.minimumLigament = 2;
		var o = options(d),
			p = CastingMoldPlanner.plan(d, o),
			s = CastingMoldCodec.encode(p);
		check(CastingMoldCodec.encode(CastingMoldCodec.decode(s)) == s, "multi-core canonical regeneration");
		check(p.features.length == d.input.features.length && p.cores.length == 1, "complete source ownership");
		var cuts = [for (f in d.input.features) CastingMoldPlanner.featureBox(d, f)],
			negative = [p.cores[0].cavity];
		for (f in p.formers)
			for (b in f.volume.pieces)
				negative.push(b);
		for (f in p.fixtures)
			for (b in f.datum.pieces)
				negative.push(b);
		for (i in d.input.inserts)
			negative.push(materia.tooling.casting.BoxMoldPlanner.box(i.width, i.depth, i.height,
				materia.assembly.AssemblyFrames.compose(d.setup.sourceToMold, i.pose)));
		var source = BoxVolumes.subtract([p.envelope], cuts),
			reconstructed = BoxVolumes.subtract([p.cavity], negative);
		check(Math.abs(BoxVolumes.volume(source) - BoxVolumes.volume(reconstructed)) < 1e-5, "orthogonal reconstruction exact volume");
		check(p.changeRequests.filter(c -> c.kind == "lifting-points").length == 1, "unqualified flip requests lifting points");
		check(p.changeRequests.filter(c -> c.kind == "interface-allowance").length == 1, "replication requires source allowance decision");
		p.features.pop();
		rejects(() -> CastingMoldPlanner.validate(p), "missing feature rejected");
		p = CastingMoldPlanner.plan(d, o);
		p.openings.pop();
		rejects(() -> CastingMoldPlanner.validate(p), "unclassified opening rejected");
		o.anchorage.pop();
		rejects(() -> CastingMoldPlanner.plan(d, o), "missing anchorage not silently qualified");
		var raw:Dynamic = haxe.Json.parse(s);
		Reflect.setField(raw, "version", 99);
		rejects(() -> CastingMoldCodec.decode(haxe.Json.stringify(raw)), "unknown mold format rejected");
		raw = haxe.Json.parse(s);
		Reflect.setField(Reflect.field(raw, "options"), "requiresFlip", 1);
		rejects(() -> CastingMoldCodec.decode(haxe.Json.stringify(raw)), "typed handling flag");
		var volume:materia.tooling.casting.CastingMoldPlan.MoldVolume = {
			id: "test-former",
			material: FormerStock,
			profiles: [],
			pieces: [
				{
					x: 0.0,
					y: 0.0,
					z: 0.0,
					width: 10.0,
					depth: 20.0,
					height: 30.0
				}
			]
		};
		var encoded = MoldVolumeCodec.encode(volume);
		check(MoldVolumeCodec.encode(MoldVolumeCodec.decode(encoded)) == encoded, "volume recipe canonical");
		raw = haxe.Json.parse(encoded);
		Reflect.setField(Reflect.field(raw, "volume"), "material", "mystery");
		rejects(() -> MoldVolumeCodec.decode(haxe.Json.stringify(raw)), "unknown material rejected");
		var a = {
			x: 0.0,
			y: 0.0,
			z: 0.0,
			width: 10.0,
			depth: 10.0,
			height: 10.0
		}, b = {
			x: 3.0,
			y: 3.0,
			z: 0.0,
			width: 4.0,
			depth: 4.0,
			height: 10.0
		};
		check(Math.abs(BoxVolumes.volume(BoxVolumes.subtract([a], [b])) - 840) < 1e-8, "through-hole box subtraction");
		check(BoxVolumes.groups(BoxVolumes.subtract([a], [
			{
				x: 4,
				y: -1,
				z: -1,
				width: 2,
				depth: 12,
				height: 12
			}
		])).length == 2, "disconnected formers split into physical members");
		Sys.println('CastingMoldTests: $assertions assertions passed');
	}
}
