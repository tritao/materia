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
						holeProtectionReference: null,
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
			serviceOrientation: materia.assembly.AssemblyFrames.identity(),
			overlapRule: ReservedThenPriority,
			liftingEvidence: []
		};

	public static function run(d:ToolingDocument):Void {
		d = ToolingCodec.snapshot(d);
		d.recipe.minimumLigament = 2;
		var o = options(d),
			p = CastingMoldPlanner.plan(d, o),
			s = CastingMoldCodec.encode(p);
		FabricationTests.run(p);
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
		check(BoxVolumes.volume(BoxVolumes.subtract(source, reconstructed)) + BoxVolumes.volume(BoxVolumes.subtract(reconstructed, source)) < 1e-5,
			"orthogonal source/reconstruction symmetric difference");
		check(p.changeRequests.filter(c -> c.kind == "lifting-points").length == 1, "unqualified flip requests lifting points");
		check(p.changeRequests.filter(c -> c.id == "allowance-" + d.input.inserts[0].id).length == 1, "replication requires source allowance decision");
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
			cuts: [],
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
		var shuffled = ToolingCodec.snapshot(d);
		shuffled.input.features.reverse();
		shuffled.setup.features.reverse();
		var permutation = CastingMoldPlanner.plan(shuffled, options(shuffled));
		check(ToolingCodec.canonical([for (f in permutation.features) {id: f.featureId, members: f.memberIds}]) == ToolingCodec.canonical([
			for (f in CastingMoldPlanner.plan(d, options(d)).features)
				{id: f.featureId, members: f.memberIds}
		]), "feature permutation preserves exact ownership IDs");
		var unsupported = ToolingCodec.snapshot(d);
		unsupported.setup.features[0].strategy = materia.tooling.casting.CastingSetup.CoreStrategy.Unsupported;
		var unsupportedPlan = CastingMoldPlanner.plan(unsupported, options(unsupported));
		check(unsupportedPlan.features.filter(f -> f.kind == UnsupportedFeature).length == 1 && !unsupportedPlan.fabrication.ready,
			"unsupported strategy records a blocking decision without a fake solid");
		check(unsupportedPlan.validation.filter(v -> v.status == "unsupported").length == 1, "unsupported strategy has a typed validation");
		var rigid = ToolingCodec.snapshot(d);
		rigid.setup.features[0].strategy = materia.tooling.casting.CastingSetup.CoreStrategy.ReusableFormer;
		rigid.setup.features[0].withdrawal = {x: 0, y: 0, z: 1};
		var rigidPlan = CastingMoldPlanner.plan(rigid, options(rigid));
		check(rigidPlan.changeRequests.filter(v -> v.kind == DraftOrSplit).length > 0, "zero draft emits a physical-release request");
		var overlap = ToolingCodec.snapshot(d);
		overlap.input.inserts[0].pose.x = 30;
		overlap.input.inserts[0].pose.z = 40;
		overlap.input.features.filter(f -> f.kind == "insert-recess")[0].pose.x = 30;
		overlap.input.features.filter(f -> f.kind == "insert-recess")[0].pose.z = 40;
		overlap.setup.sourceFingerprint = ToolingCodec.inputFingerprint(overlap.input);
		rejects(() -> CastingMoldPlanner.plan(overlap, options(overlap)), "EPS/insert overlap cannot silently allocate away either body");
		var rotated = ToolingCodec.snapshot(d);
		rotated.setup.sourceToMold = materia.assembly.AssemblyFrames.fromRotationMatrix(0, 0, 100, [0, -1, 0, -1, 0, 0, 0, 0, -1]);
		var rotatedPlan = CastingMoldPlanner.plan(rotated, options(rotated));
		var rotatedNegative = [for (c in rotatedPlan.cores) c.cavity];
		for (f in rotatedPlan.formers)
			for (b in f.volume.pieces)
				rotatedNegative.push(b);
		for (f in rotatedPlan.fixtures)
			for (b in f.datum.pieces)
				rotatedNegative.push(b);
		for (i in rotated.input.inserts)
			rotatedNegative.push(materia.tooling.casting.BoxMoldPlanner.box(i.width, i.depth, i.height,
				materia.assembly.AssemblyFrames.compose(rotated.setup.sourceToMold, i.pose)));
		var rotatedSource = BoxVolumes.subtract([rotatedPlan.envelope], [for (f in rotated.input.features) CastingMoldPlanner.featureBox(rotated, f)]),
			rotatedRebuilt = BoxVolumes.subtract([rotatedPlan.cavity], rotatedNegative);
		check(BoxVolumes.volume(BoxVolumes.subtract(rotatedSource, rotatedRebuilt))
			+ BoxVolumes.volume(BoxVolumes.subtract(rotatedRebuilt, rotatedSource)) < 1e-5,
			"changed setup orientation preserves exact source reconstruction");
		check(CastingMoldCodec.fingerprint(rotatedPlan) != CastingMoldCodec.fingerprint(CastingMoldPlanner.plan(d, options(d))),
			"setup orientation regenerates tooling");
		var service = options(d);
		service.serviceOrientation = d.setup.sourceToMold;
		check(!CastingMoldPlanner.plan(d, service).handling.requiresFlip, "service orientation derives flip requirement");
		var physical = CastingMoldPlanner.plan(d, options(d));
		check(physical.handling.castingMassKg > 0, "casting handling includes density-weighted resin and bored-steel mass");
		for (change in physical.changeRequests)
			for (rule in change.blockingChecks)
				check(physical.validation.filter(v -> v.rule == rule).length == 1, "every change request references a real validation rule");
		var qualified = options(d), handling = physical.handling;
		qualified.liftingEvidence = [
			{
				method: ExternalCradle,
				equipmentReference: "cradle-study",
				capacityKg: handling.castingMassKg * d.recipe.handlingLoadFactor,
				verifiedMassKg: handling.castingMassKg,
				centerOfMass: handling.centerOfMassMoldMm,
				positionToleranceMm: 0.01,
				attachmentIds: ["body"],
				verificationReference: "verified-handling-record"
			}
		];
		check(CastingMoldPlanner.plan(d, qualified).changeRequests.filter(c -> c.kind == LiftingPoints).length == 0,
			"typed capacity/CoM/cradle qualification can resolve handling request");
		qualified.liftingEvidence[0].capacityKg = 1;
		check(CastingMoldPlanner.plan(d, qualified).changeRequests.filter(c -> c.kind == LiftingPoints).length == 1,
			"insufficient typed lifting capacity stays blocking");
		var grout = options(d);
		var joint:materia.tooling.casting.CastingMoldPlan.GroutedJoint = {
			id: "duplicate",
			insertIds: [d.input.inserts[0].id],
			gapMm: null,
			jackingReference: null,
			boltSleeveReference: null,
			cureBeforeTorqueEvidence: []
		};
		grout.groutedJoints = [joint, joint];
		rejects(() -> CastingMoldOptionsCodec.snapshot(grout), "duplicate grout IDs rejected");
		volume.pieces.push({
			x: 10,
			y: 0,
			z: 0,
			width: 10,
			depth: 20,
			height: 30
		});
		var ordered = MoldVolumeCodec.encode(volume);
		volume.pieces.reverse();
		check(MoldVolumeCodec.encode(volume) == ordered, "mold box pieces encode in canonical coordinate order");
		volume.pieces.push({
			x: 5,
			y: 0,
			z: 0,
			width: 10,
			depth: 20,
			height: 30
		});
		rejects(() -> MoldVolumeCodec.encode(volume), "overlapping mold box pieces rejected");
		var assigned = ToolingCodec.snapshot(rigid);
		assigned.input.features.push({
			id: "overlap-detail",
			kind: materia.tooling.casting.CastingToolingInput.CastingFeatureKind.Void,
			insertId: null,
			width: 40,
			depth: 40,
			height: 30,
			pose: materia.assembly.AssemblyFrames.translation(0, 0, 20)
		});
		assigned.setup.features.push({
			featureId: "overlap-detail",
			strategy: materia.tooling.casting.CastingSetup.CoreStrategy.ReusableFormer,
			withdrawal: {x: 0, y: 0, z: 1},
			draftDegrees: 0,
			priority: 10
		});
		assigned.setup.sourceFingerprint = ToolingCodec.inputFingerprint(assigned.input);
		var allocated = CastingMoldPlanner.plan(assigned, options(assigned));
		check(allocated.features.filter(f -> f.featureId == "pocket")[0].memberIds.indexOf("former-overlap-detail-0") >= 0,
			"partial overlaps include every shaping former, not only the newly allocated remainder");
		var machining = options(d);
		machining.precision[0].route = Machined;
		check(CastingMoldPlanner.plan(d, machining).changeRequests.filter(c -> c.id == "allowance-" + d.input.inserts[0].id).length == 1,
			"machining requires an allowance decision");
		machining.precision[0].allowance = 1;
		check(CastingMoldPlanner.plan(d, machining).changeRequests.filter(c -> c.id == "allowance-" + d.input.inserts[0].id).length == 0,
			"explicit finishing stock satisfies the allowance contract");
		var finishing = ToolingCodec.snapshot(d);
		finishing.input.inserts[0].finish = materia.tooling.ToolingEnums.InsertFinish.ReplicatedAfterCasting;
		finishing.setup.sourceFingerprint = ToolingCodec.inputFingerprint(finishing.input);
		check(CastingMoldPlanner.plan(finishing, options(finishing)).changeRequests.filter(c -> StringTools.startsWith(c.id, "finish-")).length == 0,
			"replication route matches a replicated source finish");
		var invalidEvidence:Dynamic = haxe.Json.parse(s);
		Reflect.setField(Reflect.field(invalidEvidence, "options"), "liftingEvidence", ["nonempty string is not qualification"]);
		rejects(() -> CastingMoldCodec.decode(haxe.Json.stringify(invalidEvidence)), "lifting requires a typed qualification record");
		var invalidRule = CastingMoldCodec.decode(s);
		invalidRule.changeRequests[0].blockingChecks = ["missing-rule"];
		rejects(() -> CastingMoldPlanner.validate(invalidRule), "change requests cannot name nonexistent validation rules");
		Sys.println('CastingMoldTests: $assertions assertions passed');
	}
}
