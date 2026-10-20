import materia.casting.BoxAlgebra;
import machinekit.milling.MillFrame;
import machinekit.tooling.CastingMoldCad;
import machinekit.tooling.ToolingVolume;
import machinekit.tooling.SweptRemoval;
import machinekit.assembly.PosedParts;
import cadkit.modeling.Vector;
import materia.assembly.AssemblyFrames as F;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingMoldPlanner;
import materia.tooling.casting.CastingMoldCodec;
import materia.tooling.process.OrthogonalSections.SectionAxis;

/** Real mill acceptance cases; source CAD is an independent reference to the planner. */
class MillMoldChecks {
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
		catch (e:Dynamic) {
			failed = true;
			Sys.println("Expected mold rejection: " + Std.string(e));
		}
		check(failed, label);
	}

	static function equivalent(frame:MillFrame, p:CastingMoldPlan):Void {
		var local = frame.base.body.geometry(),
			original = PosedParts.moved(local, p.document.setup.sourceToMold);
		local.close();
		var rebuilt = CastingMoldCad.reconstructed(p),
			a = original.subtractAll([rebuilt]),
			b = rebuilt.subtractAll([original]);
		check(a.volume() + b.volume() < 1e-3, "actual foundation CAD symmetric difference");
		a.close();
		b.close();
		rebuilt.close();
		original.close();
	}

	static function junctionGeometry(frame:MillFrame, p:CastingMoldPlan):Void {
		var boundaries:Array<materia.tooling.process.FillAnalysis.FillBoundary> = [
			for (feature in p.document.input.features)
				{id: feature.id, box: CastingMoldPlanner.featureBox(p.document, feature)}
		];
		var resin = materia.casting.BoxAlgebra.subtract([p.envelope], [for (boundary in boundaries) boundary.box]);
		var report = materia.tooling.process.JunctionThickness.analyze(p.document.input.id, CastingMoldCodec.fingerprint(p), resin, boundaries);
		check(report.converged, "actual mill junction diameter reaches its declared numerical tolerance");
		var owned:Array<cadkit.modeling.Part> = [];
		try {
			var local = frame.base.body.geometry();
			owned.push(local);
			var source = PosedParts.moved(local, p.document.setup.sourceToMold);
			owned.push(source);
			var area = 0.0;
			for (face in report.faces) {
				var b = face.region;
				area += face.axis == SectionAxis.X ? b.depth * b.height : face.axis == SectionAxis.Y ? b.width * b.height : b.width * b.depth;
			}
			check(Math.abs(area - source.shape.area()) < 1e-3, "mill exposed rectangles agree with actual foundation BREP area");
			for (extra in [-1e-5, 0.05]) {
				var ball = new cadkit.modeling.Part(cadkit.Shape.sphere((extra < 0 ? report.lowerDiameterMm : report.upperDiameterMm) / 2 + extra));
				owned.push(ball);
				var placed = ball.translated(new Vector(report.witness.x, report.witness.y, report.witness.z));
				owned.push(placed);
				var outside = placed.subtract(source);
				owned.push(outside);
				check(extra < 0?outside.volume()<1e-3:outside.volume()>1e-3,
					extra < 0 ? "mill witness sphere lies inside actual foundation CAD" : "enlarged mill witness crosses actual foundation CAD");
			}
		} catch (error:Dynamic) {
			for (part in owned)
				if (!part.shape.isClosed())
					part.close();
			throw error;
		}
		for (part in owned)
			if (!part.shape.isClosed())
				part.close();
	}

	public static function run(frame:MillFrame, p:CastingMoldPlan, report:machinekit.tooling.CastingMoldCad.CastingMoldReview):Void {
		check(p.cores.length == 6 && p.fixtures.length == 12, "actual mill pocket and insert inventory");
		check(p.features.length == p.document.input.features.length && p.features.length == 31, "every full-frame feature classified");
		for (name in ["reconstruction", "coverage", "assembly-graph", "demolding-graph"])
			check(report.validation.filter(v -> v.rule == "tooling.cad." + name && v.status == "pass").length == 1,
				"CAD review owns its " + name + " evidence");
		var design = machinekit.structural.FoundationIntegrity.castingReview(frame.base, p.document);
		check(design.validation.length > 0
			&& design.metrics.liftingPointCount == p.document.input.inserts.filter(i -> i.role == materia.casting.InsertSpec.InsertRole.LiftingMount).length,
			"source-level design metrics map into foundation integrity");
		check(report.symmetricDifferenceMm3 < 1e-3 && !report.fabrication.ready, "review geometry passes without fabricating readiness");
		check(report.assemblyGroups.length == 2 && report.permanentGroups.length == 2, "two shared row fixtures");
		check(report.assembly.assemblyOrder[0] == "bed", "bed installed first");
		check(p.changeRequests.filter(c -> c.kind == "lifting-points").length == 1, "handling lift/flip request");
		check(p.options.precision.filter(s -> s.route != CastingMoldPlanner.sourceRoute(p.document.input.inserts.filter(i -> i.id == s.insertId)[0].finish))
			.length == 0 && p.options.groutedJoints.length == 1,
			"finishing routes follow typed source intent");
		equivalent(frame, p);
		junctionGeometry(frame, p);
		for (c in p.cores) {
			check(c.foamLayers.length == 2, "EPS layers fit 100 mm stock");
			var first = c.foamLayers[0],
				last = c.foamLayers[c.foamLayers.length - 1];
			check(Math.abs(last.z + last.height + c.adhesiveThickness - c.cavity.z - c.cavity.height) < 1e-7, "glue line included in foam height");
			check(Math.abs(c.foamLayers[1].z - first.z - first.height - c.adhesiveThickness) < 1e-7, "interlayer glue line included");
		}
		for (f in p.formers) {
			var solid = ToolingVolume.geometryOf(f.volume);
			check(solid.valid() && solid.shape.subshapeCount(CadKit.ShapeKind.Solid) == 1, "each former is one physical connected solid");
			solid.close();
		}
		check(p.formers.filter(f -> f.volume.profiles.length > 0).length == 4, "all access formers own exact router-relief sealing profiles");
		var mismatched = CastingMoldCodec.decode(CastingMoldCodec.encode(p));
		var profiled = mismatched.formers.filter(f -> f.volume.profiles.length > 0)[0];
		profiled.volume.profiles[0].depth += 1;
		rejects(() -> CastingMoldPlanner.validate(mismatched), "former sealing profile must match panel operation");
		check(p.changeRequests.filter(c -> c.kind == "release-alternative")
			.length == materia.tooling.casting.CastingStrategies.resolve(p.document).filter(f -> f.strategy == "reusable-former")
			.length
				+ p.formers.filter(f -> f.featureIds.length == 0)
					.length,
			"every zero-draft rigid feature has a release decision");
		check(p.validation.filter(v -> StringTools.startsWith(v.rule, "tooling.fixture.attachment.")).length == 12,
			"fixture attachments are tooling pending records");
		check(p.changeRequests.filter(c -> c.kind == "fixture-attachment").length == 0, "tool-only fixture attachments are not casting changes");
		check(report.handling.castingMassKg > 0 && report.demoldingSteps[report.demoldingSteps.length - 1].kind == "flip-to-service",
			"mass/CoM and flip appear in demolding sequence");
		for (f in p.fixtures) {
			var datum = ToolingVolume.geometryOf(f.datum);
			check(datum.valid() && datum.solidCount() == 1, "bored datum support remains one valid connected solid");
			datum.close();
			check(f.lateralLocation.baselineMm == p.document.recipe.hardware.fixturePinSpacing && f.datum.cuts.length >= 2,
				"fixture has pin/slot and clearance-only fixing cuts");
			check(f.penetrations.length == 2 + f.fixings.length, "fixture bed cuts have explicit sealing face records");
		}
		for (f in p.fixtures) {
			for (id in [f.lateralLocation.primaryId, f.lateralLocation.secondaryId]) {
				check(report.demolding.removalOrder.indexOf(id) < report.demolding.removalOrder.indexOf(f.datum.id),
					"through-bed pins withdraw before datum supports");
				check(Math.abs(f.withdrawal.x) < 0.9
					|| report.assembly.assemblyOrder.indexOf(f.datum.id) < report.assembly.assemblyOrder.indexOf(id),
					"datum supports seat before pins enter from below");
			}
		}
		var savedAssembly = CastingMoldCad.assembly(p).encode();
		var restoredAssembly = machinekit.assembly.MachineAssembly.decode(savedAssembly);
		restoredAssembly.validate();
		check(restoredAssembly.encode() == savedAssembly, "full hardware assembly round-trips within connector limits");
		var unknown = "";
		try {
			var invalid = materia.tooling.ToolingCodec.inputSnapshot(p.document.input);
			invalid.features[0].kind = cast "future-feature";
			materia.tooling.ToolingCodec.inputSnapshot(invalid);
		} catch (e:Dynamic) {
			unknown = Std.string(e);
		}
		check(unknown.indexOf("Unknown casting feature kind") >= 0, "unknown feature kinds cannot inherit a strategy");
		var permuted = p.document.input.features.copy();
		permuted.reverse();
		var strategies = materia.tooling.casting.CastingStrategies.resolve(p.document);
		check(materia.tooling.ToolingCodec.canonical(CastingMoldPlanner.ownershipOrder(permuted, strategies))
			== materia.tooling.ToolingCodec.canonical(CastingMoldPlanner.ownershipOrder(p.document.input.features, strategies)),
			"raw feature permutation preserves the actual ownership ordering before codec sorting");
		var unsupported = materia.tooling.ToolingCodec.snapshot(p.document);
		unsupported.setup.features[0].strategy = materia.tooling.casting.CastingSetup.CoreStrategy.Unsupported;
		var partial = CastingMoldPlanner.plan(unsupported, p.options),
			assessment = CastingMoldCad.assess(partial);
		check(assessment.geometry == null && assessment.diagnostics.hasErrors() && !partial.fabrication.ready,
			"unsupported feature reports through shared Diagnostics without CAD fabrication claims");
		for (i in p.document.input.inserts) {
			var steel = CastingMoldCad.steel(p, i.id);
			for (protection in p.fixtures.filter(f -> f.insertId == i.id)[0].boreMasks) {
				var hole = i.holes.filter(h -> h.id == protection.sourceHoleId)[0];
				var hardware = p.hardware.filter(h -> h.id == protection.hardwareId)[0];
				var local = machinekit.tooling.ToolingPin.boreMask(hole).geometry();
				var world = PosedParts.moved(local, hardware.pose);
				local.close();
				var overlap = steel.intersect(world);
				check(overlap.volume() < 1e-5, "each bore mask follows its actual source hole and frame");
				overlap.close();
				world.close();
			}
			steel.close();
		}
		var changed = CastingMoldPlanner.plan(MillMoldSetup.document(frame, 17.8), MillMoldSetup.options(frame));
		check(CastingMoldCodec.fingerprint(changed) != CastingMoldCodec.fingerprint(p), "measured stock invalidates package");
		equivalent(frame, changed);
		var bare = new MillFrame(frame.layout),
			simple = CastingMoldPlanner.plan(MillMoldSetup.document(bare), MillMoldSetup.options(bare));
		check(simple.features.length < p.features.length && p.features.filter(f -> f.featureId == "rear-y-drive-seat").length == 1,
			"optional XY interfaces come from full source instance");
		equivalent(bare, simple);
		var encoded = CastingMoldCodec.encode(p),
			restored = CastingMoldCodec.decode(encoded);
		check(CastingMoldCodec.encode(restored) == encoded, "complete plan recipe regeneration");
		restored.openings.pop();
		rejects(() -> CastingMoldPlanner.validate(restored), "unclassified contact opening");
		restored = CastingMoldCodec.decode(encoded);
		restored.cores[0].cavity.height -= 2;
		rejects(() -> CastingMoldCad.review(restored), "missing pocket geometry");
		restored = CastingMoldCodec.decode(encoded);
		restored.fixtures[0].datum.pieces[0].z += 30;
		rejects(() -> CastingMoldCad.review(restored), "displaced insert datum fixture");
		var f = p.formers.filter(f -> f.featureIds.length > 0 && StringTools.startsWith(f.featureIds[0], "foot-access-"))[0];
		var former = ToolingVolume.geometryOf(f.volume),
			castingPart = CastingMoldCad.source(p);
		rejects(() -> SweptRemoval.analyze([
			{id: f.volume.id, part: former, direction: new Vector(-f.withdrawal.x, 0, 0)},
			{
				id: "casting",
				part: castingPart,
				direction: new Vector(0, 0, 1)
			}
		]), "inward foot former trapped against actual casting");
		former.close();
		castingPart.close();
		var review = report;
		var memberIds = [for (m in CastingMoldCad.assembly(p).components()) m.id];
		var playback = machinekit.tooling.MoldPlayback.frames(p, review, memberIds);
		var install = playback.filter(f -> f.phase == machinekit.tooling.MoldPlayback.MoldPlaybackPhase.Assembly);
		var demold = playback.filter(f -> f.phase == machinekit.tooling.MoldPlayback.MoldPlaybackPhase.Demolding);
		check(install.length == review.assembly.assemblyOrder.length + 1, "one playback snapshot per installation plus explicit loose-hardware step");
		for (i in 0...review.assembly.assemblyOrder.length)
			check(install[i].stepId == review.assembly.assemblyOrder[i], "playback preserves the actual checked assembly order");
		check(demold.length == review.demoldingSteps.length, "playback uses the actual demolding steps including release/flip");
		check(install[install.length - 1].visibleMemberIds.length == memberIds.length, "final installation snapshot contains every hardware component");
		check(demold[demold.length - 1].visibleMemberIds.join(",") == "cured-casting", "final scene retains the released casting");
		check(demold[demold.length - 1].status == review.demoldingSteps[review.demoldingSteps.length - 1].status,
			"pending lift qualification stays visible in playback");
		memberIds.reverse();
		check(materia.tooling.ToolingCodec.canonical(playback) == materia.tooling.ToolingCodec.canonical(machinekit.tooling.MoldPlayback.frames(p, review,
			memberIds)),
			"playback deterministic under member permutation");
		var savedFingerprint = review.fingerprint, staleError = "";
		review.fingerprint = "stale";
		try
			machinekit.tooling.MoldPlayback.frames(p, review, memberIds)
		catch (error:Dynamic)
			staleError = Std.string(error);
		review.fingerprint = savedFingerprint;
		check(staleError.indexOf("Stale playback review") >= 0, "playback rejects stale CAD evidence");
		var missingError = "";
		try
			machinekit.tooling.MoldPlayback.frames(p, review, memberIds.filter(id -> id != "bed"))
		catch (error:Dynamic)
			missingError = Std.string(error);
		check(missingError.indexOf("Unknown playback member: bed") >= 0, "playback cannot silently omit a checked member");
		var group = review.assemblyGroups[0], cycleError = "";
		group.members.push(group.id);
		try
			machinekit.tooling.MoldPlayback.frames(p, review, memberIds)
		catch (error:Dynamic)
			cycleError = Std.string(error);
		group.members.pop();
		check(cycleError.indexOf("Cyclic playback group") >= 0, "playback reports cyclic group expansion");
		Sys.println('MillMoldChecks: $assertions assertions passed');
	}
}
