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
		var support=materia.tooling.casting.CastingBoxSupport.check(p.document.input,"process.junction.box-geometry");
		if (support.filter(item -> item.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported).length>0) {
			for (insert in p.document.input.inserts)
				switch insert.anchorage {
					case Bolted(_,_):check(support.filter(item -> item.affectedIds.indexOf(insert.id)>=0
						&& item.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported).length==1,
						"box-only junction check reports the exact anchor owner: "+insert.id);
					default:
				}
			return;
		}

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
		check(p.cores.length == 6 && p.fixtures.length == 16, "actual mill pocket and insert inventory");
		check(p.features.length == p.document.input.features.length && p.features.length == 35, "every full-frame feature classified");
		for (name in ["reconstruction", "coverage", "assembly-graph", "demolding-graph"])
			check(report.validation.filter(v -> v.rule == "tooling.cad." + name && v.status == "pass").length == 1,
				"CAD review owns its " + name + " evidence");
		var missingContext="";
		try machinekit.structural.FoundationIntegrity.castingReview(frame.base,p.document)
		catch (error:Dynamic) missingContext=Std.string(error);
		check(missingContext.indexOf("Foundation casting review source differs from document")>=0,
			"design review cannot silently trust the mold document's assembly context");
		var design = machinekit.structural.FoundationIntegrity.castingReview(frame.base, p.document,
			MillMoldSetup.input(frame).interfaceContext);
		check(design.validation.length > 0
			&& design.metrics.liftingPointCount == p.document.input.inserts.filter(i -> i.role == materia.casting.InsertSpec.InsertRole.LiftingMount).length,
			"source-level design metrics map into foundation integrity");
		check(report.symmetricDifferenceMm3 < 1e-3 && !report.fabrication.ready, "review geometry passes without fabricating readiness");
		var rowGroups=report.assemblyGroups.filter(group -> group.members.filter(id -> p.cores.filter(core -> core.id==id).length==1).length>0);
		check(rowGroups.length == 2 && report.permanentGroups.length == 2, "two shared row fixtures");
		var anchored=p.document.input.inserts.filter(insert -> switch insert.anchorage {case Bolted(_,_):true;default:false;});
		check(report.assemblyGroups.length==rowGroups.length+anchored.length,
			"each anchored insert has its own rigid insertion group alongside the core rows");
		check(report.assembly.assemblyOrder[0] == "bed", "bed installed first");
		check(p.changeRequests.filter(c -> c.kind == "lifting-points").length == 0,
			"source lifting connections clear their design request");
		check(p.validation.filter(check -> check.rule=="tooling.handling.flip" && check.status=="pending").length==1,
			"rigging and flip evidence remain pending independently");
		check(p.options.precision.length==p.document.input.inserts.length && p.options.foundationJointProcesses.length==0,
			"process choices cover source inserts without inventing the undeclared source joint");
		var jointRequests=p.changeRequests.filter(request -> request.kind==materia.tooling.casting.CastingMoldPlan.DesignChangeKind.FoundationJointDefinition);
		check(jointRequests.length==1 && jointRequests[0].sourceIds.length==4
			&& jointRequests[0].blockingChecks.length==1 && p.validation.filter(record -> record.rule==jointRequests[0].blockingChecks[0]
				&& record.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending).length==1,
			"the mill's four column anchors retain a source joint request while production gap and jacking remain undeclared");

		equivalent(frame, p);
		junctionGeometry(frame, p);
		for (c in p.cores) {
			check(c.layers.length == 2, "EPS layers fit 100 mm stock");
			var first = materia.tooling.casting.MoldCores.singleBox(materia.tooling.casting.MoldCores.layerRegion(c.layers[0]),c.id);
			var last = materia.tooling.casting.MoldCores.singleBox(materia.tooling.casting.MoldCores.layerRegion(c.layers[c.layers.length-1]),c.id);
			var cavity = materia.tooling.casting.MoldCores.singleBox(c.cavity,c.id);
			check(Math.abs(last.z + last.height + c.adhesiveThickness - cavity.z - cavity.height) < 1e-7, "glue line included in foam height");
			check(Math.abs(last.z - first.z - first.height - c.adhesiveThickness) < 1e-7, "interlayer glue line included");
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
		check(p.validation.filter(v -> StringTools.startsWith(v.rule, "tooling.fixture.attachment.")).length == 16,
			"fixture attachments are tooling pending records");
		check(p.changeRequests.filter(c -> c.kind == "fixture-attachment").length == 0, "tool-only fixture attachments are not casting changes");
		check(report.handling.castingMassKg > 0 && report.demoldingSteps[report.demoldingSteps.length - 1].kind == "flip-to-service",
			"mass/CoM and flip appear in demolding sequence");
		for (corner in ["LeftFront", "LeftRear", "RightFront", "RightRear"]) {
			var stems = p.document.input.features.filter(feature -> feature.id == "foot-stem-" + corner);
			check(stems.length == 1 && stems[0].geometry.pose.z + materia.casting.CastingFeature.CastingGeometrySolids.height(stems[0].geometry.solid) == frame.layout.casting.footAccessFloor,
				"foot stem ends at its authored access-floor contact");
		}

		var sourceFixings = [for (fixture in p.fixtures) for (fixing in fixture.fixings) fixing];
		check(sourceFixings.length == 8, "column anchors and lifting connections get source-thread fixture screws");
		for (fixture in p.fixtures) for (fixing in fixture.fixings) {
			var insert = p.document.input.inserts.filter(i -> i.id == fixture.insertId)[0];
			var hole = insert.holes.filter(h -> h.id == fixing.sourceHoleId)[0];
			var penetration = fixture.penetrations.filter(pen -> pen.sourceHoleId == hole.id)[0];
			check(hole.thread != null && hole.extent == materia.casting.InsertHole.HoleExtent.Blind && penetration.plug == null
				&& fixing.engagement + materia.casting.InsertHole.InsertHoleCodec.threadLimits(hole).minTipGap <= hole.depth + 1e-7
				&& materia.casting.InsertSpec.InsertBodyCodec.height(insert.body) - hole.depth >= hole.limits.minBack,
				"source fixing preserves the blind C45 back wall and has no buried plug");
		}
		for (f in p.fixtures) {
			var datum = ToolingVolume.geometryOf(f.datum);
			check(datum.valid() && datum.solidCount() == 1, "bored datum support remains one valid connected solid");
			datum.close();
			check(f.lateralLocation.baselineMm >= p.document.recipe.hardware.fixturePinSpacing && f.datum.cuts.length >= 2,
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
		var smaller=materia.tooling.casting.MoldCores.singleBox(restored.cores[0].cavity,restored.cores[0].id);
		smaller.height -= 2;
		restored.cores[0].cavity=materia.tooling.casting.MoldRegions.box(smaller);
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
