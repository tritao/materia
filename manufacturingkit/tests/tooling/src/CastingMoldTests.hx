import materia.casting.InsertSpec.InsertBodyCodec as InsertBodies;
import materia.assembly.AssemblyFrames as F;
import materia.casting.CastingFeature.CastingDimension;
import materia.tooling.ToolingDocument;
import materia.tooling.ToolingCodec;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingMoldPlanner;
import materia.tooling.casting.CastingMoldCodec;
import materia.tooling.casting.CastingMoldOptionsCodec;
import materia.casting.BoxAlgebra;
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
			replicationJigs: [],
			foundationJointProcesses: [],
			precision: [
				for (i in d.input.inserts)
					{
						insertId: i.id,
						jigReference: null,
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

			serviceOrientation: materia.assembly.AssemblyFrames.identity(),
			overlapRule: ReservedThenPriority,
			liftingEvidence: []
		};

	static function unrealizedGeometry():Void {
		for (draft in [true, false]) {
			var d = ToolingTests.fixture();
			var feature = d.input.features.filter(f -> f.kind != materia.casting.CastingFeature.CastingFeatureKind.InsertRecess)[0];
			if (draft) feature.draft.degrees = 3; else feature.cornerRadius = 4;
			d.setup.sourceFingerprint = ToolingCodec.inputFingerprint(d.input);
			var encoded = ToolingCodec.encode(d);
			check(ToolingCodec.encode(ToolingCodec.decode(encoded)) == encoded, "non-box intent survives its strict source codec");
			var assessment = CastingMoldPlanner.assess(d, options(d));
			check(assessment.plan == null && !assessment.fabrication.ready && assessment.changeRequests.length == 1,
				"draft/radius capability gap produces an explicit unrealized assessment and source request");
			check(assessment.changeRequests[0].sourceIds.indexOf(feature.id) >= 0
				&& assessment.validation.filter(v -> v.status == "unsupported" && v.affectedIds.indexOf(feature.id) >= 0).length == 1,
				"unrealized request and Unsupported name the actual source feature");
			var levelOne = materia.tooling.process.CastingLevelOne.analyze(d);
			check(levelOne.validation.filter(v -> v.status == "unsupported" && v.affectedIds.indexOf(feature.id) >= 0).length == 1,
				"plan-free castability accepts the intent and reports its geometry capability gap");
		}
		var family = ToolingTests.fixture();
		var child = family.input.features.filter(f -> f.kind != materia.casting.CastingFeature.CastingFeatureKind.InsertRecess)[0];
		child.derivedFrom = "authored-pocket";
		child.cornerRadius = 3;
		family.setup.sourceFingerprint = ToolingCodec.inputFingerprint(family.input);
		var review = materia.tooling.process.CastingLevelOne.analyze(family);
		check(review.metrics.minimumCornerRadiusMm == 3
			&& review.validation.filter(v -> v.status == "unsupported" && v.affectedIds.indexOf("authored-pocket") >= 0).length == 1,
			"unrealized fragment castability reports its authored parent and preserves known radius metadata");
	}

	static function mountingRouteRequest():Void {
		var d=ToolingTests.fixture(),insert=d.input.inserts[0],hole=insert.holes[0];
		d.recipe.minimumLigament=2;
		hole.thread={designation:"M10",majorDiameter:10,pitch:1.5,starts:1,hand:Right,
			internal:true,toleranceClass:"6H",profile:Metric};
		hole.diameter=8.5;hole.tapDrill=8.5;hole.threadDepth=hole.depth;hole.clearanceDiameter=11;
		hole.limits.threaded={minEngagement:13,minTipGap:1.5};
		var manufacturing=switch insert.origin {case Fabricated(plan):plan;default:throw "Fabricated fixture required";};
		manufacturing.holeRoutes=[];
		manufacturing.mountingRequirements=[{holeId:hole.id,componentPath:"rail",connectorId:"mount1",
			componentFingerprint:haxe.crypto.Sha256.encode("test rail"),connectorFrame:F.alongY(0,0,20,0,0,1),
			clearanceReference:"test clearance reference",permittedRoutes:[],
			budget:{boltDiameterMm:10,clearanceDiameterMm:11,distanceFromDatumMm:0,
				placementOffsetMm:0,placementAngleRad:0,assemblyAdjustmentMm:0,cureBowMm:0,
				machiningErrorMm:0.01,pitchAccumulationMm:0,requiredMarginMm:0.01}}];
		d.setup.sourceFingerprint=ToolingCodec.inputFingerprint(d.input);
		var p=CastingMoldPlanner.plan(d,options(d));
		var rule=materia.tooling.casting.InsertManufacturingPlan.mountingRule(insert.id,hole.id);
		check(p.changeRequests.filter(request -> request.kind==DesignChangeKind.MountingHoleRoute
			&& request.sourceIds.indexOf(insert.id)>=0 && request.blockingChecks.indexOf(rule)>=0).length==1,
			"forbidden mounting routes produce a source request referencing the real failed manufacturing rule");
		var encoded=CastingMoldCodec.encode(p);
		check(CastingMoldCodec.encode(CastingMoldCodec.decode(encoded))==encoded,
			"mounting-route requests and budgets regenerate exactly through the mold codec");
		manufacturing.mountingRequirements[0].permittedRoutes=[materia.casting.InsertManufacturing.InsertHoleRoute.TransferAfterCasting];
		manufacturing.holeRoutes=[{holeId:hole.id,route:materia.casting.InsertManufacturing.InsertHoleRoute.TransferAfterCasting}];
		d.setup.sourceFingerprint=ToolingCodec.inputFingerprint(d.input);
		var resolved=CastingMoldPlanner.plan(d,options(d));
		check(resolved.changeRequests.filter(request -> request.kind==DesignChangeKind.MountingHoleRoute).length==0,
			"source permission and closed transfer budget clear the mounting-route request on replan");
		hole.extent=Blind;hole.depth=17.5;hole.threadDepth=15.5;
		manufacturing.mountingRequirements[0].permittedRoutes=[materia.casting.InsertManufacturing.InsertHoleRoute.BeforeCasting];
		manufacturing.mountingRequirements[0].budget.machiningErrorMm=null;
		manufacturing.holeRoutes=[];
		var insertPose=F.fromRotationMatrix(0,0,20,[1,0,0,0,-1,0,0,0,-1]);
		var pending=materia.tooling.casting.InsertFixturePlanner.missingFixing(insert,insertPose,F.identity(),d.recipe.hardware);
		check(pending.change==null && pending.check.status=="pending"
			&& pending.check.rule=="tooling.fixture.hole-stage."+insert.id,
			"permitted pre-cast route awaiting budget inputs blocks the fixture as Pending, without a source-hole request");
		check(manufacturing.holeRoutes.length==0 && hole.frame.z==20,
			"fixture feasibility changes neither the source stage nor its single hole frame");
	}

	static function liftingSourceDeclaration():Void {
		var d=ToolingTests.fixture();d.recipe.minimumLigament=2;
		var insert=d.input.inserts[0],hole=insert.holes[0];
		insert.role=materia.casting.InsertSpec.InsertRole.LiftingMount;
		hole.extent=Blind;hole.diameter=5;hole.depth=12;hole.tapDrill=5;hole.threadDepth=10;
		hole.thread={designation:"M6",majorDiameter:6,pitch:1,starts:1,hand:Right,
			internal:true,toleranceClass:"6H",profile:Metric};
		hole.clearanceDiameter=6.6;hole.limits.threaded={minEngagement:7.8,minTipGap:1};
		d.recipe.designReview.minimumLiftingPoints=1;
		d.setup.sourceFingerprint=ToolingCodec.inputFingerprint(d.input);
		var p=CastingMoldPlanner.plan(d,options(d));
		check(p.changeRequests.filter(change -> change.kind==LiftingPoints).length==0,
			"source lifting inserts clear the source design request independently of equipment");
		check(p.validation.filter(check -> check.rule=="tooling.handling.flip" && check.status=="pending").length==1,
			"source declarations do not qualify rigging, capacity or the flip procedure");
		var manufacturing=switch insert.origin {case Fabricated(plan):plan;default:throw "Fabricated lifting insert required";};
		manufacturing.holeRoutes=[{holeId:hole.id,route:materia.casting.InsertManufacturing.InsertHoleRoute.TransferAfterCasting}];
		d.setup.sourceFingerprint=ToolingCodec.inputFingerprint(d.input);
		check(CastingMoldPlanner.plan(d,options(d)).changeRequests.filter(change -> change.kind==LiftingPoints).length==1,
			"a future post-cast thread cannot clear the initial lifting request");
		manufacturing.holeRoutes=[{holeId:hole.id,route:materia.casting.InsertManufacturing.InsertHoleRoute.BeforeCasting}];
		insert.role=materia.casting.InsertSpec.InsertRole.GuideSeat;
		d.setup.sourceFingerprint=ToolingCodec.inputFingerprint(d.input);
		var missing=CastingMoldPlanner.plan(d,options(d));
		check(missing.changeRequests.filter(change -> change.kind==LiftingPoints
			&& change.blockingChecks.indexOf("tooling.handling.lifting-points")>=0).length==1,
			"removing the source lifting declaration restores its design request");
		d.recipe.designReview.minimumLiftingPoints=0;
		check(CastingMoldPlanner.plan(d,options(d)).changeRequests.filter(change -> change.kind==LiftingPoints).length==1,
			"a zero count limit cannot stand in for declared lifting connections or a qualified cradle");
	}

	static function straddlingDatum():Void {
		var d=ToolingTests.fixture();d.recipe.minimumLigament=2;
		var extra=ToolingCodec.snapshot(d).input.inserts[0];
		extra.id="raised-seat";extra.pose=F.translation(-70,0,86);
		d.input.inserts.push(extra);
		d.input.features.push({id:"insert-raised-seat",kind:InsertRecess,owner:extra.id,
			geometry:{solid:materia.casting.CastingFeature.CastingGeometrySolid.Insert(extra.body),pose:extra.pose},
			openings:[materia.casting.CastingPatterns.face("body.z-max")],draft:{degrees:0,axis:{x:0,y:0,z:1}},
			cornerRadius:0,adjustable:[],derivedFrom:null});
		d.setup.sourceFingerprint=ToolingCodec.inputFingerprint(d.input);
		var p=CastingMoldPlanner.plan(d,options(d));
		var fixture=p.fixtures.filter(fixture -> fixture.insertId==extra.id)[0];
		var bed=p.panels.members.filter(member -> member.panel.id=="bed")[0];
		var socket=bed.panel.operations.filter(operation -> operation.id==fixture.datum.id)[0];
		var top=bed.pose.z+bed.panel.stock.measuredThickness,below=0;
		for (piece in fixture.datum.pieces) if (piece.z<top-1e-7) {
			below++;
			var contactTolerance=1e-6;
			check(materia.casting.CastingProfileGeometry.containsRect(socket.contour,piece.x-bed.pose.x+contactTolerance,
				piece.y-bed.pose.y+contactTolerance,piece.width-2*contactTolerance,piece.depth-2*contactTolerance),
				"datum interior fits its bed socket; boundary contact is permitted");
		}
		check(below>0 && fixture.datum.pieces.filter(piece -> piece.z+piece.height>top+1e-7).length>0,
			"raised datum fixture actually straddles the bed surface");
	}

	public static function run(d:ToolingDocument):Void {
		straddlingDatum();
		liftingSourceDeclaration();
		mountingRouteRequest();
		unrealizedGeometry();
		replicationReservation(d);
		d = ToolingCodec.snapshot(d);
		d.recipe.minimumLigament = 2;
		var late=ToolingCodec.snapshot(d);
		switch late.input.inserts[0].origin {
			case Fabricated(manufacturing):manufacturing.holeRoutes[0].route=materia.casting.InsertManufacturing.InsertHoleRoute.TransferAfterCasting;
			case Catalogue(_):throw "Fabricated fixture expected";
		}
		late.setup.sourceFingerprint=ToolingCodec.inputFingerprint(late.input);
		var earlyPlan=CastingMoldPlanner.plan(d,options(d)),latePlan=CastingMoldPlanner.plan(late,options(late));
		var lateHole=late.input.inserts[0].holes[0];
		var steelAdded=Math.PI*lateHole.diameter*lateHole.diameter/4*lateHole.depth*d.recipe.steelDensityKgM3*1e-9;
		check(Math.abs(latePlan.handling.castingMassKg-earlyPlan.handling.castingMassKg-steelAdded)<1e-9,
			"handling uses solid pre-cast steel where a through bore is drilled later");
		check(latePlan.fixtures[0].boreMasks.length==0 && earlyPlan.fixtures[0].boreMasks.length==1,
			"generic mold protection follows the same manufacturing stage as handling");
		var o = options(d),
			p = CastingMoldPlanner.plan(d, o),
			s = CastingMoldCodec.encode(p);
		FabricationTests.run(p);
		var sourceThreads = ToolingCodec.snapshot(d);
		sourceThreads.recipe.hardware.fixturePinSpacing = 24;
		var threads = TestInsertHoles.through(5, InsertBodies.height(sourceThreads.input.inserts[0].body));
		threads[0].thread = {designation: "M6", majorDiameter: 6, pitch: 1, starts: 1, hand: Right,
			internal: true, toleranceClass: "6H", profile: Metric};
		threads[0].tapDrill = 5;
		threads[0].threadDepth = threads[0].depth;
		threads[0].clearanceDiameter = 6.6;
		threads[0].limits = {policyId: "steel-threaded-hole-v1", threaded: {minEngagement: 6, minTipGap: 1},
			minBack: 1.5, minWall: 3, minClearance: 0.6, overrides: []};
		threads[0].frame.y = -5;
		var second = materia.casting.InsertHole.InsertHoleCodec.snapshot(threads)[0];
		second.id = "second";
		second.frame.y = 5;
		threads.push(second);
		var unused = TestInsertHoles.through(3, InsertBodies.height(sourceThreads.input.inserts[0].body))[0];
		unused.id = "unused";
		unused.frame.x = 10;
		unused.thread = threads[0].thread;
		unused.diameter = 5;
		unused.tapDrill = 5;
		unused.threadDepth = unused.depth;
		unused.clearanceDiameter = 6.6;
		unused.limits = threads[0].limits;
		unused.frame = F.translation(10, 0, 0);
		threads.push(unused);
		sourceThreads.input.inserts[0].holes = threads;
		TestInsertHoles.beforeCasting(sourceThreads.input.inserts[0]);
		sourceThreads.setup.sourceFingerprint = ToolingCodec.inputFingerprint(sourceThreads.input);
		var fixed = CastingMoldPlanner.plan(sourceThreads, options(sourceThreads));
		check(fixed.fixtures[0].fixings.length == 2, "datum selects both accessible source steel threads");
		var fixtureLocation=fixed.fixtures[0].location;
		check(fixtureLocation.unsupportedReasons.length==0 && fixtureLocation.report.rank==3,
			"flat insert support locates three DOFs, independently of datum-to-bed pins");
		var freeCoordinates:Array<String> = [];
		for (motion in fixtureLocation.report.freeMotions) for (coordinate in motion.affectedCoordinates)
			if (freeCoordinates.indexOf(coordinate)<0) freeCoordinates.push(coordinate);
		freeCoordinates.sort(Reflect.compare);
		check(freeCoordinates.join(",")=="rotation-z,translation-x,translation-y",
			"real insert fixture exposes its two lateral translations and yaw");
		check(fixtureLocation.report.preloadIds.length==2 && fixtureLocation.report.redundantLocations.length==0,
			"both source-thread screws clamp without becoming lateral locators");
		check(fixed.validation.filter(v -> v.rule=="tooling.fixture.location."+fixed.fixtures[0].insertId
			&& v.status==Pending && v.message.indexOf("translation-x")>=0).length==1,
			"under-constrained real fixture blocks readiness with named free directions");
		var encodedFixture=CastingMoldCodec.encode(fixed);
		check(CastingMoldCodec.decode(encodedFixture).fixtures[0].location.report.rank==3,
			"fixture contact witnesses and rank survive exact regeneration");
		fixed.fixtures[0].location.report.rank=6;
		rejects(() -> CastingMoldPlanner.validate(fixed),"forged six-DOF fixture report is rejected");
		fixed.fixtures[0].location.report.rank=3;
		var reordered=ToolingCodec.snapshot(sourceThreads).input.inserts[0];
		reordered.holes.reverse();
		var datum=fixed.fixtures[0],reversedCuts=materia.tooling.casting.MoldVolumeCodec.snapshot(datum.datum);
		reversedCuts.cuts.reverse();reversedCuts.pieces.reverse();
		var permuted=materia.tooling.casting.DatumFixtureLocation.plan(reordered,sourceThreads.setup.sourceToMold,
			{mountingFace:datum.mountingFace,datum:reversedCuts,fixings:datum.fixings});
		check(ToolingCodec.canonical(permuted)==ToolingCodec.canonical(fixtureLocation),
			"fixture contact report is independent of source hole and support order");
		var clearContacts=true;
		for (contact in fixtureLocation.contacts) if (contact.role==materia.casting.FixtureConstraints.FixtureConstraintRole.Location)
			for (hole in sourceThreads.input.inserts[0].holes) {
				var entry=F.compose(F.compose(sourceThreads.setup.sourceToMold,sourceThreads.input.inserts[0].pose),hole.frame);
				var dx=contact.point.x-entry.x,dy=contact.point.y-entry.y;
				if (dx*dx+dy*dy<=9) clearContacts=false;
			}
		check(clearContacts,"support witnesses lie outside every source thread footprint");
		var missingSupport=materia.tooling.casting.MoldVolumeCodec.snapshot(datum.datum);
		missingSupport.pieces=[];
		var unsupportedLocation=materia.tooling.casting.DatumFixtureLocation.plan(reordered,sourceThreads.setup.sourceToMold,
			{mountingFace:datum.mountingFace,datum:missingSupport,fixings:datum.fixings});
		check(unsupportedLocation.report.rank==0 && unsupportedLocation.unsupportedReasons.length==1
			&& materia.tooling.casting.DatumFixtureLocation.validation(datum.insertId,unsupportedLocation).status
				==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported,
			"screws alone cannot stand in for a missing support contact model");
		check(fixed.fixtures[0].boreMasks.length == 1 && fixed.fixtures[0].boreMasks[0].sourceHoleId == "unused",
			"only the non-fixing hole receives a bore mask");
		var unusedMask = fixed.fixtures[0].boreMasks[0];
		check(unusedMask.plugLength == sourceThreads.recipe.hardware.plugDepth && unusedMask.farEndId != null
			&& fixed.hardware.filter(h -> h.id == unusedMask.farEndId).length == 1,
			"unused through-thread has separate entry and resin-side threaded plugs");
		var needsCap = ToolingCodec.snapshot(sourceThreads);
		needsCap.input.inserts[0].holes.filter(h -> h.id == "unused")[0].counterbore = {diameter: 8, depth: 2};
		needsCap.setup.sourceFingerprint = ToolingCodec.inputFingerprint(needsCap.input);
		var capPlan = CastingMoldPlanner.plan(needsCap, options(needsCap));
		check(capPlan.validation.filter(v -> v.rule == "tooling.fixture.hole-protection." + needsCap.input.inserts[0].id + ".unused"
			&& v.status == materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported).length == 1 && !capPlan.fabrication.ready,
			"threaded counterbore needs an explicit sealing cap, never a smooth-pin fallback");
		var protectedHardware = fixed.hardware;
		fixed.hardware = protectedHardware.filter(h -> h.id != unusedMask.farEndId);
		rejects(() -> CastingMoldPlanner.validate(fixed), "missing unused-hole far-end mask rejected");
		fixed.hardware = protectedHardware;
		for (fixing in fixed.fixtures[0].fixings) {
			var screw = fixed.hardware.filter(h -> h.id == fixing.hardwareId)[0];
			var penetration = fixed.fixtures[0].penetrations.filter(pen -> pen.sourceHoleId == fixing.sourceHoleId)[0];
			check(penetration.plug != null && fixing.engagement + 1 + penetration.plug.length <= threads[0].depth,
				"through-thread fixing has a resin-side plug and a separate tip gap");
			check(Math.abs(screw.length - (fixing.entry.z - screw.pose.z + 6)) < 1e-7,
				"screw clamps the bed and datum with policy thread engagement");
		}
		var farEnd = fixed.fixtures[0].penetrations.filter(pen -> pen.sourceHoleId == fixed.fixtures[0].fixings[0].sourceHoleId)[0];
		var savedPlug = farEnd.plug;
		farEnd.plug = null;
		rejects(() -> CastingMoldPlanner.validate(fixed), "missing resin-side protection rejected");
		farEnd.plug = savedPlug;
		fixed.hardware.filter(h -> h.id == fixed.fixtures[0].fixings[0].hardwareId)[0].length += 1;
		rejects(() -> CastingMoldPlanner.validate(fixed), "altered fixing stack rejected");
		check(p.fixtures[0].fixings.length == 0 && p.changeRequests.filter(r -> r.kind == "insert-fixture-hole").length == 1,
			"an insert without a usable steel thread blocks its fixture with a source change request");
		var handlingOptions = options(d);
		handlingOptions.serviceOrientation = materia.assembly.AssemblyFrames.compose(materia.assembly.AssemblyFrames.fromRotationMatrix(0, 0, 0,
			[0, -1, 0, 1, 0, 0, 0, 0, 1]), d.setup.sourceToMold);
		check(!materia.tooling.casting.CastHandling.plan(d, handlingOptions, p.envelope).requiresFlip,
			"yaw preserves the vertical and does not require lifting/flip");
		handlingOptions.serviceOrientation = materia.assembly.AssemblyFrames.compose(materia.assembly.AssemblyFrames.fromRotationMatrix(0, 0, 0,
			[1, 0, 0, 0, 0, -1, 0, 1, 0]), d.setup.sourceToMold);
		check(materia.tooling.casting.CastHandling.plan(d, handlingOptions, p.envelope).requiresFlip, "changing vertical requires handling/flip");
		var bridgePlan = CastingMoldCodec.decode(s);
		var extraCore = CastingMoldCodec.decode(s).cores[0];
		extraCore.cavity.y -= 20;
		extraCore.cavity.depth += 40;
		bridgePlan.cores.push(extraCore);
		var bridgePanels:Array<materia.tooling.panel.PanelAssemblyPlan.PanelPlacement> = [];
		materia.tooling.casting.CoreSupports.sharedBridges(bridgePlan.cavity, bridgePlan.document.recipe, bridgePlan.cores, bridgePanels, []);
		var bridgeBounds = materia.casting.CastingProfileGeometry.bounds(materia.tooling.panel.PanelContours.outside(bridgePanels.filter(v ->
			v.panel.id == "core-bridge-0")[0].panel));
		check(Math.abs(bridgeBounds.height - extraCore.cavity.depth / 2) < 1e-8, "row bridge uses the largest depth including a later core");
		var narrowBridge = ToolingCodec.snapshot(d);
		narrowBridge.recipe.hardware.minimumBridgeWidth = 1000;
		rejects(() -> CastingMoldPlanner.plan(narrowBridge, options(narrowBridge)), "undersized bridge row rejected");
		var intersecting = CastingMoldCodec.decode(s);
		var resinPieces = BoxAlgebra.subtract([p.envelope], [
			for (f in p.document.input.features)
				CastingMoldPlanner.featureBox(p.document, f)
		]);
		intersecting.fixtures[0].datum.pieces = [resinPieces[0]];
		rejects(() -> CastingMoldPlanner.validate(intersecting), "datum/resin intersection rejected before CAD");
		check(CastingMoldCodec.encode(CastingMoldCodec.decode(s)) == s, "multi-core canonical regeneration");
		var jigOptions=options(d);
		jigOptions.replicationJigs=[{id:"synthetic-jig",datumSchemeId:"synthetic-datums",frameReference:"synthetic-frame",
			pose:materia.assembly.AssemblyFrames.identity(),masters:[{id:"synthetic-bar",
				kind:materia.tooling.casting.ReplicationJig.ReplicationMasterKind.GroundBar,
				body:materia.casting.CastingSolid.Box(60,20,10),pose:materia.assembly.AssemblyFrames.identity(),
				reference:materia.assembly.AssemblyFrames.identity(),
				members:[materia.casting.InterfaceRelationship.InterfaceMember.ComponentInterface("synthetic-member","synthetic-plane")],
				finish:materia.casting.InterfaceRequirement.InterfaceFinish.JigMaster,jacks:[],releaseSpec:null,
				flatnessMm:null,inspectionReference:null}]}];
		var jigPlan=CastingMoldPlanner.plan(d,jigOptions),jigEncoded=CastingMoldCodec.encode(jigPlan);
		check(CastingMoldCodec.encode(CastingMoldCodec.decode(jigEncoded))==jigEncoded,
			"nonempty jig inventory and primitive master stock have one canonical mold encoding");
		jigPlan.validation=jigPlan.validation.filter(check -> !StringTools.startsWith(check.rule,"tooling.replication."));
		jigPlan.fabrication=materia.tooling.ToolingValidation.ToolingReadiness.evaluate(jigPlan.validation);
		rejects(() -> CastingMoldPlanner.validate(jigPlan),"Replication jig validation differs from source and authored jig inventory");

		check(p.features.length == d.input.features.length && p.cores.length == 1, "complete source ownership");
		var fragmented = ToolingCodec.snapshot(d);
		var authored = fragmented.input.features.filter(feature -> feature.kind != materia.casting.CastingFeature.CastingFeatureKind.InsertRecess)[0];
		var cut = materia.casting.CastingBounds.feature(authored);
		var children = BoxAlgebra.retain([authored], [{
			x: cut.x + cut.width / 2 - 5, y: cut.y, z: cut.z, width: 10, depth: cut.depth, height: cut.height
		}]);
		fragmented.input.features = fragmented.input.features.filter(feature -> feature.id != authored.id).concat(children);
		fragmented.setup.features = [{featureId: authored.id, strategy: SacrificialEPS, priority: 0, withdrawal: null}];
		fragmented.setup.sourceFingerprint = ToolingCodec.inputFingerprint(fragmented.input);
		var grouped = CastingMoldPlanner.plan(fragmented, options(fragmented));
		check(grouped.cores.length == 2 && grouped.coreUnits.length == 1
			&& grouped.coreUnits[0].parentFeatureId == authored.id && grouped.coreUnits[0].featureIds.length == 2,
			"retained source fragments share one core unit and accept an explicit parent override");
		fragmented.setup.features.push({featureId: children[0].id, strategy: ReusableFormer, priority: 10, withdrawal: {x: 0, y: 0, z: 1}});
		rejects(() -> ToolingCodec.snapshot(fragmented), "one authored core unit cannot mix incompatible tooling strategies");
		var cuts = [for (f in d.input.features) CastingMoldPlanner.featureBox(d, f)],
			negative = [p.cores[0].cavity];
		for (f in p.formers)
			for (b in f.volume.pieces)
				negative.push(b);
		for (f in p.fixtures)
			for (b in f.datum.pieces)
				negative.push(b);
		for (i in d.input.inserts) {
			var dimensions = InsertBodies.box(i.body, i.id);
			negative.push(materia.tooling.casting.BoxMoldPlanner.box(dimensions.width, dimensions.depth, dimensions.height,
				materia.assembly.AssemblyFrames.compose(d.setup.sourceToMold, i.pose)));
		}
		var source = BoxAlgebra.subtract([p.envelope], cuts),
			reconstructed = BoxAlgebra.subtract([p.cavity], negative);
		check(BoxAlgebra.volume(BoxAlgebra.subtract(source, reconstructed)) + BoxAlgebra.volume(BoxAlgebra.subtract(reconstructed, source)) < 1e-5,
			"orthogonal source/reconstruction symmetric difference");
		check(p.changeRequests.filter(c -> c.kind == "lifting-points").length == 1, "unqualified flip requests lifting points");
		check(p.changeRequests.filter(c -> c.id == "allowance-" + d.input.inserts[0].id).length == 1, "replication requires source allowance decision");
		p.features.pop();
		rejects(() -> CastingMoldPlanner.validate(p), "missing feature rejected");
		p = CastingMoldPlanner.plan(d, o);
		p.openings.pop();
		rejects(() -> CastingMoldPlanner.validate(p), "unclassified opening rejected");
		var legacyOptions:Dynamic=haxe.Json.parse(ToolingCodec.canonical(o));
		Reflect.setField(legacyOptions,"anchorage",new Array<Dynamic>());
		rejects(() -> CastingMoldOptionsCodec.read(legacyOptions),"obsolete duplicate process anchorage rejected");
		var raw:Dynamic = haxe.Json.parse(s);
		Reflect.setField(raw, "version", 15);
		rejects(() -> CastingMoldCodec.decode(haxe.Json.stringify(raw)), "previous mold schema rejected");
		raw = haxe.Json.parse(s);
		Reflect.setField(raw, "version", 12);
		rejects(() -> CastingMoldCodec.decode(haxe.Json.stringify(raw)), "previous mold schema rejected");
		raw = haxe.Json.parse(s);
		Reflect.setField(raw, "version", 99);
		rejects(() -> CastingMoldCodec.decode(haxe.Json.stringify(raw)), "unknown mold format rejected");
		raw = haxe.Json.parse(s);
		Reflect.setField(Reflect.field(raw, "options"), "requiresFlip", 1);
		rejects(() -> CastingMoldCodec.decode(haxe.Json.stringify(raw)), "typed handling flag");
		var volume:materia.tooling.casting.CastingMoldPlan.MoldVolume = {
			id: "test-former",
			material: FormerStock,
			cuts: [],
			regions: [],
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
		raw=haxe.Json.parse(encoded);
		Reflect.setField(raw,"version",3);
		rejects(() -> MoldVolumeCodec.decode(haxe.Json.stringify(raw)),"previous volume schema rejected");
		raw=haxe.Json.parse(encoded);
		Reflect.deleteField(Reflect.field(raw,"volume"),"regions");
		rejects(() -> MoldVolumeCodec.decode(haxe.Json.stringify(raw)),"region collection is required");
		var shaped=MoldVolumeCodec.snapshot(volume);
		shaped.pieces=[];
		shaped.regions=[materia.casting.CastingRegion.Drafted("shaped-former",
			materia.casting.CastingProfiles.rectangle(-20,-15,40,30),F.identity(),60,2)];
		var shapedText=MoldVolumeCodec.encode(shaped);
		check(MoldVolumeCodec.encode(MoldVolumeCodec.decode(shapedText))==shapedText,"region-only tooling volume round-trips without box stand-ins");
		var stock=materia.tooling.casting.MoldVolumeBounds.pieces(shaped);
		check(stock.length==1 && Math.abs(stock[0].width-40)<1e-7 && Math.abs(stock[0].depth-30)<1e-7
			&& Math.abs(stock[0].height-60)<1e-7,"draft bounds describe stock only, not displaced material");

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
		check(Math.abs(BoxAlgebra.volume(BoxAlgebra.subtract([a], [b])) - 840) < 1e-8, "through-hole box subtraction");
		check(BoxAlgebra.groups(BoxAlgebra.subtract([a], [
			{
				x: 4,
				y: -1,
				z: -1,
				width: 2,
				depth: 12,
				height: 12
			}
		])).length == 2, "disconnected formers split into physical members");
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
		check(rigidPlan.changeRequests.filter(v -> v.kind == ReleaseAlternative).length > 0, "zero draft emits a physical-release request");
		var overlap = ToolingCodec.snapshot(d);
		overlap.input.inserts[0].pose.x = 30;
		overlap.input.inserts[0].pose.z = 40;
		overlap.input.features.filter(f -> f.kind == "insert-recess")[0].geometry.pose.x = 30;
		overlap.input.features.filter(f -> f.kind == "insert-recess")[0].geometry.pose.z = 40;
		overlap.input.features.filter(f -> f.kind == "insert-recess")[0].openings = [];
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
			rotatedNegative.push(materia.tooling.casting.BoxMoldPlanner.box(InsertBodies.box(i.body, i.id).width, InsertBodies.box(i.body, i.id).depth, InsertBodies.box(i.body, i.id).height,
				materia.assembly.AssemblyFrames.compose(rotated.setup.sourceToMold, i.pose)));
		var rotatedSource = BoxAlgebra.subtract([rotatedPlan.envelope], [for (f in rotated.input.features) CastingMoldPlanner.featureBox(rotated, f)]),
			rotatedRebuilt = BoxAlgebra.subtract([rotatedPlan.cavity], rotatedNegative);
		check(BoxAlgebra.volume(BoxAlgebra.subtract(rotatedSource, rotatedRebuilt))
			+ BoxAlgebra.volume(BoxAlgebra.subtract(rotatedRebuilt, rotatedSource)) < 1e-5,
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
		var joint:materia.tooling.casting.CastingMoldPlan.FoundationJointProcess={jointId:"duplicate",materialReference:null,cureEvidence:[]};
		grout.foundationJointProcesses=[joint,joint];
		rejects(() -> CastingMoldOptionsCodec.snapshot(grout),"duplicate foundation joint process IDs rejected");
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
		assigned.input.features.push({id:"overlap-detail",kind:materia.casting.CastingFeature.CastingFeatureKind.Lightening,owner:null,geometry:{solid:materia.casting.CastingFeature.CastingGeometrySolid.Cut(materia.casting.CastingSolid.Box(40,40,30)),pose:materia.assembly.AssemblyFrames.translation(0, 0, 20)},openings:[],draft:{degrees:0,axis:{x:0,y:0,z:1}},cornerRadius:0,adjustable:[CastingDimension.Width, CastingDimension.Depth, CastingDimension.Height],derivedFrom:null});
		assigned.setup.features.push({
			featureId: "overlap-detail",
			strategy: materia.tooling.casting.CastingSetup.CoreStrategy.ReusableFormer,
			withdrawal: {x: 0, y: 0, z: 1},
			priority: 10
		});
		assigned.setup.sourceFingerprint = ToolingCodec.inputFingerprint(assigned.input);
		assigned.setup.features[assigned.setup.features.length - 1].priority = 0;
		rejects(() -> CastingMoldPlanner.plan(assigned, options(assigned)), "equal-priority overlapping formers rejected");
		assigned.setup.features[assigned.setup.features.length - 1].priority = 10;
		var allocated = CastingMoldPlanner.plan(assigned, options(assigned));
		check(allocated.features.filter(f -> f.featureId == "pocket")[0].memberIds.indexOf("former-overlap-detail-0") >= 0,
			"partial overlaps include every shaping former, not only the newly allocated remainder");
		var machiningSource=ToolingCodec.snapshot(d);
		machiningSource.input.inserts[0].finish=materia.casting.InsertSpec.InsertFinish.MachinedAfterCasting;
		machiningSource.setup.sourceFingerprint=ToolingCodec.inputFingerprint(machiningSource.input);
		var machining=options(machiningSource);
		check(CastingMoldPlanner.plan(machiningSource,machining).changeRequests.filter(c -> c.id == "allowance-" + d.input.inserts[0].id).length == 1,
			"source machining requires an allowance decision");
		var oldPrecision:Dynamic=haxe.Json.parse(ToolingCodec.canonical(machining));
		var oldPrecisionRows:Array<Dynamic> = cast Reflect.field(oldPrecision,"precision");
		Reflect.setField(oldPrecisionRows[0],"allowance",1);
		rejects(() -> CastingMoldOptionsCodec.read(oldPrecision),"mold options cannot carry source finishing allowances");
		var stockSource=ToolingCodec.snapshot(d),stockInsert=stockSource.input.inserts[0];
		stockInsert.finish=materia.casting.InsertSpec.InsertFinish.GroundAfterCasting;
		stockInsert.interfaces=[{
			id:"finished-top",geometry:materia.casting.InterfaceRequirement.PrecisionGeometry.Plane(
				materia.assembly.AssemblyFrames.translation(0,0,19)),
			route:materia.casting.InterfaceRequirement.InterfaceRoute.Ground,allowanceMm:1,finishingProcessErrorMm:null,
			permittedFinishes:[materia.casting.InterfaceRequirement.InterfaceFinish.Ground],
			positionToleranceMm:0.01,datumReference:"primary",relationshipIds:[]
		}];
		stockInsert.holes[0].frame.z=19;stockInsert.holes[0].depth=19;
		stockSource.setup.sourceFingerprint=ToolingCodec.inputFingerprint(stockSource.input);
		var stockOptions=options(stockSource);
		var stockPlan=CastingMoldPlanner.plan(stockSource,stockOptions);
		var stockRule="tooling.precision.allowance."+stockInsert.id;
		check(stockPlan.validation.filter(v -> v.rule==stockRule && v.status==Pass).length==1
			&& stockPlan.changeRequests.filter(c -> c.id=="allowance-"+stockInsert.id).length==0,
			"source finishing layer clears the allowance request");
		stockInsert.interfaces[0].allowanceMm=2;
		stockInsert.holes[0].frame.z=20;stockInsert.holes[0].depth=20;
		stockSource.setup.sourceFingerprint=ToolingCodec.inputFingerprint(stockSource.input);
		var badStock=CastingMoldPlanner.plan(stockSource,stockOptions);
		check(badStock.validation.filter(v -> v.rule==stockRule && v.status==Fail).length==1
			&& badStock.changeRequests.filter(c -> c.id=="allowance-"+stockInsert.id).length==1,
			"allowance request returns when modeled stock no longer matches");
		switch stockSource.input.inserts[0].origin {
			case Fabricated(manufacturing):
				manufacturing.holeRoutes=[{holeId:stockSource.input.inserts[0].holes[0].id,
					route:materia.casting.InsertManufacturing.InsertHoleRoute.TransferAfterCasting}];
			case Catalogue(_):throw "Fabricated source expected";
		}
		stockSource.setup.sourceFingerprint=ToolingCodec.inputFingerprint(stockSource.input);
		var manufacturingReview=CastingMoldPlanner.assess(stockSource,stockOptions);
		check(manufacturingReview.plan!=null && manufacturingReview.validation.filter(v ->
			v.rule=="fabrication.insert.hole-geometry."+stockSource.input.inserts[0].id
			&& v.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass).length==1,
			"resolved post-cast holes preserve nominal mold review with distinct pre-cast geometry");
		check(manufacturingReview.changeRequests.filter(request -> request.blockingChecks.indexOf(
			"fabrication.insert.hole-geometry."+stockSource.input.inserts[0].id)>=0).length==0,
			"manufacturing work is not turned into a casting geometry change request");
		var finishing = ToolingCodec.snapshot(d);
		finishing.input.inserts[0].finish = materia.casting.InsertSpec.InsertFinish.ReplicatedAfterCasting;
		finishing.setup.sourceFingerprint = ToolingCodec.inputFingerprint(finishing.input);
		var sourceReplication=CastingMoldPlanner.plan(finishing,options(finishing));
		check(sourceReplication.validation.filter(check -> check.rule=="tooling.precision.replication."+finishing.input.inserts[0].id
			&& check.status==Pending).length==1,
			"source replication requires its master and release without an options route");
		var invalidEvidence:Dynamic = haxe.Json.parse(s);
		Reflect.setField(Reflect.field(invalidEvidence, "options"), "liftingEvidence", ["nonempty string is not qualification"]);
		rejects(() -> CastingMoldCodec.decode(haxe.Json.stringify(invalidEvidence)), "lifting requires a typed qualification record");
		var invalidRule = CastingMoldCodec.decode(s);
		invalidRule.changeRequests[0].blockingChecks = ["missing-rule"];
		rejects(() -> CastingMoldPlanner.validate(invalidRule), "change requests cannot name nonexistent validation rules");
		ReviewContractTests.run(CastingMoldCodec.decode(s));
		ProcessTests.run(CastingMoldCodec.decode(s));
		Sys.println('CastingMoldTests: $assertions assertions passed');
	}

	static function replicationReservation(document:ToolingDocument):Void {
		var d=ToolingCodec.snapshot(document),insert=d.input.inserts[0];
		insert.body=materia.casting.InsertSpec.InsertBody.Primitive(materia.casting.CastingSolid.Box(40,30,14));
		insert.pose=F.translation(80,0,85);
		insert.holes=[];
		TestInsertHoles.beforeCasting(insert);
		insert.finish=materia.casting.InsertSpec.InsertFinish.ReplicatedAfterCasting;
		insert.interfaces=[{
			id:"replication-face",geometry:materia.casting.InterfaceRequirement.PrecisionGeometry.Plane(F.translation(0,0,15)),
			route:materia.casting.InterfaceRequirement.InterfaceRoute.Replicated,allowanceMm:1,finishingProcessErrorMm:null,
			permittedFinishes:[materia.casting.InterfaceRequirement.InterfaceFinish.JigMaster],
			positionToleranceMm:0.01,datumReference:"primary",relationshipIds:[]
		}];
		var recess=d.input.features.filter(f -> f.owner==insert.id)[0];
		recess.geometry={solid:materia.casting.CastingFeature.CastingGeometrySolid.Insert(insert.body),pose:insert.pose};
		recess.openings=[materia.casting.CastingPatterns.face("body.x-max")];
		var occupied=materia.tooling.casting.InterfaceQualification.finishingStock(d.input,insert);
		check(occupied.status==Fail && occupied.affectedIds.join(",")==insert.id
			&& occupied.message.indexOf("replication-face")>=0 && occupied.message.indexOf("casting material")>=0,
			"a nominal replication gap fails when it is still occupied by source casting material");
		var gap=materia.casting.CastingFeatureCodec.snapshot([recess])[0];
		gap.id="replication-gap";gap.kind=materia.casting.CastingFeature.CastingFeatureKind.InterfaceRecess;
		gap.geometry={solid:materia.casting.CastingFeature.CastingGeometrySolid.Cut(materia.casting.CastingSolid.Box(40,30,1)),
			pose:F.translation(80,0,99)};
		gap.openings=[materia.casting.CastingPatterns.face("body.x-max"),materia.casting.CastingPatterns.face("body.z-max")];
		d.input.features.push(gap);
		var reserved=materia.tooling.casting.InterfaceQualification.finishingStock(d.input,insert);
		check(reserved.status==Pass,"explicit source gap clears the nominal allowance decision");
		var obstruction=materia.tooling.ToolingCodec.snapshot(document).input.inserts[0];
		obstruction.id="gap-obstruction";obstruction.holes=[];TestInsertHoles.beforeCasting(obstruction);
		obstruction.body=materia.casting.InsertSpec.InsertBody.Primitive(materia.casting.CastingSolid.Box(2,2,1));
		obstruction.pose=F.translation(80,0,99);
		d.input.inserts.push(obstruction);
		var blocked=materia.tooling.casting.InterfaceQualification.finishingStock(d.input,insert);
		check(blocked.status==Fail && blocked.message.indexOf("gap-obstruction")>=0,
			"a second insert cannot occupy the reserved replication gap");
	}
}
