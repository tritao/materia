import materia.tooling.casting.CastingMoldPlan.MoldOpenings;
import materia.tooling.casting.MoldCores;
import materia.casting.InsertSpec.InsertBodyCodec as InsertBodies;
import materia.assembly.AssemblyFrames as F;
import haxe.Json;
import materia.tooling.ToolingDocument;
import materia.tooling.ToolingCodec;
import materia.tooling.casting.CastingMoldPlanner;
import materia.tooling.panel.PanelCodec;
import materia.tooling.casting.CoreSupports;
import materia.tooling.casting.CoreSupport.CoreSupportKind;

/** Planning/persistence remain buildable with ProjectKit alone, without native CAD. */
class BoxPlannerTests {
	static var assertions = 0;

	static function check(ok:Bool, label:String):Void {
		assertions++;
		if (!ok)
			throw label;
	}

	static function rejects(fn:Void->Void, label:String):Void {
		var caught = false;
		try
			fn()
		catch (e:Dynamic)
			caught = true;
		check(caught, label);
	}

	public static function main():Void run(ToolingTests.fixture());

	public static function run(d:ToolingDocument):Void {
		d = ToolingCodec.copy(d);
		d.recipe.minimumLigament = 2;
		var insertSize = InsertBodies.box(d.input.inserts[0].body, "seat");
		d.input.inserts[0].body = materia.casting.InsertSpec.InsertBody.Primitive(materia.casting.CastingSolid.Box(insertSize.width, insertSize.depth, 15));
		d.input.inserts[0].pose.z = 85;
		d.input.inserts[0].holes = TestInsertHoles.through(5,InsertBodies.height(d.input.inserts[0].body));
		var hole = d.input.inserts[0].holes[0];
		hole.thread = {designation: "M6", majorDiameter: 6, pitch: 1, starts: 1, hand: Right,
			internal: true, toleranceClass: "6H", profile: Metric};
		hole.tapDrill = 5;
		hole.threadDepth = hole.depth;
		hole.clearanceDiameter = 6.6;
		hole.limits = {policyId: "steel-threaded-hole-v1", threaded: {minEngagement: 6, minTipGap: 1},
			minBack: 1.5, minWall: 3, minClearance: 0.6, overrides: []};
		var recess = d.input.features.filter(f -> f.kind == "insert-recess")[0];
		recess.geometry.solid = materia.casting.CastingFeature.CastingGeometrySolid.Insert(d.input.inserts[0].body);
		recess.geometry.pose.z = 85;
		d.setup.sourceFingerprint = ToolingCodec.inputFingerprint(d.input);
		var withDatums=ToolingCodec.copy(d);
		withDatums.input.datumScheme={id:"fixture-datums",foundationId:d.input.id,
			primary:{id:"primary",reference:materia.casting.FoundationDatumScheme.DatumReference.BodyFace("body.z-max"),frame:F.translation(0,0,100)},
			secondary:{id:"secondary",reference:materia.casting.FoundationDatumScheme.DatumReference.BodyFace("body.x-min"),
				frame:F.fromRotationMatrix(-100,0,50,[0,0,-1,0,1,0,1,0,0])},
			tertiary:{id:"tertiary",reference:materia.casting.FoundationDatumScheme.DatumReference.BodyFace("body.y-min"),
				frame:F.fromRotationMatrix(0,-90,50,[1,0,0,0,0,-1,0,1,0])}};
		withDatums.setup.sourceFingerprint=ToolingCodec.inputFingerprint(withDatums.input);
		var datumPlan=CastingMoldPlanner.planSinglePocket(withDatums);
		check(datumPlan.validation.filter(v -> v.rule=="tooling.datum.contacts."+d.input.id
			&& v.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending).length==1,
			"box planner preserves datum contact qualification gate");
		var carrierPlan=CastingMoldPlanner.planSinglePocket(d),carrierBody=carrierPlan.cavity;
		var sourceBefore=ToolingCodec.encode(carrierPlan.document),carrierFace=F.translation(0,70,102);
		var carrier=CoreSupports.insertCarrier(carrierPlan,"dry-fixture",carrierFace,
			{x:-20,y:60,z:90,width:40,depth:20,height:12});
		check(carrier.panel.contactFace==materia.tooling.ToolingEnums.ContactFace.NegativeZ,
			"Upward source face contacts the dry carrier underside");
		check(materia.tooling.casting.DatumFixtures.panelOnFace([carrier],carrierFace)==carrier,
			"Dry carrier uses the same source-plane receiver selection");
		var pads=carrierPlan.formers.filter(former -> former.volume.id.indexOf("fixture-carrier-dry-fixture-")==0);
		check(pads.length==2,"Dry carrier has one physical support pad per side wall");
		for (pad in pads) {
			check(pad.volume.pieces.length==1 && pad.volume.pieces[0].height==2,
				"Pad height comes from the source contact plane and actual wall top");
			var piece=pad.volume.pieces[0];
			check(piece.x+piece.width<=carrierBody.x+1e-7 || piece.x>=carrierBody.x+carrierBody.width-1e-7,
				"Dry pads rest on wall tops outside the resin footprint");
		}
		check(sourceBefore==ToolingCodec.encode(carrierPlan.document),"Carrier construction never changes casting source geometry");
		check(carrierPlan.validation.filter(row -> row.rule=="loads-handling" && row.status=="pending").length==1,
			"Carrier construction preserves the existing load and handling qualification gate");
		rejects(() -> CoreSupports.insertCarrier(carrierPlan,"low",F.translation(0,70,101),
			{x:-20,y:60,z:90,width:40,depth:20,height:11}),"Carrier below dry clearance is rejected");
		var blockedContact="";
		try CoreSupports.insertCarrier(carrierPlan,"blocked-contact",F.translation(0,0,102),
			{x:-20,y:-10,z:90,width:40,depth:20,height:12}) catch (error:Dynamic) blockedContact=Std.string(error);
		check(blockedContact.indexOf("crosses the source insert contact plane")>=0,
			"An existing core support cannot be cut through the insert contact face");
		var crossing=materia.tooling.panel.PanelJoints.blank("cross-beam",80,40,d.recipe.panel,d.recipe.routerDiameter,2);
		carrierPlan.supportPanels.push({panel:crossing,pose:F.translation(-40,50,118),withdrawal:{x:0,y:0,z:1}});
		carrierPlan.cores[0].support.kind=Bridge;
		carrierPlan.cores[0].support.bondedPanelIds=[crossing.id];
		var cross=CoreSupports.insertCarrier(carrierPlan,"dry-cross",carrierFace,
			{x:-20,y:60,z:90,width:40,depth:20,height:12});
		var clearances=cross.panel.operations.filter(operation -> operation.id=="clear-cross-beam");
		check(clearances.length==1 && clearances[0].kind==materia.tooling.panel.PanelPlan.PanelOperationKind.Rebate,
			"Crossed dry carrier receives an explicit open rebate for the core bridge");
		check(Math.abs(clearances[0].depth-(d.recipe.panel.measuredThickness-16+d.recipe.jointClearance))<1e-7,
			"Carrier rebate depth follows actual support height plus the declared clearance");
		check(cross.panel.contactFace==materia.tooling.ToolingEnums.ContactFace.NegativeZ,
			"Crossing relief preserves the source insert contact face");
		crossing.operations[0].contour=materia.tooling.panel.PanelContours.circle(40,20,20);
		var nonRectangle="";
		try CoreSupports.insertCarrier(carrierPlan,"round-cross",carrierFace,
			{x:-20,y:60,z:90,width:40,depth:20,height:12}) catch (error:Dynamic) nonRectangle=Std.string(error);
		check(nonRectangle.indexOf("exact non-rectangular support intersection")>=0,
			"A curved support is never cleared with its bounding rectangle");
		var sidePlan=CastingMoldPlanner.planSinglePocket(d);
		var arm=CoreSupports.insertCarrier(sidePlan,"right-arm",F.translation(60,70,102),
			{x:40,y:60,z:90,width:40,depth:20,height:12});
		check(arm.pose.x==40-d.recipe.minimumLigament,
			"A source footprint wholly on one side gets a carrier arm to its adjacent wall");
		var armPads=sidePlan.formers.filter(former -> former.volume.id.indexOf("fixture-carrier-right-arm-")==0);
		check(armPads.length==1 && armPads[0].volume.pieces[0].x==sidePlan.cavity.x+sidePlan.cavity.width,
			"A carrier arm has one real wall-top pad, with no floating opposite support");
		var pattern = ToolingCodec.copy(d);
		var second = materia.casting.InsertHole.InsertHoleCodec.snapshot([hole])[0];
		pattern.input.inserts[0].holes[0].frame.x = -8;
		second.id = "second";
		second.frame.x = 8;
		pattern.input.inserts[0].holes.push(second);
		TestInsertHoles.beforeCasting(pattern.input.inserts[0]);
		pattern.setup.sourceFingerprint = ToolingCodec.inputFingerprint(pattern.input);
		var patterned = CastingMoldPlanner.planSinglePocket(pattern);
		check(patterned.fixtures[0].fixings.length == 2, "both source threads produce independent fixings");
		var shallow = materia.casting.InsertHole.InsertHoleCodec.snapshot([hole])[0];
		shallow.depth = materia.casting.InsertHole.InsertHoleCodec.threadLimits(shallow).minEngagement
			+ materia.casting.InsertHole.InsertHoleCodec.threadLimits(shallow).minTipGap + pattern.recipe.hardware.plugDepth - 0.1;
		check(materia.tooling.casting.InsertFixturePlanner.select(shallow,
			F.compose(pattern.setup.sourceToMold, pattern.input.inserts[0].pose), patterned.fixtures[0].mountingFace,
			pattern.recipe.hardware) == null, "screw and far-end plug cannot consume the same depth");
		check(MoldOpenings.records(patterned).filter(p -> p.insertId == "seat" && p.sourceHoleId != null).length == 2,
			"each insert penetration traces to its source hole");
		patterned.fixtures[0].fixings[0].engagement = 0.001;
		rejects(() -> CastingMoldPlanner.validate(patterned), "engagement must match the numeric source policy");
		var counterbored=ToolingCodec.copy(d);
		counterbored.input.inserts[0].holes[0].counterbore={diameter:9,depth:2};
		counterbored.setup.sourceFingerprint=ToolingCodec.inputFingerprint(counterbored.input);
		var counterboredPlan=CastingMoldPlanner.planSinglePocket(counterbored);
		var selected=materia.tooling.casting.InsertFixturePlanner.select(counterbored.input.inserts[0].holes[0],
			F.compose(counterbored.setup.sourceToMold,counterbored.input.inserts[0].pose),counterboredPlan.fixtures[0].mountingFace,counterbored.recipe.hardware);
		check(selected!=null && selected.engagement==6 && selected.insertion==8,
			"source fixing selection separates full-thread engagement from insertion through a counterbore");
		var noThread = ToolingCodec.copy(d);
		noThread.input.inserts[0].holes[0].thread = null;
		noThread.input.inserts[0].holes[0].tapDrill = null;
		noThread.input.inserts[0].holes[0].threadDepth = null;
		noThread.input.inserts[0].holes[0].limits.threaded = null;
		noThread.setup.sourceFingerprint = ToolingCodec.inputFingerprint(noThread.input);
		var blocked = CastingMoldPlanner.planSinglePocket(noThread);
		check(blocked.fixtures[0].fixings.length == 0 && blocked.changeRequests.filter(r -> r.kind == "insert-fixture-hole").length == 1
			&& !blocked.fabrication.ready, "plain bore never becomes an invented thread");
		var late=ToolingCodec.copy(d);
		switch late.input.inserts[0].origin {
			case Fabricated(manufacturing):manufacturing.holeRoutes[0].route=materia.casting.InsertManufacturing.InsertHoleRoute.TransferAfterCasting;
			case Catalogue(_):throw "Fabricated fixture expected";
		}
		late.setup.sourceFingerprint=ToolingCodec.inputFingerprint(late.input);
		var latePlan=CastingMoldPlanner.planSinglePocket(late);
		check(latePlan.fixtures[0].fixings.length==0 && latePlan.fixtures[0].boreMasks.length==0,
			"post-cast thread supplies neither a pour-time fixing nor a fictitious bore mask");
		check(latePlan.changeRequests.filter(r -> r.kind=="insert-fixture-hole").length==1,
			"a fixture needs a usable pre-cast source hole when its only service thread is late");
		var forgedSeal=CastingMoldPlanner.planSinglePocket(d);
		var sourcePenetration=MoldOpenings.records(forgedSeal).filter(pen -> pen.sourceHoleId==d.input.inserts[0].holes[0].id)[0];
		switch forgedSeal.document.input.inserts[0].origin {
			case Fabricated(manufacturing):manufacturing.holeRoutes[0].route=materia.casting.InsertManufacturing.InsertHoleRoute.TransferAfterCasting;
			case Catalogue(_):throw "Fabricated fixture expected";
		}
		var stageSealError="";
		try materia.tooling.casting.DatumFixtures.validatePanelSeal(forgedSeal,sourcePenetration) catch (error:Dynamic) stageSealError=Std.string(error);
		check(stageSealError.indexOf("Insert penetration has no unique source hole")>=0,
			"a service-only thread cannot qualify a pre-cast insert-sealed penetration");
		var undecided=ToolingCodec.copy(d);
		switch undecided.input.inserts[0].origin {
			case Fabricated(manufacturing):manufacturing.holeRoutes=[];
			case Catalogue(_):throw "Fabricated fixture expected";
		}
		undecided.setup.sourceFingerprint=ToolingCodec.inputFingerprint(undecided.input);
		var undecidedPlan=CastingMoldPlanner.planSinglePocket(undecided);
		check(undecidedPlan.fixtures[0].fixings.length==0 && undecidedPlan.fixtures[0].boreMasks.length==0
			&& undecidedPlan.validation.filter(v -> v.rule=="tooling.fixture.hole-stage."+d.input.inserts[0].id
				&& v.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending).length==1,
			"undecided manufacturing stage cannot become an existing fixture thread");
		check(undecidedPlan.changeRequests.filter(r -> r.kind=="insert-fixture-hole").length==0,
			"an undecided source hole route requests manufacturing planning rather than a casting change");
		var p = CastingMoldPlanner.planSinglePocket(d),initialFingerprint=CastingMoldPlanner.fingerprint(p);
		var staleManufacturing=CastingMoldPlanner.planSinglePocket(d);
		staleManufacturing.validation.filter(v -> v.rule=="fabrication.insert.inspection."+d.input.inserts[0].id)[0].status
			=materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass;
		var manufacturingError="";
		try CastingMoldPlanner.validate(staleManufacturing) catch (error:Dynamic) manufacturingError=Std.string(error);
		check(manufacturingError.indexOf("Insert manufacturing validation differs from source")>=0,
			"a planned inspection cannot be forged into physical inspection evidence");
		var stockRule="tooling.precision.allowance."+d.input.inserts[0].id;
		check(p.validation.filter(v -> v.rule==stockRule && v.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending).length==1
			&& p.changeRequests.filter(r -> r.id=="allowance-"+d.input.inserts[0].id).length==1,
			"single-pocket mold keeps an undeclared source finishing layer pending");
		var stocked=ToolingCodec.copy(d);
		stocked.input.inserts[0].interfaces=[{
			id:"ground-top",geometry:materia.casting.InterfaceRequirement.PrecisionGeometry.Plane(F.translation(0,0,14)),
			route:materia.casting.InterfaceRequirement.InterfaceRoute.Ground,allowanceMm:1,finishingProcessErrorMm:materia.casting.CastingInput.Unresolved("Finishing process error allocation"),
			permittedFinishes:[materia.casting.InterfaceRequirement.InterfaceFinish.Ground],
			positionToleranceMm:0.01,datumReference:"primary",relationshipIds:[]
		}];
		stocked.input.inserts[0].holes[0].frame.z=14;
		stocked.input.inserts[0].holes[0].depth=14;
		stocked.input.inserts[0].holes[0].threadDepth=14;
		stocked.setup.sourceFingerprint=ToolingCodec.inputFingerprint(stocked.input);
		var reserved=CastingMoldPlanner.planSinglePocket(stocked);
		check(reserved.validation.filter(v -> v.rule==stockRule && v.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass).length==1
			&& reserved.changeRequests.filter(r -> r.id=="allowance-"+stocked.input.inserts[0].id).length==0,
			"single-pocket mold uses the same source finishing-geometry decision");
		var fixing=p.fixtures[0].fixings[0];
		var screws=p.hardware.filter(item -> item.id==fixing.hardwareId);
		check(screws.length==1 && screws[0].kind==materia.tooling.casting.CoreSupport.SupportHardwareKind.FixtureScrew,
			"The shared receiver emits exactly one physical source-thread screw");
		var badLength=CastingMoldPlanner.planSinglePocket(d);
		badLength.hardware.filter(item -> item.id==fixing.hardwareId)[0].length+=1;
		rejects(() -> CastingMoldPlanner.validate(badLength),"A forged fixture screw stack is rejected");
		var missingCap=CastingMoldPlanner.planSinglePocket(d);
		missingCap.hardware=missingCap.hardware.filter(item -> item.kind!=materia.tooling.casting.CoreSupport.SupportHardwareKind.BorePlug);
		rejects(() -> CastingMoldPlanner.validate(missingCap),"A missing far-end source-thread plug is rejected");
		check(p.fixtures[0].location.report.rank==3,"direct bed contact leaves three insert freedoms");
		check(p.validation.filter(check -> check.rule=="tooling.fixture.location."+p.fixtures[0].insertId
			&& check.status=="pending" && check.message.indexOf("translation-x")>=0).length==1,
			"box fixture names its unconstrained lateral direction");
		var forged=CastingMoldPlanner.planSinglePocket(d),locationError="";
		forged.fixtures[0].location.report.rank=6;
		try CastingMoldPlanner.validate(forged) catch(error:Dynamic) locationError=Std.string(error);
		check(locationError.indexOf("Panel fixture location differs from source geometry")>=0,
			"a forged fully constrained box fixture is rejected");
		check(p.panels.members.length == 5 && p.openings.length == 9, "all sockets/lands and fixing hole classified");
		check(p.sourceFingerprint == ToolingCodec.documentFingerprint(d), "mold provenance");
		var forgedSolid=CastingMoldPlanner.planSinglePocket(d),solidError="";
		forgedSolid.cores[0].layers[0].solid=materia.tooling.casting.MoldRegions.box({x:0,y:0,z:0,width:1,depth:1,height:1});
		try CastingMoldPlanner.validate(forgedSolid) catch (error:Dynamic) solidError=Std.string(error);
		check(solidError.indexOf("Invalid ruled core cutting definition")>=0,
			"finished cutting solid cannot change behind its source foam geometry");
		var missingSolid=CastingMoldPlanner.planSinglePocket(d);
		Reflect.setField(missingSolid.cores[0].layers[0],"solid",null);
		rejects(() -> CastingMoldPlanner.validate(missingSolid),"Invalid ruled core cutting definition");
		p.cores[0].layers[0].wireDirection = {x: 1, y: 1, z: 0};
		rejects(() -> CastingMoldPlanner.validate(p), "unsupported wire direction rejected");
		p = CastingMoldPlanner.planSinglePocket(d);
		p.openings.pop();
		rejects(() -> CastingMoldPlanner.validate(p), "missing penetration classification");
		var panel:Dynamic = Json.parse(PanelCodec.encode(p.panels.members[0].panel));
		Reflect.setField(panel, "version", 1);
		rejects(() -> PanelCodec.decode(Json.stringify(panel)), "old panel encoding rejected");
		panel = Json.parse(PanelCodec.encode(p.panels.members[0].panel));
		Reflect.setField(Reflect.field(panel, "panel"), "contactOpenings", null);
		rejects(() -> PanelCodec.decode(Json.stringify(panel)), "null opening list rejected");
		d.recipe.panel.measuredThickness = 18.2;
		check(CastingMoldPlanner.fingerprint(CastingMoldPlanner.planSinglePocket(d)) != initialFingerprint, "stock changes invalidate package");
		var bridge = CastingMoldPlanner.planSinglePocket(d);
		check(bridge.cores[0].support.kind == CoreSupportKind.Bridge
			&& bridge.supportPanels.length == 4, "suspended core receives carrier and pinned bridge");
		var wrongBed = CastingMoldPlanner.planSinglePocket(d);
		wrongBed.cores[0].support.kind = CoreSupportKind.RoutedSocket;
		rejects(() -> CastingMoldPlanner.validate(wrongBed), "bed socket cannot alter a solid casting floor");
		var bedDoc = ToolingCodec.copy(d);
		bedDoc.input.features.filter(f -> f.kind != "insert-recess")[0].geometry.pose.z = 40;
        bedDoc.input.features.filter(f -> f.kind != "insert-recess")[0].openings=[materia.casting.CastingPatterns.face("body.z-max")];
		bedDoc.setup.sourceFingerprint = ToolingCodec.inputFingerprint(bedDoc.input);
		var bed = CastingMoldPlanner.planSinglePocket(bedDoc), core = bed.cores[0];
		check(core.support.kind == CoreSupportKind.RoutedSocket && bed.supportPanels.length == 0, "bed opening selects routed socket");
		check(bed.openings.length == 10 && core.support.foot != null, "bed socket classified and separate foot defined");
		var foot = core.support.foot;
		if (foot == null)
			throw "Missing foot";
		check(Math.abs(MoldCores.singleBox(core.foamEnvelope,core.id).z - bed.cavity.z) < 1e-7 && foot.foam.z < bed.cavity.z, "main core height preserved and foot buried");
		var wrongBridge = CastingMoldPlanner.planSinglePocket(bedDoc);
		wrongBridge.cores[0].support.kind = CoreSupportKind.Bridge;
		rejects(() -> CastingMoldPlanner.validate(wrongBridge), "bridge cannot pass a closed top");
		var escaped = CastingMoldPlanner.planSinglePocket(bedDoc);
		var wide = ToolingCodec.copy(bedDoc).recipe;
		wide.jointClearance = 30;
		rejects(() -> CoreSupports.build(escaped.cores[0], escaped.cavity, wide, escaped.panels.members[0], [], []),
			"socket clearance cannot escape core sealing land");
		var noAdhesive = ToolingCodec.copy(bedDoc);
		noAdhesive.recipe.adhesiveLineThickness = 0;
		rejects(() -> CastingMoldPlanner.planSinglePocket(noAdhesive), "zero adhesive thickness rejected");
		foot.cutting.wireDirection = {x: 1, y: 0, z: 0};
		rejects(() -> CastingMoldPlanner.validate(bed), "foot cutting direction validated");
		var constrained = ToolingCodec.copy(d);
		constrained.recipe.jointClearance = 0.1;
		rejects(() -> ToolingCodec.copy(constrained), "clearance must include stock variation and allowances");
		var sealPlan = CastingMoldPlanner.planSinglePocket(d);
		MoldOpenings.sourceProtection(sealPlan,"insert-fixing-fixing").sealMargin = 100;
		rejects(() -> CastingMoldPlanner.validate(sealPlan), "seal margin rejected before native CAD");
		for (axis in [
			materia.tooling.ToolingRecipe.CoreWireAxis.X,
			materia.tooling.ToolingRecipe.CoreWireAxis.Y,
			materia.tooling.ToolingRecipe.CoreWireAxis.Z
		]) {
			var direction = ToolingCodec.copy(d);
			direction.recipe.coreWireAxis = axis;
			var cut = CastingMoldPlanner.planSinglePocket(direction).cores[0].layers[0];
			check(cut.wireDirection.x * cut.wireDirection.x + cut.wireDirection.y * cut.wireDirection.y + cut.wireDirection.z * cut.wireDirection.z == 1,
				"declared wire axis and faces");
		}
		var circular=materia.casting.CastingProfiles.circle(7,-3,8);
		var prism=CoreSupports.prismCutting(circular,20,F.translation(3,4,5),materia.tooling.ToolingRecipe.CoreWireAxis.X);
		check(prism.block.width==20 && prism.block.depth==16 && prism.block.height==16 && prism.wireDirection.x==1,
			"a curved prism aligns its extrusion with the declared stock wire axis");
		check(materia.tooling.casting.MoldRegions.boxes(prism.solid)==null,
			"the rectangular blank never substitutes for curved foam material");
		var before=ToolingCodec.canonical(prism);
		circular[0].start.x+=1;
		check(ToolingCodec.canonical(prism)==before,"cutting records own their finished profiles");
		var depthError="";
		try CoreSupports.prismCutting(materia.casting.CastingProfiles.circle(0,0,8),0,F.identity(),
			materia.tooling.ToolingRecipe.CoreWireAxis.Z) catch (error:Dynamic) depthError=Std.string(error);
		check(depthError.indexOf("Invalid core cutting depth")>=0,"zero-depth prism stock is rejected before CAD");
		check(!p.fabrication.ready && p.validation.length > 0, "readiness separate from successful planning");
		check(p.changeRequests.filter(x -> x.kind == "lifting-points").length == 1, "inverted box handling requires lifting points");
		var longId = "";
		for (i in 0...128)
			longId += "a";
		var derived = materia.tooling.ToolingIds.derive("core-", longId, "-socket");
		materia.tooling.panel.PanelCodec.identifier(derived);
		check(derived.length <= 128, "derived source IDs always satisfy shared vocabulary");
		var oversized = ToolingCodec.copy(d);
		oversized.recipe.hardware.locatingPinDiameter = 4;
		rejects(() -> CastingMoldPlanner.planSinglePocket(oversized), "support obround must fit the selected router");
		var padPlan = CastingMoldPlanner.planSinglePocket(d);
		var pad = padPlan.supportPanels.filter(x -> StringTools.endsWith(x.panel.id, "-north-pad"))[0];
		pad.pose.y = MoldCores.singleBox(padPlan.cores[0].foamEnvelope,padPlan.cores[0].id).y;
		rejects(() -> CastingMoldPlanner.validate(padPlan), "carrier pad overlap rejected by neutral planner");
		Sys.println('BoxPlannerTests: $assertions assertions passed');
	}
}
