import materia.assembly.AssemblyFrames as F;
import materia.tooling.process.CastabilityAnalysis.CastabilityAction;
import materia.casting.InsertSpec;
import materia.casting.InsertAnchorage;
import materia.casting.InsertAnchorage.InsertAnchorageCodec as C;
import machinekit.structural.CastInsert;
import machinekit.structural.InsertHoles;
import machinekit.structural.InsertAnchorHardware;

/** Synthetic anchor choices exercise rules, not a production fastening pattern. */
class InsertAnchorTests {
	static var assertions=0;
	static function check(ok:Bool,message:String):Void {assertions++;if (!ok) throw message;}
	static function rejects(fn:Void->Void,expected:String):Void {
		var message="";try fn() catch (error:Dynamic) message=Std.string(error);
		check(message.indexOf(expected)>=0,'Expected "$expected", got "$message"');
	}
	static function bolts(spec:InsertSpec):Array<InsertBoltAnchor> return switch spec.anchorage {
		case Bolted(_,anchors):anchors;default:throw "Expected bolted source";
	};
	public static function fixture():CastInsert {
		var thread=InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(4).mating());
		var holes=new Array<materia.casting.InsertHole>();
		for (x in [-15.0,15.0]) {
			var hole=InsertHoles.records(InsertHoles.tappedBlind(thread,20))[0];
			hole.id=x<0 ? "back-left" : "back-right";hole.frame=F.translation(x,0,0);holes.push(hole);
		}
		var front=InsertHoles.records(InsertHoles.tappedBlind(InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(6).mating()),20))[0];
		front.id="mounting";holes.push(front);
		var values=CastInsert.precastHoles(CastInsert.boxValues(60,50,20),InsertHoles.fromNeutral(holes))
			.setToken("role",materia.casting.InsertSpec.InsertRole.GenericMount);
		var base=new CastInsert(values),spec=base.insertSpec();
		var anchorage=InsertAnchorHardware.choose(spec,
			{minimumEmbedmentMm:10,minimumBearingAreaMm2:20,minimumSpacingMm:5,minimumCoverMm:8},
			[for (hole in holes) if (hole.id!="mounting") {id:hole.id,holeId:hole.id,lengthMm:25,
				insertionMm:materia.casting.InsertHole.InsertHoleCodec.threadLimits(hole).minEngagement+0.5,
				locking:InsertAnchorLock.RetainingCompound,lockingReference:null}]);
		values.setToken("anchorage",haxe.Json.stringify(C.record(anchorage)));return new CastInsert(values);
	}
	static function foundationGeometry(source:CastInsert):Void {
		var foundation=new machinekit.structural.MineralFoundation(new machinekit.component.ComponentValues()
			.setNumber("width",120).setNumber("depth",100).setNumber("height",80),
			[{id:"seat",part:source,pose:F.translation(0,0,60),openings:[materia.casting.CastingPatterns.face("body.z-max")]}]);
		var spec=source.insertSpec(),anchors=bolts(spec),bill=foundation.billOfMaterials();
		check(bill.quantity(anchors[0].screw.partNumber)==2 && bill.quantity(anchors[0].washer.partNumber)==2,
			"foundation BOM includes real anchor screws and washers once each");
		var context=machinekit.tooling.FoundationAssemblyContext.snapshot(foundation,"casting",[{id:"seat",member:"seat"}]);
		var input=machinekit.tooling.FoundationToolingAdapter.snapshot(foundation,"anchored-foundation",1,context);
		check(input.inserts.length==1
			&& materia.tooling.ToolingCodec.canonical(input.inserts[0].anchorage)==materia.tooling.ToolingCodec.canonical(spec.anchorage),"adapter copies anchor source, not extra dummy inserts");
		var native=machinekit.tooling.InsertAnchorGeometry.review(input);
		check(native.length==6 && native.filter(record -> record.status!=materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass).length==0,
			"both actual anchor washers bear on resin and have their declared source casting cover");
		var sourceRules=switch input.inserts[0].anchorage {case Bolted(rules,_):rules;default:throw "Expected bolted source";};
		sourceRules.minimumCoverMm=50;
		check(machinekit.tooling.InsertAnchorGeometry.review(input).filter(record -> StringTools.endsWith(record.rule,".cover")
			&& record.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Fail && record.affectedIds[0]=="seat").length==2,
			"casting cover is checked against native boundary faces and reports the insert owner");
		sourceRules.minimumCoverMm=8;sourceRules.minimumBearingAreaMm2=100;
		check(machinekit.tooling.InsertAnchorGeometry.review(input).filter(record -> StringTools.endsWith(record.rule,".bearing-area")
			&& record.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Fail).length==2,
			"bearing area cannot be supplied by nominal washer diameter alone");
		sourceRules.minimumBearingAreaMm2=20;
		var badInput=materia.tooling.ToolingCodec.inputSnapshot(input);badInput.inserts[0].holes[0].depth+=0.1;
		rejects(() -> materia.tooling.ToolingCodec.inputSnapshot(badInput),"Invalid insert seat: Anchor");
		check(foundation.body.collisionHullQualification.status==machinekit.structural.CastingCollisionQualification.CastingCollisionStatus.Unsupported,
			"anchor voids are not approximated by box collision hulls");
		rejects(() -> foundation.body.minimumSectionIy(-20,20),"Unsupported non-box anchor material for inserts: seat");
		check(materia.tooling.process.FillAnalysis.sourceSupport(input).filter(c -> c.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported
			&& c.affectedIds.indexOf("seat")>=0).length==1,"fill box consumers report the anchor owner explicitly");
		var document=SinglePocketSetup.document();document.input=input;
		document.setup.sourceFingerprint=materia.tooling.ToolingCodec.inputFingerprint(input);
		document.setup.fillHeightMm=80;document.setup.features=[];
		document.setup.sourceToMold=F.fromRotationMatrix(0,0,80,[1,0,0,0,-1,0,0,0,-1]);
		var options:materia.tooling.casting.CastingMoldPlan.CastingMoldOptions={replicationJigs:[],foundationJointProcesses:[],
			precision:[{insertId:"seat",masterReference:null,jigReference:null,masterFlatnessMm:null,releaseSpec:null,holeProtectionReference:null,evidence:[]}],
			compaction:{method:materia.tooling.casting.CastingMoldPlan.CompactionMethod.Undeclared,bedInterfaceReference:null,
				frequencyHz:null,amplitudeMm:null,durationSeconds:null,evidence:[]},
			serviceOrientation:F.identity(),overlapRule:materia.tooling.casting.CastingMoldPlan.OverlapRule.ReservedThenPriority,liftingEvidence:[]};
		var sourceReview=machinekit.structural.FoundationIntegrity.castingReview(foundation,document,context);
		check(sourceReview.validation.filter(record -> record.rule.indexOf(".cad.")>=0 && record.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass).length==6,
			"native anchor source geometry reaches foundation design diagnostics");
		var capability=sourceReview.validation.filter(record -> record.rule=="process.level1.geometry.seat");
		check(capability.length==1 && capability[0].status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported,
			"one source geometry capability finding covers anchor occupancy without duplicating bearing/capacity qualifications");
		var sourceOnly=materia.tooling.ToolingCodec.snapshot(document);
		sourceOnly.recipe.designReview.minimumLiftingPoints=4;
		var previous=materia.tooling.casting.CastingMoldPlanner.geometry;
		materia.tooling.casting.CastingMoldPlanner.setGeometry(null);
		try {
			var screening=materia.tooling.process.CastingLevelOne.analyze(sourceOnly);
			check(screening.proposals.filter(proposal -> proposal.action==CastabilityAction.DeclareLiftingPoint).length==4,
				"box-plus-anchor source moments still produce lifting review positions without a native fill kernel");
			check(screening.validation.filter(record -> record.rule=="process.level1.geometry.seat"
				&& record.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported).length==1,
				"exact source moments do not falsely qualify unsupported box-only anchor filling or sweeps");
		} catch (error:Dynamic) {
			materia.tooling.casting.CastingMoldPlanner.setGeometry(previous);
			throw error;
		}
		materia.tooling.casting.CastingMoldPlanner.setGeometry(previous);
		var assessment=materia.tooling.casting.CastingMoldPlanner.assess(document,options);
		check(assessment.plan!=null && !assessment.fabrication.ready,
			"panel mold planning retains exact native anchors while non-box fill remains a step-3 requirement");
		var interference=machinekit.tooling.InsertAnchorGeometry.reviewMold(cast assessment.plan);
		check(interference.length==2 && interference.filter(record -> record.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Fail).length==0,
			"source anchor pairs clear every actual mold member");
		var processSpec=ProcessTests.spec();processSpec.sections=[];
		processSpec.recipeFingerprint=materia.tooling.ToolingCodec.recipeFingerprint(document.recipe);
		var process=materia.tooling.process.CastingProcessAnalysis.analyze(cast assessment.plan,processSpec,9.80665);
		check(process.fill==null && process.sections==null && process.pour==null && !process.fabrication.ready,
			"non-box source keeps unavailable process reports explicit without a box substitute or aborting the kit");
		check(process.validation.filter(record -> record.rule=="process.cure.section-coverage"
			&& record.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported).length==1,
			"non-box section coverage cannot become a thermal pass");
		var nativeProcess=materia.tooling.process.CastingProcessAnalysis.analyze(cast assessment.plan,processSpec,9.80665,null,
			machinekit.tooling.CastingFillAnalysis.fromPlan);
		check(nativeProcess.fill!=null && nativeProcess.pour!=null && nativeProcess.sections==null,
			"native fill enters the shared process and pour pipeline while orthogonal sections remain unsupported");
		var anchorFill:materia.tooling.process.FillAnalysis.FillReport=cast nativeProcess.fill;
		check(anchorFill.trappedPockets.length==anchors.length,"each downward-open anchor washer retains its real air trap");
		for (anchor in anchors) {
			var members=machinekit.tooling.CastingMoldCad.anchorComponents(cast assessment.plan,"seat")
				.filter(member -> member.anchorId==anchor.id);
			var pockets=anchorFill.trappedPockets.filter(pocket -> pocket.sourceIds.indexOf("seat")>=0
				&& pocket.sourceIds.indexOf(anchor.id)>=0);
			check(pockets.length==1 && members.length==2
				&& members.filter(member -> pockets[0].sourceIds.indexOf(member.id)>=0).length==2,
				"planned fill names the insert, its source anchor and both bounding hardware members");
			var proposals=nativeProcess.castability.proposals.filter(proposal -> proposal.rule=="process.fill.trap."+pockets[0].id);
			check(proposals.length==1 && proposals[0].action==CastabilityAction.RouteVent
				&& proposals[0].sourceIds.join(",")==pockets[0].sourceIds.join(",")
				&& proposals[0].instruction.indexOf("pre-pour fill/seal")>=0,
				"actual anchor and insert IDs reach the vent or explicitly modelled seal proposal");
		}
		check(nativeProcess.validation.filter(record -> record.rule=="process.fill.geometry"
			&& record.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass).length==1,
			"native source geometry replaces the unavailable fill projection");
		check(nativeProcess.cure.validation.filter(record -> record.rule=="process.cure.section-coverage"
			&& record.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported).length==1,
			"exact fill cannot clear unsupported thermal coverage");
		var processDirectory=Sys.getCwd()+"/build/tooling/anchor-process-"+nativeProcess.sourceFingerprint.substr(0,12);
		machinekit.tooling.ToolingFiles.directory(processDirectory);
		var exportChecks=machinekit.tooling.ToolingProcessExport.write(cast assessment.plan,processSpec,9.80665,
			processDirectory,null,null,foundation,nativeProcess,context);
		check(exportChecks.filter(record -> record.rule=="process.fill.cad-volume-agreement"
			&& record.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass).length==1,
			"exact fill export checks the actual foundation CAD independently of a swept-graph review");
		var exportedFill:Dynamic=haxe.Json.parse(sys.io.File.getContent(processDirectory+"/fill-analysis.json"));
		check(exportedFill!=null && exportedFill.model=="exact-brep-flat-free-surface",
			"export retains the available native fill report when section geometry is unsupported");
		check(haxe.Json.parse(sys.io.File.getContent(processDirectory+"/pour-plan.json"))!=null,
			"export retains native fill quantities and pending mixer planning");
		check(haxe.Json.parse(sys.io.File.getContent(processDirectory+"/section-geometry.json"))==null,
			"unimplemented non-box orthogonal sections remain explicitly unavailable");
		var changedSpec=materia.tooling.process.ProcessSpecCodec.snapshot(processSpec);
		changedSpec.pourRateMm3PerSecond=12345;
		rejects(() -> machinekit.tooling.ToolingProcessExport.write(cast assessment.plan,changedSpec,9.80665,
			processDirectory,null,null,foundation,nativeProcess,context),"Stale process export report");
		rejects(() -> machinekit.tooling.ToolingProcessExport.write(cast assessment.plan,processSpec,9.80665,
			processDirectory,null,null,foundation,nativeProcess),"Process CAD foundation differs from source revision");
		var layeredDocument=SinglePocketSetup.document();layeredDocument.recipe.core.height=15;
		// This synthetic bridge fits the 80 mm pocket; the library minimum is not weakened.
		layeredDocument.recipe.hardware.minimumBridgeWidth=40;
		var layered=materia.tooling.casting.CastingMoldPlanner.plan(layeredDocument,options),core=layered.cores[0];
		check(core.layers.length>1 && materia.tooling.casting.MoldRegions.volume(materia.tooling.casting.MoldCores.foam(core),core.id)
			< materia.tooling.casting.MoldRegions.volume(core.cavity,core.id),"uplift fixture really has lined and glued stock layers");
		var layeredSettings:materia.tooling.process.FillAnalysis.FillSourceSettings={
			sourceFingerprint:materia.tooling.casting.CastingMoldCodec.fingerprint(layered),processFingerprint:"layered-core",
			cores:[],densityKgM3:2400,gravityMps2:9.80665,pourRateMm3PerSecond:null,maximumAirEscapeWidthMm:null,
			extraLevels:[],fillObstructions:[],panels:[]};
		var layeredFill=machinekit.tooling.CastingFillAnalysis.fromPlan(layered,layeredSettings).report;
		check(layeredFill!=null,"layered core fill has an exact native report");
		var layeredReport:materia.tooling.process.FillAnalysis.FillReport=cast layeredFill;
		var finalLevel=layeredReport.levels[layeredReport.levels.length-1],coreLift=finalLevel.cores.filter(row -> row.id==core.id)[0];
		check(Math.abs(coreLift.submergedVolumeMm3-materia.tooling.casting.MoldRegions.volume(core.cavity,core.id))<1e-5,
			"native gross uplift includes lining and adhesive instead of only the separated foam stock");
		var restored=machinekit.assembly.MachineAssembly.decode(foundation.encode());
		check(restored.billOfMaterials().quantity(anchors[0].screw.partNumber)==2,"source assembly round trip preserves actual anchor hardware");
		var owned:Array<cadkit.modeling.Part> = [];
		try {
			var mold:materia.tooling.casting.CastingMoldPlan=cast assessment.plan;
			var resin=machinekit.tooling.CastingMoldCad.source(mold);owned.push(resin);
			check(Math.abs(materia.tooling.casting.CastHandling.resinVolume(mold.document)-resin.volume())<1e-6,
				"effort resin volume subtracts real exposed anchors from the shared source cavities");
			var effort=materia.tooling.process.MoldOrientationStudy.metrics(mold,FabricationTests.spec());
			check(Math.abs(effort.mixMassKg-resin.volume()*mold.document.recipe.mixDensityKgM3*1e-9)<1e-6,
				"anchored orientation mix mass agrees with actual foundation CAD");
			var reconstructed=machinekit.tooling.CastingMoldCad.reconstructed(mold);owned.push(reconstructed);
			var missing=resin.subtract(reconstructed);owned.push(missing);
			var added=reconstructed.subtract(resin);owned.push(added);
			check(missing.volume()+added.volume()<1e-6,"panel reconstruction includes exact anchor occupancy");
			var metal=machinekit.tooling.CastingMoldCad.steel(mold,"seat");owned.push(metal);
			var nominalMass=(resin.volume()*mold.document.recipe.mixDensityKgM3+metal.volume()*mold.document.recipe.steelDensityKgM3)*1e-9;
			check(Math.abs(nominalMass-mold.handling.castingMassKg)<1e-6,"source anchor moments agree with exact nominal metal/resin mass");
			var actual=foundation.body.geometry();owned.push(actual);
			var baseline=new machinekit.structural.MineralCasting(foundation.body.values().copy().setToken("embeddedAnchors","[]")).geometry();
			owned.push(baseline);
			check(baseline.volume()-actual.volume()>100,"embedded hardware removes real resin volume beyond the insert recess");
			var detached=machinekit.tooling.ToolingSourceCasting.fromInput(input).geometry();owned.push(detached);
			var a=actual.subtractAll([detached]);owned.push(a);var b=detached.subtractAll([actual]);owned.push(b);
			check(a.volume()+b.volume()<1e-6,"detached source resin matches foundation CAD by symmetric difference with real anchor voids");
			var cache=new machinekit.assembly.PosedParts();
			try {
				for (hardware in InsertAnchorHardware.parts(spec)) {
					var part=cache.posed(hardware.part,F.compose(F.translation(0,0,60),hardware.pose));owned.push(part);
					part.shape.setThreadedFaces([]);
					var common=new cadkit.modeling.Part(actual.shape.common(part.shape));owned.push(common);
					check(common.volume()<1e-6,"casting resin excludes actual "+hardware.kind+" hardware");
				}
			} catch (error:Dynamic) {cache.close();throw error;}
			cache.close();
		} catch (error:Dynamic) {for (part in owned) part.close();throw error;}
		for (part in owned) part.close();
	}

	static function sourcePattern():Void {
		var values=CastInsert.boxValues(60,50,20).setToken("role",materia.casting.InsertSpec.InsertRole.GenericMount)
			.setToken("holes",InsertHoles.tappedBlind(InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(6).mating()),20));
		var source=new CastInsert(values),origin=source.insertSpec().origin;
		switch origin {case Fabricated(plan):plan.holeRoutes=[{holeId:"fixing",route:materia.casting.InsertManufacturing.InsertHoleRoute.TransferAfterCasting}];default:throw "Expected fabricated source";}
		values.setToken("origin",haxe.Json.stringify(materia.casting.InsertOrigin.InsertOriginCodec.record(origin)));
		source=new CastInsert(values);
		var rules:materia.casting.InsertAnchorage.InsertAnchorRules={minimumEmbedmentMm:10,minimumBearingAreaMm2:20,minimumSpacingMm:5,minimumCoverMm:8};
		var points:Array<machinekit.structural.InsertAnchorHardware.InsertBackAnchorChoice> = [for (x in [-15.0,15.0]) {
			id:x<0 ? "back-left" : "back-right",xMm:x,yMm:0,lengthMm:25,insertionMm:5.7,
			locking:InsertAnchorLock.RetainingCompound,lockingReference:null}];
		var anchored=InsertAnchorHardware.backPattern(source,machinekit.standard.FastenerThreads.coarse(4).mating(),rules,points);
		check(source.insertSpec().holes.length==1 && anchored.insertSpec().holes.length==3,"source back pattern owns its new holes without mutating the input insert");
		var plan=switch anchored.insertSpec().origin {case Fabricated(plan):plan;default:throw "Expected source plan";};
		check(plan.holeRoutes.filter(route -> route.holeId=="fixing" && route.route==materia.casting.InsertManufacturing.InsertHoleRoute.TransferAfterCasting).length==1,
			"adding anchors never changes a bought-part mounting-hole route");
		check(plan.holeRoutes.filter(route -> route.route==materia.casting.InsertManufacturing.InsertHoleRoute.BeforeCasting).length==2,
			"only the new source anchor holes are declared pre-cast");
		check(InsertAnchorHardware.parts(anchored.insertSpec()).length==4,"source pattern selects actual standard bolts and washers");
		rejects(() -> InsertAnchorHardware.backPattern(anchored,machinekit.standard.FastenerThreads.coarse(4).mating(),rules,points),"already declared");
		points[0].id="fixing";
		rejects(() -> InsertAnchorHardware.backPattern(source,machinekit.standard.FastenerThreads.coarse(4).mating(),rules,points),"Duplicate source anchor hole");
	}

	public static function main():Void run();

	public static function run():Void {
		sourcePattern();
		var source=fixture(),spec=source.insertSpec();
		var colliding=InsertSpecCodec.snapshot(InsertSpecCodec.record(spec));
		colliding.holes.filter(hole -> hole.id=="mounting")[0].frame.x=14;
		rejects(() -> InsertAnchorHardware.parts(colliding),"source hole mounting clearance");
		var collisionValues=source.values().copy().setToken("holes",InsertHoles.fromNeutral(colliding.holes));
		rejects(() -> new machinekit.structural.MineralFoundation(new machinekit.component.ComponentValues()
			.setNumber("width",120).setNumber("depth",100).setNumber("height",80),
			[{id:"seat",part:new CastInsert(collisionValues),pose:F.translation(0,0,60),
				openings:[materia.casting.CastingPatterns.face("body.z-max")]}]),"Insert seat: Anchor");
		var encoded=haxe.Json.stringify(C.record(spec.anchorage));
		check(haxe.Json.stringify(C.record(C.snapshot(haxe.Json.parse(encoded))))==encoded,"strict source anchor round trip");
		var copy=InsertSpecCodec.snapshot(InsertSpecCodec.record(spec));
		bolts(copy).reverse();
		check(haxe.Json.stringify(C.record(C.snapshot(copy.anchorage)))==encoded,"anchor source order is canonical");
		bolts(copy)[0].screw.headHeightMm+=1;
		check(haxe.Json.stringify(C.record(spec.anchorage))==encoded,"source hardware facts are defensive copies");
		rejects(() -> InsertAnchorHardware.parts(copy),"Anchor hardware differs from selected standard components");
		copy=InsertSpecCodec.snapshot(InsertSpecCodec.record(spec));bolts(copy)[0].screw.socketDepthMm+=0.1;
		rejects(() -> InsertAnchorHardware.parts(copy),"Anchor hardware differs from selected standard components");
		var parts=InsertAnchorHardware.parts(spec);
		var screwGeometry=parts[0].part.geometry();
		try check(screwGeometry.shape.threadedFaces().length==1,"real anchor component retains its external thread metadata")
		catch (error:Dynamic) {screwGeometry.close();throw error;}
		screwGeometry.close();
		check(parts.length==4,"two real screws and two real washers");
		check(parts.filter(p -> Std.isOfType(p.part,machinekit.standard.SocketHeadCapScrew)).length==2
			&& parts.filter(p -> Std.isOfType(p.part,machinekit.standard.FlatWasher)).length==2,"anchors use standard BOM components");
		for (anchor in bolts(spec)) {
			var screw=parts.filter(p -> p.anchorId==anchor.id && p.kind==InsertAnchorComponentKind.Bolt)[0];
			var tip=F.transformPoint(screw.pose,0,0,-anchor.screw.lengthMm);
			var hole=spec.holes.filter(h -> h.id==anchor.holeId)[0];
			check(Math.abs(tip.z-anchor.insertionMm)<1e-8 && Math.abs(tip.x-hole.frame.x)<1e-8,
				"actual screw tip has its declared insertion in the source hole");
			var washer=parts.filter(p -> p.anchorId==anchor.id && p.kind==InsertAnchorComponentKind.Washer)[0];
			check(Math.abs(washer.pose.z-(anchor.insertionMm-anchor.screw.lengthMm))<1e-8,"washer starts on the real screw bearing plane");
		}
		copy=InsertSpecCodec.snapshot(InsertSpecCodec.record(spec));bolts(copy)[0].insertionMm=1;
		rejects(() -> InsertAnchorHardware.parts(copy),"Anchor source thread fit fails");
		copy=InsertSpecCodec.snapshot(InsertSpecCodec.record(spec));
		copy.interfaces=[{id:"back-stock",geometry:materia.casting.InterfaceRequirement.PrecisionGeometry.Plane(
			F.fromRotationMatrix(0,0,1,[1,0,0,0,-1,0,0,0,-1])),route:materia.casting.InterfaceRequirement.InterfaceRoute.Ground,
			allowanceMm:1,finishingProcessErrorMm:null,permittedFinishes:[materia.casting.InterfaceRequirement.InterfaceFinish.Ground],
			positionToleranceMm:0.01,datumReference:"primary",relationshipIds:[]}];
		for (hole in copy.holes) {
			hole.depth-=1;if (hole.threadDepth!=null) hole.threadDepth=hole.threadDepth-1;
			if (hole.id!="mounting") hole.frame.z=1;
		}
		var stocked=InsertAnchorHardware.parts(copy),first=stocked[0],anchor=bolts(copy)[0];
		var stockedTip=F.transformPoint(first.pose,0,0,-anchor.screw.lengthMm);
		check(Math.abs(stockedTip.z-1-anchor.insertionMm)<1e-8,"disposable back stock never counts toward permanent source thread engagement");
		var rules=switch copy.anchorage {case Bolted(r,_):r;default:throw "Expected bolt rules";};
		copy.anchorage=InsertAnchorHardware.choose(copy,rules,[for (a in bolts(copy)) {id:a.id,holeId:a.holeId,lengthMm:12,
			insertionMm:12-a.washer.thicknessMm-0.5,locking:InsertAnchorLock.RetainingCompound,lockingReference:null}]);
		var occupied=bolts(copy)[0],serviceHole=copy.holes.filter(h -> h.id==occupied.holeId)[0];
		var rawHole=materia.casting.InsertStages.holes(copy,materia.casting.InsertStages.InsertStage.BeforeCasting)
			.filter(h -> h.id==occupied.holeId)[0];
		var bearing=C.bearingFrame(serviceHole,occupied);
		var fit=materia.casting.ThreadedFastenerFit.checkIntervals(serviceHole,serviceHole.frame,
			{shankLength:occupied.screw.lengthMm,threadedLength:occupied.screw.threadedLengthMm,bearingFrame:bearing},occupied.id)
			.concat(materia.casting.ThreadedFastenerFit.checkIntervals(rawHole,rawHole.frame,
				{shankLength:occupied.screw.lengthMm,threadedLength:occupied.screw.threadedLengthMm,bearingFrame:bearing},occupied.id));
		check(fit.filter(f -> !f.passed).length==0,"raw-stock collision control passes every source and pre-cast thread-fit rule");
		rejects(() -> InsertAnchorHardware.parts(copy),"Anchor washer intersects pre-cast steel");
		copy=InsertSpecCodec.snapshot(InsertSpecCodec.record(spec));bolts(copy)[0].holeId="missing";
		rejects(() -> InsertSpecCodec.snapshot(InsertSpecCodec.record(copy)),"Unknown anchor source hole");
		copy=InsertSpecCodec.snapshot(InsertSpecCodec.record(spec));copy.holes[0].extent=materia.casting.InsertHole.HoleExtent.Through;
		rejects(() -> C.references(copy.anchorage,copy.body,copy.holes,copy.origin,copy.interfaces),"Anchor requires a blind source tap");
		copy=InsertSpecCodec.snapshot(InsertSpecCodec.record(spec));copy.holes[0].frame=F.translation(-15,0,20);
		rejects(() -> C.references(copy.anchorage,copy.body,copy.holes,copy.origin,copy.interfaces),"back face");
		copy=InsertSpecCodec.snapshot(InsertSpecCodec.record(spec));copy.holes[0].depth+=0.1;
		rejects(() -> InsertSpecCodec.snapshot(InsertSpecCodec.record(copy)),"back-wall rule");
		copy=InsertSpecCodec.snapshot(InsertSpecCodec.record(spec));copy.holes[0].frame.x=29;
		rejects(() -> InsertSpecCodec.snapshot(InsertSpecCodec.record(copy)),"radial-wall rule");
		copy=InsertSpecCodec.snapshot(InsertSpecCodec.record(spec));bolts(copy)[0].locking=InsertAnchorLock.Shoulder;
		check(switch C.snapshot(copy.anchorage) {case Bolted(_,a):a[0].locking==InsertAnchorLock.Shoulder;default:false;},
			"shoulder intent never invents a washer-to-insert clamp or a thread-runout stop");
		rejects(() -> C.snapshot({kind:"bolted",rules:{minimumEmbedmentMm:1,minimumBearingAreaMm2:1,minimumSpacingMm:1,minimumCoverMm:1},anchors:new Array<Dynamic>()}),"requires 1..256 anchors");
		var old:Dynamic=InsertSpecCodec.record(spec);Reflect.deleteField(old,"anchorage");
		rejects(() -> InsertSpecCodec.snapshot(old),"Unexpected casting record fields");
		var input:materia.tooling.casting.CastingToolingInput.CastingToolingInsert={id:"seat",body:spec.body,holes:spec.holes,
			origin:spec.origin,anchorage:spec.anchorage,interfaces:spec.interfaces,material:source.insertMaterial(),pose:F.identity(),
			role:materia.casting.InsertSpec.InsertRole.GenericMount,label:"synthetic anchored seat",positionTolerance:0.01,
			finish:materia.casting.InsertSpec.InsertFinish.AsSupplied};
		check(materia.tooling.casting.InsertFixturePlanner.holes(input).length==3
			&& materia.tooling.casting.InsertFixturePlanner.availableHoles(input).length==1,
			"occupied anchor taps remain source drilling but receive no fixture screw or duplicate bore mask");
		var manufacturing=materia.tooling.casting.InsertManufacturingPlan.build(input);
		check(manufacturing.operations.filter(op -> op.kind==materia.tooling.casting.InsertManufacturingPlan.InsertOperationKind.FitAnchors
			&& op.stage==materia.tooling.casting.InsertManufacturingPlan.InsertOperationStage.BeforeCasting).length==2,
			"each source anchor appears in the pre-cast operation sheet");
		var checks=materia.tooling.casting.InsertAnchorageChecks.check(input);
		check(checks.filter(c -> c.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Fail).length==0,"valid nominal anchor limits pass their implemented checks");
		check(checks.filter(c -> c.rule.indexOf(".capacity")>=0 && c.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending).length==1,"capacity requires physical evidence");
		check(!materia.tooling.ToolingValidation.ToolingReadiness.evaluate(checks).ready,"nominal thread fit cannot qualify casting occupancy or capacity");
		var locked=bolts(spec)[0];locked.locking=InsertAnchorLock.Shoulder;
		check(materia.tooling.casting.InsertAnchorageChecks.check(input).filter(c -> c.rule.indexOf(".locking")>=0
			&& c.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported).length==1,
			"an undeclared physical shoulder stays Unsupported even with nominal standard hardware");
		locked.locking=InsertAnchorLock.RetainingCompound;
		foundationGeometry(source);
		Sys.println('InsertAnchorTests: $assertions assertions passed');
	}
}
