import materia.tooling.casting.CastingMoldPlanner as P;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingMoldCodec;
import materia.tooling.ToolingCodec;
import materia.assembly.AssemblyFrames as F;

class RegionMoldPlannerTests {
	static var assertions=0;
	static function check(value:Bool,message:String):Void {assertions++;if (!value) throw message;}
	static function sideFormers(options:CastingMoldOptions):Void {
		for (quarterTurn in [false,true]) {
			var document=SinglePocketSetup.document(18,quarterTurn,true);
			document.input.inserts=[];
			var feature=materia.casting.CastingPatterns.box("side-window",
				materia.casting.CastingFeature.CastingFeatureKind.ServiceOpening,
				[materia.casting.CastingPatterns.face("body.x-max")],81,40,40,80.5,0,30);
			feature.cornerRadius=8;
			feature.draft={degrees:2,axis:{x:1,y:0,z:0}};
			document.input.features=[feature];
			var direction=F.transformVector(document.setup.sourceToMold,1,0,0);
			document.setup.features=[{featureId:feature.id,strategy:materia.tooling.casting.CastingSetup.CoreStrategy.ReusableFormer,
				priority:0,withdrawal:direction}];
			document.setup.sourceFingerprint=ToolingCodec.inputFingerprint(document.input);
			var authored=materia.tooling.casting.CastingMoldOptionsCodec.snapshot(options);
			authored.serviceOrientation=document.setup.sourceToMold;
			var plan=machinekit.tooling.CastingMoldCad.plan(document,authored),former=plan.formers[0];
			check(former.volume.profiles.length==1,"side former carries one panel-operation sealing profile");
			var profile=former.volume.profiles[0],wall=plan.panels.members.filter(member -> member.panel.id==profile.panelId)[0];
			var operation=wall.panel.operations.filter(op -> op.id==profile.operationId)[0];
			check(ToolingCodec.canonical(profile.contour)==ToolingCodec.canonical(operation.contour),
				"side access CAD and panel drawings/CAM use the same exact operation contour");
			check(Math.abs(materia.casting.CastingProfileGeometry.area(operation.contour)-(1600-(4-Math.PI)*64))<1e-7,
				"side access preserves the authored rounded opening area under setup rotation");
			var review=machinekit.tooling.CastingMoldCad.review(plan);
			check(review.symmetricDifferenceMm3<1e-5,"side former reconstructs the exact tapered source cavity");
			for (rule in ["assembly-graph","demolding-graph"])
				check(review.validation.filter(record -> record.rule=="tooling.cad."+rule
					&& record.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass).length==1,
					"side former passes complete "+rule+" sweeps under either setup orientation");
			feature.draft.degrees=0;
			document.setup.sourceFingerprint=ToolingCodec.inputFingerprint(document.input);
			check(P.assess(document,authored).plan==null,
				"a transverse zero-draft rounded box is not replaced by a reoriented corner profile");
			var failure="";
			feature.cornerRadius=2;
			feature.draft.degrees=2;
			feature.geometry.solid=materia.casting.CastingFeature.CastingGeometrySolid.Cut(materia.casting.CastingSolid.Box(41,40,40));
			feature.geometry.pose.x=100.5;
			document.setup.sourceFingerprint=ToolingCodec.inputFingerprint(document.input);
			try machinekit.tooling.CastingMoldCad.plan(document,authored) catch (error:Dynamic) failure=Std.string(error);
			check(failure.indexOf("side-window")>=0 && failure.indexOf("Cutter exceeds internal radius")>=0,
				"uncuttable side opening fails with its source feature ID");
		}
	}

	public static function main():Void {
		var d=SinglePocketSetup.document(18,false,true);
		d.input.inserts=[];
		d.input.features=d.input.features.filter(feature -> feature.id=="pocket");
		var feature=d.input.features[0];
		feature.kind=materia.casting.CastingFeature.CastingFeatureKind.ServiceOpening;
		feature.draft={degrees:2,axis:{x:0,y:0,z:1}};
		d.setup.features=[{featureId:feature.id,strategy:materia.tooling.casting.CastingSetup.CoreStrategy.ReusableFormer,
			priority:0,withdrawal:{x:0,y:0,z:-1}}];
		d.setup.sourceFingerprint=ToolingCodec.inputFingerprint(d.input);
		var options:CastingMoldOptions={replicationJigs:[],foundationJointProcesses:[],precision:[],
			compaction:{method:CompactionMethod.Undeclared,bedInterfaceReference:null,frequencyHz:null,amplitudeMm:null,
				durationSeconds:null,evidence:[]},serviceOrientation:d.setup.sourceToMold,
			overlapRule:OverlapRule.ReservedThenPriority,liftingEvidence:[]};
		var previous=P.geometry;
		try {
			var plan=machinekit.tooling.CastingMoldCad.plan(d,options);
			check(plan.formers.length==1 && plan.formers[0].volume.regions.length==1 && plan.formers[0].volume.pieces.length==0,
				"drafted former uses exact allocated source geometry rather than a stock box");
			check(plan.changeRequests.filter(request -> request.blockingChecks.indexOf("tooling.former.draft.pocket")>=0).length==0,
				"positive aligned draft clears its release change request");
			check(plan.validation.filter(record -> record.rule=="tooling.former.draft.pocket"
				&& record.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass).length==1,"source draft has an explicit passing rule");
			var sections=machinekit.tooling.ToolingSectionReview.build(plan);
			check(sections.records.length==plan.formers.length+plan.fixtures.length
				&& sections.validation.filter(record -> record.status!=materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass).length==0,
				"every finished tooling member has exact source-bound section drawing evidence: "+ToolingCodec.canonical(sections.validation));
			check(ToolingCodec.canonical(machinekit.tooling.ToolingSectionReview.record(sections,plan.sourceFingerprint))
				==ToolingCodec.canonical(machinekit.tooling.ToolingSectionReview.record(
					machinekit.tooling.ToolingSectionReview.build(plan),plan.sourceFingerprint)),
				"rebuilding a drawing report preserves its exact records and fingerprint");
			var encoded=CastingMoldCodec.encode(plan);
			check(CastingMoldCodec.encode(CastingMoldCodec.decode(encoded))==encoded,"native region plan regenerates through the strict codec");
			P.setGeometry(null);
			check(CastingMoldCodec.encode(machinekit.tooling.CastingMoldCad.decode(encoded))==encoded,
				"a fresh native entry point restores exact regions without prior global kernel state");
			var review=machinekit.tooling.CastingMoldCad.review(plan);
			for (name in ["reconstruction","coverage","assembly-graph","demolding-graph"])
				check(review.validation.filter(record -> record.rule=="tooling.cad."+name
					&& record.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass).length==1,
					"drafted mold passes exact "+name);
			var rounded=ToolingCodec.snapshot(d);
			rounded.input.features[0].cornerRadius=8;
			rounded.setup.sourceFingerprint=ToolingCodec.inputFingerprint(rounded.input);
			var roundedReview=machinekit.tooling.CastingMoldCad.review(P.plan(rounded,options));
			check(roundedReview.symmetricDifferenceMm3<1e-5
				&& roundedReview.validation.filter(record -> record.rule=="tooling.cad.demolding-graph"
					&& record.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass).length==1,
				"rounded drafted walls retain exact conical surfaces through reconstruction and withdrawal");
			var mixed=SinglePocketSetup.document(18,false,true);
			mixed.input.envelope[0].profile=[{x:-200.0,y:-90.0},{x:200.0,y:-90.0},{x:200.0,y:90.0},{x:-200.0,y:90.0}];
			mixed.input.features.filter(value -> value.id=="pocket")[0].draft.degrees=2;
			mixed.input.features.filter(value -> value.id=="pocket")[0].kind=materia.casting.CastingFeature.CastingFeatureKind.ServiceOpening;
			mixed.input.features.push(materia.casting.CastingPatterns.box("fill-core",
				materia.casting.CastingFeature.CastingFeatureKind.Lightening,
				[materia.casting.CastingPatterns.face("body.z-min")],100,160,31,-140,0,-1));
			mixed.setup.features=d.setup.features.copy();
			mixed.setup.sourceFingerprint=ToolingCodec.inputFingerprint(mixed.input);
			var mixedOptions=materia.tooling.casting.CastingMoldOptionsCodec.snapshot(options);
			mixedOptions.precision=[for (insert in mixed.input.inserts) {insertId:insert.id,jigReference:null,
				masterReference:null,masterFlatnessMm:null,releaseSpec:null,holeProtectionReference:null,evidence:[]}];
			var mixedPlan=P.plan(mixed,mixedOptions);
			check(mixedPlan.fixtures.length==1 && mixedPlan.cores.length==1
				&& mixedPlan.formers.filter(former -> former.volume.regions.length>0).length==1,
				"the shared planner combines exact drafted formers with source-thread fixtures and box EPS");
			var mixedReview=machinekit.tooling.CastingMoldCad.review(mixedPlan);
			check(mixedReview.symmetricDifferenceMm3<1e-5,"mixed tooling reconstructs the actual source casting");
			check(mixedReview.validation.filter(record -> record.rule=="tooling.cad.demolding-graph"
				&& record.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass).length==1,
				"mixed drafted formers and fixtures pass complete demolding sweeps");
			sideFormers(options);
			var oblique=ToolingCodec.snapshot(d);
			oblique.input.features[0].draft.axis={x:0.6,y:0,z:0.8};
			oblique.setup.sourceFingerprint=ToolingCodec.inputFingerprint(oblique.input);
			var declined=P.assess(oblique,options);
			check(declined.plan==null && declined.validation.filter(record -> record.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported
				&& record.affectedIds.indexOf("pocket")>=0).length==1,
				"an unrealized oblique draft remains Unsupported by feature ID with the native bridge enabled");
			var flat=ToolingCodec.snapshot(d);
			flat.input.features[0].draft.degrees=0;
			flat.setup.sourceFingerprint=ToolingCodec.inputFingerprint(flat.input);
			check(P.plan(flat,options).changeRequests.filter(request -> request.blockingChecks.indexOf("tooling.former.draft.pocket")>=0).length==1,
				"removing source draft restores its release request");
			var reversed=ToolingCodec.snapshot(d);
			reversed.setup.features[0].withdrawal={x:0,y:0,z:1};
			check(P.plan(reversed,options).changeRequests.filter(request -> request.blockingChecks.indexOf("tooling.former.draft.pocket")>=0).length==1,
				"a contradictory withdrawal cannot clear a source draft request");
		} catch (error:Dynamic) {
			P.setGeometry(previous);
			throw error;
		}
		P.setGeometry(previous);
		Sys.println("RegionMoldPlannerTests: "+assertions+" assertions passed");
	}
}
