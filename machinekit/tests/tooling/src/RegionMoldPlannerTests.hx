import materia.tooling.casting.CastingMoldPlanner as P;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingMoldCodec;
import materia.tooling.ToolingCodec;
import materia.assembly.AssemblyFrames as F;

class RegionMoldPlannerTests {
	static var assertions=0;
	static function check(value:Bool,message:String):Void {assertions++;if (!value) throw message;}
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
