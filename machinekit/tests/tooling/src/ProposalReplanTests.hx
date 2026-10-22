import materia.assembly.AssemblyFrames as F;
import materia.casting.CastingPatterns;
import materia.casting.InsertSpec.InsertFinish;
import materia.casting.InsertSpec.InsertRole;
import materia.tooling.ToolingCodec;
import materia.tooling.ToolingDocument;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus as Status;
import materia.tooling.casting.CastingMoldPlan.CastingMoldOptions;
import materia.tooling.casting.CastingMoldPlan.DesignChangeKind;
import materia.tooling.process.CastabilityAnalysis.CastabilityAction;
import materia.tooling.process.CastingLevelOne;
import machinekit.component.ComponentValues;
import machinekit.structural.CastInsert;
import machinekit.structural.InsertHoles;
import machinekit.structural.MineralFoundation;
import machinekit.structural.MineralFoundation.CastInsertPlacement;
import machinekit.tooling.CastingMoldCad;
import machinekit.tooling.FoundationToolingAdapter;

/** Explicit source authoring accepts a review location; no automatic insert edit. */
class ProposalReplanTests {
	static var assertions=0;
	static inline var INSERT_HEIGHT=20.0;

	static function check(value:Bool,message:String):Void {
		assertions++;
		if (!value) throw message;
	}

	static function foundation(placements:Array<CastInsertPlacement>):MineralFoundation
		return new MineralFoundation(new ComponentValues().setNumber("width",240)
			.setNumber("depth",180).setNumber("height",100),placements);

	static function document(source:MineralFoundation,revision:Int):ToolingDocument {
		var result=SinglePocketSetup.document();
		result.input=FoundationToolingAdapter.snapshot(source,"proposal-lifting-fixture",revision);
		result.setup.features=[];
		result.setup.revision=revision;
		result.setup.sourceFingerprint=ToolingCodec.inputFingerprint(result.input);
		result.recipe.designReview.minimumLiftingPoints=4;
		return ToolingCodec.snapshot(result);
	}

	static function options(source:ToolingDocument):CastingMoldOptions {
		var result=RegionCorePlannerTests.options(source);
		result.serviceOrientation=F.identity();
		result.precision=[for (insert in source.input.inserts) {
			insertId:insert.id,jigReference:null,masterReference:null,masterFlatnessMm:null,
			releaseSpec:null,holeProtectionReference:null,evidence:[]
		}];
		return result;
	}

	public static function main():Void run();
	public static function run():Void {
		var initial=document(foundation([]),1),before=ToolingCodec.canonical(initial);
		var first=CastingMoldCad.plan(initial,options(initial));
		check(first.handling.requiresFlip,"the fixture genuinely requires source lifting connections");
		check(first.changeRequests.filter(row -> row.kind==DesignChangeKind.LiftingPoints).length==1,
			"the unmodified casting has a lifting design request");
		var candidates=CastingLevelOne.analyze(initial).proposals.filter(row -> row.action==CastabilityAction.DeclareLiftingPoint);
		check(candidates.length==4,"the accepted locations come from the actual CoM-based proposal");
		var placements:Array<CastInsertPlacement> =[];
		var sourceFromMold=F.inverse(initial.setup.sourceToMold);
		for (index in 0...candidates.length) {
			var candidate=candidates[index],point=candidate.point;
			if (point==null) throw "Missing lifting proposal location";
			var position=F.transformPoint(sourceFromMold,point.x,point.y,point.z);
			var thread=InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(6).mating());
			var values=CastInsert.precastHoles(CastInsert.boxValues(40,40,INSERT_HEIGHT),
				InsertHoles.tappedBlind(thread,INSERT_HEIGHT));
			values.setToken("role",InsertRole.LiftingMount).setToken("finish",InsertFinish.AsSupplied)
				.setToken("label","accepted lifting connection");
			var insert=new CastInsert(values);
			placements.push({id:"lift-"+index,part:insert,
				pose:F.translation(position.x,position.y,position.z-INSERT_HEIGHT),
				openings:[CastingPatterns.face("body.z-max")]});
			var returned=F.transformPoint(initial.setup.sourceToMold,position.x,position.y,position.z);
			check(Math.abs(returned.x-point.x)+Math.abs(returned.y-point.y)+Math.abs(returned.z-point.z)<1e-7,
				"source authoring preserves the proposed mounting point through the setup frame");
			var holes=machinekit.design.DesignInterfaces.of(insert);
			check(holes!=null && holes.holes.length==1,
				"each accepted connection declares a real policy-derived source HoleInterface");
		}
		var accepted=document(foundation(placements),initial.input.revision+1);
		check(accepted.input.revision==2 && accepted.setup.revision==2,
			"accepted source authoring creates a new casting and setup revision");
		check(ToolingCodec.inputFingerprint(accepted.input)!=ToolingCodec.inputFingerprint(initial.input),
			"new lifting geometry changes the source fingerprint");
		var revised=CastingMoldCad.plan(accepted,options(accepted));
		check(revised.changeRequests.filter(row -> row.kind==DesignChangeKind.LiftingPoints).length==0,
			"accepting the actual proposed lifting locations clears the request on replan");
		check(revised.fixtures.length==4 && revised.fixtures.filter(row -> row.fixings.length>0).length==4,
			"all proposed source threads have actual mold fixing plans");
		check(revised.validation.filter(row -> row.rule=="tooling.handling.flip" && row.status==Status.Pending).length==1,
			"source acceptance never qualifies equipment capacity or the flip procedure");
		var review=CastingMoldCad.review(revised);
		check(review.symmetricDifferenceMm3<1e-5,"the revised mold reconstructs the source casting");
		for (name in ["coverage","assembly-graph","demolding-graph"])
			check(review.validation.filter(row -> row.rule=="tooling.cad."+name && row.status==Status.Pass).length==1,
				"the accepted source revision passes complete "+name);
		var removed=document(foundation(placements.slice(0,3)),accepted.input.revision+1);
		var missing=CastingMoldCad.plan(removed,options(removed));
		check(missing.changeRequests.filter(row -> row.kind==DesignChangeKind.LiftingPoints).length==1,
			"removing an accepted lifting connection restores the design request");
		check(ToolingCodec.canonical(initial)==before,"review and explicit acceptance leave the original revision intact");
		Sys.println("ProposalReplanTests: "+assertions+" assertions passed");
	}
}
