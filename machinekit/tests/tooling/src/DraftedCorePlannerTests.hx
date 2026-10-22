import materia.assembly.AssemblyFrames as F;
import materia.casting.CastingRegion;
import materia.casting.CastingProfiles;
import materia.tooling.ToolingCodec;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus as Status;
import materia.tooling.casting.MoldRegions as R;
import materia.tooling.casting.MoldCores;

class DraftedCorePlannerTests {
	static var assertions=0;
	static function check(value:Bool,message:String):Void {assertions++;if (!value) throw message;}
	static function near(actual:Float,expected:Float,message:String):Void
		check(Math.abs(actual-expected)<1e-5,message+": "+actual+" vs "+expected);
	static function volume(width:Float,depth:Float,radius:Float,height:Float,degrees:Float):Float {
		var inset=height*Math.tan(degrees*Math.PI/180);
		var opening=width*depth-(4-Math.PI)*radius*radius;
		var linear=2*(width+depth)-2*(4-Math.PI)*radius;
		return height*(opening-linear*inset/2+Math.PI*inset*inset/3);
	}
	public static function main():Void run();
	public static function run():Void {
		var document=SinglePocketSetup.document(),feature=document.input.features.filter(row -> row.id=="pocket")[0];
		document.input.inserts=[];document.input.features=[feature];
		feature.cornerRadius=6;feature.draft={degrees:2,axis:{x:0,y:0,z:-1}};
		document.recipe.coreWireAxis=materia.tooling.ToolingRecipe.CoreWireAxis.Z;document.recipe.core.height=16;
		document.recipe.hardware.minimumBridgeWidth=40;
		document.setup.sourceFingerprint=ToolingCodec.inputFingerprint(document.input);
		var options=RegionCorePlannerTests.options(document);
		var plan=machinekit.tooling.CastingMoldCad.plan(document,options),core=plan.cores[0];
		check(plan.cores.length==1 && core.layers.length==4,"drafted EPS uses four full-cross-section stock layers");
		check(switch core.foamEnvelope {case Drafted(_,_,_,_,degrees):degrees==2;default:false;},
			"lining preserves exact draft rather than flattening it to a prism");
		var expected=volume(100,80,6,60,2);
		near(R.volume(core.cavity,core.id),expected,"gross rounded draft agrees with analytic frustum volume");
		var cover=document.recipe.linerThickness,glue=document.recipe.adhesiveLineThickness;
		var radians=2*Math.PI/180,inset=cover/Math.cos(radians)+glue*Math.tan(radians);
		near(R.volume(core.foamEnvelope,core.id),volume(100-2*inset,80-2*inset,6-inset,60-cover-glue,2),
			"foam includes normal lining thickness on drafted walls and the top adhesive line");
		var top=MoldCores.section(core.foamEnvelope,100-glue,core.id);
		near(materia.casting.CastingProfileGeometry.bounds(top[0].contour).width,100-2*inset,
			"authored opening dimensions stay at withdrawal, with explicit foam allowances");
		near(R.volume(CastingRegion.Difference(MoldCores.foam(core),[core.foamEnvelope]),core.id),0,
			"every posed wire layer stays inside the finished lined foam envelope");
		check(R.volume(MoldCores.foam(core),core.id)<R.volume(core.foamEnvelope,core.id),
			"adhesive gaps between drafted layers remain separate from EPS material");
		for (layer in core.layers) check(layer.block.height<=16+1e-7,"drafted stock fits the declared block thickness");
		var drawings=machinekit.tooling.ToolingSectionReview.build(plan);
		check(drawings.records.length==4 && drawings.validation.filter(row -> row.status!=Status.Pass).length==0,
			"all drafted foam layers retain exact line/arc section drawings");
		var encoded=materia.tooling.casting.CastingMoldCodec.encode(plan);
		check(materia.tooling.casting.CastingMoldCodec.encode(machinekit.tooling.CastingMoldCad.decode(encoded))==encoded,
			"drafted core geometry regenerates through the same strict source schema");
		var review=machinekit.tooling.CastingMoldCad.review(plan);
		near(review.symmetricDifferenceMm3,0,"drafted EPS reconstructs the actual source casting");
		for (rule in ["coverage","assembly-graph","demolding-graph"])
			check(review.validation.filter(row -> row.rule=="tooling.cad."+rule && row.status==Status.Pass).length==1,
				"drafted EPS passes complete "+rule);
		var processSpec=ProcessTests.spec();processSpec.sections=[];
		processSpec.recipeFingerprint=ToolingCodec.recipeFingerprint(document.recipe);
		var process=materia.tooling.process.CastingProcessAnalysis.analyze(plan,processSpec,9.80665,null,
			machinekit.tooling.CastingFillAnalysis.fromPlan),fill=process.fill;
		check(fill!=null,"drafted EPS shares the native fill pipeline");
		if (fill==null) throw "Missing drafted EPS fill report";
		var uplift=fill.levels[fill.levels.length-1].cores.filter(row -> row.id==core.id)[0];
		near(uplift.submergedVolumeMm3,expected,"drafted gross uplift uses the exact cavity volume");
		var effort=materia.tooling.process.MoldOrientationStudy.metrics(plan,FabricationTests.spec());
		near(effort.mixMassKg,(240*180*100-expected)*document.recipe.mixDensityKgM3*1e-9,
			"drafted orientation effort uses exact resin geometry");
		var study=materia.tooling.process.MoldOrientationStudy.run([
			{id:"drafted",document:document,options:options,process:processSpec}
		],FabricationTests.spec(),candidate -> {
			var checked=machinekit.tooling.CastingMoldCad.review(candidate);
			return {planFingerprint:checked.fingerprint,assemblySteps:checked.assembly.assemblyOrder.length,
				demoldingSteps:checked.demoldingSteps.length,assemblyBlockingEdges:checked.assembly.blocking.length,
				demoldingBlockingEdges:checked.demolding.blocking.length,validation:checked.validation};
		},machinekit.tooling.CastingFillAnalysis.fromPlan);
		var candidate=study.rankedCandidates[0],quantities=candidate.effort;
		check(candidate.planningSucceeded && candidate.geometryChecked,
			"non-box orientation uses the real planner and complete CAD graph evidence");
		check(candidate.fill!=null && candidate.sections==null,
			"non-box orientation retains exact fill without inventing orthogonal sections");
		check(quantities!=null,"non-box orientation keeps its effort metrics");
		if (quantities==null) throw "Missing drafted orientation effort";
		near(quantities.mixMassKg,effort.mixMassKg,"orientation and direct effort share exact resin quantities");
		var shortStock=ToolingCodec.snapshot(document);shortStock.recipe.coreWireAxis=materia.tooling.ToolingRecipe.CoreWireAxis.Y;
		var message="";
		try machinekit.tooling.CastingMoldCad.plan(shortStock,options) catch (error:Dynamic) message=Std.string(error);
		check(message.indexOf("EPS core pocket: Unsupported drafted EPS cross-profile tiling")>=0,
			"impossible cross-profile stock splitting fails with the source feature ID");
		Sys.println("DraftedCorePlannerTests: "+assertions+" assertions passed");
	}
}
