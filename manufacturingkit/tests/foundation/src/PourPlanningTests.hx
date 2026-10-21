import materia.tooling.process.PourPlanning;
import materia.tooling.process.PourPlanning.MixComposition;
import materia.tooling.process.PourPlanning.PourMixerSpec;
import materia.tooling.process.PourPlanning.PourAssessment;
import materia.tooling.process.PourPlanning.PourSchedule;
import materia.tooling.process.FillAnalysis;
import materia.tooling.process.FillAnalysis.FillReport;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;

/** Synthetic ratios and equipment exercise planning, not production selections. */
class PourPlanningTests {
	static var assertions = 0;
	static function check(ok:Bool, message:String):Void {
		assertions++;
		if (!ok) throw message;
	}
	static function near(a:Float,b:Float,message:String):Void
		check(Math.isFinite(a) && Math.abs(a-b)<1e-9,message+': $a vs $b');
	static function rejects(fn:Void -> Void, expected:String):Void {
		var error="";
		try fn() catch(e:Dynamic) error=Std.string(e);
		check(error.indexOf(expected)>=0,'Expected "$expected", got "$error"');
	}
	static function composition():MixComposition return {
		reference:"synthetic mix by weight", cureReference:"synthetic resin system", cureMassIngredientIds:["resin"], ingredients:[
			{id:"resin",kind:Resin,materialReference:"test resin",massFraction:0.1},
			{id:"hardener",kind:Hardener,materialReference:"test hardener",massFraction:0.05},
			{id:"aggregate",kind:Aggregate,materialReference:"test aggregate",massFraction:0.85}
		]
	};
	static function mixer():PourMixerSpec return {
		reference:"synthetic single mixer",capacityKg:0.7,mixingSeconds:1,transferSeconds:1,
		workingTimeSeconds:20,workingTimeReference:"synthetic working-time input"
	};
	static function cube(rate:Null<Float>, ?extra:Array<Float>):FillReport
		return FillAnalysis.analyze("cast","source","fill-spec",
			[{x:0.0,y:0,z:0,width:100.0,depth:100.0,height:100.0}],[],[],100,2000,9.80665,rate,null,extra);
	static function schedule(report:PourAssessment):PourSchedule {
		var result=report.schedule;
		if(result==null)throw "Expected a computed pour schedule";
		return result;
	}
	public static function run():Void {
		var fill=cube(100000),recipe=composition(),equipment=mixer();
		var result=PourPlanning.analyze("cast",fill,recipe,equipment),plan=schedule(result);
		near(result.totalMixMassKg,2,"total mass comes from the existing corrected fill curve");
		check(plan.batches.length==3,"capacity selects three batches");
		near(plan.batches[0].massKg,0.7,"first batch usable capacity");
		near(plan.batches[2].massKg,0.6,"final partial batch");
		near(plan.batches[0].upperHeightMm,35,"first batch reaches its actual fill level");
		near(plan.batches[1].lowerHeightMm,35,"level ranges join without a gap");
		near(plan.batches[1].upperHeightMm,70,"second fill level");
		near(plan.batches[2].upperHeightMm,100,"final fill level");
		near(plan.totalElapsedSeconds,16,"timeline includes mixing and transfer for each batch");
		var total=0.0;
		for(batch in plan.batches) {
			total+=batch.massKg;
			var ingredients=0.0;
			for(q in batch.quantities)ingredients+=q.massKg;
			near(ingredients,batch.massKg,"each batch balances by weight");
		}
		near(total,result.totalMixMassKg,"batch mass conservation");
		near(result.quantities.filter(q->q.ingredientId=="resin")[0].massKg,0.2,"resin quantity");
		near(result.quantities.filter(q->q.ingredientId=="hardener")[0].massKg,0.1,"hardener quantity");
		near(result.quantities.filter(q->q.ingredientId=="aggregate")[0].massKg,1.7,"aggregate quantity");
		var basisMass = result.cureBasisMassKg;
		if (basisMass == null) throw "Expected a declared cure mass basis";
		near(basisMass,0.2,"heat denominator follows the explicitly selected ingredient mass");
		check(PourPlanning.cureConsistency("cast",recipe,"synthetic resin system",0.1).status==ToolingCheckStatus.Pass,
			"base resin heat denominator is explicit");
		recipe.cureMassIngredientIds=["resin","hardener"];
		check(PourPlanning.cureConsistency("cast",recipe,"synthetic resin system",0.15).status==ToolingCheckStatus.Pass,
			"mixed binder heat denominator is explicit");
		check(PourPlanning.cureConsistency("cast",recipe,"synthetic resin system",0.1).status==ToolingCheckStatus.Fail,
			"mixed binder and base resin denominators cannot silently disagree");
		check(PourPlanning.cureConsistency("cast",recipe,"other system",0.15).status==ToolingCheckStatus.Fail,
			"cure resin identity must match the mixture");
		check(PourPlanning.cureConsistency("cast",recipe,"synthetic resin system",null).status==ToolingCheckStatus.Pending,
			"unknown physical heat basis remains pending");
		recipe.cureMassIngredientIds=["resin"];
		var dense=schedule(PourPlanning.analyze("cast",cube(100000,[for(i in 0...1001)i/10.0]),recipe,equipment));
		for(i in 0...plan.batches.length) {
			near(dense.batches[i].lowerHeightMm,plan.batches[i].lowerHeightMm,"dense level curve preserves batch start");
			near(dense.batches[i].upperHeightMm,plan.batches[i].upperHeightMm,"dense level curve preserves batch end");
		}
		equipment.capacityKg=10;equipment.workingTimeSeconds=5;
		var limited=schedule(PourPlanning.analyze("cast",fill,recipe,equipment));
		check(limited.batches.length==8,"working time and following-batch arrival constrain capacity");
		near(limited.totalElapsedSeconds,26,"working-time-constrained sequential timeline");
		check(limited.minimumWorkingTimeMarginSeconds>=0,"both transfer and first interface contact meet the declared deadline");
		for(i in 1...limited.batches.length)
			check(limited.batches[i].pourStartSeconds-limited.batches[i-1].mixingStartSeconds<=5+1e-9,
				"every incoming batch reaches the prior batch before its deadline");
		equipment.workingTimeSeconds=12;
		check(schedule(PourPlanning.analyze("cast",fill,recipe,equipment)).batches.length==1,
			"one batch does not reserve an unnecessary following-batch mixing cycle");
		equipment.workingTimeSeconds=3;
		var impossible=PourPlanning.analyze("cast",fill,recipe,equipment);
		check(impossible.schedule==null && impossible.validation.filter(v->v.status==ToolingCheckStatus.Fail).length==1,
			"impossible sequential handover fails explicitly");
		var missing=PourPlanning.analyze("cast",fill,null,null);
		check(missing.schedule==null && missing.quantities.length==0
			&& missing.validation.filter(v->v.status==ToolingCheckStatus.Pending).length==2,"undeclared inputs stay pending without invented ratios");
		missing=PourPlanning.analyze("cast",fill,recipe,null);
		check(missing.quantities.length==3 && missing.schedule==null,"known quantities survive missing equipment");
		missing=PourPlanning.analyze("cast",cube(null),recipe,mixer());
		check(missing.schedule==null && missing.validation.filter(v->v.status==ToolingCheckStatus.Pending).length==1,"unknown pour rate stays pending");
		var unknown=cube(100000);unknown.model="future exact non-box model";
		var unsupported=PourPlanning.analyze("cast",unknown,recipe,mixer());
		check(unsupported.schedule==null && unsupported.validation.filter(v->v.status==ToolingCheckStatus.Unsupported).length==1,
			"non-box level inversion is never approximated by a line");
		var stepped=FillAnalysis.analyze("cast","source","fill-spec",[
			{x:0.0,y:0,z:0,width:10.0,depth:10.0,height:5.0},
			{x:0.0,y:0,z:5,width:20.0,depth:10.0,height:5.0}],[],[],10,2000,9.80665,1000,null);
		var small=mixer();small.capacityKg=0.0012;small.mixingSeconds=0.01;small.transferSeconds=0.01;
		var stepPlan=schedule(PourPlanning.analyze("cast",stepped,recipe,small));
		near(stepPlan.batches[0].upperHeightMm,5.5,"batch level inversion spans a real section-area event");
		near(stepPlan.batches[1].upperHeightMm,8.5,"subsequent batch follows the larger area");
		var trapped=FillAnalysis.analyze("cast","source","fill-spec",[
			{x:0.0,y:0,z:0,width:2.0,depth:2.0,height:10.0},
			{x:2.0,y:0,z:0,width:2.0,depth:2.0,height:2.0},
			{x:4.0,y:0,z:0,width:4.0,depth:2.0,height:6.0}],
			[{id:"ceiling",box:{x:4.0,y:0,z:6,width:4.0,depth:2.0,height:4.0}}],[],10,2400,9.80665,2,null);
		var trapMixer=mixer();trapMixer.capacityKg=0.00004;trapMixer.mixingSeconds=0.1;trapMixer.transferSeconds=0.1;
		var trapPlan=PourPlanning.analyze("cast",trapped,recipe,trapMixer);
		near(trapPlan.totalMixMassKg,64*2400/1e9,"weighed quantities exclude the sealed air volume");
		near(schedule(trapPlan).batches[1].upperHeightMm,7.0/3,"batch heights follow the corrected curve past neck closure");
		check(!materia.tooling.ToolingValidation.ToolingReadiness.evaluate(trapped.validation.concat(trapPlan.validation)).ready,
			"a calculable batch proposal does not qualify a trapped casting");
		var tooSmall=mixer();tooSmall.capacityKg=1e-12;
		var bounded=PourPlanning.analyze("cast",fill,recipe,tooSmall);
		check(bounded.schedule==null && bounded.validation.filter(v->v.status==ToolingCheckStatus.Unsupported).length==1,
			"huge batch counts are bounded before integer conversion or allocation");
		recipe.ingredients.reverse();
		check(PourPlanning.analyze("cast",fill,recipe,mixer()).mixingFingerprint==result.mixingFingerprint,
			"ingredient order does not change fingerprint");
		equipment=mixer();equipment.capacityKg=0.6;
		check(PourPlanning.analyze("cast",fill,recipe,equipment).mixingFingerprint!=result.mixingFingerprint,
			"equipment choices enter fingerprint");
		var bad=composition();bad.ingredients[0].massFraction=0.5;
		rejects(() -> PourPlanning.copyComposition(bad),"fractions must sum to one");
		bad=composition();bad.ingredients[0].id=bad.ingredients[1].id;
		rejects(() -> PourPlanning.copyComposition(bad),"duplicate mix ingredient");
		equipment=mixer();equipment.capacityKg=0;
		rejects(() -> PourPlanning.copyMixer(equipment),"Invalid mixer capacity");
		var times=cube(100000,[50]);times.levels[1].timeSeconds=4;
		rejects(() -> PourPlanning.analyze("cast",times,recipe,mixer()),"Fill times disagree");
		var raw:Dynamic=haxe.Json.parse(haxe.Json.stringify(mixer()));Reflect.setField(raw,"capacityLitres",1);
		rejects(() -> PourPlanning.copyMixer(raw),"Unexpected casting record fields");
		Sys.println('PourPlanningTests: $assertions assertions passed');
	}
}
