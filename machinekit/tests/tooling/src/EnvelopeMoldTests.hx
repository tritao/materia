import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingCodec;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingMoldPlanner;
import machinekit.tooling.CastingMoldCad;
import machinekit.tooling.ToolingVolume;
import machinekit.structural.MineralFoundation;
import machinekit.component.ComponentValues;

/** The shell bounds size stock; exact source body and filler regions size resin. */
class EnvelopeMoldTests {
	static var assertions=0;
	static function check(value:Bool,message:String):Void {
		assertions++;
		if (!value) throw message;
	}

	static function options():CastingMoldOptions return {
		replicationJigs:[],foundationJointProcesses:[],precision:[],
		compaction:{method:Undeclared,bedInterfaceReference:null,frequencyHz:null,
			amplitudeMm:null,durationSeconds:null,evidence:[]},
		serviceOrientation:F.identity(),overlapRule:ReservedThenPriority,liftingEvidence:[]
	};

	static function fixture(flared:Bool):Void {
		var values=new ComponentValues().setNumber("width",100).setNumber("depth",80).setNumber("height",120);
		if (flared) values.setNumber("footWidth",160).setNumber("footHeight",40).setNumber("footShoulder",15);
		else values.setNumber("cornerChamfer",12);
		var foundation=new MineralFoundation(values,[]);
		var document=SinglePocketSetup.document();
		document.input=machinekit.tooling.FoundationToolingAdapter.snapshot(foundation,flared ? "flared-shell" : "chamfered-shell",1);
		document.setup.sourceFingerprint=ToolingCodec.inputFingerprint(document.input);
		document.setup.sourceToMold=F.identity();
		document.setup.features=[];
		document.setup.fillHeightMm=120;
		var boxes=materia.tooling.casting.CastingSourceRegions.derive(document);
		check(boxes.regions==null && boxes.validation.filter(row -> row.status=="unsupported"
			&& row.affectedIds.indexOf(document.input.id)>=0).length==1,
			"Box-only consumers retain an explicit source-envelope Unsupported result");
		var plan=CastingMoldCad.plan(document,options());
		check(plan.formers.length==(flared ? 2 : 4),"Each envelope complement island is an independent former");
		check(plan.document.input.envelope.length==document.input.envelope.length,"Shell planning preserves source envelope pieces");
		check(plan.validation.filter(row -> row.rule=="tooling.feature.box-geometry" && row.status=="pass").length==1,
			"Native regions qualify envelope tooling without claiming box-only source support");
		var source=foundation.body.geometry(),resin=CastingMoldCad.reconstructed(plan);
		var missing=source.subtractAll([resin]),excess=resin.subtractAll([source]);
		check(missing.volume()+excess.volume()<1e-5,"Shell and exact fillers reproduce actual foundation CAD");
		missing.close();excess.close();
		check(Math.abs(plan.handling.castingMassKg-source.volume()*document.recipe.mixDensityKgM3*1e-9)<1e-7,
			"Handling uses actual envelope mass rather than shell bounds");
		check(source.volume()<CastingMoldPlanner.volume(plan.cavity),"Fixture distinguishes actual resin from shell bounding stock");
		source.close();resin.close();
		for (former in plan.formers) {
			var shape=ToolingVolume.geometryOf(former.volume);
			check(shape.solidCount()==1,"Each filler is one physical connected body");
			shape.close();
		}
		var drawings=machinekit.tooling.ToolingSectionReview.build(plan);
		check(drawings.records.length==plan.formers.length
			&& drawings.validation.filter(row -> row.status!="pass").length==0,
			"Every exact envelope filler has line/arc section drawing evidence");
		var review=CastingMoldCad.review(plan);
		check(review.symmetricDifferenceMm3<1e-5,"Full review proves exact envelope reconstruction");
		for (name in ["reconstruction","coverage","assembly-graph","demolding-graph"])
			check(review.validation.filter(row -> row.rule=="tooling.cad."+name && row.status=="pass").length==1,
				"Envelope mold retains native "+name+" proof");
		check(review.demolding.removalOrder.length>plan.formers.length,"Complete graph contains walls, fillers and cured casting");
		plan.formers.pop();
		var message="";
		try CastingMoldCad.review(plan) catch (error:Dynamic) message=Std.string(error);
		check(message.indexOf("Casting reconstruction differs by")>=0,"Missing envelope filler fails exact reconstruction: "+message);
	}

	static function stepped():Void {
		var document=SinglePocketSetup.document();
		document.input.id="stepped-shell";
		document.input.inserts=[];
		document.input.features=[];
		function rectangle(width:Float,depth:Float):Array<{x:Float,y:Float}>
			return [{x:-width/2,y:-depth/2},{x:width/2,y:-depth/2},
				{x:width/2,y:depth/2},{x:-width/2,y:depth/2}];
		document.input.envelope=[
			{id:"lower",profile:rectangle(200,160),height:20,pose:F.identity()},
			{id:"upper",profile:rectangle(100,80),height:80,pose:F.translation(0,0,20)}
		];
		document.setup.features=[];
		document.setup.sourceToMold=F.identity();
		document.setup.fillHeightMm=100;
		document.setup.sourceFingerprint=ToolingCodec.inputFingerprint(document.input);
		var plan=CastingMoldCad.plan(document,options());
		var volume=200*160*20+100*80*80;
		check(Math.abs(plan.handling.castingMassKg-volume*document.recipe.mixDensityKgM3*1e-9)<1e-7,
			"Disjoint-box handling retains the exact stepped body rather than its stock bounds");
		var resin=CastingMoldPlanner.resinAtFill(document,plan.envelope);
		check(Math.abs(materia.casting.BoxAlgebra.volume(resin)-volume)<1e-7,
			"Box resin analysis uses every exact body piece");
		check(plan.formers.length==1 && plan.formers[0].withdrawal.z==1,
			"Connected stepped-envelope collar withdraws through the fill side");
		var drawings=machinekit.tooling.ToolingSectionReview.build(plan);
		check(drawings.records.length==plan.formers.length
			&& drawings.validation.filter(row -> row.status!="pass").length==0,
			"Every exact envelope filler has line/arc section drawing evidence");
		var review=CastingMoldCad.review(plan);
		check(review.symmetricDifferenceMm3<1e-5,"Stepped shell reconstructs its exact source union");
		check(review.demolding.removalOrder.indexOf(plan.formers[0].volume.id)>=0,
			"Collar has a real swept removal path");
	}

	static function main():Void {
		fixture(true);
		fixture(false);
		stepped();
		Sys.println('EnvelopeMoldTests: $assertions assertions passed');
	}
}
