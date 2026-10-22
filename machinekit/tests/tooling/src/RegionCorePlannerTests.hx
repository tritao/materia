import materia.assembly.AssemblyFrames as F;
import materia.casting.CastingFeature.CastingGeometrySolid;
import materia.casting.CastingProfiles as Profiles;
import materia.tooling.ToolingCodec;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus as Status;
import materia.tooling.casting.MoldCores;
import materia.tooling.casting.MoldRegions as R;
import materia.tooling.casting.CastingMoldPlan;

class RegionCorePlannerTests {
	static var assertions=0;
	static function check(value:Bool,message:String):Void {assertions++;if (!value) throw message;}
	static function rejects(fn:Void->Void,expected:String):Void {
		var message="";try fn() catch (error:Dynamic) message=Std.string(error);
		check(message.indexOf(expected)>=0,'Expected "$expected", got "$message"');
	}

	static function variants(document:materia.tooling.ToolingDocument,options:CastingMoldOptions,original:CastingMoldPlan):Void {
		var changed=ToolingCodec.snapshot(document);changed.recipe.core.height=20;
		var alternate=machinekit.tooling.CastingMoldCad.plan(changed,options);
		check(alternate.cores[0].layers.length!=original.cores[0].layers.length,
			"a stock-thickness change rebuilds the actual foam pieces");
		check(ToolingCodec.canonical(alternate.cores[0].cavity)==ToolingCodec.canonical(original.cores[0].cavity),
			"stock changes never modify the design cavity");
		var mixed=ToolingCodec.snapshot(document);
		var service=materia.casting.CastingPatterns.box("side-service",
			materia.casting.CastingFeature.CastingFeatureKind.ServiceOpening,
			[materia.casting.CastingPatterns.face("body.x-min"),materia.casting.CastingPatterns.feature("pocket")],
			100,20,30,-71,0,20);
		mixed.input.features.push(service);
		mixed.setup.features.push({featureId:service.id,strategy:ReusableFormer,priority:10,withdrawal:{x:-1,y:0,z:0}});
		mixed.setup.sourceFingerprint=ToolingCodec.inputFingerprint(mixed.input);
		var mixedPlan=machinekit.tooling.CastingMoldCad.plan(mixed,options);
		check(mixedPlan.cores.length==1 && mixedPlan.formers.length==1,
			"an intersecting side former reserves the exact triangular EPS region");
		var mixedReview=machinekit.tooling.CastingMoldCad.review(mixedPlan);
		check(mixedReview.symmetricDifferenceMm3<1e-5,"mixed core/former ownership reconstructs the source union");
		var pair=ToolingCodec.snapshot(document);
		var round=materia.casting.CastingFeatureCodec.snapshot(pair.input.features)[0];
		round.id="round-pocket";round.geometry.solid=Cut(materia.casting.CastingSolid.Cylinder(80,61));
		round.geometry.pose=F.translation(75,10,-1);
		pair.input.features.push(round);
		pair.setup.sourceFingerprint=ToolingCodec.inputFingerprint(pair.input);
		var plan=machinekit.tooling.CastingMoldCad.plan(pair,options);
		check(plan.cores.length==2,"two different non-box cores share the generic support planner");
		var forward:Array<materia.tooling.panel.PanelAssemblyPlan.PanelPlacement> =[],backward:Array<materia.tooling.panel.PanelAssemblyPlan.PanelPlacement> =[];
		var reversed=plan.cores.copy();reversed.reverse();
		materia.tooling.casting.CoreSupports.sharedBridges(plan.cavity,pair.recipe,plan.cores.copy(),forward,[]);
		materia.tooling.casting.CoreSupports.sharedBridges(plan.cavity,pair.recipe,reversed,backward,[]);
		forward.sort((a,b) -> Reflect.compare(a.panel.id,b.panel.id));
		backward.sort((a,b) -> Reflect.compare(a.panel.id,b.panel.id));
		check(ToolingCodec.canonical(forward)==ToolingCodec.canonical(backward),
			"real core-order reversal after decoding preserves support geometry and IDs");
		var review=machinekit.tooling.CastingMoldCad.review(plan);
		check(review.symmetricDifferenceMm3<1e-5,"curved and polygon cores reconstruct the shared casting");
		for (rule in ["assembly-graph","demolding-graph"])
			check(review.validation.filter(row -> row.rule=="tooling.cad."+rule && row.status==Status.Pass).length==1,
				"curved carriers clear their bounding-box neighbours in complete "+rule);
	}

	static function packageEvidence(plan:CastingMoldPlan,process:materia.tooling.process.CastingProcessAnalysis.CastingProcessReport):Void {
		var fabrication=materia.tooling.export.ToolingFabrication.plan(plan,FabricationTests.spec());
		var directory=Sys.getCwd()+"/build/tooling/nonbox-core-"+fabrication.fingerprint.substr(0,12);
		machinekit.tooling.ToolingFiles.directory(directory);
		sys.io.File.saveContent(directory+"/mold-plan.json",materia.tooling.casting.CastingMoldCodec.encode(plan));
		var checks=machinekit.tooling.ToolingFabricationExport.write(plan,fabrication,directory);
		checks=checks.concat(machinekit.tooling.ToolingProcessExport.write(plan,process.spec,9.80665,directory,
			null,null,null,process));
		var castability:Dynamic=haxe.Json.parse(sys.io.File.getContent(directory+"/castability-analysis.json"));
		check(castability!=null && Reflect.field(castability,"version")==materia.tooling.process.CastabilityAnalysis.VERSION,
			"native process export retains the real castability report without orthogonal section support");
		check(Reflect.field(castability,"sourceFingerprint")==process.sourceFingerprint
			&& Reflect.field(castability,"processFingerprint")==process.processFingerprint,
			"exported castability remains bound to its actual source and process");
		check(Reflect.field(castability,"minimumOpposingFaceSpanMm")==null
			&& Reflect.field(castability,"maximumDiameterUpperMm")==null,
			"exported unsupported section measurements remain null");
		var version=process.castability.version;
		try {
			process.castability.version=version-1;
			rejects(() -> machinekit.tooling.ToolingProcessExport.write(plan,process.spec,9.80665,directory,
				null,null,null,process),"Stale process export report");
		} catch (error:Dynamic) {process.castability.version=version;throw error;}
		process.castability.version=version;
		var markdown=sys.io.File.getContent(directory+"/castability-review.md");
		check(markdown.indexOf("process.castability.release.")>=0 && markdown.indexOf("process.level1.lifting-points")>=0,
			"human review includes available release and source findings rather than only an Unsupported note");
		machinekit.tooling.ToolingPackageManifest.write(directory,fabrication,checks);
		machinekit.tooling.ToolingPackageManifest.verify(directory);
		var core=plan.cores[0],name=MoldCores.layerId(core,0)+".cutting.json";
		var path=directory+"/"+name,original=sys.io.File.getContent(path);
		var changed:Dynamic=haxe.Json.parse(original);
		Reflect.deleteField(Reflect.field(changed,"cutting"),"solid");
		try {
			sys.io.File.saveContent(path,ToolingCodec.canonical(changed));
			rejects(() -> machinekit.tooling.ToolingPackageManifest.write(directory,fabrication,checks),
				"Core cutting artifact differs from source");
		} catch (error:Dynamic) {sys.io.File.saveContent(path,original);throw error;}
		sys.io.File.saveContent(path,original);
		var manifestPath=directory+"/manifest.json",manifestText=sys.io.File.getContent(manifestPath);
		var manifest:Dynamic=haxe.Json.parse(manifestText),files:Array<Dynamic> =cast Reflect.field(manifest,"files");
		try {
			Reflect.setField(manifest,"files",files.filter(file -> file.path!=name));
			sys.io.File.saveContent(manifestPath,ToolingCodec.canonical(manifest));
			rejects(() -> machinekit.tooling.ToolingPackageManifest.verify(directory),"Missing hashed tooling drawing");
		} catch (error:Dynamic) {sys.io.File.saveContent(manifestPath,manifestText);throw error;}
		sys.io.File.saveContent(manifestPath,manifestText);
		machinekit.tooling.ToolingPackageManifest.verify(directory);
		check(!fabrication.fabrication.ready,"geometric cutting evidence never grants physical fabrication readiness");
	}

	public static function options(document:materia.tooling.ToolingDocument):CastingMoldOptions
		return {foundationJointProcesses:[],replicationJigs:[],precision:[],
			compaction:{method:Undeclared,bedInterfaceReference:null,frequencyHz:null,amplitudeMm:null,durationSeconds:null,evidence:[]},
			serviceOrientation:document.setup.sourceToMold,overlapRule:ReservedThenPriority,liftingEvidence:[]};

	public static function main():Void run();
	public static function run():Void {
		var document=SinglePocketSetup.document(),feature=document.input.features.filter(row -> row.id=="pocket")[0];
		feature.geometry.solid=Prism(Profiles.polygon([{x:-40,y:-30},{x:40,y:-30},{x:0,y:50}]),61);
		feature.geometry.pose=F.translation(0,0,-1);
		document.input.features=[feature];document.input.inserts=[];
		document.recipe.core.height=16;
		document.recipe.hardware.minimumBridgeWidth=40;
		document.setup.sourceFingerprint=ToolingCodec.inputFingerprint(document.input);
		var options=RegionCorePlannerTests.options(document);
		var plan=machinekit.tooling.CastingMoldCad.plan(document,options);
		check(plan.cores.length==1 && plan.formers.length==0,"a triangular source cavity uses one exact EPS core");
		var core=plan.cores[0];
		check(core.layers.length>1,"thin EPS stock creates real manufacturing pieces");
		check(R.boxes(core.cavity)==null && R.boxes(MoldCores.foam(core))==null,
			"neither gross displacement nor finished foam becomes a stock box");
		var stock=0.0;
		for (layer in core.layers) {
			check(layer.block.width<=document.recipe.core.width+1e-7
				&& layer.block.depth<=document.recipe.core.depth+1e-7 && layer.block.height<=16+1e-7,
				"each explicit stock piece fits the declared EPS block");
			stock+=layer.block.width*layer.block.depth*layer.block.height;
		}
		check(stock>R.volume(MoldCores.foam(core),core.id),"cutting stock includes genuine offcuts outside the triangular foam");
		check(R.volume(core.foamEnvelope,core.id)>R.volume(MoldCores.foam(core),core.id),
			"inter-piece glue gaps remain distinct from EPS material");
		check(R.volume(core.cavity,core.id)>R.volume(core.foamEnvelope,core.id),
			"lining and the carrier glue line stay inside the finished cavity");
		var carrier=plan.supportPanels.filter(row -> row.panel.id==core.carrierId)[0];
		var carrierArea=Math.abs(materia.casting.CastingProfileGeometry.area(
			materia.tooling.panel.PanelContours.outside(carrier.panel)));
		var topArea=MoldCores.area(MoldCores.section(core.foamEnvelope,100-core.adhesiveThickness,core.id));
		check(Math.abs(carrierArea-topArea)<1e-6,"carrier follows the actual triangular contact footprint");
		check(core.bridgeBondAreaMm2>0 && core.bridgeBondAreaMm2<=core.bondAreaMm2+1e-7,
			"beam bonding credits only exact foam contact");
		var encoded=materia.tooling.casting.CastingMoldCodec.encode(plan);
		check(materia.tooling.casting.CastingMoldCodec.encode(machinekit.tooling.CastingMoldCad.decode(encoded))==encoded,
			"exact core cutting records regenerate through the strict mold codec");
		var review=machinekit.tooling.CastingMoldCad.review(plan);
		check(review.symmetricDifferenceMm3<1e-5,"non-box EPS reconstructs the source casting exactly");
		for (rule in ["coverage","assembly-graph","demolding-graph"])
			check(review.validation.filter(row -> row.rule=="tooling.cad."+rule && row.status==Status.Pass).length==1,
				"non-box EPS passes complete "+rule);
		var sections=machinekit.tooling.ToolingSectionReview.build(plan);
		check(sections.records.length==core.layers.length
			&& sections.validation.filter(row -> row.status!=Status.Pass).length==0,
			"every real EPS stock piece has exact finished section drawings");
		check(Math.abs(R.volume(core.cavity,core.id)-192000)<1e-5,
			"gross triangular cavity agrees with its independent analytic volume");
		var spec=ProcessTests.spec();spec.sections=[];
		spec.recipeFingerprint=ToolingCodec.recipeFingerprint(document.recipe);
		var process=materia.tooling.process.CastingProcessAnalysis.analyze(plan,spec,9.80665,null,
			machinekit.tooling.CastingFillAnalysis.fromPlan);
		var fill=process.fill;
		check(fill!=null,"non-box EPS keeps the native fill analysis in the shared process pipeline");
		if (fill==null) throw "Missing exact triangular-core fill report";
		var level=fill.levels[fill.levels.length-1];
		var uplift=level.cores.filter(row -> row.id==core.id)[0];
		check(Math.abs(uplift.submergedVolumeMm3-192000)<1e-5,
			"native uplift displaces the gross cavity, rather than foam tiles or stock blanks");
		check(process.castability.sourceFingerprint==process.sourceFingerprint
			&& process.castability.processFingerprint==process.processFingerprint,
			"native non-box processing preserves source-bound castability");
		check(process.castability.validation.filter(row -> row.rule=="process.castability.release.pocket"
			&& row.status==Status.Pending).length==1,
			"non-box EPS keeps its physical breakout qualification alongside exact fill evidence");
		check(process.castability.validation.filter(row -> row.rule=="process.castability.minimum-passage"
			&& row.status==Status.Unsupported).length==1,
			"native fill does not turn unsupported section measurements into a pass");
		check(process.sections==null && process.validation.filter(row -> row.status==Status.Unsupported
			&& StringTools.startsWith(row.rule,"process.sections.geometry")).length>0,
			"non-box orthogonal sections stay explicitly Unsupported");
		var ring=materia.casting.CastingRegion.Difference(
			materia.casting.CastingRegion.Placed({solid:Cut(materia.casting.CastingSolid.Cylinder(40,40)),pose:F.identity()}),
			[materia.casting.CastingRegion.Placed({solid:Cut(materia.casting.CastingSolid.Cylinder(20,40)),pose:F.identity()})]);
		var ringCore=materia.tooling.casting.MoldCoreLayers.build(ring,"annular-core",document.recipe)[0];
		var expectedRing=Math.PI*(19.9*19.9-10.1*10.1)*(40-0.1-0.2);
		check(Math.abs(R.volume(ringCore.foamEnvelope,ringCore.id)-expectedRing)<1e-5,
			"lining shrinks the outside and grows retained-material openings by their true normal thickness");
		variants(document,options,plan);
		packageEvidence(plan,process);
		Sys.println("RegionCorePlannerTests: "+assertions+" assertions passed");
	}
}
