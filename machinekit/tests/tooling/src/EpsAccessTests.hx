import materia.assembly.AssemblyFrames as F;
import materia.casting.CastingRegion;
import materia.tooling.casting.MoldRegions;
import materia.tooling.casting.MoldCoreLayers;
import materia.tooling.casting.MoldCores;
import machinekit.tooling.NativeCastingMoldKernel;

/** Access EPS retains its independent fixture notch and its registered flange. */
class EpsAccessTests {
	static var assertions=0;
	static function check(value:Bool,message:String):Void {
		assertions++;
		if (!value) throw message;
	}
	static function box(x:Float,y:Float,z:Float,w:Float,d:Float,h:Float):CastingRegion
		return MoldRegions.box({x:x,y:y,z:z,width:w,depth:d,height:h});

	static function options():materia.tooling.casting.CastingMoldPlan.CastingMoldOptions return {precision:[],replicationJigs:[],
		foundationJointProcesses:[],compaction:{method:Undeclared,bedInterfaceReference:null,
			frequencyHz:null,amplitudeMm:null,durationSeconds:null,evidence:[]},
		serviceOrientation:F.identity(),overlapRule:ReservedThenPriority,liftingEvidence:[]};

	static function fixture(stepped:Bool):Void {
		var kernel=new NativeCastingMoldKernel();
		materia.tooling.casting.CastingMoldPlanner.geometry=kernel;
		var recipe=SinglePocketSetup.document().recipe;
		recipe.core.width=1000;recipe.core.depth=500;recipe.core.height=100;
		var region:CastingRegion,frame:materia.assembly.AssemblyRecord.AssemblyFrame;
		if (stepped) {
			region=Union([box(0,0,0,100,100,10),box(20,20,10,60,60,20),box(0,0,30,100,100,10)]);
			frame=F.identity();
		} else {
			region=Difference(Union([box(0,0,0,80,90,100),box(-30,-10,-10,30,110,120)]),
				[box(-40,20,0,100,50,18)]);
			frame=F.fromRotationMatrix(0,0,0,[0,0,1,1,0,0,0,1,0]);
		}
		var core=MoldCoreLayers.registeredFlange(region,"access","core-access",recipe,frame,"west");
		check(core.support.kind==materia.tooling.casting.CoreSupport.CoreSupportKind.RegisteredFlange,
			"The access core uses one EPS type with a registered flange support");
		check(core.layers.length>1 && core.layers.length<=32,"Wire stock is split at exact profile changes and stock limits");
		check(MoldRegions.volume(Difference(core.foamEnvelope,[core.cavity]),core.id)<1e-6,
			"Lined foam stays inside the full registered access envelope");
		check(core.support.clampBearingAreaMm2>0,"Clamp bearing has a finite geometric limit, not a capacity claim");
		var pieces:Array<CastingRegion>=[];
		for (layer in core.layers) {
			check(kernel.prismaticSection(layer.solid,layer.wireDirection,1e-6,core.id)!=null,
				"Every stock piece is proved ruled in its declared wire direction");
			var piece=MoldCores.layerRegion(layer);
			check(MoldRegions.volume(Difference(piece,[core.foamEnvelope]),core.id)<1e-6,
				"Every wire-cut piece fits the globally lined envelope");
			for (previous in pieces) check(MoldRegions.volume(Intersection([piece,previous]),core.id)<1e-6,
				"EPS stock pieces have no overlapping interiors");
			pieces.push(piece);
		}
		check(MoldRegions.volume(Difference(core.foamEnvelope,[Union(pieces)]),core.id)>0,
			"Declared adhesive gaps remain distinct from the global covering");
		if (stepped) {
			var error="";
			try machinekit.tooling.FormerManufacturing.machining("undercut",region,F.identity(),recipe.panel,0,recipe)
			catch (caught:Dynamic) error=Std.string(caught);
			check(error.indexOf("Upper material blocks the cutter shaft")>=0,
				"Sharing the geometry bands does not remove the router's shaft-access rejection");
		}
	}
	static function bedAccess():Void {
		var document=SinglePocketSetup.document(18,false,true);
		var plan=materia.tooling.casting.CastingMoldPlanner.planSinglePocket(document);
		var feature=plan.document.input.features.filter(feature -> feature.id=="pocket")[0];
		var bed=plan.panels.members[0],oldSocket=plan.cores[0].support.socketOperationId;
		bed.panel.operations=bed.panel.operations.filter(operation -> operation.id!=oldSocket);
		bed.panel.contactOpenings=bed.panel.contactOpenings.filter(id -> id!=oldSocket);
		var sourceBefore=materia.tooling.ToolingCodec.encode(plan.document);
		var cavity=MoldRegions.feature(plan.document,feature.id);
		var volume:materia.tooling.casting.CastingMoldPlan.MoldVolume={id:"bed-access",material:FormerStock,
			pieces:[],profiles:[],cuts:[],regions:[cavity]};
		var extensions=materia.tooling.casting.MoldFormerAccess.registeredBedAccess(plan,feature,volume,[]);
		check(extensions.length==2 && volume.profiles.length==1,"Bed access includes a neck, flange and traced bed profile");
		check(volume.profiles[0].panelId=="bed","Bed access names its actual receiver");
		var operations=bed.panel.operations.filter(operation -> operation.id==volume.profiles[0].operationId);
		check(operations.length==1 && operations[0].through,"Bed access uses one through operation");
		check(materia.tooling.ToolingCodec.canonical(operations[0].contour)
			==materia.tooling.ToolingCodec.canonical(volume.profiles[0].contour),
			"EPS registration, CAD, drawing and CAM share the bed operation profile");
		check(MoldRegions.volume(Intersection([Union(extensions),MoldRegions.envelope(plan.document)]),feature.id)<1e-6,
			"Neither neck nor flange changes source casting material");
		check(sourceBefore==materia.tooling.ToolingCodec.encode(plan.document),"Bed access preserves the authored source and recipe");
		var core=MoldCoreLayers.registeredFlange(MoldRegions.tooling(volume),feature.id,"core-bed-access",
			plan.document.recipe,F.identity(),"bed");
		check(core.layers.length>0 && core.support.clampBearingAreaMm2>0,"Bed EPS retains the shared ruled cutting and restraint route");
		var wrong=materia.tooling.casting.CastingMoldPlanner.planSinglePocket(SinglePocketSetup.document());
		var wrongFeature=wrong.document.input.features.filter(feature -> feature.id=="pocket")[0],message="";
		try materia.tooling.casting.MoldFormerAccess.registeredBedAccess(wrong,wrongFeature,volume,[])
		catch (error:Dynamic) message=Std.string(error);
		check(message.indexOf("would change source casting geometry")>=0,"Bed access cannot invent an opening through source resin");
	}

	static function overlapOwnership():Void {
		var document=SinglePocketSetup.document();
		document.input.inserts=[];
		document.input.features=[];
		document.setup.sourceToMold=F.identity();
		var definitions=[{id:"bed-void",z:0.0,y:0.0,strategy:materia.tooling.casting.CastingSetup.CoreStrategy.SacrificialEPS,
			priority:10,sign:-1.0,face:"body.z-min"},
			{id:"fill-tool",z:40.0,y:0.0,strategy:materia.tooling.casting.CastingSetup.CoreStrategy.ReusableFormer,
				priority:20,sign:1.0,face:"body.z-max"}];
		for (item in definitions) document.input.features.push({id:item.id,
			kind:materia.casting.CastingFeature.CastingFeatureKind.ServiceOpening,
			geometry:{solid:materia.casting.CastingFeature.CastingGeometrySolid.Cut(materia.casting.CastingSolid.Box(40,40,60)),
				pose:F.translation(item.id=="bed-void" ? -10 : 10,item.y,item.z)},
			openings:[materia.casting.CastingPatterns.face(item.face)],draft:{degrees:0,axis:{x:0,y:0,z:1}},
			cornerRadius:0,owner:null,adjustable:[],derivedFrom:null});
		document.setup.features=[for (item in definitions) {featureId:item.id,strategy:item.strategy,
			priority:item.priority,withdrawal:{x:0.0,y:0.0,z:item.sign}}];
		document.setup.sourceFingerprint=materia.tooling.ToolingCodec.inputFingerprint(document.input);

		var plan=machinekit.tooling.CastingMoldCad.plan(document,options());
		check(plan.cores.length==1 && plan.cores[0].featureId=="bed-void","Only the authored EPS owner becomes a core");
		var rigid=plan.formers.filter(former -> former.featureIds.indexOf("fill-tool")>=0);
		check(rigid.length==1 && rigid[0].featureIds.indexOf("bed-void")>=0,
			"Higher-priority rigid tool retains both cavity attributions without becoming EPS");
		check(plan.features.filter(feature -> feature.featureId=="bed-void")[0].memberIds.indexOf(rigid[0].volume.id)>=0,
			"Shared cavity coverage still names the rigid neighbour");
		var source=machinekit.tooling.CastingMoldCad.source(plan),rebuilt=machinekit.tooling.CastingMoldCad.reconstructed(plan);
		var missing=source.subtractAll([rebuilt]),excess=rebuilt.subtractAll([source]);
		check(missing.volume()+excess.volume()<1e-5,"Mixed-strategy ownership preserves exact source resin");
		missing.close();excess.close();source.close();rebuilt.close();
	}

	static function rotatedBridge():Void {
		var document=SinglePocketSetup.document();
		document.input.inserts=[];
		document.input.features=document.input.features.filter(feature -> feature.kind!=materia.casting.CastingFeature.CastingFeatureKind.InsertRecess);
		document.recipe.hardware.minimumBridgeWidth=30;
		document.setup.sourceFingerprint=materia.tooling.ToolingCodec.inputFingerprint(document.input);

		var plan=machinekit.tooling.CastingMoldCad.plan(document,options());
		plan.panels=materia.tooling.casting.MoldPanels.box(plan.cavity,document.recipe,
			materia.tooling.casting.MoldPanels.RaisedWalls.NorthSouth).panels;
		plan.supportPanels=[];plan.hardware=[];
		materia.tooling.casting.CoreSupports.sharedBridges(plan.cavity,document.recipe,plan.cores,
			plan.supportPanels,plan.hardware,materia.tooling.casting.MoldPanels.RaisedWalls.NorthSouth);
		var review=machinekit.tooling.CastingMoldCad.review(plan);
		check(review.symmetricDifferenceMm3<1e-5,"Rotated bridge keeps exact resin reconstruction");
		for (rule in ["assembly-graph","demolding-graph"])
			check(review.validation.filter(row -> row.rule=="tooling.cad."+rule && row.status=="pass").length==1,
				"Rotated bridge clears the real native "+rule+", including long-wall corner laps");
	}

	static function transverseBore():Void {
		var scope=new cadkit.modeling.Scope();
		try {
			var outer=scope.own(cadkit.modeling.Part.cylinder(5,8)),hole=scope.own(cadkit.modeling.Part.cylinder(3,8));
			var tube=scope.own(outer.subtractAll([hole])),delta=new cadkit.modeling.Vector(20,0,0);
			var swept=scope.own(machinekit.tooling.SweptRemoval.sweep(tube,delta));
			var expected=scope.own(machinekit.tooling.SweptRemoval.sweep(outer,delta));
			var missing=scope.own(expected.subtractAll([swept])),excess=scope.own(swept.subtractAll([expected]));
			check(missing.volume()+excess.volume()<1e-5,"Long transverse bore sweep agrees with the filled exterior sweep");
			check(tube.valid(),"A transverse bore sweep keeps the borrowed source valid");
			scope.close();
		} catch (error:Dynamic) {scope.close();throw error;}
	}

	static function sideMaskAssembly():Void {
		var document=SinglePocketSetup.document(),insert=document.input.inserts[0];
		var size=materia.casting.InsertSpec.InsertBodyCodec.box(insert.body,insert.id);
		var right=Math.NEGATIVE_INFINITY;
		for (point in document.input.envelope[0].profile) right=Math.max(right,point.x);
		insert.pose=F.fromRotationMatrix(right-size.height,0,50,[0,0,1,0,1,0,-1,0,0]);
		insert.holes=TestInsertHoles.through(5,size.height);TestInsertHoles.beforeCasting(insert);
		var recess=document.input.features.filter(feature -> feature.kind==materia.casting.CastingFeature.CastingFeatureKind.InsertRecess)[0];
		recess.geometry.pose=insert.pose;recess.openings=[materia.casting.CastingPatterns.face("body.x-max")];
		document.input.features=[recess,{id:"fixing-access",kind:materia.casting.CastingFeature.CastingFeatureKind.ServiceOpening,
			geometry:{solid:materia.casting.CastingFeature.CastingGeometrySolid.Cut(materia.casting.CastingSolid.Box(25,40,70)),
				pose:F.translation(right-size.height-12.5,0,30)},openings:[materia.casting.CastingPatterns.face("body.z-max")],
			draft:{degrees:0,axis:{x:0,y:0,z:1}},cornerRadius:0,owner:null,adjustable:[],derivedFrom:null}];
		document.setup.sourceToMold=F.identity();
		document.setup.features=[{featureId:"fixing-access",strategy:materia.tooling.casting.CastingSetup.CoreStrategy.ReusableFormer,
			priority:10,withdrawal:{x:0,y:0,z:1}}];
		document.setup.sourceFingerprint=materia.tooling.ToolingCodec.inputFingerprint(document.input);
		var process=options();
		process.precision=[{insertId:insert.id,jigReference:null,masterReference:null,masterFlatnessMm:null,
			releaseSpec:null,holeProtectionReference:null,evidence:[]}];
		var plan=machinekit.tooling.CastingMoldCad.plan(document,process),review=machinekit.tooling.CastingMoldCad.review(plan);
		var mask=plan.fixtures[0].boreMasks[0].hardwareId,steel="insert-"+insert.id;
		var motion=review.assemblyMotions.filter(motion -> motion.id==mask);
		check(motion.length==1 && Math.abs(motion[0].x-1)<1e-8 && Math.abs(motion[0].y)+Math.abs(motion[0].z)<1e-8,
			"Side-facing mask assembly follows its source-hole withdrawal axis");
		check(review.assembly.blocking.filter(edge -> edge.movingId==mask && edge.blockerId==steel).length==0,
			"The mask pulls through its actual bore without an invented transverse blocker");
		check(review.assembly.blocking.filter(edge -> edge.movingId==steel && edge.blockerId==mask).length==1,
			"The exact graph still requires the insert before its side mask during assembly");
	}

	static function main():Void {
		transverseBore();
		fixture(false);fixture(true);
		bedAccess();
		overlapOwnership();
		rotatedBridge();
		sideMaskAssembly();
		Sys.println('EpsAccessTests: $assertions assertions passed');
	}
}
