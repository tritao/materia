import materia.assembly.AssemblyFrames as F;
import materia.casting.CastingRegion;
import materia.casting.BoxAlgebra;
import materia.tooling.casting.MoldRegions;
import materia.tooling.casting.MoldCoreLayers;
import materia.tooling.casting.MoldCores;
import machinekit.tooling.NativeCastingMoldKernel;

/** Access EPS retains its independent fixture notch and its registered flange. */
@:access(machinekit.tooling.CastingMoldCad)
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
		core.support.registrationProfiles=volume.profiles;
		plan.cores=[core];
		var fits=machinekit.tooling.CastingMoldCad.corePanelFit(plan);
		check(fits.length==1 && fits[0].status=="pass"
			&& fits[0].affectedIds.indexOf(core.id)>=0 && fits[0].affectedIds.indexOf("bed")>=0,
			"Exact relieved neck and outside flange clear the actual machined bed");
		var layer=core.layers[0],original=layer.solid;
		var intrusion=MoldRegions.moved(box(14,14,8,2,2,2),bed.pose);
		layer.solid=Union([original,MoldRegions.moved(intrusion,F.inverse(layer.stockToMold))]);
		var fitError="";
		try machinekit.tooling.CastingMoldCad.corePanelFit(plan)
		catch (error:Dynamic) fitError=Std.string(error);
		check(fitError.indexOf("EPS access intersects receiver: "+core.id+"/bed")>=0,
			"Oversized finished foam fails even when its nominal sealed envelope still fits");
		layer.solid=original;
		var cavityBefore=core.cavity;
		core.cavity=Union([cavityBefore,intrusion]);
		fitError="";
		try machinekit.tooling.CastingMoldCad.corePanelFit(plan)
		catch (error:Dynamic) fitError=Std.string(error);
		check(fitError.indexOf("EPS access intersects receiver: "+core.id+"/bed")>=0,
			"An oversized sealing neck fails even when the actual foam still fits");
		core.cavity=cavityBefore;
		check(machinekit.tooling.CastingMoldCad.corePanelFit(plan).length==1,
			"Restoring only the offending geometry restores the exact fit proof");
		var wrong=materia.tooling.casting.CastingMoldPlanner.planSinglePocket(SinglePocketSetup.document());
		var wrongFeature=wrong.document.input.features.filter(feature -> feature.id=="pocket")[0],message="";
		try materia.tooling.casting.MoldFormerAccess.registeredBedAccess(wrong,wrongFeature,volume,[])
		catch (error:Dynamic) message=Std.string(error);
		check(message.indexOf("would change source casting geometry")>=0,"Bed access cannot invent an opening through source resin");
	}

	static function sideRegistration():Void {
		var document=SinglePocketSetup.document();
		document.input.inserts=[];document.input.features=[];
		document.setup.sourceToMold=F.identity();
		var bounds=MoldRegions.exactBounds(MoldRegions.envelope(document),"side-registration");
		var feature:materia.casting.CastingFeature={id:"side-pocket",kind:ServiceOpening,
			geometry:{solid:Cut(materia.casting.CastingSolid.Box(30,40,30)),
				pose:F.translation(0,bounds.y+bounds.depth-20,30)},
			openings:[materia.casting.CastingPatterns.face("body.y-max")],
			draft:{degrees:0,axis:{x:0,y:0,z:1}},cornerRadius:0,owner:null,adjustable:[]};
		document.input.features=[feature];
		ToolingTestRules.set(document,feature.id,SacrificialEPS,10,{x:0,y:1,z:0});
		document.setup.sourceFingerprint=materia.tooling.ToolingCodec.inputFingerprint(document.input);
		var plan=machinekit.tooling.CastingMoldCad.plan(document,options());
		check(plan.cores.length==1 && plan.cores[0].support.kind==materia.tooling.casting.CoreSupport.CoreSupportKind.RegisteredFlange,
			"Sharp side-open EPS uses its matched dry registration neck");
		var core=plan.cores[0],profiles=core.support.registrationProfiles;
		check(profiles.length==1 && profiles[0].panelId=="north","Side registration names its actual receiver");
		var wall=plan.panels.members.filter(member -> member.panel.id=="north")[0];
		var operation=wall.panel.operations.filter(operation -> operation.id==profiles[0].operationId)[0];
		check(operation.reliefToolDiameter==document.recipe.routerDiameter,
			"The declared relief cutter is shared by the receiver and the EPS neck");
		check(materia.tooling.ToolingCodec.canonical(operation.contour)==materia.tooling.ToolingCodec.canonical(profiles[0].contour),
			"Side EPS, drawings and CAM use the same exact operation contour");
		check(plan.document.input.features[0].cornerRadius==0,"Dry relief does not round the source cavity");
		var source=machinekit.tooling.CastingMoldCad.source(plan),rebuilt=machinekit.tooling.CastingMoldCad.reconstructed(plan);
		var missing=source.subtract(rebuilt),extra=rebuilt.subtract(source);
		check(missing.volume()+extra.volume()<1e-5,"Relieved side registration preserves exact source resin");
		missing.close();extra.close();source.close();rebuilt.close();
		for (fit in machinekit.tooling.CastingMoldCad.corePanelFit(plan))
			check(fit.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass,"Side EPS fits the real machined receiver");
	}

	static function rebatedSteel():Void {
		var document=SinglePocketSetup.document(),insert=document.input.inserts[0];
		insert.body=materia.casting.InsertSpec.InsertBody.Primitive(materia.casting.CastingSolid.Box(40,35,19));
		insert.interfaces=[{id:"finished-seat",geometry:materia.casting.InterfaceRequirement.PrecisionGeometry.Plane(F.translation(0,0,15)),
			route:materia.casting.InterfaceRequirement.InterfaceRoute.Ground,allowanceMm:4,
			finishingProcessErrorMm:materia.casting.CastingInput.Unresolved("Finishing error allocation"),
			permittedFinishes:[materia.casting.InterfaceRequirement.InterfaceFinish.Ground],positionToleranceMm:0.01,
			datumReference:"primary",relationshipIds:[]}];
		var recess=document.input.features.filter(feature -> feature.owner==insert.id)[0];
		recess.geometry.solid=materia.casting.CastingFeature.CastingGeometrySolid.Insert(insert.body);
		document.setup.sourceFingerprint=materia.tooling.ToolingCodec.inputFingerprint(document.input);
		var process=options();
		process.precision=[{insertId:insert.id,jigReference:null,masterReference:null,masterFlatnessMm:null,
			releaseSpec:null,holeProtectionReference:null,evidence:[]}];
		var plan=machinekit.tooling.CastingMoldCad.plan(document,process);
		check(Math.abs(plan.cavity.z-plan.envelope.z)<1e-8,"Proud steel does not lower the mold bed");
		check(plan.formers.filter(former -> StringTools.startsWith(former.volume.id,"bed-backing-")).length==0,
			"The rebated bed needs no sliver backing formers");
		check(plan.seals.length==1,"Routed clearance and corner relief have an explicit cured closure");
		var closure=machinekit.tooling.ToolingVolume.fromVolume(plan.seals[0].volume,plan.seals[0].materialSpec);
		check(closure.materialSpec()=="unspecified" && closure.parameterToken("stockMaterial")==plan.seals[0].materialSpec,
			"Unresolved seal chemistry remains a shop reference, with no fabricated physical material claim");
		var bed=plan.panels.members[0],pockets=bed.panel.operations.filter(operation -> operation.id==materia.tooling.ToolingIds.derive("steel-rebate-",insert.id,"-bed"));
		check(pockets.length==1 && pockets[0].depth==4 && !pockets[0].through,"Bed rebate matches the raw proud finishing stock");
		var fixture=plan.fixtures.filter(fixture -> fixture.insertId==insert.id)[0];
		check(materia.tooling.casting.CastingMoldPlan.MoldFixtures.panelId(fixture)=="bed" && fixture.fixings.length==1,
			"The insert fixes through its rebated bed into its own declared source thread");
		var hardware=plan.hardware.filter(hardware -> hardware.id==fixture.fixings[0].hardwareId)[0];
		var hole=materia.tooling.casting.InsertFixturePlanner.availableHoles(insert)[0];
		var selected=materia.tooling.casting.InsertFixturePlanner.select(hole,F.compose(document.setup.sourceToMold,insert.pose),
			fixture.mountingFace,document.recipe.hardware);
		check(selected!=null && Math.abs(hardware.length-(14+selected.insertion))<1e-8,
			"Fixing length uses the remaining 14 mm board stack, not the full 18 mm blank");
		var drill=bed.panel.operations.filter(operation -> operation.id==fixture.fixings[0].operationId)[0];
		check(drill.accessVia==pockets[0].id,"CAM cuts the receiver before its dependent clearance bore");
		var review=machinekit.tooling.CastingMoldCad.review(plan);
		check(review.symmetricDifferenceMm3<1e-5,"Steel rebate and closure preserve exact source resin");
		check(review.assemblyMotions.filter(motion -> motion.id=="bed-sealed").length==1
			&& review.assemblyMotions.filter(motion -> motion.id=="insert-"+insert.id).length==1,
			"Bed-mounted steel installs separately; side-receiver preassembly does not load the whole bed");
		check(review.permanentGroups.filter(group -> group.members.indexOf("bed")>=0
			&& group.members.indexOf(plan.seals[0].volume.id)>=0).length==1,"Cured closure moves with its bed in the exact removal graphs");
		var profile=plan.seals[0].volume.profiles[0],pocket=MoldRegions.profile(profile);
		// Leave only a positive-volume bottom shard: the traced operation is
		// unchanged, but the actual closure no longer reaches the wet boundary.
		profile.subtract=[Difference(pocket,[box(100.05,-0.05,-4,0.1,0.1,1)])];
		var failure="";try machinekit.tooling.CastingMoldCad.review(plan) catch(error:Dynamic) failure=Std.string(error);
		check(failure.indexOf("Uncovered mold boundary")>=0,"Removing the real closure exposes the leak to the existing exact coverage check");
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
			kind:item.id=="bed-void" ? materia.casting.CastingFeature.CastingFeatureKind.Lightening : materia.casting.CastingFeature.CastingFeatureKind.ServiceOpening,
			geometry:{solid:materia.casting.CastingFeature.CastingGeometrySolid.Cut(materia.casting.CastingSolid.Box(40,40,60)),
				pose:F.translation(item.id=="bed-void" ? -10 : 10,item.y,item.z)},
			openings:[materia.casting.CastingPatterns.face(item.face)],draft:{degrees:0,axis:{x:0,y:0,z:1}},
			cornerRadius:0,owner:null,adjustable:[]});
		for(item in definitions)ToolingTestRules.set(document,item.id,item.strategy,item.priority,{x:0.0,y:0.0,z:item.sign});
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

	static function elevatedThroughMask():Void {
		var hole=TestInsertHoles.through(8,20)[0];
		var protection:materia.tooling.casting.CastingMoldPlan.BoreMaskProtection={
			sourceHoleId:hole.id,hardwareId:"mask",plugLength:null,farEndId:null,withdrawal:{x:0,y:0,z:-1}};
		var volume:materia.tooling.casting.CastingMoldPlan.MoldVolume={id:"datum",material:materia.tooling.casting.CastingMoldPlan.MoldPartMaterial.FormerStock,
			pieces:[],profiles:[],cuts:[],regions:[]};
		var support:materia.tooling.casting.CastingMoldPlan.MoldFixtureSupport=
			OnDatum(volume,{x:0,y:0,z:-1},{primaryId:"pin",secondaryId:"slot",baselineMm:20});
		var direction=machinekit.tooling.CastingMoldCad.maskAssemblyDirection(hole,protection,support);
		check(direction.z==1,"A plain through mask enters from the open far end above its bed datum");
		var scope=new cadkit.modeling.Scope();
		try {
			var bed=scope.own(cadkit.modeling.Part.box(100,100,18));
			bed=scope.own(bed.translated(new cadkit.modeling.Vector(0,0,-18)));
			var datum=scope.own(cadkit.modeling.Part.box(40,40,40));
			var blank=scope.own(cadkit.modeling.Part.box(60,60,20));
			var bore=scope.own(cadkit.modeling.Part.cylinder(4,20));
			var steel=scope.own(blank.subtract(bore));
			steel=scope.own(steel.translated(new cadkit.modeling.Vector(0,0,40)));
			var mask=scope.own(bore.translated(new cadkit.modeling.Vector(0,0,40)));
			var members:Array<machinekit.tooling.SweptRemoval.RemovalMember>=[
				{id:"bed",part:bed,direction:new cadkit.modeling.Vector(0,0,1)},
				{id:"datum",part:datum,direction:new cadkit.modeling.Vector(0,0,1)},
				{id:"steel",part:steel,direction:new cadkit.modeling.Vector(0,0,1)},
				{id:"mask",part:mask,direction:direction}];
			var assembly=machinekit.tooling.SweptRemoval.analyze(members,1e-4);
			var edges=[for(edge in assembly.blocking)edge.movingId+"->"+edge.blockerId];edges.sort(Reflect.compare);
			check(edges.join(",")=="bed->datum,bed->mask,bed->steel,datum->mask,datum->steel",
				"Elevated datum assembly has the exact expected blocking edges");
			for(member in members)if(member.id!="steel")member.direction=new cadkit.modeling.Vector(0,0,-1);
			var removal=machinekit.tooling.SweptRemoval.analyze(members,1e-4);
			edges=[for(edge in removal.blocking)edge.movingId+"->"+edge.blockerId];edges.sort(Reflect.compare);
			check(edges.join(",")=="datum->bed,mask->bed,mask->datum",
				"After cure the bed and datum release before the entry-side mask");
			scope.close();
		} catch(error:Dynamic) {scope.close();throw error;}
		hole.counterbore={diameter:10,depth:2};
		check(machinekit.tooling.CastingMoldCad.maskAssemblyDirection(hole,protection,support).z== -1,
			"A stepped mask cannot be installed through its smaller far end");
		hole.counterbore=null;hole.extent=materia.casting.InsertHole.HoleExtent.Blind;
		check(machinekit.tooling.CastingMoldCad.maskAssemblyDirection(hole,protection,support).z== -1,
			"A blind mask has only entry-side access");
	}

	static function sideReceiverDocument(proud:Float):materia.tooling.ToolingDocument {
		var document=SinglePocketSetup.document(),insert=document.input.inserts[0];
		var size=materia.casting.InsertSpec.InsertBodyCodec.box(insert.body,insert.id);
		var right=Math.NEGATIVE_INFINITY;
		for (point in document.input.envelope[0].profile) right=Math.max(right,point.x);
		insert.pose=F.fromRotationMatrix(right-size.height+proud,0,50,[0,0,1,0,1,0,-1,0,0]);
		insert.holes=TestInsertHoles.through(5,size.height);TestInsertHoles.beforeCasting(insert);
		var recess=document.input.features.filter(feature -> feature.kind==materia.casting.CastingFeature.CastingFeatureKind.InsertRecess)[0];
		recess.geometry.pose=insert.pose;recess.openings=[materia.casting.CastingPatterns.face("body.x-max")];
		document.input.features=[recess,{id:"fixing-access",kind:materia.casting.CastingFeature.CastingFeatureKind.ServiceOpening,
			geometry:{solid:materia.casting.CastingFeature.CastingGeometrySolid.Cut(materia.casting.CastingSolid.Box(25,40,70)),
				pose:F.translation(right-size.height-12.5+proud,0,30)},openings:[materia.casting.CastingPatterns.face("body.z-max")],
			draft:{degrees:0,axis:{x:0,y:0,z:1}},cornerRadius:0,owner:null,adjustable:[]}];
		document.setup.sourceToMold=F.identity();
		ToolingTestRules.set(document,"fixing-access",materia.tooling.casting.CastingSetup.CoreStrategy.ReusableFormer,10,{x:0,y:0,z:1});
		document.setup.sourceFingerprint=materia.tooling.ToolingCodec.inputFingerprint(document.input);
		return document;
	}

	static function sideMaskAssembly(proud:Float=0):Void {
		var document=sideReceiverDocument(proud),insert=document.input.inserts[0];
		var process=options();
		process.precision=[{insertId:insert.id,jigReference:null,masterReference:null,masterFlatnessMm:null,
			releaseSpec:null,holeProtectionReference:null,evidence:[]}];
		var plan=machinekit.tooling.CastingMoldCad.plan(document,process),review=machinekit.tooling.CastingMoldCad.review(plan);
		var mask=plan.fixtures[0].boreMasks[0].hardwareId,steel="insert-"+insert.id;
		if(proud>0) {
			var seals=plan.seals.filter(seal->seal.panelId=="east" && seal.insertId==insert.id);
			check(seals.length==1,"Proud wall steel has the same exact clearance/relief closure as bed steel");
			var east=plan.panels.members.filter(member->member.panel.id=="east")[0];
			check(east.withdrawal.x>1-1e-8 && Math.abs(east.withdrawal.y)+Math.abs(east.withdrawal.z)<1e-8,
				"Proud side receivers pull outward after their bed-slot restraint releases");
			var pocket=east.panel.operations.filter(operation->operation.id==seals[0].volume.profiles[0].operationId)[0];
			check(pocket.depth==proud && pocket.side==materia.tooling.ToolingEnums.MachiningSide.NegativeZ,
				"The wall rebate is machined from its actual resin-facing side");
			var access=east.panel.operations.filter(operation->operation.id==materia.tooling.ToolingIds.derive("access-",mask))[0];
			check(access.side==pocket.side && access.setupId==pocket.setupId && access.accessVia==pocket.id,
				"Negative-face bore access follows its actual receiver-floor pocket");
			check(review.permanentGroups.filter(group->group.members.indexOf("east")>=0 && group.members.indexOf(seals[0].volume.id)>=0).length==1,
				"The wall and its cured closure stay together in the exact removal graph");
		}
		var motion=review.assemblyMotions.filter(motion -> motion.id==mask);
		check(motion.length==1 && Math.abs(motion[0].x-1)<1e-8 && Math.abs(motion[0].y)+Math.abs(motion[0].z)<1e-8,
			"Side-facing mask assembly follows its source-hole withdrawal axis");
		check(review.assembly.blocking.filter(edge -> edge.movingId==mask && edge.blockerId==steel).length==0,
			"The mask pulls through its actual bore without an invented transverse blocker");
		if(proud==0)check(review.assembly.blocking.filter(edge -> edge.movingId==steel && edge.blockerId==mask).length==1,
			"The exact graph still requires the insert before its side mask during vertical assembly");
		else {
			var steelMotion=review.assemblyMotions.filter(motion->motion.id==steel);
			check(steelMotion.length==1 && steelMotion[0].x< -1+1e-8,
				"Proud steel installs laterally into its wall receiver before casting");
		}
	}

	static function clampedSideReceiver():Void {
		var document=sideReceiverDocument(4),insert=document.input.inserts[0];
		var size=materia.casting.InsertSpec.InsertBodyCodec.box(insert.body,insert.id);
		insert.holes=machinekit.structural.InsertHoles.records(machinekit.structural.InsertHoles.tappedBlind(
			machinekit.structural.InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(6).mating()),size.height));
		TestInsertHoles.beforeCasting(insert);
		document.setup.sourceFingerprint=materia.tooling.ToolingCodec.inputFingerprint(document.input);
		var process=options();
		process.precision=[{insertId:insert.id,jigReference:null,masterReference:null,masterFlatnessMm:null,
			releaseSpec:null,holeProtectionReference:null,evidence:[]}];
		var plan=machinekit.tooling.CastingMoldCad.plan(document,process),review=machinekit.tooling.CastingMoldCad.review(plan);
		check(plan.fixtures[0].fixings.length==1,"Source steel thread clamps the insert to its wall");
		var groups=review.assemblyGroups.filter(group->group.members.indexOf("east-sealed")>=0
			&& group.members.indexOf("insert-"+insert.id)>=0);
		check(groups.length==1,"A screw-fixed side insert preassembles with its receiver panel");
		var motion=review.assemblyMotions.filter(motion->motion.id==groups[0].id);
		check(motion.length==1 && motion[0].z==1,"The loaded wall installs vertically in its bed slot");
		var wall=review.demoldingMotions.filter(motion->motion.id=="east-sealed");
		check(wall.length==1 && wall[0].x==1,"After unclamping, the wall withdraws outward from cured steel");
	}

	/** Keep the generic shared-row check when deleting the obsolete strut pads. */
	static function overlappingBridgeRows():Void {
		var recipe=SinglePocketSetup.document().recipe;
		recipe.hardware.minimumBridgeWidth=80;
		var cavity={x:-300.0,y:-200.0,z:0.0,width:600.0,depth:400.0,height:60.0};
		var cores=[for(index in 0...2) MoldCoreLayers.box({x:index==0 ? -180.0 : 100.0,y:-40.0+20*index,
			z:0.0,width:80.0,depth:80.0,height:60.0},"row-"+index,recipe,
			materia.tooling.casting.CoreSupport.CoreSupportKind.Bridge)];
		var panels:Array<materia.tooling.panel.PanelAssemblyPlan.PanelPlacement>=[];
		var hardware:Array<materia.tooling.casting.CoreSupport.SupportHardware>=[];
		materia.tooling.casting.CoreSupports.sharedBridges(cavity,recipe,cores,panels,hardware);
		check(panels.length==5 && hardware.length==4,"Overlapping beam bands share two rails, one beam and two carriers");
		var id=MoldCores.bridgeId(cores[0]);
		check(id==MoldCores.bridgeId(cores[1]),"Different row centres can share one physical beam");
		var beam=panels.filter(panel->panel.panel.id==id)[0];
		var bounds=materia.casting.CastingProfileGeometry.bounds(materia.tooling.panel.PanelContours.outside(beam.panel));
		check(bounds.height>=100-1e-7,"The common beam covers both minimum-width bands");
	}

	static function contactAndLocatingFoot():Void {
		var recipe=SinglePocketSetup.document().recipe;
  // A socket must not use the bounding rectangle over an internal datum hole
  // or across a re-entrant branch of the actual foam contact.
  var profile=materia.casting.CastingProfiles.rectangle(0,0,120,100);
  var hole=materia.casting.CastingProfiles.rectangle(45,35,30,30);
  var receiver={x:0.0,y:0.0,z:0.0,width:120.0,depth:100.0,height:1.0};
  var patch=materia.tooling.casting.EPSAccessNetworks.contactRectangle([
   {depth:0,contour:profile},{depth:1,contour:hole}],receiver,'contact-with-datum',20);
  if(BoxAlgebra.intersection(patch,{x:45.0,y:35.0,z:0.0,width:30.0,depth:30.0,height:1.0})!=null)
   throw 'Certified EPS contact includes an internal datum hole';
  var concave=materia.tooling.panel.PanelContours.polygon([
   {x:0.0,y:0.0},{x:120.0,y:0.0},{x:120.0,y:40.0},
   {x:40.0,y:40.0},{x:40.0,y:100.0},{x:0.0,y:100.0}]);
  patch=materia.tooling.casting.EPSAccessNetworks.contactRectangle([{depth:0,contour:concave}],receiver,'contact-with-branch',20);
  if(BoxAlgebra.volume(BoxAlgebra.subtract([patch],[
   {x:0.0,y:0.0,z:0.0,width:120.0,depth:40.0,height:1.0},
   {x:0.0,y:40.0,z:0.0,width:40.0,depth:60.0,height:1.0}]))>1e-7)
   throw 'Certified EPS foot crosses a concave branch: '+haxe.Json.stringify(patch);
  var gapCore=materia.tooling.casting.MoldCoreLayers.box(
   {x:20,y:20,z:10,width:100,depth:100,height:20},'bed-gap',recipe,
   materia.tooling.casting.CoreSupport.CoreSupportKind.RoutedSocket);
  var bed={panel:materia.tooling.panel.PanelJoints.blank('gap-bed',160,160,recipe.panel,
   recipe.routerDiameter,recipe.minimumLigament),
   pose:materia.assembly.AssemblyFrames.translation(0,0,-recipe.panel.measuredThickness),
   withdrawal:{x:0.0,y:0.0,z:-1.0}};
  var foam=materia.tooling.casting.MoldCores.singleBox(gapCore.foamEnvelope,gapCore.id);
  materia.tooling.casting.CoreSupports.socketFoot(gapCore,
   {x:foam.x,y:foam.y,width:foam.width,depth:foam.depth},
   {x:20.0,y:20.0,width:100.0,depth:100.0},0,10,recipe,bed,[]);
  var foot=gapCore.support.foot;
  if(foot==null || foot.foam.z>=0 || Math.abs(foot.foam.z+foot.foam.height-10+recipe.adhesiveLineThickness)>1e-7)
   throw 'EPS locating foot must span the dry bed gap and stop one glue line below the foam';
  var occupied=materia.tooling.casting.MoldRegions.boxes(materia.tooling.casting.MoldCores.occupied(gapCore));
  var glue={x:foot.foam.x,y:foot.foam.y,z:foot.foam.z+foot.foam.height,
   width:foot.foam.width,depth:foot.foam.depth,height:recipe.adhesiveLineThickness};
  if(occupied==null || BoxAlgebra.volume(BoxAlgebra.subtract([glue],occupied))>1e-7)
   throw 'Backing must leave the declared foot-to-core glue line free of wood';
  var jobs=materia.tooling.casting.MoldCores.cuttingJobs(gapCore);
  if(jobs.length!=gapCore.layers.length+1 || jobs[jobs.length-1].cutting.block.height<10)
   throw 'The separate gap-spanning locating foot must be supplied in the wire-cut stock jobs';
  gapCore.support.assemblyByLayer=true;gapCore.support.breakoutAfterLift=true;
  var half=(foam.width-recipe.adhesiveLineThickness)/2;
  gapCore.layers=[for(right in [false,true])materia.tooling.casting.CoreSupports.cutting({
   x:foam.x+(right?half+recipe.adhesiveLineThickness:0),y:foam.y,z:foam.z,
   width:half,depth:foam.depth,height:foam.height},recipe.coreWireAxis)];
  var baseMembers=materia.tooling.casting.MoldCores.baseAssemblyMembers(gapCore);
  if(baseMembers.length!=2 || baseMembers.indexOf(gapCore.id)<0
   || materia.tooling.casting.MoldCores.assemblyMembers(gapCore).length!=3)
   throw 'A locating foot spanning two stock tiles must prebond both actual contacts';
  var topCore=materia.tooling.casting.MoldCoreLayers.box(
   {x:0,y:0,z:0,width:60,depth:60,height:50},'layered-top',recipe,
   materia.tooling.casting.CoreSupport.CoreSupportKind.Bridge);
  var topFoam=materia.tooling.casting.MoldCores.singleBox(topCore.foamEnvelope,topCore.id);
  var halfHeight=(topFoam.height-recipe.adhesiveLineThickness)/2;
  topCore.layers=[for(upper in [false,true])materia.tooling.casting.CoreSupports.cutting({
   x:topFoam.x,y:topFoam.y,z:topFoam.z+(upper?halfHeight+recipe.adhesiveLineThickness:0),
   width:topFoam.width,depth:topFoam.depth,height:halfHeight},recipe.coreWireAxis)];
  topCore.support.assemblyByLayer=true;
  if(materia.tooling.casting.MoldCores.assemblyAnchor(topCore)!=1
   || materia.tooling.casting.MoldCores.bridgeAssemblyMembers(topCore).join(',')!=topCore.id)
   throw 'Only the actual carrier-facing stock preassembles with a bridge';
		assertions+=7;
	}

	static function declaredBedNetwork():Void {
		var document=SinglePocketSetup.document();
		document.input.inserts=[];
		document.setup.sourceToMold=F.identity();
		document.input.features=[for(index in 0...2) {
			id:index==0?"shaft":"drive-channel",kind:materia.casting.CastingFeature.CastingFeatureKind.ServiceOpening,
			geometry:{solid:materia.casting.CastingFeature.CastingGeometrySolid.Cut(materia.casting.CastingSolid.Box(60,index==0?60:30,index==0?50:30)),
				pose:F.translation(index==0?-20:40,0,index==0?0:10)},
			openings:index==0 ? [materia.casting.CastingPatterns.face("body.z-min"),materia.casting.CastingPatterns.feature("drive-channel")]
				: [materia.casting.CastingPatterns.feature("shaft")],draft:{degrees:0,axis:{x:0,y:0,z:1}},
			cornerRadius:0,owner:null,adjustable:[]
		}];
		for(feature in document.input.features)ToolingTestRules.set(document,feature.id,SacrificialEPS,0,{x:0.0,y:0.0,z:-1.0});
		document.setup.sourceFingerprint=materia.tooling.ToolingCodec.inputFingerprint(document.input);
		var before=materia.tooling.ToolingCodec.encodeInput(document.input);
		var plan=machinekit.tooling.CastingMoldCad.plan(document,options());
		check(plan.cores.length==1 && plan.features.filter(feature->feature.memberIds.indexOf(plan.cores[0].id)>=0).length==2,
			"Face-connected declared cuts share one real portal and retain both source IDs");
		check(materia.tooling.ToolingCodec.encodeInput(plan.document.input)==before,"Access grouping changes no source geometry");
		var source=machinekit.tooling.CastingMoldCad.source(plan),rebuilt=machinekit.tooling.CastingMoldCad.reconstructed(plan);
		var missing=source.subtractAll([rebuilt]),excess=rebuilt.subtractAll([source]);
		check(missing.volume()+excess.volume()<1e-5,"Shared face-open EPS access exactly reconstructs source resin");
		missing.close();excess.close();source.close();rebuilt.close();
		document.input.features.push({id:"round-neighbour",kind:materia.casting.CastingFeature.CastingFeatureKind.ServiceOpening,
			geometry:{solid:materia.casting.CastingFeature.CastingGeometrySolid.Cut(materia.casting.CastingSolid.Cylinder(20,50)),pose:F.translation(90,0,0)},
			openings:[materia.casting.CastingPatterns.face("body.z-min")],draft:{degrees:0,axis:{x:0,y:0,z:1}},
			cornerRadius:0,owner:null,adjustable:[]});
		ToolingTestRules.set(document,"round-neighbour",SacrificialEPS,0,{x:0.0,y:0.0,z:-1.0});
		document.setup.sourceFingerprint=materia.tooling.ToolingCodec.inputFingerprint(document.input);
		var networks=materia.tooling.casting.EPSAccessNetworks.plan(document,materia.tooling.casting.CastingStrategies.resolve(document),[]);
		check(networks.length==1 && networks[0].featureIds.length==2 && networks[0].featureIds.indexOf("round-neighbour")<0,
			"Unrelated non-box cuts retain their native route without disabling exact box networks");
	}

	static function main():Void {
		declaredBedNetwork();
		transverseBore();elevatedThroughMask();
		contactAndLocatingFoot();
		fixture(false);fixture(true);
		bedAccess();
		sideRegistration();
		rebatedSteel();
		overlapOwnership();
		rotatedBridge();
		sideMaskAssembly();sideMaskAssembly(4);clampedSideReceiver();
		overlappingBridgeRows();
		Sys.println('EpsAccessTests: $assertions assertions passed');
	}
}
