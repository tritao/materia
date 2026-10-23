import materia.assembly.AssemblyFrames as F;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.FormerConstruction;
import machinekit.tooling.FormerManufacturing;

class FormerManufacturingTests {
	static var assertions=0;
	static function check(ok:Bool,message:String):Void {assertions++;if (!ok) throw message;}
	static function box(id:String,width:Float,depth:Float,height:Float):MoldFormer
		return {volume:{id:id,material:FormerStock,profiles:[],regions:[],cuts:[],
			pieces:[{x:0,y:0,z:0,width:width,depth:depth,height:height}]},
			featureIds:[],withdrawal:{x:0,y:0,z:-1},draftDegrees:0};

	static function removal(former:MoldFormer,recipe:materia.tooling.ToolingRecipe,pitch:Float=0.0002):Void {
		var construction=FormerManufacturing.plan(former,recipe);
		var stock=switch construction.build {case Solid(stock,_,_):stock;default:throw "Expected solid former";};
		var record=FormerManufacturing.machining(former.volume.id,
			materia.tooling.casting.MoldRegions.tooling(former.volume),construction.stockToMold,stock,former.draftDegrees,recipe);
		var settings=MillFabricationSetup.cam([materia.tooling.panel.PanelJoints.blank("board",40,30,recipe.panel,6,2)]);
		var programs=camkit.CamToolingFormers.build(record,settings);
		check(programs.length==(former.draftDegrees==0 ? 1 : 2),"separate tool touch-offs share the existing contour CAM");
		var shape=machinekit.tooling.ToolingVolume.geometryOf(former.volume);
		var local=machinekit.assembly.PosedParts.moved(shape,F.inverse(record.stockToMold));shape.close();
		var meshTolerance=recipe.former.toleranceMm/20;
		if (former.draftDegrees>0) meshTolerance*=Math.tan(former.draftDegrees*Math.PI/180);
		var mesh=local.shape.tessellate(meshTolerance,0.1);local.close();
		var base:Float=cast settings.spoilboardPlane;
		var positions=[for (i in 0...mesh.vertexCount*3)
			mesh.vertices.getDouble(i*8)*0.001+(i%3==2 ? base : settings.blankMargin)];
		var indices=[for (i in 0...mesh.indexCount) mesh.indices.getInt32(4*i)];
		var width=record.widthMm*0.001+2*settings.blankMargin;
		var depth=record.depthMm*0.001+2*settings.blankMargin;
		var height=stock.measuredThickness*0.001;
		var lattice=new stockkit.StockLattice(pitch*0.137,pitch*0.293,base+pitch*0.419,pitch,
			Std.int(Math.ceil(width/pitch)),Std.int(Math.ceil(depth/pitch)),Std.int(Math.ceil(height/pitch)));
		var target=stockkit.Stock.fromTriangles(lattice,positions,indices);
		var actual=stockkit.Stock.box(lattice,0,0,base,width,depth,base+height);
		actual.setThreads(1);
		try {
			for (program in programs) {
				var code=cnckit.CncWriter.write(program.program,new cnckit.CncController(cnckit.CncDialect.LinuxCnc,program.program.tools));
				check(code.indexOf("G54")>=0,"former programs export with the declared controller work offset");
				var report=actual.cut(stockkit.CutMoves.fromProgram(program.program));
				check(report.rapidContacts().length==0,"former CAM never rapids through uncut material");
				check(report.collisions().length==0,"former CAM keeps non-cutting tool zones clear");
			}
			for (axis in actual.compareAll(target)) {
				check(axis.deepestGouge()<=record.toleranceMm*0.001,"former gouge stays within declared tolerance: "+axis.deepestGouge());
				check(axis.thickestLeftover()<=record.toleranceMm*0.001,"former leftover stays within declared tolerance: "+axis.thickestLeftover());
			}
		} catch (error:Dynamic) {target.dispose();actual.dispose();throw error;}
		target.dispose();actual.dispose();
	}

	public static function main():Void {
		materia.tooling.casting.CastingMoldPlanner.geometry=new machinekit.tooling.NativeCastingMoldKernel();
		var recipe=SinglePocketSetup.document().recipe;
		var small=FormerManufacturing.plan(box("shallow",40,30,8),recipe);
		check(switch small.build {case Solid(stock,depth,_):stock.measuredThickness==18 && depth==10;default:false;},
			"shallow former is faced from one declared sheet");
		var stem=FormerManufacturing.plan(box("stem",20,20,20),recipe);
		check(switch stem.build {case Solid(stock,depth,_):stock.measuredThickness==25 && depth==5;default:false;},
			"stem uses the next declared thickness without laminations");
		var tall=box("tall",111,110,120);
		var construction=FormerManufacturing.plan(tall,recipe);
		check(switch construction.build {case PanelBox(_,panels,_):panels.members.length==5;default:false;},
			"tall former is an open five-panel shell");
		var evidence=FormerManufacturing.boxEvidence(tall,construction,recipe.minimumLigament);
		check(evidence.differenceMm3<1e-6,"boards plus declared air and rabbet seals exactly reconstruct the former envelope");
		check(evidence.minimumWallMm>=recipe.minimumLigament-1e-7,"hollow interior stays away from closed cavity faces");
		var targets=FormerManufacturing.panelTargets(tall,construction);
		for (target in targets) {
			check(target.part.valid() && target.part.solidCount()==1,"each former board is one manufacturable connected part");
			target.part.close();
		}
		var longId="";for (index in 0...128) longId+="a";
		var neck=box(longId,90,100,90);
		neck.volume.pieces=[{x:10,y:10,z:30,width:90,depth:100,height:90},
			{x:0,y:0,z:0,width:110,depth:120,height:12}];
		neck.volume.profiles=[{panelId:"test-wall",operationId:"neck-opening",contour:materia.tooling.panel.PanelContours.rectangle(10,10,90,100),
			pose:F.translation(0,0,12),depth:18,subtract:[]}];
		var castingEnvelope=materia.tooling.casting.MoldRegions.box({x:-10,y:-10,z:30,width:130,depth:140,height:100});
		var neckConstruction=FormerManufacturing.plan(neck,recipe,null,castingEnvelope);
		check(switch neckConstruction.build {case PanelBox(_,panels,body):panels.members.length==5 && body.heightMm==90;
			case Unsupported(reason):throw reason;default:false;},"necked former uses panels around its body only");
		check(neckConstruction.attachments.length==2,"neck and flange are removable single-sheet parts");
		for (attachment in neckConstruction.attachments) {
			materia.tooling.ToolingIds.validate(attachment.id);
			check(attachment.id.length<=128,"derived sheet IDs reserve suffix space for maximum-length source IDs");
		}
		var neckEvidence=FormerManufacturing.boxEvidence(neck,neckConstruction,recipe.minimumLigament);
		check(neckEvidence.differenceMm3<1e-6,"necked panel former exactly reconstructs with its separate sheet parts");
		removal(box("solid-cam",40,30,8),recipe);
		var tapered=box("tapered-cam",40,30,8);
		tapered.draftDegrees=3;
		tapered.volume.pieces=[];
		tapered.volume.regions=[materia.casting.CastingRegion.Drafted("tapered-cam",
			materia.casting.CastingProfiles.rectangle(0,0,40,30),
			F.fromRotationMatrix(0,30,8,[1,0,0,0,-1,0,0,0,-1]),8,3)];
		removal(tapered,recipe);
		removal(tapered,recipe,0.0001);
		var rounded=box("rounded-taper-cam",40,30,8);
		rounded.draftDegrees=3;rounded.volume.pieces=[];
		rounded.volume.regions=[materia.casting.CastingRegion.Drafted("rounded-taper-cam",
			materia.casting.CastingProfiles.roundedRectangle(0,0,40,30,6),
			F.fromRotationMatrix(0,30,8,[1,0,0,0,-1,0,0,0,-1]),8,3)];
		removal(rounded,recipe,0.0001);
		var document=SinglePocketSetup.document();
		document.input.inserts=[];
		document.input.features=document.input.features.filter(feature -> feature.kind!=materia.casting.CastingFeature.CastingFeatureKind.InsertRecess);
		document.setup.sourceFingerprint=materia.tooling.ToolingCodec.inputFingerprint(document.input);
		document.setup.features[0].strategy=ReusableFormer;
		document.setup.features[0].withdrawal={x:0,y:0,z:1};
		var options:materia.tooling.casting.CastingMoldPlan.CastingMoldOptions={foundationJointProcesses:[],replicationJigs:[],precision:[],
			compaction:{method:materia.tooling.casting.CastingMoldPlan.CompactionMethod.Undeclared,bedInterfaceReference:null,frequencyHz:null,amplitudeMm:null,durationSeconds:null,evidence:[]},
			serviceOrientation:document.setup.sourceToMold,overlapRule:materia.tooling.casting.CastingMoldPlan.OverlapRule.ReservedThenPriority,liftingEvidence:[]};
		var mold=materia.tooling.casting.CastingMoldPlanner.plan(document,options);
		var report=machinekit.tooling.FormerShopExport.build(mold);
		check(report.formers.length==1 && report.formers[0].boards.length==5,"shop report includes the complete panel former stock");
		var fabrication=materia.tooling.export.ToolingFabrication.plan(mold,MillFabricationSetup.spec());
		machinekit.tooling.FormerShopExport.applyValidation(report,fabrication,mold);
		for (board in report.formers[0].boards) {
			check(fabrication.loads.estimates.filter(load -> load.id==board.id && load.pressurePa>0).length==1,
				"each hollow-former board carries the existing peak-pressure demand");
			check(fabrication.validation.filter(row -> row.rule=="fabrication.plate."+board.id
				&& row.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported).length==1,
				"each finished hollow-former board exposes the existing structural-model gap");
		}
		for (board in report.formers[0].boards) {
			check(board.machining!=null,"each panel has an accessible neutral machining record: "+board.reasons.join("; "));
			check(board.sealingFaces.length>0,"every routed panel lists its sealing faces");
		}
		var directory=Sys.getCwd()+"/build/former-manufacturing-review";
		machinekit.tooling.ToolingFiles.directory(directory);
		machinekit.tooling.FormerShopExport.write(report,directory);
		var encoded=sys.io.File.getContent(directory+"/"+report.formers[0].id+".assembly.json");
		var restored=machinekit.assembly.MachineAssembly.decode(encoded,machinekit.component.MachineKitComponents.standard());
		check(restored.components().length==5,"finished clipped boards restore as a repairable assembly");

		var drafted=box("drafted-tall",111,110,120);drafted.draftDegrees=3;
		var unsupported=FormerManufacturing.plan(drafted,recipe);
		check(switch unsupported.build {case Unsupported(reason):reason.indexOf("leaning wall panels")>=0;default:false;},
			"drafted tall former reports the missing leaning-panel construction explicitly");
		if (Sys.args().indexOf("--mill")>=0) {
			var started=Sys.time();
			var frame=MillMoldSetup.frame();
			Sys.println("FORMER production frame seconds "+(Sys.time()-started));
			var plan=materia.tooling.casting.CastingMoldPlanner.plan(MillMoldSetup.document(frame),MillMoldSetup.options(frame));
			Sys.println("FORMER production plan seconds "+(Sys.time()-started));
			var source=machinekit.tooling.ToolingSourceCasting.fromInput(plan.document.input).geometry();
			var resin=machinekit.assembly.PosedParts.moved(source,plan.document.setup.sourceToMold);source.close();
			for (former in plan.formers) {
				var build=FormerManufacturing.plan(former,plan.document.recipe,resin,materia.tooling.casting.MoldRegions.envelope(plan.document));
				Sys.println(former.volume.id+" "+switch build.build {case Solid(stock,_,_):"solid / "+stock.measuredThickness+" mm";
					case PanelBox(stock,panels,body):"panel box / "+panels.members.length+" panels / "+stock.measuredThickness+" mm";
					case Unsupported(reason):"unsupported / "+reason;});
				check(switch build.build {case Unsupported(_):false;default:true;},"production former construction: "+former.volume.id);
			}
			if (Sys.args().indexOf("--shop")>=0) {
				var report=machinekit.tooling.FormerShopExport.build(plan);
				for (item in report.formers) for (board in item.boards) {
					var machining=board.machining;
					if (machining==null) Sys.println("FORMER-SHOP "+board.id+" unsupported "+board.reasons.join("; "));
					else {
						var settings=MillFabricationSetup.cam([materia.tooling.panel.PanelJoints.blank("board",40,30,recipe.panel,6,2)]);
						var programs=camkit.CamToolingFormers.build(machining,settings);
						var accepted=programs.length>0;
						check(accepted,"Every production board passes cutter-fit without changing its source contact: "+board.id);
						Sys.println("FORMER-SHOP "+board.id+" programs "+programs.length);
						var solid=switch item.construction.build {case Solid(_,_,_):true;default:false;};
						if (accepted && solid) {
							var former=plan.formers.filter(former -> former.volume.id==item.id)[0];
							removal(former,plan.document.recipe);
							Sys.println("FORMER-STOCKKIT "+board.id+" passed");
						}
					}
				}
			}
			resin.close();
		}
		Sys.println('FormerManufacturingTests: $assertions assertions passed');
	}
}
