package machinekit.tooling;

import cadkit.modeling.Scope;
import cadkit.modeling.Model;
import materia.assembly.AssemblyFrames as F;
import materia.casting.CastingRegion;
import materia.tooling.ToolingCodec;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.FormerConstruction;
import materia.tooling.casting.MoldRegions;
import materia.tooling.panel.PanelStockSpec;

typedef FormerSealFace = {
	var id:String;
	var faceIndex:Int;
	var areaMm2:Float;
	var bounds:materia.casting.Box3;
	var faced:Bool;
}

typedef FormerBoardJob = {
	var id:String;
	var stock:PanelStockSpec;
	var region:CastingRegion;
	var pose:materia.assembly.AssemblyRecord.AssemblyFrame;
	var machining:Null<FormerMachiningPlan>;
	var sealingFaces:Array<FormerSealFace>;
	var programs:Array<String>;
	var reasons:Array<String>;
}

typedef FormerShopItem = {
	var id:String;
	var sourceIds:Array<String>;
	var construction:FormerConstructionPlan;
	var boards:Array<FormerBoardJob>;
	var reconstructionDifferenceMm3:Null<Float>;
	var minimumInteriorCoverMm:Null<Float>;
}

typedef FormerShopReport = {
	var format:String;
	var version:Int;
	var sourceFingerprint:String;
	var sealSpec:String;
	var releaseSpec:String;
	var formers:Array<FormerShopItem>;
	var validation:Array<ToolingValidation>;
	var pocketScrews:Array<{formerId:String,jointId:String,fixedBoard:String,movingBoard:String,
		sitesMm:Array<Float>,choice:FormerPocketScrew,access:String,qualification:String}>;
}

/** Board construction, exact sections and sealing instructions. CAM stays in
 * CamKit; this exporter consumes the same neutral machining records it draws.
 */
class FormerShopExport {
	public static function build(plan:CastingMoldPlan):FormerShopReport {
		if (materia.tooling.casting.CastingMoldPlanner.geometry==null)
			materia.tooling.casting.CastingMoldPlanner.setGeometry(new NativeCastingMoldKernel());
		var report:FormerShopReport={format:"materia.tooling.former-manufacturing",version:1,
			sourceFingerprint:materia.tooling.casting.CastingMoldPlanner.fingerprint(plan),sealSpec:plan.document.recipe.hardware.sealMaterial,
			releaseSpec:plan.document.recipe.releaseSpec,formers:[],validation:[],pocketScrews:[]};
		var scope=new Scope();
		try {
			var source=scope.own(ToolingSourceCasting.fromInput(plan.document.input).geometry());
			var resin=scope.own(machinekit.assembly.PosedParts.moved(source,plan.document.setup.sourceToMold));
			for (former in plan.formers) {
				var construction=FormerManufacturing.plan(former,plan.document.recipe,resin,MoldRegions.envelope(plan.document));
				var item:FormerShopItem={id:former.volume.id,sourceIds:former.featureIds.copy(),construction:construction,
					boards:[],reconstructionDifferenceMm3:null,minimumInteriorCoverMm:null};
				var status:ToolingCheckStatus=Pending,message="Declared board construction and machining require shop/tool/workholding, sealing, pocket-jig and board/grip qualification.";
				function board(id:String,stock:PanelStockSpec,region:CastingRegion,pose:materia.assembly.AssemblyRecord.AssemblyFrame,draft:Float):Void {
					var job:FormerBoardJob={id:id,stock:stock,region:region,pose:pose,machining:null,sealingFaces:[],programs:[],reasons:[]};
					try {
						var machining=FormerManufacturing.machining(id,region,pose,stock,draft,plan.document.recipe);
						job.machining=machining;job.pose=machining.stockToMold;
					} catch (error:Dynamic) {
						job.reasons.push(Std.string(error));status=ToolingCheckStatus.Unsupported;message=Std.string(error);
					}
					job.sealingFaces=sealingFaces(job);
					item.boards.push(job);
				}
				switch construction.build {
					case Solid(stock,_,_):board(item.id,stock,MoldRegions.tooling(former.volume),construction.stockToMold,former.draftDegrees);
					case PanelBox(stock,panels,body):
						for (joint in panels.joints) {
							var screw=plan.document.recipe.former.pocketScrew;
							var seat=joint.seat;
							if (seat==null) throw "Former pocket screw has no joint seat: "+joint.id;
							var span=seat.memberLength-2*screw.edgeDistanceMm;
							if (span<0) throw "Former joint lacks pocket-screw spacing: "+joint.id;
							var sites:Array<Float>;
							var intervals=Std.int(Math.ceil(span/screw.maximumPitchMm));
							if (intervals<1) intervals=1;
							sites=span<2*screw.diameterMm ? [seat.memberLength/2]
								: [for (index in 0...intervals+1) screw.edgeDistanceMm+span*index/intervals];
							report.pocketScrews.push({formerId:item.id,jointId:joint.id,fixedBoard:joint.fixedId,movingBoard:joint.movingId,
								sitesMm:sites,choice:screw,
								access:"Heads and jig access inside the dry cavity; no resin-side counterbores",
								qualification:"Pending board/grip and jig trial; remove screws after withdrawal for storage"});
						}
						var proof=FormerManufacturing.boxEvidence(former,construction,plan.document.recipe.minimumLigament,resin);
						item.reconstructionDifferenceMm3=proof.differenceMm3;
						item.minimumInteriorCoverMm=proof.minimumWallMm;
						for (member in panels.members) board(member.panel.id,stock,FormerManufacturing.boardRegion(former,member),member.pose,0);
					for (attachment in construction.attachments) board(attachment.id,attachment.stock,attachment.region,attachment.pose,0);
					case Unsupported(reason):status=ToolingCheckStatus.Unsupported;message=reason;
				}
				report.formers.push(item);
				report.validation.push(materia.tooling.export.ToolingLoads.check("fabrication.former-machining."+item.id,
					[item.id].concat(item.sourceIds),status,message,["former-manufacturing.json"]));
			}
			scope.close();return report;
		} catch (error:Dynamic) {scope.close();throw error;}
	}

	static function sealingFaces(board:FormerBoardJob):Array<FormerSealFace> {
		var scope=new Scope(),result:Array<FormerSealFace>=[];
		try {
			var global=scope.own(machinekit.structural.CastingRegionCad.part(board.region));
			var local=scope.own(machinekit.assembly.PosedParts.moved(global,F.inverse(board.pose)));
			for (index in 0...local.shape.subshapeCount(CadKit.ShapeKind.Face)) {
				var face=scope.own(new Model(local.shape.subshape(CadKit.ShapeKind.Face,index)));
				var b=face.shape.bounds(),low=b.get_min(),high=b.get_max();
				var faced=face.shape.surfaceKind()==CadKit.SurfaceKind.Plane && Math.abs(face.shape.faceNormal().get_z())>1-1e-7;
				var rawBottom=faced && Math.abs(low.get_z())<1e-6 && Math.abs(high.get_z())<1e-6;
				var rawTop=faced && Math.abs(low.get_z()-board.stock.measuredThickness)<1e-6
					&& Math.abs(high.get_z()-board.stock.measuredThickness)<1e-6;
				var coated=board.stock.coatedFace;
				if ((rawBottom && (coated==materia.tooling.ToolingEnums.CoatedFace.Both || coated==materia.tooling.ToolingEnums.CoatedFace.NegativeZ)) || (rawTop && (coated==materia.tooling.ToolingEnums.CoatedFace.Both || coated==materia.tooling.ToolingEnums.CoatedFace.PositiveZ))) continue;
				result.push({id:materia.tooling.ToolingIds.derive("",board.id,"-seal-face-"+index),faceIndex:index,
					areaMm2:face.shape.area(),faced:faced,bounds:{x:low.get_x(),y:low.get_y(),z:low.get_z(),
						width:high.get_x()-low.get_x(),depth:high.get_y()-low.get_y(),height:high.get_z()-low.get_z()}});
			}
			scope.close();return result;
		} catch (error:Dynamic) {scope.close();throw error;}
	}

	public static function applyValidation(report:FormerShopReport,f:materia.tooling.export.ToolingFabrication.FabricationPlan,
			plan:CastingMoldPlan):Void {
		if (report.sourceFingerprint!=f.sourceFingerprint) throw "Stale former manufacturing report";
		var panels:Array<materia.tooling.panel.PanelAssemblyPlan.PanelPlacement>=[];
		for (item in report.formers) switch item.construction.build {
			case PanelBox(_,box,_):panels=panels.concat(box.members);
			case Solid(_,_,_),Unsupported(_):
		}
		var oldLoadRules=[for (row in f.loads.validation) row.rule];
		f.loads=materia.tooling.export.ToolingLoads.report(plan,f.spec,panels);
		f.validation=f.validation.filter(row -> oldLoadRules.indexOf(row.rule)<0).concat(f.loads.validation);
		var rules=[for (row in report.validation) row.rule];
		f.validation=f.validation.filter(row -> rules.indexOf(row.rule)<0).concat(report.validation);
		f.validation.sort((a,b) -> Reflect.compare(a.rule,b.rule));
		f.fabrication=materia.tooling.ToolingValidation.ToolingReadiness.evaluate(f.validation);
	}

	public static function write(report:FormerShopReport,dir:String):Void {
		sys.io.File.saveContent(dir+"/former-manufacturing.json",ToolingCodec.canonical(report)+"\n");
		var rows=[["former_id","board_id","material","stock_thickness_mm","width_mm","depth_mm","finished_height_mm","panel_count","sealing_faces"]];
		for (item in report.formers) switch item.construction.build {
			case PanelBox(_,panels,_):
				var components:Map<String,machinekit.component.MachineComponent>=[];
				for (member in panels.members) {
					var board=item.boards.filter(board -> board.id==member.panel.id)[0];
					components.set(board.id,ToolingVolume.fromVolume({id:board.id,material:FormerStock,profiles:[],cuts:[],pieces:[],
						regions:[MoldRegions.moved(board.region,F.inverse(member.pose))]}));
				}
				var assembly=PanelKit.assembly(panels,"declared former board",components);
				for (attachment in item.construction.attachments) {
					var component=ToolingVolume.fromVolume({id:attachment.id,material:FormerStock,
						profiles:[],cuts:[],pieces:[],regions:[MoldRegions.moved(attachment.region,F.inverse(attachment.pose))]});
					assembly.addComponent(attachment.id,component,attachment.pose);
					var fixed=panels.members[0];
					var fixedId=materia.tooling.ToolingIds.derive("",attachment.id,"-fixed");
					var movingId=materia.tooling.ToolingIds.derive("",attachment.id,"-moving");
					assembly.addMemberConnector(fixed.panel.id,fixedId,F.compose(F.inverse(fixed.pose),attachment.pose));
					assembly.addMemberConnector(attachment.id,movingId,F.identity());
					assembly.addMate(materia.tooling.ToolingIds.derive("",attachment.id,"-joint"),"fixed",fixed.panel.id,fixedId,attachment.id,movingId);
				}
				sys.io.File.saveContent(dir+"/"+item.id+".assembly.json",assembly.encode());
			case Solid(_,_,_),Unsupported(_):
		}
		for (item in report.formers) for (board in item.boards) {
			var machining=board.machining;
			rows.push([item.id,board.id,board.stock.materialSpec,Std.string(board.stock.measuredThickness),
				machining==null ? "unsupported" : Std.string(machining.widthMm),machining==null ? "unsupported" : Std.string(machining.depthMm),
				machining==null ? "unsupported" : Std.string(machining.heightMm),Std.string(item.boards.length),Std.string(board.sealingFaces.length)]);
			if (machining==null) continue;
			var part=machinekit.structural.CastingRegionCad.part(board.region),sheet:ToolingSectionDrawing.ToolingSectionSheet;
			try sheet=ToolingSectionDrawing.draw(part,board.id,machining.stockToMold,[0,machining.heightMm/2,machining.heightMm])
			catch (error:Dynamic) {part.close();throw error;}
			part.close();
			var entities=sheet.drawing.entities();
			entities.push(cadkit.drawing.DrawingEntity.Label(Text,new cadkit.sheetmetal.Point2(0,-18),2.5,
				"Stock: "+board.stock.materialSpec+" / "+board.stock.measuredThickness+" mm; seal all routed edges and faced surfaces"));
			entities.push(cadkit.drawing.DrawingEntity.Label(Text,new cadkit.sheetmetal.Point2(0,-25),2.5,
				"Sealing face IDs and bounds: former-manufacturing.json; machine and sealing qualification pending"));
			var drawing=new cadkit.drawing.Drawing2D(entities),name=materia.tooling.ToolingIds.derive("shop-",board.id);
			sys.io.File.saveContent(dir+"/"+name+".svg",cadkit.drawing.Svg.write(drawing));
			sys.io.File.saveContent(dir+"/"+name+".dxf",cadkit.drawing.Dxf.write(drawing));
		}
		sys.io.File.saveContent(dir+"/former-board-stock.csv",materia.tooling.export.ToolingFabrication.csv(rows));
		var screws=[["former_id","joint_id","fixed_board","moving_board","diameter_mm","length_mm","count","sites_from_joint_end_mm","jig_angle_degrees","access","qualification"]];
		for (fixing in report.pocketScrews) screws.push([fixing.formerId,fixing.jointId,fixing.fixedBoard,fixing.movingBoard,
			Std.string(fixing.choice.diameterMm),Std.string(fixing.choice.lengthMm),Std.string(fixing.sitesMm.length),
			fixing.sitesMm.map(value -> Std.string(value)).join(" / "),Std.string(fixing.choice.jigAngleDegrees),fixing.access,fixing.qualification]);
		sys.io.File.saveContent(dir+"/former-pocket-screws.csv",materia.tooling.export.ToolingFabrication.csv(screws));
	}
}
