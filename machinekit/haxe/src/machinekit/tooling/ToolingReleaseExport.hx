package machinekit.tooling;

import cadkit.modeling.Part;
import cadkit.modeling.Model;
import cadkit.modeling.Scope;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.tooling.ReleasePlan;
import materia.tooling.ReleaseSystem;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingMoldPlan.MoldFixtures;
import materia.tooling.casting.MoldRegions;
import materia.tooling.panel.PanelStockSpec;

typedef ReleaseStockFrame = {var stock:PanelStockSpec;var pose:AssemblyFrame;}
typedef ReleaseGeometryPart = {
	var id:String;
	var sourceIds:Array<String>;
	var part:Part;
	var substrate:ReleaseSubstrate;
	var stock:Null<ReleaseStockFrame>;
}

/** Actual face/solid intersections define wet tooling and steel bond surfaces.
 * Bounds only cull impossible masking contacts; surviving pairs use exact CAD
 * distance. Native resources remain owned until the final contact query closes.
 */
class ToolingReleaseExport {
	static inline var CONTACT_MM=1e-6;

	static function bounds(part:Model):materia.casting.Box3 {
		var box=part.shape.bounds(),a=box.get_min(),b=box.get_max();
		return {x:a.get_x(),y:a.get_y(),z:a.get_z(),width:b.get_x()-a.get_x(),
			depth:b.get_y()-a.get_y(),height:b.get_z()-a.get_z()};
	}

	static function mayTouch(a:materia.casting.Box3,b:materia.casting.Box3):Bool
		return a.x<=b.x+b.width+CONTACT_MM && b.x<=a.x+a.width+CONTACT_MM
			&& a.y<=b.y+b.depth+CONTACT_MM && b.y<=a.y+a.depth+CONTACT_MM
			&& a.z<=b.z+b.height+CONTACT_MM && b.z<=a.z+a.height+CONTACT_MM;

	static function exposed(face:Model,context:ReleaseStockFrame):Bool {
		if (face.shape.surfaceKind()!=CadKit.SurfaceKind.Plane) return true;
		var inverse=F.inverse(context.pose),worldNormal=face.shape.faceNormal();
		var normal=F.transformVector(inverse,worldNormal.get_x(),worldNormal.get_y(),worldNormal.get_z());
		if (Math.abs(Math.abs(normal.z)-1)>1e-8) return true;
		var center=face.shape.center();
		var z=F.transformPoint(inverse,center.get_x(),center.get_y(),center.get_z()).z;
		var stock=context.stock,bottom=Math.abs(z)<=CONTACT_MM;
		var top=Math.abs(z-stock.measuredThickness)<=CONTACT_MM,coated=stock.coatedFace;
		if (bottom && (coated==materia.tooling.ToolingEnums.CoatedFace.Both
			|| coated==materia.tooling.ToolingEnums.CoatedFace.NegativeZ)) return false;
		if (top && (coated==materia.tooling.ToolingEnums.CoatedFace.Both
			|| coated==materia.tooling.ToolingEnums.CoatedFace.PositiveZ)) return false;
		return true;
	}

	/** Borrowed geometry; a single exact surface query serves mold and fixture
	 * assemblies. Each contact query uses its own copies and closes on throw.
	 */
	public static function fromGeometry(sourceId:String,system:ReleaseSystem,coveringThicknessMm:Float,
			resin:Part,parts:Array<ReleaseGeometryPart>):ReleasePlan {
		var scope=new Scope(),faces:Array<ReleaseFace>=[];
		var contacts:Array<{face:ReleaseFace,part:Model,bounds:materia.casting.Box3}>=[];
		try {
			function add(id:String,sourceIds:Array<String>,part:Part,substrate:ReleaseSubstrate,stock:Null<ReleaseStockFrame>):Void {
				part=scope.own(machinekit.assembly.PosedParts.moved(part,F.identity()));
				// These are physical contact queries, not machining/thread records.
				part.shape.setThreadedFaces([]);
				for (index in 0...part.shape.subshapeCount(CadKit.ShapeKind.Face)) {
					var face=scope.own(new Model(part.shape.subshape(CadKit.ShapeKind.Face,index)));
					var area=face.shape.area();
					if (area<=0) continue;
					var wet=scope.own(new Model(face.shape.common(resin.shape))),wetArea=wet.shape.area();
					var seal=substrate==Board && (stock==null || exposed(face,cast stock));
					// Keep dry exposed board faces for sealing and all source steel
					// for clean handling; only real wet patches receive release.
					if (wetArea<=0 && !seal && substrate!=BondingSteel && substrate!=EPSCovering) continue;
					var record:ReleaseFace={id:materia.tooling.ToolingIds.derive("",id,"-release-face-"+index),
						partId:id,sourceIds:sourceIds.copy(),faceIndex:index,areaMm2:area,wettedAreaMm2:wetArea,
						bounds:bounds(face),substrate:substrate,needsSealing:seal,adjacentCoatedFaceIds:[]};
					faces.push(record);
					if (wetArea>0) contacts.push({face:record,part:wet,bounds:bounds(wet)});
				}
			}
			for (entry in parts) add(entry.id,entry.sourceIds,entry.part,entry.substrate,entry.stock);
			for (steel in contacts) if (steel.face.substrate==BondingSteel) {
				for (tool in contacts) if (tool.face.substrate!=BondingSteel && mayTouch(steel.bounds,tool.bounds)
					&& steel.part.shape.distance(tool.part.shape)<=CONTACT_MM) steel.face.adjacentCoatedFaceIds.push(tool.face.id);
				steel.face.adjacentCoatedFaceIds.sort(Reflect.compare);
			}
			var result=ReleasePlans.build(sourceId,system,coveringThicknessMm,faces);
			scope.close();return result;
		} catch (error:Dynamic) {scope.close();throw error;}
	}

	public static function build(plan:CastingMoldPlan,shop:FormerShopExport.FormerShopReport):ReleasePlan {
		var scope=new Scope(),parts:Array<ReleaseGeometryPart>=[],seen=new Map<String,Bool>();
		try {
			var source=scope.own(CastingMoldCad.source(plan));
			var box=bounds(source),height=Math.min(box.height,plan.document.setup.fillHeightMm-box.z);
			if (height<=0) throw "Release surfaces need resin below the declared fill plane";
			var fill=scope.own(ToolingBlock.placedBox({x:box.x,y:box.y,z:box.z,
				width:box.width,depth:box.depth,height:height}));
			var resin=scope.own(source.intersect(fill));
			function add(id:String,sourceIds:Array<String>,part:Part,substrate:ReleaseSubstrate,stock:Null<ReleaseStockFrame>):Void {
				scope.own(part);seen.set(id,true);
				parts.push({id:id,sourceIds:sourceIds,part:part,substrate:substrate,stock:stock});
			}
			for (member in plan.panels.members.concat(plan.supportPanels)) {
				var local=scope.own(ToolingPanel.fromPlan(member.panel).geometry());
				add(member.panel.id,[plan.document.input.id],machinekit.assembly.PosedParts.moved(local,member.pose),Board,
					{stock:member.panel.stock,pose:member.pose});
			}
			for (core in plan.cores) add(core.id,[core.featureId],machinekit.structural.CastingRegionCad.part(core.cavity),EPSCovering,null);
			for (former in plan.formers) {
				var items=shop.formers.filter(item -> item.id==former.volume.id);
				if (items.length!=1) throw "Missing former construction for release surfaces: "+former.volume.id;
				var item=items[0];
				seen.set(former.volume.id,true);
				if (item.boards.length==0) {
					// The missing manufacturing route stays Unsupported elsewhere;
					// its required exposed-board sealing is still explicit here.
					add(former.volume.id,former.featureIds,ToolingVolume.geometryOf(former.volume),Board,
						{stock:plan.document.recipe.panel,pose:item.construction.stockToMold});
				} else for (board in item.boards)
					add(board.id,former.featureIds,machinekit.structural.CastingRegionCad.part(board.region),Board,
						{stock:board.stock,pose:board.pose});
			}
			for (fixture in plan.fixtures) for (volume in MoldFixtures.volumes(fixture))
				add(volume.id,[fixture.insertId],ToolingVolume.geometryOf(volume),ToolingMetal,null);
			for (insert in plan.document.input.inserts) add(materia.tooling.ToolingIds.derive("insert-",insert.id),
				[insert.id],CastingMoldCad.steel(plan,insert.id),BondingSteel,null);
			for (hardware in CastingMoldCad.anchorComponents(plan)) {
				var local=scope.own(hardware.part.geometry());
				add(hardware.id,[hardware.insertId,hardware.anchorId],machinekit.assembly.PosedParts.moved(local,hardware.pose),BondingSteel,null);
			}
			// Reuse the actual shared hardware builders, including threaded masks,
			// end caps and fixture screws. No second screw-stack implementation.
			var assembly=CastingMoldCad.assembly(plan),poses=assembly.solvedPoses();
			for (row in assembly.components()) if (!seen.exists(row.id)) {
				var local=scope.own(row.component.geometry());
				add(row.id,[plan.document.input.id],machinekit.assembly.PosedParts.moved(local,poses.get(row.id)),ToolingMetal,null);
			}
			var result=fromGeometry(plan.document.input.id,plan.document.recipe.releaseSystem,
				plan.document.recipe.linerThickness,resin,parts);
			scope.close();return result;
		} catch (error:Dynamic) {scope.close();throw error;}
	}

	/** Fixture assemblies supply their already-built source casting and parts.
	 * Surface extraction and process planning are the same path as mold tooling.
	 */
	public static function fromAssembly(document:materia.tooling.ToolingDocument,
			assembly:machinekit.assembly.MachineAssembly):ReleasePlan {
		var scope=new Scope(),parts:Array<ReleaseGeometryPart>=[],resin:Null<Part>=null;
		var poses=assembly.solvedPoses();
		try {
			for (row in assembly.components()) {
				var local=scope.own(row.component.geometry()),pose=poses.get(row.id);
				var part=scope.own(machinekit.assembly.PosedParts.moved(local,pose));
				if (Std.isOfType(row.component,ToolingSourceCasting)) {
					if (resin!=null) throw "Release fixture has more than one source casting";
					resin=part;continue;
				}
				var insert=document.input.inserts.filter(item -> materia.tooling.ToolingIds.derive("insert-",item.id)==row.id);
				var kind:ReleaseSubstrate=insert.length>0 ? BondingSteel : ToolingMetal;
				var stock:Null<ReleaseStockFrame>=null;
				if (Std.isOfType(row.component,ToolingPanel)) {
					var panel:ToolingPanel=cast row.component;
					stock={stock:panel.plan().stock,pose:pose};kind=Board;
				}
				parts.push({id:row.id,sourceIds:insert.length>0 ? [insert[0].id] : [document.input.id],
					part:part,substrate:kind,stock:stock});
			}
			if (resin==null) throw "Release fixture needs its source casting geometry";
			var result=fromGeometry(document.input.id,document.recipe.releaseSystem,document.recipe.linerThickness,cast resin,parts);
			scope.close();return result;
		} catch (error:Dynamic) {scope.close();throw error;}
	}

	static function timing(input:Null<materia.casting.CastingInput>):String {
		if (input==null) return "";
		var value:materia.casting.CastingInput=cast input;
		return switch value {case Known(seconds):Std.string(seconds)+" s";case Unresolved(reason):"Pending: "+reason;};
	}

	public static function write(plan:CastingMoldPlan,shop:FormerShopExport.FormerShopReport,dir:String,
			?chemical:materia.tooling.ToolingValidation):Array<materia.tooling.ToolingValidation>
		return writeReport(build(plan,shop),materia.tooling.casting.CastingMoldPlanner.fingerprint(plan),dir,chemical);

	public static function writeAssembly(document:materia.tooling.ToolingDocument,assembly:machinekit.assembly.MachineAssembly,
			dir:String):Array<materia.tooling.ToolingValidation> {
		var fingerprint=haxe.crypto.Sha256.encode(materia.tooling.ToolingCodec.canonical({
			document:materia.tooling.ToolingCodec.documentFingerprint(document),assembly:assembly.encode()}));
		return writeReport(fromAssembly(document,assembly),fingerprint,dir,null);
	}

	static function writeReport(release:ReleasePlan,fingerprint:String,dir:String,
			chemical:Null<materia.tooling.ToolingValidation>):Array<materia.tooling.ToolingValidation> {
		var checks=ReleasePlans.validate(release);
		if (chemical==null) {
			chemical=materia.tooling.ToolingValidation.ToolingReadiness.pending("fabrication.materials.compatibility",
				[release.sourceId],"Physical resin/EPS/adhesive/release coupon evidence has not been supplied to this review export.");
			checks.push(cast chemical);
		}
		var record:Dynamic=ReleasePlans.record(release);
		record.format="materia.tooling.release-system";record.version=1;
		record.sourceFingerprint=fingerprint;
		record.chemicalCompatibilityRule="fabrication.materials.compatibility";
		record.chemicalCompatibility=chemical;
		record.validation=checks;
		sys.io.File.saveContent(dir+"/release-system.json",materia.tooling.ToolingCodec.canonical(record)+"\n");
		var rows=[["sequence","part_id","action","coat","time","face_ids"]],lines=[
			"# Release and sealing procedure",
			"",
			"Complete every preparation step below before fitting any insert or anchored steel assembly. Then follow the "
				+ "checked geometric assembly order. Keep source steel out of the coating area; never coat insert or anchor "
				+ "bonding faces.",
			"",
			"Resolve the declared product references and manufacturer times before work. Chemical compatibility of resin, "
				+ "EPS, adhesive and release remains Pending until the existing fabrication materials rule has a passing physical"
				+ " coupon-trial reference. Product declarations alone are insufficient.",
			"",
			"Cover the resin-wetted EPS portions continuously across stock glue seams, and seal the covering at mold "
				+ "penetrations with the declared seam process. Keep dry carrier/foot adhesive and clamp contacts free of "
				+ "covering and release agent. Complete and cure support bonds before applying release. The covering thickness is"
				+ " already included in core geometry; do not add another dimensional allowance.",
			""
		];
		for (index in 0...release.steps.length) {
			var step=release.steps[index],time=timing(step.seconds);
			rows.push([Std.string(index+1),step.partId,(step.action:String),step.coat==0 ? "" : Std.string(step.coat),time,step.faceIds.join(";")]);
			lines.push((index+1)+". `"+step.partId+"`: "+(step.action:String)+(step.coat==0 ? "" : " coat "+step.coat)
				+(time=="" ? "" : " — "+time)+". Faces: "+step.faceIds.join(", ")+".");
		}
		lines.push("\nDo not coat the must-not-coat faces in release-system.json. Mask the listed adjacent bonding faces, protect "
			+ "clean steel during handling, and remove temporary masks without contaminating bonding surfaces before the "
			+ "pour. Board/grip, product timing and physical coupon qualification remain required.");
		sys.io.File.saveContent(dir+"/release-operations.csv",materia.tooling.export.ToolingFabrication.csv(rows));
		sys.io.File.saveContent(dir+"/release-system.md",lines.join("\n")+"\n");
		return checks;
	}
}
