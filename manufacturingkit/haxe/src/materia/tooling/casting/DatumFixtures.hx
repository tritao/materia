package materia.tooling.casting;

import materia.casting.BoxAlgebra;

import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingIds;
import materia.tooling.ToolingCodec;
import materia.tooling.panel.PanelJoints;
import materia.tooling.panel.PanelCodec;
import materia.tooling.panel.PanelContours;
import materia.tooling.ToolingEnums.MachiningSide;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingToolingInput.CastingToolingInsert;
import materia.tooling.casting.CoreSupport;
import materia.tooling.panel.PanelAssemblyPlan.PanelPlacement;

/** Nominal supports, located by pins and clamped into source steel threads.
 * Loose pins/screws/masks must be released before the solid-only removal graphs.
 * No source insert thread or pin hole is invented.
 */
class DatumFixtures {
	/** Match an authored mounting plane to a real shell contact plane. The
	 * subsequent contact and penetration checks prove the footprint and holes.
	 */
	public static function panelOnFace(panels:Array<PanelPlacement>,face:materia.assembly.AssemblyRecord.AssemblyFrame)
			:Null<PanelPlacement> {
		var matches:Array<PanelPlacement>=[];
		for (panel in panels) {
			var surface:Null<Float>=switch panel.panel.contactFace {
				case materia.tooling.ToolingEnums.ContactFace.PositiveZ:panel.panel.stock.measuredThickness;
				case materia.tooling.ToolingEnums.ContactFace.NegativeZ:0.0;
				case materia.tooling.ToolingEnums.ContactFace.None:null;
				default:throw "Unknown fixture panel contact face: "+panel.panel.id;
			};
			if (surface==null) continue;
			var local=F.compose(F.inverse(panel.pose),face),normal=F.transformVector(local,0,0,1);
			var expected=panel.panel.contactFace==materia.tooling.ToolingEnums.ContactFace.PositiveZ ? -1.0 : 1.0;
			if (Math.abs(local.z-surface)<1e-7 && normal.z*expected>1-1e-8
				&& materia.casting.CastingProfileGeometry.inside(PanelContours.outside(panel.panel),local.x,local.y))
				matches.push(panel);
		}
		if (matches.length>1) throw "Insert mounting plane matches multiple receiver panels";
		return matches.length==0 ? null : matches[0];
	}

	/** Nominal access preference only; the native swept-removal graph verifies it.
	 * Side fixtures expose the through bore toward the open core behind the seat.
	 * Bed fixtures expose the mounting face. Blind holes always withdraw at entry.
	 */
	static function maskWithdrawal(p:CastingMoldPlan, insert:CastingToolingInsert, hole:materia.casting.InsertHole,
			withdrawal:{x:Float, y:Float, z:Float}):{x:Float, y:Float, z:Float} {
		var stock = F.compose(p.document.setup.sourceToMold, insert.pose);
		var entry = F.compose(stock, hole.frame), axis = F.transformVector(entry, 0, 0, 1);
		var sign = -1.0;
		if (hole.thread == null && hole.extent == materia.casting.InsertHole.HoleExtent.Through) {
			var preferred = F.transformVector(stock, 0, 0, Math.abs(withdrawal.z) < 0.9 ? -1 : 1);
			if (axis.x * preferred.x + axis.y * preferred.y + axis.z * preferred.z > 0)
				sign = 1;
		}
		return {x: sign * axis.x, y: sign * axis.y, z: sign * axis.z};
	}
	public static function add(p:CastingMoldPlan, i:CastingToolingInsert, face:materia.assembly.AssemblyRecord.AssemblyFrame, datum:MoldVolume,
			withdrawal:{x:Float, y:Float, z:Float}, bed:PanelPlacement, pocketId:Null<String>):Void {
		var precastHoles=InsertFixturePlanner.availableHoles(i);
		p.validation = p.validation.concat(InsertFixturePlanner.protectionValidation(i));
		var r = p.document.recipe,
			h = r.hardware,
			dt = h.datumThickness,
			pw = InsertFixturePlanner.requiredPostWidth(p.document.input, r);
		var cx = face.x, cy = face.y, floor = p.cavity.z - dt;
		if (Math.abs(withdrawal.x) > 0.9) {
			cx = withdrawal.x > 0 ? p.cavity.x + p.cavity.width + 2 * r.panel.measuredThickness : p.cavity.x - 2 * r.panel.measuredThickness;
			floor = p.cavity.z;
			var top = face.z - dt;
			if (top > floor)
				datum.pieces.push({
					x: cx - pw / 2,
					y: cy - pw / 2,
					z: floor,
					width: pw,
					depth: pw,
					height: top - floor
				});
			datum.pieces = BoxAlgebra.subtract(datum.pieces, []);
		}
		var id = datum.id, pinDiameter = h.fixturePinDiameter, d = pinDiameter + r.jointClearance, embed = h.fixturePinEmbed;
		var primary = ToolingIds.derive("", id, "-primary-pin"),
			secondary = ToolingIds.derive("", id, "-secondary-pin");
		var pinSpacing = h.fixturePinSpacing, pinAlongY = false;
		for (hole in precastHoles) {
			var fixing = InsertFixturePlanner.select(hole, F.compose(p.document.setup.sourceToMold, i.pose), face, h);
			if (fixing != null)
				pinSpacing = Math.max(pinSpacing, fixing.clearanceDiameter + d + h.secondaryPinTravel + 2 * r.minimumLigament);
		}
		var feet=datum.pieces.filter(piece -> Math.abs(piece.z-floor)<1e-7);
		var requiredSpan=pinSpacing+d+h.secondaryPinTravel+2*r.minimumLigament;
		var alongX=feet.filter(piece -> piece.width>=requiredSpan).length>0;
		var alongY=feet.filter(piece -> piece.depth>=requiredSpan).length>0;
		function clearFixings(alongY:Bool):Int {
			var count=0;
			for (hole in precastHoles) {
				var fixing=InsertFixturePlanner.select(hole,F.compose(p.document.setup.sourceToMold,i.pose),face,h);
				if (fixing==null) continue;
				var bore=PanelContours.circle(fixing.entry.x,fixing.entry.y,fixing.clearanceDiameter/2),clear=true;
				for (index in 0...2) {
					var offset=index==0 ? -pinSpacing/2 : pinSpacing/2;
					var x=cx+(alongY ? 0 : offset),y=cy+(alongY ? offset : 0);
					var locator=index==0 ? PanelContours.circle(x,y,d/2)
						: PanelContours.obround(x,y,d+(alongY ? 0 : h.secondaryPinTravel),d+(alongY ? h.secondaryPinTravel : 0));
					if (materia.casting.CastingProfileGeometry.distanceBetween(bore,locator)<r.minimumLigament-1e-7)
						clear=false;
				}
				if (clear) count++;
			}
			return count;
		}
		if (!alongX && !alongY) {
			// An undersized support remains blocked by the bore/ligament checks.
			pinSpacing=h.fixturePinSpacing;
		} else if (alongY && (!alongX || clearFixings(true)>clearFixings(false))) {
			pinAlongY=true;
		}
		for (index in 0...2) {
			var operationId = index == 0 ? primary : secondary;
			var offset = index == 0 ? -pinSpacing / 2 : pinSpacing / 2;
			var x = cx + (pinAlongY ? 0 : offset), y = cy + (pinAlongY ? offset : 0);
			var diameter = d;
			var depth = r.panel.measuredThickness;
			var operation = PanelJoints.operation(operationId, Drill, PanelContours.circle(x - bed.pose.x, y - bed.pose.y, diameter / 2), depth, diameter);
			operation.through = true;
			operation.accessVia = pocketId;
			bed.panel.operations.push(operation);
			bed.panel.contactOpenings.push(operation.id);
			p.openings.push({
				panelId: bed.panel.id,
				operationId: operation.id,
				kind: ToolingOpening,
				sealingIds: [datum.id],
				protection:null, sealSpec: h.sealMaterial
			});
			MoldOpenings.attach(p,{
				panelId: bed.panel.id,
				operationId: operation.id,
				kind: InsertSealed,
				insertId: i.id,
				coreId: null,
				sealMargin: 0,
				sealMaterial: h.sealMaterial,
				sealingFace: F.translation(x, y, floor),
				plug: null
			});
			var receiver = index == 1 ? PanelContours.obround(x, y, d + (pinAlongY ? 0 : h.secondaryPinTravel), d + (pinAlongY ? h.secondaryPinTravel : 0)) : PanelContours.circle(x, y, d / 2);
			if (index == 1 && h.fixtureSlotToolDiameter > d + 1e-7)
				throw "Fixture secondary-slot cutter exceeds slot width";
			var receiverDepth = embed;
			var machining = PanelJoints.operation(operationId, index == 1 ? Pocket : Drill, receiver, receiverDepth,
				index == 1 ? h.fixtureSlotToolDiameter : d, MachiningSide.NegativeZ,
				Math.abs(floor + receiverDepth - face.z) < 1e-7);
			datum.cuts.push({operation: machining, pose: F.translation(0, 0, floor)});
			p.hardware.push({
				id: operationId,
				kind: FixturePin,
				diameter: pinDiameter,
				length: floor - bed.pose.z + embed,
				pose: F.translation(x, y, bed.pose.z)
			});
		}
		var fixings:Array<InsertFixtureFixing> = [];
		var sourceCandidates = 0;
		for (hole in precastHoles) {
			var selected = InsertFixturePlanner.select(hole, F.compose(p.document.setup.sourceToMold, i.pose), face, h);
			if (selected == null)
				continue;
			var entry = selected.entry, engagement = selected.engagement;
			var thread:materia.casting.InsertHole.InsertThread = cast hole.thread;
			sourceCandidates++;
			var diameter = selected.clearanceDiameter;
			// A straight clearance bore must remain inside the supporting datum.
			var path:Array<MoldBox> = [{x: entry.x - diameter / 2, y: entry.y - diameter / 2,
				z: Math.max(floor, bed.pose.z + r.panel.measuredThickness), width: diameter, depth: diameter,
				height: entry.z - Math.max(floor, bed.pose.z + r.panel.measuredThickness)}];
			if (path[0].height > 1e-7 && BoxAlgebra.volume(BoxAlgebra.subtract(path, datum.pieces)) > 1e-7)
				continue;
			var fixingId = ToolingIds.derive("fixing-", i.id + "-" + hole.id);
			var clearance = PanelJoints.operation(fixingId, Drill,
				PanelContours.circle(entry.x - bed.pose.x, entry.y - bed.pose.y, diameter / 2),
				r.panel.measuredThickness, diameter);
			clearance.through = true;
			clearance.accessVia = pocketId;
			var conflicts = bed.panel.operations.filter(other -> other.kind != OutsideProfile && other.id != pocketId
				&& materia.casting.CastingProfileGeometry.distanceBetween(clearance.contour, other.contour) < r.minimumLigament - 1e-7);
			if (conflicts.length > 0)
				continue;
			bed.panel.operations.push(clearance);
			bed.panel.contactOpenings.push(fixingId);
			p.openings.push({panelId: bed.panel.id, operationId: fixingId, kind: ToolingOpening,
				sealingIds: [ToolingIds.derive("insert-", i.id), datum.id], protection:null, sealSpec: h.sealMaterial});
			MoldOpenings.attach(p,{panelId: bed.panel.id, operationId: fixingId, kind: InsertSealed,
				insertId: i.id, sourceHoleId: hole.id, coreId: null, sealMargin: h.sealMargin, sealMaterial: h.sealMaterial,
				sealingFace: F.translation(entry.x, entry.y, floor), plug: selected.plug});
			if (selected.plug != null) {
				var plug = selected.plug;
				p.hardware.push({id: ToolingIds.derive("resin-plug-", i.id + "-" + hole.id), kind: BorePlug,
					diameter: plug.diameter, length: plug.length,
					pose: F.compose(entry, F.translation(0, 0, hole.depth - plug.length))});
			}
			var depth = entry.z - floor;
			if (depth > 1e-7) {
				var receiver = PanelJoints.operation(fixingId, Drill, PanelContours.circle(entry.x, entry.y, diameter / 2),
					depth, diameter, MachiningSide.NegativeZ, true);
				datum.cuts.push({operation: receiver, pose: F.translation(0, 0, floor)});
			}
			p.hardware.push({id: fixingId, kind: FixtureScrew, diameter: thread.majorDiameter,
				length: entry.z - bed.pose.z + selected.insertion,
				pose: F.fromRotationMatrix(entry.x, entry.y, bed.pose.z, [1, 0, 0, 0, -1, 0, 0, 0, -1])});
			fixings.push({sourceHoleId: hole.id, hardwareId: fixingId, panelId: bed.panel.id,
				operationId: fixingId, entry: entry, engagement: engagement, threadSpec: thread.designation});
		}
		if (fixings.length == 0 && sourceCandidates == 0) {
			var decision=InsertFixturePlanner.missingFixing(i,F.compose(p.document.setup.sourceToMold,i.pose),face,h),change=decision.change;
			if (change!=null) p.changeRequests.push(change);
			p.validation.push(decision.check);
		}
		if (fixings.length == 0 && sourceCandidates > 0)
			p.validation.push(materia.tooling.ToolingValidation.ToolingReadiness.pending("tooling.fixture.clearance." + i.id, [i.id, datum.id],
				"Source threads exist, but the support/locator layout leaves no clear screw path; revise the tooling fixture."));
		var masks:Array<BoreMaskProtection> = [];
		for (hole in precastHoles) {
			if (fixings.filter(f -> f.sourceHoleId == hole.id).length > 0)
				continue;
			if (h.plugMaterial != "steel")
				throw "Unsupported nominal bore-mask material";
			var entry = F.compose(F.compose(p.document.setup.sourceToMold, i.pose), hole.frame);
			var protection = InsertFixturePlanner.mask(hole, ToolingIds.derive("mask-", i.id + "-" + hole.id), h.plugDepth,
				maskWithdrawal(p, i, hole, withdrawal));
			var mask:SupportHardware = {
				id: ToolingIds.derive("mask-", i.id + "-" + hole.id),
				kind: BoreMask,
				diameter: hole.thread == null ? hole.diameter : hole.thread.majorDiameter,
				length: protection.plugLength == null ? hole.depth : protection.plugLength,
				pose: entry
			};
			p.hardware.push(mask);
			masks.push(protection);
			if (protection.farEndId != null)
				p.hardware.push({id: protection.farEndId, kind: BorePlug, diameter: mask.diameter, length: mask.length,
					pose: F.compose(entry, F.translation(0, 0, hole.depth - mask.length))});
		}
		var location=DatumFixtureLocation.plan(i,p.document.setup.sourceToMold,{mountingFace:face,datum:datum,fixings:fixings});
		p.validation.push(DatumFixtureLocation.validation(i.id,location));
		p.fixtures.push({
			insertId: i.id,
			location:location,
			support: OnDatum(datum,withdrawal,{primaryId:primary,secondaryId:secondary,baselineMm:pinSpacing}),
			mountingFace: face,
			attachmentEvidence: [],
			assemblyClearance: {
				x: withdrawal.x,
				y: withdrawal.y,
				z: Math.abs(withdrawal.x) > 0.9 ? 0 : 1
			},
			fixings: fixings,
			boreMasks: masks
		});
	}

	static function fixingId(p:CastingMoldPlan,insertId:String,holeId:String):String
		return p.construction==SinglePocket ? ToolingIds.derive("insert-fixing-",holeId)
			: ToolingIds.derive("fixing-",insertId+"-"+holeId);

	static function maskId(p:CastingMoldPlan,insertId:String,holeId:String):String
		return ToolingIds.derive("mask-",p.construction==SinglePocket ? holeId : insertId+"-"+holeId);

	static function plugId(p:CastingMoldPlan,insertId:String,holeId:String):String
		return ToolingIds.derive("resin-plug-",p.construction==SinglePocket ? holeId : insertId+"-"+holeId);

	static function memberId(p:CastingMoldPlan,insertId:String):String
		return p.construction==SinglePocket ? insertId : ToolingIds.derive("insert-",insertId);

	static function panelMaskDiameter(hole:materia.casting.InsertHole,r:materia.tooling.ToolingRecipe):Float {
		var maskDiameter=hole.thread==null ? hole.diameter : hole.thread.majorDiameter;
		return Math.max(hole.clearanceDiameter,maskDiameter+r.jointClearance);
	}

	/** One direct receiver for a bed, wall or dry carrier. Every source-facing
	 * mask has a clearance access hole, so release never needs a resin-side
	 * counterbore or a thread invented in the tooling.
	 */
	public static function addPanel(p:CastingMoldPlan,insert:CastingToolingInsert,
			face:materia.assembly.AssemblyRecord.AssemblyFrame,panel:PanelPlacement):Void {
		var r=p.document.recipe,h=r.hardware,thickness=panel.panel.stock.measuredThickness;
		var stock=F.compose(p.document.setup.sourceToMold,insert.pose),inverse=F.inverse(panel.pose);
		var fixings:Array<InsertFixtureFixing>=[];
		var masks:Array<BoreMaskProtection>=[];
		function access(id:String,hole:materia.casting.InsertHole,entry:materia.assembly.AssemblyRecord.AssemblyFrame,
				diameter:Float,plug:Null<{diameter:Float,length:Float,materialSpec:String}>):Void {
			var point=F.transformPoint(inverse,entry.x,entry.y,entry.z);
			var operation=PanelJoints.operation(id,Drill,PanelContours.circle(point.x,point.y,diameter/2),
				thickness,diameter,MachiningSide.PositiveZ,true);
			panel.panel.operations.push(operation);
			panel.panel.contactOpenings.push(id);
			p.openings.push({panelId:panel.panel.id,operationId:id,kind:ToolingOpening,
				sealingIds:[memberId(p,insert.id)],protection:null,sealSpec:h.sealMaterial});
			MoldOpenings.attach(p,{panelId:panel.panel.id,operationId:id,kind:InsertSealed,
				insertId:insert.id,sourceHoleId:hole.id,coreId:null,sealMargin:h.sealMargin,
				sealMaterial:h.sealMaterial,sealingFace:face,plug:plug});
		}
		for (hole in InsertFixturePlanner.availableHoles(insert)) {
			var entry=F.compose(stock,hole.frame),axis=F.transformVector(entry,0,0,1);
			var selected=InsertFixturePlanner.select(hole,stock,face,h);
			if (selected!=null) {
				var id=fixingId(p,insert.id,hole.id),thread:materia.casting.InsertHole.InsertThread=cast hole.thread;
				access(id,hole,entry,selected.clearanceDiameter,selected.plug);
				var head=F.compose(F.compose(entry,F.translation(0,0,-thickness)),
					F.fromRotationMatrix(0,0,0,[1,0,0,0,-1,0,0,0,-1]));
				p.hardware.push({id:id,kind:FixtureScrew,diameter:thread.majorDiameter,
					length:thickness+selected.insertion,pose:head});
				fixings.push({sourceHoleId:hole.id,hardwareId:id,panelId:panel.panel.id,operationId:id,
					entry:entry,engagement:selected.engagement,threadSpec:thread.designation});
				var cap:Null<{diameter:Float,length:Float,materialSpec:String}>=selected.plug;
				if (cap!=null) p.hardware.push({id:plugId(p,insert.id,hole.id),kind:BorePlug,
					diameter:cap.diameter,length:cap.length,
					pose:F.compose(entry,F.translation(0,0,hole.depth-cap.length))});
				continue;
			}
			if (h.plugMaterial!="steel") throw "Unsupported nominal bore-mask material";
			var id=maskId(p,insert.id,hole.id);
			var mask=InsertFixturePlanner.mask(hole,id,h.plugDepth,{x:-axis.x,y:-axis.y,z:-axis.z});
			masks.push(mask);
			var diameter=hole.thread==null ? hole.diameter : hole.thread.majorDiameter;
			var length=mask.plugLength==null ? hole.depth : mask.plugLength;
			p.hardware.push({id:id,kind:BoreMask,diameter:diameter,length:length,pose:entry});
			if (mask.farEndId!=null) p.hardware.push({id:mask.farEndId,kind:BorePlug,
				diameter:diameter,length:length,pose:F.compose(entry,F.translation(0,0,hole.depth-length))});
			if (InsertFixturePlanner.facing(entry,face))
				access(ToolingIds.derive("access-",id),hole,entry,panelMaskDiameter(hole,r),null);
		}
		panel.panel=PanelCodec.copy(panel.panel);
		var location=DatumFixtureLocation.onPanel(insert,p.document.setup.sourceToMold,face,panel,fixings);
		p.fixtures.push({insertId:insert.id,location:location,support:OnPanel(panel.panel.id),mountingFace:face,
			fixings:fixings,boreMasks:masks,attachmentEvidence:[],assemblyClearance:{x:0,y:0,z:1}});
		p.validation=p.validation.concat(InsertFixturePlanner.protectionValidation(insert));
		p.validation.push(DatumFixtureLocation.validation(insert.id,location));
		if (fixings.length==0) {
			var decision=InsertFixturePlanner.missingFixing(insert,stock,face,h);
			p.validation.push(decision.check);
			if (decision.change!=null) p.changeRequests.push(decision.change);
		}
	}

	static function receiver(p:CastingMoldPlan,id:String):PanelPlacement {
		var matches=p.panels.members.concat(p.supportPanels).filter(member -> member.panel.id==id);
		if (matches.length!=1) throw "Fixture needs one receiver panel: "+id;
		return matches[0];
	}

	static function sourceInsert(p:CastingMoldPlan,id:String):CastingToolingInsert {
		var matches=p.document.input.inserts.filter(insert -> insert.id==id);
		if (matches.length!=1) throw "Fixture needs one source insert: "+id;
		return matches[0];
	}

	/** Check an insert-covered panel bore before any native CAD is built. */
	public static function validatePanelSeal(p:CastingMoldPlan,record:PenetrationPlan):Void {
		var fixtures=p.fixtures.filter(fixture -> fixture.insertId==record.insertId);
		if (fixtures.length!=1) throw "Insert panel seal needs one fixture";
		var fixture=fixtures[0],insert=sourceInsert(p,fixture.insertId);
		if (record.panelId!=MoldFixtures.panelId(fixture)) throw "Insert seal references wrong receiver panel";
		var panel=receiver(p,record.panelId),operations=panel.panel.operations.filter(op -> op.id==record.operationId);
		var holes=InsertFixturePlanner.availableHoles(insert).filter(hole -> hole.id==record.sourceHoleId);
		if (operations.length!=1 || holes.length!=1) throw "Insert penetration has no unique source hole/operation";
		var operation=operations[0],hole=holes[0],face=fixture.mountingFace;
		var entry=F.compose(F.compose(p.document.setup.sourceToMold,insert.pose),hole.frame);
		if (!InsertFixturePlanner.facing(entry,face)
			|| ToolingCodec.canonical(record.sealingFace)!=ToolingCodec.canonical(face))
			throw "Insert sealing face differs from source fixture";
		var fixings=fixture.fixings.filter(fixing -> fixing.sourceHoleId==hole.id && fixing.operationId==operation.id);
		var masks=fixture.boreMasks.filter(mask -> mask.sourceHoleId==hole.id
			&& ToolingIds.derive("access-",mask.hardwareId)==operation.id);
		if (fixings.length+masks.length!=1) throw "Insert seal references wrong fixing or mask access hole";
		var diameter=fixings.length==1 ? hole.clearanceDiameter : panelMaskDiameter(hole,p.document.recipe);
		var local=F.transformPoint(F.inverse(panel.pose),entry.x,entry.y,entry.z);
		var expected=PanelContours.circle(local.x,local.y,diameter/2);
		if (operation.kind!=Drill || !operation.through || operation.depth!=panel.panel.stock.measuredThickness
			|| ToolingCodec.canonical(operation.contour)!=ToolingCodec.canonical(expected))
			throw "Panel fixture needs a source-sized clearance/access bore";
		var selected=fixings.length==1 ? InsertFixturePlanner.select(hole,
			F.compose(p.document.setup.sourceToMold,insert.pose),face,p.document.recipe.hardware) : null;
		if (fixings.length==1 && selected==null
			|| ToolingCodec.canonical(record.plug)!=ToolingCodec.canonical(selected==null ? null : selected.plug))
			throw "Panel fixture protection differs from source depth budget";
		var margin=record.sealMargin;
		if (!Math.isFinite(margin) || margin<p.document.recipe.hardware.sealMargin
			|| record.sealMaterial!=p.document.recipe.hardware.sealMaterial)
			throw "Panel fixture seal differs from recipe";
		var at=F.transformPoint(F.inverse(face),entry.x,entry.y,entry.z),radius=diameter/2+margin;
		switch materia.casting.InsertSpec.InsertBodyCodec.mouth(insert.body) {
			case Box(width,depth,_):
				if (Math.abs(at.x)+radius>width/2+1e-7 || Math.abs(at.y)+radius>depth/2+1e-7)
					throw "Insert sealing margin exceeded";
			case Cylinder(outside,_):
				if (Math.sqrt(at.x*at.x+at.y*at.y)+radius>outside/2+1e-7)
					throw "Insert sealing margin exceeded";
		}
	}

	public static function validatePanel(p:CastingMoldPlan,fixture:MoldInsertFixture):Void {
		var source=sourceInsert(p,fixture.insertId),panel=receiver(p,MoldFixtures.panelId(fixture));
		var stock=F.compose(p.document.setup.sourceToMold,source.pose);
		var face=F.compose(stock,F.translation(0,0,materia.casting.InsertSpec.InsertBodyCodec.height(source.body)));
		if (ToolingCodec.canonical(fixture.mountingFace)!=ToolingCodec.canonical(face))
			throw "Panel fixture face differs from source mouth: "+source.id;
		var holes=InsertFixturePlanner.availableHoles(source);
		if (fixture.fixings.length+fixture.boreMasks.length!=holes.length)
			throw "Every pre-cast source hole needs a fixing screw or bore mask";
		var openings=MoldOpenings.records(p,source.id),expectedOpenings=0;
		for (hole in holes) {
			var entry=F.compose(stock,hole.frame),axis=F.transformVector(entry,0,0,1);
			var fixings=fixture.fixings.filter(fixing -> fixing.sourceHoleId==hole.id);
			var masks=fixture.boreMasks.filter(mask -> mask.sourceHoleId==hole.id);
			if (fixings.length+masks.length!=1) throw "Source hole protection must be unique";
			if (fixings.length==1) {
				var thread:materia.casting.InsertHole.InsertThread=cast hole.thread;
				var fixing=fixings[0],selected=InsertFixturePlanner.select(hole,stock,face,p.document.recipe.hardware);
				if (selected==null || hole.thread==null || fixing.hardwareId!=fixingId(p,source.id,hole.id)
					|| fixing.panelId!=panel.panel.id || fixing.operationId!=fixing.hardwareId
					|| fixing.engagement!=selected.engagement || fixing.threadSpec!=thread.designation
					|| ToolingCodec.canonical(fixing.entry)!=ToolingCodec.canonical(entry))
					throw "Fixture fixing differs from source hole";
				var hardware=p.hardware.filter(item -> item.id==fixing.hardwareId);
				var head=F.compose(F.compose(entry,F.translation(0,0,-panel.panel.stock.measuredThickness)),
					F.fromRotationMatrix(0,0,0,[1,0,0,0,-1,0,0,0,-1]));
				if (hardware.length!=1 || hardware[0].kind!=FixtureScrew || hardware[0].diameter!=thread.majorDiameter
					|| hardware[0].length!=panel.panel.stock.measuredThickness+selected.insertion
					|| ToolingCodec.canonical(hardware[0].pose)!=ToolingCodec.canonical(head))
					throw "Panel fixing hardware differs from source engagement and stack";
				var cap:Null<{diameter:Float,length:Float,materialSpec:String}>=selected.plug;
				if (cap!=null) {
					var caps=p.hardware.filter(item -> item.id==plugId(p,source.id,hole.id));
					var position=F.compose(entry,F.translation(0,0,hole.depth-cap.length));
					if (caps.length!=1 || caps[0].kind!=BorePlug || caps[0].diameter!=cap.diameter
						|| caps[0].length!=cap.length || ToolingCodec.canonical(caps[0].pose)!=ToolingCodec.canonical(position))
						throw "Missing or inconsistent panel fixing end plug";
				}
				expectedOpenings++;
			} else {
				var mask=masks[0],id=maskId(p,source.id,hole.id);
				var expected=InsertFixturePlanner.mask(hole,id,p.document.recipe.hardware.plugDepth,
					{x:-axis.x,y:-axis.y,z:-axis.z});
				var hardware=p.hardware.filter(item -> item.id==id),length=mask.plugLength==null ? hole.depth : mask.plugLength;
				if (ToolingCodec.canonical(mask)!=ToolingCodec.canonical(expected) || hardware.length!=1
					|| hardware[0].kind!=BoreMask || hardware[0].diameter!=(hole.thread==null ? hole.diameter : hole.thread.majorDiameter)
					|| hardware[0].length!=length || ToolingCodec.canonical(hardware[0].pose)!=ToolingCodec.canonical(entry))
					throw "Bore mask differs from source hole depth budget";
				if (mask.farEndId!=null) {
					var caps=p.hardware.filter(item -> item.id==mask.farEndId);
					var position=F.compose(entry,F.translation(0,0,hole.depth-length));
					if (caps.length!=1 || caps[0].kind!=BorePlug || caps[0].length!=length
						|| ToolingCodec.canonical(caps[0].pose)!=ToolingCodec.canonical(position))
						throw "Missing source mask end plug";
				}
				if (InsertFixturePlanner.facing(entry,face)) expectedOpenings++;
			}
		}
		if (openings.length!=expectedOpenings) throw "Missing or extra panel fixture access records";
		for (opening in openings) validatePanelSeal(p,opening);
		if (fixture.fixings.length==0)
			InsertFixturePlanner.validateMissingFixing(source,stock,face,p.document.recipe.hardware,p.validation,p.changeRequests);
		var location=DatumFixtureLocation.onPanel(source,p.document.setup.sourceToMold,face,panel,fixture.fixings);
		if (ToolingCodec.canonical(location)!=ToolingCodec.canonical(fixture.location))
			throw "Panel fixture location differs from source geometry: "+source.id;
		var expected=DatumFixtureLocation.validation(source.id,location);
		var checks=p.validation.filter(check -> check.rule==expected.rule);
		if (checks.length!=1 || ToolingCodec.canonical(checks[0])!=ToolingCodec.canonical(expected))
			throw "Panel fixture location validation differs from source geometry: "+source.id;
	}

	public static function validate(p:CastingMoldPlan):Void {
		for (f in p.fixtures) {
			switch f.support {
				case OnPanel(_):validatePanel(p,f);continue;
				case OnDatum(_,_,_):
			}
			if (MoldOpenings.records(p,f.insertId).length != 2 + f.fixings.length || MoldFixtures.datum(f).cuts.length < 2)
				throw "Incomplete fixture locator/fixing records";
			var ids = [for (c in MoldFixtures.datum(f).cuts) c.operation.id];
			if (ids.indexOf(MoldFixtures.lateral(f).primaryId) < 0
				|| ids.indexOf(MoldFixtures.lateral(f).secondaryId) < 0)
				throw "Fixture locator references unknown receiver";
			for (pen in MoldOpenings.records(p,f.insertId)) {
				var panel = p.panels.members.filter(m -> m.panel.id == pen.panelId),
					face = pen.sealingFace;
				if (panel.length != 1 || pen.insertId != f.insertId || face == null)
					throw "Invalid insert-sealed fixture penetration";
				var operations = panel[0].panel.operations.filter(o -> o.id == pen.operationId);
				if (operations.length != 1)
					throw "Missing fixture penetration operation";
				var b = materia.casting.CastingProfileGeometry.bounds(operations[0].contour);
				var center = F.transformPoint(panel[0].pose, b.x + b.width / 2, b.y + b.height / 2, panel[0].panel.stock.measuredThickness);
				if (Math.abs(center.x - face.x) + Math.abs(center.y - face.y) > 1e-7)
					throw "Fixture seal face misses penetration";
				var covered = false;
				for (foot in MoldFixtures.datum(f).pieces)
					if (Math.abs(foot.z - face.z) < 1e-7
						&& center.x - b.width / 2 >= foot.x - 1e-7
						&& center.x + b.width / 2 <= foot.x + foot.width + 1e-7
						&& center.y - b.height / 2 >= foot.y - 1e-7
						&& center.y + b.height / 2 <= foot.y + foot.depth + 1e-7)
						covered = true;
				if (!covered)
					throw "Fixture sealing footprint does not cover bed penetration";
			}
			var source = p.document.input.inserts.filter(i -> i.id == f.insertId)[0];
			var precastHoles=InsertFixturePlanner.availableHoles(source);
			var location=DatumFixtureLocation.plan(source,p.document.setup.sourceToMold,
				{mountingFace:f.mountingFace,datum:MoldFixtures.datum(f),fixings:f.fixings});
			if (ToolingCodec.canonical(location)!=ToolingCodec.canonical(f.location))
				throw "Insert fixture location differs from source geometry: "+source.id;
			var locationCheck=DatumFixtureLocation.validation(source.id,location);
			var locationChecks=p.validation.filter(check -> check.rule==locationCheck.rule);
			if (locationChecks.length!=1 || ToolingCodec.canonical(locationChecks[0])!=ToolingCodec.canonical(locationCheck))
				throw "Insert fixture location validation differs from source geometry: "+source.id;
			if (f.fixings.length==0) {
				var pose=F.compose(p.document.setup.sourceToMold,source.pose);
				var candidates=precastHoles.filter(hole -> InsertFixturePlanner.select(hole,pose,f.mountingFace,p.document.recipe.hardware)!=null);
				if (candidates.length==0)
					InsertFixturePlanner.validateMissingFixing(source,pose,f.mountingFace,p.document.recipe.hardware,p.validation,p.changeRequests);
				else if (p.validation.filter(v -> v.rule=="tooling.fixture.clearance."+source.id).length!=1)
					throw "Fixture without clear source screw path needs a tooling clearance decision";
			}
			if (f.boreMasks.length + f.fixings.length != precastHoles.length)
				throw "Every pre-cast source hole needs a fixing screw or explicit release mask";
			for (hole in precastHoles) {
				var fixings = f.fixings.filter(fixing -> fixing.sourceHoleId == hole.id);
				if (fixings.length > 0) {
					var thread:materia.casting.InsertHole.InsertThread = cast hole.thread;
					var fixing = fixings[0], screws = p.hardware.filter(h -> h.id == fixing.hardwareId);
					var entry = F.compose(F.compose(p.document.setup.sourceToMold, source.pose), hole.frame);
					var selected = InsertFixturePlanner.select(hole, F.compose(p.document.setup.sourceToMold, source.pose), f.mountingFace,
						p.document.recipe.hardware);
					var penetrations = MoldOpenings.records(p,f.insertId).filter(pen -> pen.sourceHoleId == hole.id && pen.operationId == fixing.operationId);
					if (selected == null || penetrations.length != 1
						|| ToolingCodec.canonical(penetrations[0].plug) != ToolingCodec.canonical(selected.plug))
						throw "Fixture fixing resin-side protection differs from source depth budget: " + hole.id;
					if (selected.plug != null) {
						var plug = selected.plug;
						var hardware = p.hardware.filter(h -> h.id == ToolingIds.derive("resin-plug-", source.id + "-" + hole.id));
						if (hardware.length != 1 || hardware[0].kind != BorePlug || hardware[0].diameter != plug.diameter
							|| hardware[0].length != plug.length
							|| ToolingCodec.canonical(hardware[0].pose) != ToolingCodec.canonical(F.compose(entry, F.translation(0, 0, hole.depth - plug.length))))
							throw "Missing or inconsistent resin-side fixing plug hardware: " + hole.id;
					}
					var receivers = MoldFixtures.datum(f).cuts.filter(cut -> cut.operation.id == fixing.operationId);
					var expected = PanelContours.circle(entry.x, entry.y,
						hole.clearanceDiameter / 2);
					if (receivers.length != 1 || receivers[0].operation.kind != Drill || !receivers[0].operation.through
						|| ToolingCodec.canonical(receivers[0].operation.contour) != ToolingCodec.canonical(expected))
						throw "Datum fixing receiver must be a source-sized clearance bore";
					if (fixings.length != 1 || hole.thread == null || screws.length != 1
						|| screws[0].kind != FixtureScrew || screws[0].diameter != thread.majorDiameter
						|| Math.abs(screws[0].length - (entry.z - screws[0].pose.z + InsertFixturePlanner.minimumInsertion(hole))) > 1e-7
						|| Math.abs(screws[0].pose.x - entry.x) + Math.abs(screws[0].pose.y - entry.y) > 1e-7
						|| MoldOpenings.records(p,f.insertId).filter(pen -> pen.sourceHoleId == hole.id && pen.operationId == fixing.operationId).length != 1
						|| fixing.engagement != materia.casting.InsertHole.InsertHoleCodec.threadLimits(hole).minEngagement
						|| fixing.threadSpec != thread.designation
						|| ToolingCodec.canonical(fixing.entry) != ToolingCodec.canonical(entry)
						|| f.boreMasks.filter(mask -> mask.sourceHoleId == hole.id).length != 0)
						throw "Fixture fixing differs from source thread: " + hole.id;
					continue;
				}
				var protections = f.boreMasks.filter(mask -> mask.sourceHoleId == hole.id);
				if (protections.length != 1)
					throw "Source hole needs one unique release mask: " + hole.id;
				var protection = protections[0], hardware = p.hardware.filter(h -> h.id == protection.hardwareId);
				var expectedProtection = InsertFixturePlanner.mask(hole, protection.hardwareId, p.document.recipe.hardware.plugDepth,
					maskWithdrawal(p, source, hole, MoldFixtures.withdrawal(f)));
				if (ToolingCodec.canonical(protection) != ToolingCodec.canonical(expectedProtection))
					throw "Source thread protection differs from its depth budget: " + hole.id;
				if (protection.farEndId != null) {
					var far = p.hardware.filter(h -> h.id == protection.farEndId);
					var entry = F.compose(F.compose(p.document.setup.sourceToMold, source.pose), hole.frame);
					if (far.length != 1 || far[0].kind != BorePlug || far[0].length != protection.plugLength
						|| ToolingCodec.canonical(far[0].pose) != ToolingCodec.canonical(F.compose(entry, F.translation(0, 0, hole.depth - far[0].length))))
						throw "Missing resin-side thread mask: " + hole.id;
				}
				var entry = F.compose(F.compose(p.document.setup.sourceToMold, source.pose), hole.frame);
				if (hardware.length != 1 || hardware[0].kind != BoreMask || hardware[0].diameter != (hole.thread == null ? hole.diameter : hole.thread.majorDiameter)
					|| hardware[0].length != (protection.plugLength == null ? hole.depth : protection.plugLength) || ToolingCodec.canonical(hardware[0].pose) != ToolingCodec.canonical(entry)
					|| ToolingCodec.canonical(protection.withdrawal) != ToolingCodec.canonical(maskWithdrawal(p, source, hole, MoldFixtures.withdrawal(f))))
					throw "Bore mask differs from its source hole: " + hole.id;
			}
		}
	}
}
