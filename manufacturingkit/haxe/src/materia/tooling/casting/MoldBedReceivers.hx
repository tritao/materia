package materia.tooling.casting;

import materia.assembly.AssemblyFrames as F;
import materia.tooling.casting.CastingMoldPlan;
import materia.casting.CastingRegion;
import materia.tooling.panel.PanelContours;
import materia.tooling.panel.PanelJoints;
import materia.tooling.panel.PanelCodec;

/** Proud raw steel sits in a routed bed rebate. The matched cured closure fills
 * the cutter relief and clearance, so no resin-side gap is silently ignored. */
class MoldBedReceivers {
	/** Cure the actual clearance/relief around a dry-fitted datum. Its drilled
	 * holes remain dry access passages, not compound-filled receiver volume. */
	public static function datum(plan:CastingMoldPlan,insertId:String,
		bed:materia.tooling.panel.PanelAssemblyPlan.PanelPlacement,volume:MoldVolume,pocketId:String):Void {
		var operations=bed.panel.operations.filter(operation -> operation.id==pocketId);
		if (operations.length!=1 || operations[0].through || operations[0].side!=materia.tooling.ToolingEnums.MachiningSide.PositiveZ)
			throw "Datum closure needs its actual blind receiver: "+pocketId;
		var operation=operations[0],gross=MoldRegions.MoldVolumeRecords.copy(volume);
		gross.cuts=[];
		var sealId=materia.tooling.ToolingIds.derive("receiver-seal-",volume.id);
		plan.seals.push({panelId:bed.panel.id,insertId:insertId,materialSpec:plan.document.recipe.hardware.sealMaterial,
			volume:{id:sealId,material:SealantStock,pieces:[],cuts:[],regions:[],
				profiles:[{panelId:bed.panel.id,operationId:pocketId,contour:operation.contour,depth:operation.depth,
					pose:F.compose(bed.pose,F.translation(0,0,bed.panel.stock.measuredThickness-operation.depth)),
					subtract:[MoldRegions.tooling(gross)]}]}});
		var openings=plan.openings.filter(opening -> opening.panelId==bed.panel.id && opening.operationId==pocketId);
		if (openings.length!=1) throw "Datum closure needs one receiver opening: "+pocketId;
		openings[0].sealingIds.push(sealId);
	}

	public static function add(plan:CastingMoldPlan):Void {
		var d=plan.document,r=d.recipe,bed=plan.panels.members[0],t=bed.panel.stock.measuredThickness;
		for (insert in d.input.inserts) {
			var pose=F.compose(d.setup.sourceToMold,insert.pose);
			var face=F.compose(pose,F.translation(0,0,materia.casting.InsertSpec.InsertBodyCodec.height(insert.body)));
			var local=F.compose(F.inverse(bed.pose),face),normal=F.transformVector(local,0,0,1);
			var depth=t-local.z;
			if (depth<=1e-7) continue;
			if (normal.z> -1+1e-7) throw "Steel below the bed needs an aligned mounting face: "+insert.id;
			if (depth+r.minimumLigament>t) throw "Proud steel consumes the bed's remaining ligament: "+insert.id;
			var clearance=r.jointClearance;
			var contour=switch materia.casting.InsertSpec.InsertBodyCodec.mouth(insert.body) {
				case Box(width,height,_):PanelContours.relievedRectangle(0,0,width+2*clearance,height+2*clearance,r.routerDiameter/2);
				case Cylinder(diameter,_):PanelContours.circle(0,0,diameter/2+clearance);
			};
			var u=F.transformVector(local,1,0,0);
			contour=PanelContours.placed(contour,Math.atan2(u.y,u.x),local.x,local.y);
			var id=materia.tooling.ToolingIds.derive("steel-rebate-",insert.id);
			var operation=PanelJoints.operation(id,Pocket,contour,depth,r.routerDiameter,PositiveZ,false,r.routerDiameter);
			bed.panel.operations.push(operation);bed.panel.contactOpenings.push(id);
			var sealId=materia.tooling.ToolingIds.derive("bed-seal-",insert.id);
			plan.seals.push({panelId:bed.panel.id,insertId:insert.id,materialSpec:r.hardware.sealMaterial,
				volume:{id:sealId,material:SealantStock,pieces:[],cuts:[],regions:[],
					profiles:[{panelId:bed.panel.id,operationId:id,contour:operation.contour,depth:depth,
						pose:F.compose(bed.pose,F.translation(0,0,local.z)),subtract:[MoldRegions.insert(d,insert.id)]}]}});
			plan.openings.push({panelId:bed.panel.id,operationId:id,kind:ToolingOpening,
				sealingIds:[materia.tooling.ToolingIds.derive("insert-",insert.id),sealId],protection:null,sealSpec:r.hardware.sealMaterial});
		}
		bed.panel=PanelCodec.copy(bed.panel);
	}
}
