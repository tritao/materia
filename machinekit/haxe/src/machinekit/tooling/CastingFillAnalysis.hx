package machinekit.tooling;

import cadkit.modeling.Model;
import cadkit.modeling.Part;
import materia.tooling.process.FillAnalysis;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingIds;
import materia.tooling.ToolingCodec;

typedef NativeFillPart = {var id:String; var part:Part;}

/** The native BREP backend of the shared flat-free-surface process model.
 * Input parts are borrowed. Pocket geometry names an exact source component,
 * never a box approximation. Additional output levels cannot affect trap IDs.
 */
class CastingFillAnalysis {
	static inline var EPS=1e-7;

	static function own<T:Model>(resources:Array<Model>,value:T):T {
		resources.push(value);
		return value;
	}

	static function commonMeasure(a:Model,b:Model,volume:Bool):Float {
		var common=new Model(a.shape.common(b.shape));
		try {
			var result=volume ? common.shape.volume() : common.shape.area();
			common.close();return result;
		} catch (error:Dynamic) {common.close();throw error;}
	}

	static function unique(values:Array<Float>):Array<Float> {
		values.sort(Reflect.compare);
		var result:Array<Float> = [];
		for (value in values)
			if (result.length==0 || value-result[result.length-1]>EPS) result.push(value);
		return result;
	}

	/** Bridge actual source and actual mold members into the one process pipeline. */
	public static function fromPlan(plan:materia.tooling.casting.CastingMoldPlan,settings:FillSourceSettings):FillAssessment {
		materia.tooling.casting.CastingMoldPlanner.validate(plan);
		var resources:Array<Model> = [];
		try {
			var source=own(resources,CastingMoldCad.source(plan));
			var bounds=source.shape.bounds(),lo=bounds.get_min(),hi=bounds.get_max();
			var boundaries:Array<NativeFillPart> = [],blockers:Array<NativeFillPart> = [],cores:Array<NativeFillPart> = [];
			for (cavity in materia.casting.CastingCavities.derive(plan.document.input.envelope,plan.document.input.features)) {
				var local=own(resources,machinekit.structural.CastingCavityCad.part(cavity));
				boundaries.push({id:cavity.parentId,part:own(resources,machinekit.assembly.PosedParts.moved(local,plan.document.setup.sourceToMold))});
			}
			var owners=new Map<String,String>();
			for (anchor in CastingMoldCad.anchorComponents(plan)) owners.set(anchor.id,anchor.insertId);
			var assembly=CastingMoldCad.assembly(plan),poses=assembly.solvedPoses();
			for (member in assembly.components()) if (member.component.hasGeometry()) {
				var local=own(resources,member.component.geometry());
				var world=own(resources,machinekit.assembly.PosedParts.moved(local,poses.get(member.id)));
				world.shape.setThreadedFaces([]);
				if (owners.exists(member.id)) boundaries.push({id:owners.get(member.id),part:world});
				var box=world.shape.bounds(),a=box.get_min(),b=box.get_max();
				if (b.get_z()>=plan.document.setup.fillHeightMm-EPS
					&& b.get_x()>lo.get_x()-EPS && a.get_x()<hi.get_x()+EPS
					&& b.get_y()>lo.get_y()-EPS && a.get_y()<hi.get_y()+EPS)
					blockers.push({id:member.id,part:world});
			}
			// Gross displacement is the finished, lined/glued core, not the foam
			// stock pieces with their fabrication gaps. These cavities are exact
			// boxes in the current planner, rather than bounding approximations.
			for (core in plan.cores) cores.push({id:core.id,part:own(resources,ToolingBlock.placedBox(core.cavity))});
			var report=analyze(plan.document.input.id,source,plan.document.setup.fillHeightMm,settings,boundaries,blockers,cores);
			for (resource in resources) resource.close();
			return {report:report,validation:report.validation.copy()};
		} catch (error:Dynamic) {
			for (resource in resources) resource.close();
			var message=Std.string(error);
			if (!StringTools.startsWith(message,"Unsupported fill critical-height curve:")
				&& !StringTools.startsWith(message,"Unsupported exact fill ceiling point:")
				&& message!="Unsupported removal boundary surface") throw error;
			var ids=[plan.document.input.id];
			for (feature in plan.document.input.features) ids.push(feature.id);
			for (insert in plan.document.input.inserts) ids.push(insert.id);
			var check=FillAnalysis.check("process.fill.geometry",ids,Unsupported,message,[settings.sourceFingerprint]);
			return {report:null,validation:[check]};
		}
	}

	public static function analyze(sourceId:String,source:Part,fillHeight:Float,settings:FillSourceSettings,
			?boundaries:Array<NativeFillPart>,?blockers:Array<NativeFillPart>,?coreParts:Array<NativeFillPart>):FillReport {
		ToolingIds.validate(sourceId);
		if (!Math.isFinite(fillHeight) || !Math.isFinite(settings.densityKgM3) || settings.densityKgM3<=0
			|| !Math.isFinite(settings.gravityMps2) || settings.gravityMps2<=0
			|| settings.pourRateMm3PerSecond!=null && (!Math.isFinite(settings.pourRateMm3PerSecond) || settings.pourRateMm3PerSecond<=0)
			|| settings.maximumAirEscapeWidthMm!=null && (!Math.isFinite(settings.maximumAirEscapeWidthMm) || settings.maximumAirEscapeWidthMm<=0)
			|| settings.extraLevels.length>4096)
			throw "Invalid fill analysis inputs";
		var resources:Array<Model> = [],geometries:Array<CastingFillGeometry> = [];
		try {
			var geometry=new CastingFillGeometry(sourceId,source);geometries.push(geometry);
			var critical=geometry.criticalHeights(),floor=critical[0];
			if (critical[critical.length-1]>fillHeight+EPS) throw "Invalid fill geometry or insufficient fill height";
			critical=unique(critical.concat([fillHeight]));
			var open=own(resources,geometry.section(fillHeight));
			var actualBlockers=blockers==null ? [] : blockers.copy();
			if (blockers==null) {
				var footprints=settings.fillObstructions.copy();
				footprints.sort((a,b) -> Reflect.compare(ToolingCodec.canonical(a),ToolingCodec.canonical(b)));
				for (index in 0...footprints.length) {
					var box=footprints[index];
					actualBlockers.push({id:"fill-blocker-"+index,part:own(resources,ToolingBlock.placedBox({
						x:box.x,y:box.y,z:fillHeight,width:box.width,depth:box.depth,height:1
					}))});
				}
			}
			actualBlockers.sort((a,b) -> Reflect.compare(a.id,b.id));
			var blockerIds=new Map<String,Bool>();
			for (blocker in actualBlockers) {
				ToolingIds.validate(blocker.id);
				if (blockerIds.exists(blocker.id)) throw "Duplicate fill blocker ID";
				blockerIds.set(blocker.id,true);
				var context=new CastingFillGeometry(blocker.id,blocker.part);
				try {
					var footprint=own(resources,context.projectedOpening(fillHeight));
					open=own(resources,new Model(open.shape.cut(footprint.shape)));
					context.close();
				} catch (error:Dynamic) {context.close();throw error;}
			}
			var openArea=open.shape.area();
			if (openArea<=EPS) throw "Fill face has zero open area";
			var pockets:Array<AirPocket> = [],trapped:Array<Part> = [];
			for (height in critical) {
				var components=geometry.airComponents(height),keys=new Map<String,Bool>();
				for (part in components) own(resources,part);
				for (part in components) {
					var key=CastingFillGeometry.componentKey(part);
					if (keys.exists(key)) throw "Ambiguous exact fill components: "+sourceId;
					keys.set(key,true);
					var vented=commonMeasure(part,open,false)>EPS;
					var known=false;
					for (previous in trapped)
						if (commonMeasure(part,previous,true)>EPS) known=true;
					if (vented || known) {
						resources.remove(part);part.close();continue;
					}
					var ceiling=new CastingFillGeometry(sourceId,part);geometries.push(ceiling);
					var point=ceiling.highestPoint(),ids:Array<String> = [];
					if (boundaries!=null) for (boundary in boundaries)
						if (commonMeasure(part,boundary.part,false)>EPS && ids.indexOf(boundary.id)<0) ids.push(boundary.id);
					if (ids.length==0) ids.push(sourceId);
					ids.sort(Reflect.compare);
					pockets.push({id:"air-"+haxe.crypto.Sha256.encode(ToolingCodec.canonical({heightMm:height,componentKey:key})).substr(0,16),
						sealedAtMm:height,volumeMm3:part.volume(),geometry:CadAirAbove(height,key),sourceIds:ids,
						proposedVentPoint:{x:point.x,y:point.y,z:point.z}});
					trapped.push(part);
				}
			}
			pockets.sort((a,b) -> Reflect.compare(a.id,b.id));
			var fluid=own(resources,source.subtractAll(trapped));
			var fluidGeometry=new CastingFillGeometry(sourceId,fluid);geometries.push(fluidGeometry);
			var actualCores=coreParts==null ? [] : coreParts.copy();
			if (coreParts==null) for (core in settings.cores) {
				var pieces=[for (box in core.boxes) own(resources,ToolingBlock.placedBox(box))];
				actualCores.push({id:core.id,part:own(resources,Part.fuseAll(pieces))});
			}
			actualCores.sort((a,b) -> Reflect.compare(a.id,b.id));
			var coreGeometries:Array<{id:String,geometry:CastingFillGeometry}> = [];
			var outputs=critical.copy(),coreIds=new Map<String,Bool>();
			for (core in actualCores) {
				ToolingIds.validate(core.id);
				if (coreIds.exists(core.id)) throw "Duplicate fill core ID";
				coreIds.set(core.id,true);
				var context=new CastingFillGeometry(core.id,core.part);geometries.push(context);
				outputs=outputs.concat(context.criticalHeights());coreGeometries.push({id:core.id,geometry:context});
			}
			for (height in settings.extraLevels) {
				if (!Math.isFinite(height) || height<floor-EPS || height>fillHeight+EPS) throw "Invalid extra fill height";
				outputs.push(height);
			}
			var panels=settings.panels.copy();panels.sort((a,b) -> Reflect.compare(a.id,b.id));
			var levels:Array<FillLevel> = [];
			for (height in unique(outputs)) if (height>=floor-EPS && height<=fillHeight+EPS) {
				var below=fluidGeometry.below(height),volume=below.volume();below.close();
				var cores=[];
				for (core in coreGeometries) {
					var submerged=core.geometry.below(height),displaced=submerged.volume();submerged.close();
					cores.push({id:core.id,submergedVolumeMm3:displaced,grossUpliftN:displaced*settings.densityKgM3*settings.gravityMps2*1e-9});
				}
				levels.push({heightMm:height,mixVolumeMm3:volume,mixMassKg:volume*settings.densityKgM3*1e-9,
					timeSeconds:settings.pourRateMm3PerSecond==null ? null : volume/settings.pourRateMm3PerSecond,
					maximumHeadPa:Math.max(0,height-floor)*settings.densityKgM3*settings.gravityMps2*0.001,
					panelHeads:[for (panel in panels) {id:panel.id,headPa:Math.max(0,height-panel.bottomMm)*settings.densityKgM3*settings.gravityMps2*0.001}],
					cores:cores});
			}
			var checks:Array<ToolingValidation> = [FillAnalysis.check("process.fill.geometry",[sourceId],Pass,
				"Exact BREP sections and connected solids at critical levels; no box approximation.",[settings.sourceFingerprint])];
			if (pockets.length==0) checks.push(FillAnalysis.check("process.fill.connected-air",[sourceId],Pass,
				"Every air component reaches the open fill plane at every critical height.",[settings.sourceFingerprint]));
			for (pocket in pockets) checks.push(FillAnalysis.check("process.fill.trap."+pocket.id,pocket.sourceIds,Fail,
				"Air pocket closes at Z="+pocket.sealedAtMm+" mm; limiting trapped volume="+pocket.volumeMm3+" mm3.",[settings.sourceFingerprint]));
			if (settings.maximumAirEscapeWidthMm==null)
				checks.push(FillAnalysis.check("process.fill.air-escape-width",[sourceId],Pending,"No recipe air-escape-width limit declared.",[]));
			else {
				var widthLimit:Float=cast settings.maximumAirEscapeWidthMm;
				try {
					var ceilings=geometry.ceilingWidths(open),offending:Array<String> = [];
					for (ceiling in ceilings) own(resources,ceiling.face);
					for (ceiling in ceilings) if (ceiling.widthMm>widthLimit+1e-6) {
						var ids:Array<String> = [];
						if (boundaries!=null) for (boundary in boundaries)
							if (commonMeasure(ceiling.face,boundary.part,false)>EPS && ids.indexOf(boundary.id)<0) ids.push(boundary.id);
						if (ids.length==0) ids.push(sourceId);
						for (id in ids) if (offending.indexOf(id)<0) offending.push(id);
					}
					offending.sort(Reflect.compare);
					checks.push(FillAnalysis.check("process.fill.air-escape-width",offending.length==0 ? [sourceId] : offending,
						offending.length==0 ? Pass : Fail,
						"Conservative projected ceiling widths compared with the declared limit after same-domain face merging; non-axial curved patches use their full projection.",
						[settings.processFingerprint]));
				} catch (error:Dynamic) {
					var message=Std.string(error);
					if (!StringTools.startsWith(message,"Unsupported fill ceiling surface:")) throw error;
					checks.push(FillAnalysis.check("process.fill.air-escape-width",[sourceId],Unsupported,message,[]));
				}
			}
			var report:FillReport={sourceFingerprint:settings.sourceFingerprint,processFingerprint:settings.processFingerprint,
				model:Brep,criticalHeightsMm:critical,totalMixVolumeMm3:fluid.volume(),
				geometricResinVolumeMm3:source.volume(),openFillAreaMm2:openArea,
				openFillGeometry:CadOpening(fillHeight,[for (blocker in actualBlockers) blocker.id]),
				levels:levels,trappedPockets:pockets,validation:checks};
			for (context in geometries) context.close();
			for (resource in resources) resource.close();
			return report;
		} catch (error:Dynamic) {
			for (context in geometries) context.close();
			for (resource in resources) resource.close();
			throw error;
		}
	}
}
