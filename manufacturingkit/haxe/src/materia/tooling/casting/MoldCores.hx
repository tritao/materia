package materia.tooling.casting;

import materia.casting.CastingRegion;
import materia.casting.Box3;
import materia.tooling.casting.CastingMoldPlan.CastingMoldCore;
import materia.tooling.casting.CoreSupport.CoreCutting;

/** Finished foam and gross cavity occupancy are distinct from cutting stock. */
class MoldCores {
	public static function carrierId(core:CastingMoldCore):String {
		if (core.support.kind!=materia.tooling.casting.CoreSupport.CoreSupportKind.Bridge)
			throw "Non-bridge core has no carrier: "+core.id;
		return materia.tooling.ToolingIds.derive("",core.id,"-carrier");
	}

	public static function spreaderIds(core:CastingMoldCore):Array<String> {
		if(core.support.kind!=materia.tooling.casting.CoreSupport.CoreSupportKind.PrecastStruts)
			throw "Only precast struts use dry bearing pads: "+core.id;
		return [for(contact in core.support.precastContacts)if(contact.contact==materia.casting.PrecastStrut.StrutContact.Upper)
			materia.tooling.ToolingIds.derive("precast-pad-",contact.strutId)];
	}

	public static function bridgeId(core:CastingMoldCore):String {
		var pads=core.support.kind==materia.tooling.casting.CoreSupport.CoreSupportKind.PrecastStruts ? spreaderIds(core) : [carrierId(core)];
		var other=core.support.bondedPanelIds.filter(id -> pads.indexOf(id)<0);
		if (other.length!=1) throw "Incomplete bonded core support: "+core.id;
		return other[0];
	}

	public static function receiverId(core:CastingMoldCore):String {
		if ((core.support.kind!=materia.tooling.casting.CoreSupport.CoreSupportKind.RoutedSocket
			&& core.support.kind!=materia.tooling.casting.CoreSupport.CoreSupportKind.RegisteredFlange)
			|| core.support.panelIds.length!=1) throw "Routed core needs one receiving panel: "+core.id;
		return core.support.panelIds[0];
	}

	public static function removal(core:CastingMoldCore):materia.tooling.casting.CastingMoldPlan.CoreRemoval
		return core.support.kind==materia.tooling.casting.CoreSupport.CoreSupportKind.PrecastStruts
			? materia.tooling.casting.CastingMoldPlan.CoreRemoval.LeaveInCasting
			: core.support.kind!=materia.tooling.casting.CoreSupport.CoreSupportKind.Bridge
			? receiverId(core)=="bed" ? materia.tooling.casting.CastingMoldPlan.CoreRemoval.BreakoutThroughBed
				: materia.tooling.casting.CastingMoldPlan.CoreRemoval.BreakoutThroughPanel
			: materia.tooling.casting.CastingMoldPlan.CoreRemoval.BreakoutThroughFill;

	public static function physical(core:CastingMoldCore):CastingRegion {
		var foot=core.support.foot;
		return foot==null ? foam(core) : CastingRegion.Union([foam(core),MoldRegions.box(foot.foam)]);
	}

	/** Gross exclusion includes the foot's two declared glue lines. Otherwise
	 * backing allocation would put a wood shim between the foot and its core.
	 */
	public static function occupied(core:CastingMoldCore):CastingRegion {
		var foot=core.support.foot;
		if(foot==null)return core.cavity;
		var gross=materia.casting.BoxAlgebra.copy(foot.foam),glue=core.support.adhesiveThickness;
		gross.z-=glue;gross.height+=2*glue;
		return CastingRegion.Union([core.cavity,MoldRegions.box(gross)]);
	}

	public static function withdrawal(core:CastingMoldCore,panels:Array<materia.tooling.panel.PanelAssemblyPlan.PanelPlacement>)
			:{x:Float,y:Float,z:Float} {
		if (core.support.kind==materia.tooling.casting.CoreSupport.CoreSupportKind.PrecastStruts)
			throw "Retained EPS has no withdrawal path: "+core.id;
		if (core.support.kind==materia.tooling.casting.CoreSupport.CoreSupportKind.Bridge) return {x:0,y:0,z:1};
		var matches=panels.filter(panel -> panel.panel.id==receiverId(core));
		if (matches.length!=1) throw "Core socket references an unknown panel: "+core.id;
		return materia.assembly.AssemblyFrames.transformVector(CoreSupports.socketFrame(matches[0]),0,0,-1);
	}

	/** The delivered single-pocket cutting sheet keeps its shop layout while the
	 * allocation uses the common region/layer/support records in memory. */
	public static function singlePocketRecord(core:CastingMoldCore,recipe:materia.tooling.ToolingRecipe):Dynamic {
		if (core.layers.length!=1) throw "Single-pocket shop record requires one layer";
		var support=core.support;
		var evidence:Array<String>=[];
		return {id:core.id,featureId:core.featureId,cavity:singleBox(core.cavity,core.id),
			foam:singleBox(core.foamEnvelope,core.id),materialSpec:recipe.core.materialSpec,
			coveringThickness:core.coveringThickness,removal:removal(core),cutting:core.layers[0],
			support:{kind:support.kind,adhesiveSpec:support.adhesiveSpec,adhesiveThickness:support.adhesiveThickness,
				fitClearance:support.fitClearance,embedDepth:support.embedDepth,bondAreaMm2:support.bondAreaMm2,clampBearingAreaMm2:support.clampBearingAreaMm2,
				socketOperationId:support.socketOperationId,registrationProfiles:support.registrationProfiles,foot:support.foot,panelIds:support.panelIds,
				bondedPanelIds:support.bondedPanelIds,hardware:support.hardware,padClosure:support.padClosure},
			restraintEvidence:evidence};
	}

	public static function layerId(core:CastingMoldCore,index:Int):String {
		if (index<0 || index>=core.layers.length) throw "Invalid core layer index: "+core.id;
		return materia.tooling.ToolingIds.derive("",core.id,"-layer-"+index);
	}

	/** All wire-cut stock supplied for a core, including its separate glued foot. */
	public static function cuttingJobs(core:CastingMoldCore):Array<{id:String,cutting:CoreCutting}> {
		var result=[for(index in 0...core.layers.length){id:layerId(core,index),cutting:core.layers[index]}];
		var foot=core.support.foot;
		if(foot!=null)result.push({id:materia.tooling.ToolingIds.derive("",core.id,"-foot"),cutting:foot.cutting});
		return result;
	}

	public static function cuttingJobRecord(core:CastingMoldCore,cutting:CoreCutting):Dynamic
		return {featureId:core.featureId,coreId:core.id,cutting:cutting,
			adhesiveThickness:core.support.adhesiveThickness,coveringThickness:core.coveringThickness};

	/** The certified foot rectangle lies in the first foam band. Intersecting
	 * its footprint with exact stock tiles identifies the actual glued pieces;
	 * stock bounds are used only for this known partition of that rectangle.
	 */
	static function baseLayerIndices(core:CastingMoldCore):Array<Int> {
		var foot=core.support.foot;
		if(foot==null)return [];
		var level=foot.foam.z+foot.foam.height+core.support.adhesiveThickness,indices:Array<Int>=[];
		for(index in 0...core.layers.length){
			var layer=core.layers[index];
			var boxes=MoldRegions.boxes(MoldRegions.moved(MoldRegions.box(layer.block),layer.stockToMold));
			if(boxes==null || boxes.length!=1)throw "Locating foot needs axis-aligned stock tiles: "+core.id;
			var tile=boxes[0];
			if(Math.abs(tile.z-level)<1e-7
				&& Math.min(tile.x+tile.width,foot.foam.x+foot.foam.width)-Math.max(tile.x,foot.foam.x)>1e-7
				&& Math.min(tile.y+tile.depth,foot.foam.y+foot.foam.depth)-Math.max(tile.y,foot.foam.y)>1e-7)indices.push(index);
		}
		if(indices.length==0)throw "Locating foot has no actual stock contact: "+core.id;
		return indices;
	}

	public static function baseAssemblyMembers(core:CastingMoldCore):Array<String>
		return [for(index in baseLayerIndices(core))assemblyLayerId(core,index)];

	/** Keep the logical core ID on its bed-facing or carrier-facing stock piece.
	 * Other pieces keep their cutting IDs. Thus existing core references remain
	 * physical members while in-mold stock has separate checked approaches.
	 */
	public static function assemblyAnchor(core:CastingMoldCore):Int {
		if(core.layers.length==0)throw "Core has no assembly stock: "+core.id;
		if(core.support.foot!=null && core.support.kind==materia.tooling.casting.CoreSupport.CoreSupportKind.RoutedSocket)
			return baseLayerIndices(core)[0];
		var selected=0,value=core.support.kind==materia.tooling.casting.CoreSupport.CoreSupportKind.Bridge
			? Math.NEGATIVE_INFINITY : Math.POSITIVE_INFINITY;
		for(index in 0...core.layers.length){
			var bounds=stockBounds(layerRegion(core.layers[index]),core.id);
			var candidate=core.support.kind==materia.tooling.casting.CoreSupport.CoreSupportKind.Bridge ? bounds.z+bounds.height : bounds.z;
			if(core.support.kind==materia.tooling.casting.CoreSupport.CoreSupportKind.Bridge ? candidate>value : candidate<value){
				selected=index;value=candidate;
			}
		}
		return selected;
	}

	public static function assemblyLayerId(core:CastingMoldCore,index:Int):String
		return index==assemblyAnchor(core) ? core.id : layerId(core,index);

	public static function assemblyMembers(core:CastingMoldCore):Array<String> {
		if(!core.support.assemblyByLayer)return [core.id];
		var ids=[for(index in 0...core.layers.length)assemblyLayerId(core,index)];
		if(core.support.foot!=null)ids.push(materia.tooling.ToolingIds.derive("",core.id,"-foot"));
		return ids;
	}

	/** These top stock pieces really bond to the dry carrier before installation. */
	public static function bridgeAssemblyMembers(core:CastingMoldCore):Array<String> {
		if(!core.support.assemblyByLayer)return [core.id];
		var anchor=stockBounds(layerRegion(core.layers[assemblyAnchor(core)]),core.id),top=anchor.z+anchor.height;
		return [for(index in 0...core.layers.length)if({
			var bounds=stockBounds(layerRegion(core.layers[index]),core.id);
			Math.abs(bounds.z+bounds.height-top)<1e-7;
		})assemblyLayerId(core,index)];
	}

	public static function cuttingRecord(core:CastingMoldCore,index:Int):Dynamic {
		layerId(core,index);
		return cuttingJobRecord(core,core.layers[index]);
	}

	public static function layerRegion(layer:CoreCutting):CastingRegion
		return MoldRegions.moved(layer.solid,layer.stockToMold);

	public static function foam(core:CastingMoldCore):CastingRegion
		return CastingRegion.Union([for (layer in core.layers) layerRegion(layer)]);

	public static function section(region:CastingRegion,z:Float,id:String):Array<materia.casting.CastingSection.CastingSectionContour> {
		var material=MoldRegions.boxes(region);
		if (material!=null) {
			var active=material.filter(box -> z>=box.z-1e-7 && z<=box.z+box.height+1e-7);
			if (active.length<=1) return [for (box in active) {
				depth:0,contour:materia.casting.CastingProfiles.rectangle(box.x,box.y,box.width,box.depth)}];
		}
		var kernel=CastingMoldPlanner.geometry;
		if (kernel==null) throw "Exact core section kernel required: "+id;
		return kernel.section(region,materia.assembly.AssemblyFrames.translation(0,0,z),id);
	}

	public static function area(contours:Array<materia.casting.CastingSection.CastingSectionContour>):Float {
		var result=0.0;
		for (row in contours) result+=(row.depth%2==0 ? 1 : -1)*Math.abs(materia.casting.CastingProfileGeometry.area(row.contour));
		return result;
	}

	/** The carrier follows the finished top footprint, including retained-material
	 * openings. Stock bounds position its drawing; they are never its outline.
	 */
	public static function carrier(core:CastingMoldCore,r:materia.tooling.ToolingRecipe,z:Float)
			:materia.tooling.panel.PanelAssemblyPlan.PanelPlacement {
		try {
			var rows=section(core.foamEnvelope,z-core.support.adhesiveThickness,core.id);
			var outers=rows.filter(row -> row.depth==0);
			if (outers.length!=1 || rows.filter(row -> row.depth>1).length>0)
				throw "Core carrier needs one connected top footprint";
			var bounds=materia.casting.CastingProfileGeometry.bounds(outers[0].contour);
			var width=bounds.width,originX=bounds.x;
			var outline=materia.tooling.panel.PanelContours.placed(outers[0].contour,0,-bounds.x,-bounds.y);
			if(width<materia.tooling.panel.PanelCodec.LABEL_WIDTH_MM){
				// A small square stem needs a wider dry carrier to carry its part
				// mark. This is new board material, not a foam-envelope estimate.
				if(rows.length!=1 || Math.abs(Math.abs(materia.casting.CastingProfileGeometry.area(outers[0].contour))
					-bounds.width*bounds.height)>1e-7)
					throw "Narrow non-rectangular carrier needs a separate label land";
				width=materia.tooling.panel.PanelCodec.LABEL_WIDTH_MM;
				originX-=(width-bounds.width)/2;
				outline=materia.tooling.panel.PanelContours.rectangle(0,0,width,bounds.height);
			}
			var panel=materia.tooling.panel.PanelJoints.blank(MoldCores.carrierId(core),width,bounds.height,
				r.panel,r.routerDiameter,r.minimumLigament);
			panel.operations[0].contour=outline;
			for (index in 0...rows.length) if (rows[index].depth==1) {
				var hole=materia.tooling.panel.PanelContours.placed(rows[index].contour,0,-originX,-bounds.y);
				panel.operations.push(materia.tooling.panel.PanelJoints.operation("core-opening-"+index,
					InsideProfile,hole,r.panel.measuredThickness,r.routerDiameter));
			}
			return {panel:materia.tooling.panel.PanelCodec.copy(panel),
				pose:materia.assembly.AssemblyFrames.translation(originX,bounds.y,z),withdrawal:{x:0,y:0,z:1}};
		} catch (error:Dynamic) {throw "Core carrier "+core.featureId+": "+Std.string(error);}
	}

	/** Layout/blank bounds only: never material or displacement. */
	public static function stockBounds(region:CastingRegion,id:String):Box3 {
		var result=materia.casting.CastingRegionBounds.box(region);
		if (result==null) throw "Empty core stock bounds: "+id;
		return result;
	}

	public static function boxes(region:CastingRegion,id:String):Array<Box3> {
		var result=MoldRegions.boxes(region);
		if (result==null) throw "Unsupported non-box core in orthogonal analysis: "+id;
		return result;
	}

	public static function singleBox(region:CastingRegion,id:String):Box3 {
		var result=boxes(region,id);
		if (result.length!=1) throw "Core needs an exact single-box support: "+id;
		return result[0];
	}
}
