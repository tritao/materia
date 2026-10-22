package materia.tooling.casting;

import materia.casting.CastingRegion;
import materia.casting.Box3;
import materia.tooling.casting.CastingMoldPlan.CastingMoldCore;
import materia.tooling.casting.CoreSupport.CoreCutting;

/** Finished foam and gross cavity occupancy are distinct from cutting stock. */
class MoldCores {
	public static function layerId(core:CastingMoldCore,index:Int):String {
		if (index<0 || index>=core.layers.length) throw "Invalid core layer index: "+core.id;
		return materia.tooling.ToolingIds.derive("",core.id,"-layer-"+index);
	}

	public static function cuttingRecord(core:CastingMoldCore,index:Int):Dynamic {
		layerId(core,index);
		return {featureId:core.featureId,coreId:core.id,cutting:core.layers[index],
			adhesiveThickness:core.adhesiveThickness,coveringThickness:core.coveringThickness};
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
			var rows=section(core.foamEnvelope,z-core.adhesiveThickness,core.id);
			var outers=rows.filter(row -> row.depth==0);
			if (outers.length!=1 || rows.filter(row -> row.depth>1).length>0)
				throw "Core carrier needs one connected top footprint";
			var bounds=materia.casting.CastingProfileGeometry.bounds(outers[0].contour);
			var panel=materia.tooling.panel.PanelJoints.blank(core.carrierId,bounds.width,bounds.height,
				r.panel,r.routerDiameter,r.minimumLigament);
			panel.operations[0].contour=materia.tooling.panel.PanelContours.placed(outers[0].contour,0,-bounds.x,-bounds.y);
			for (index in 0...rows.length) if (rows[index].depth==1) {
				var hole=materia.tooling.panel.PanelContours.placed(rows[index].contour,0,-bounds.x,-bounds.y);
				panel.operations.push(materia.tooling.panel.PanelJoints.operation("core-opening-"+index,
					InsideProfile,hole,r.panel.measuredThickness,r.routerDiameter));
			}
			return {panel:materia.tooling.panel.PanelCodec.snapshot(panel),
				pose:materia.assembly.AssemblyFrames.translation(bounds.x,bounds.y,z),withdrawal:{x:0,y:0,z:1}};
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
