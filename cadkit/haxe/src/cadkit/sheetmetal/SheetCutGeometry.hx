package cadkit.sheetmetal;

/** One authored blank profile trims every affected developed domain before folding. */
class SheetCutGeometry {
	public static function apply(source:Sheet, wall:Int, tool:WallOutline, owner:String, ?origin:BoundaryOrigin):Sheet {
		return applyEdits(source, [], [new SheetDevelopedCut(wall, tool, owner, origin)]);
	}
	/** Publish contour changes and all adjoining cuts together, verifying the complete material graph once. */
	public static function applyEdits(source:Sheet, edits:Array<SheetContourEdit>, cuts:Array<SheetDevelopedCut>):Sheet {
		var developed = source.evaluate(source.flat());
		var result = source.copy();
		var seen = new Map<Int, Bool>();
		for (edit in edits) {
			if (seen.exists(edit.wall)) throw new SheetDiagnosis("sheet.duplicate-contour-edit", "One material transaction cannot replace a wall twice", ["wall:" + edit.wall]);
			seen.set(edit.wall, true); var changed = result.updateWallMaterial(edit.wall, edit.outline);
		}
		for (cut in cuts) {
			var plane = developed.wall(cut.wall).plane;
			for (target in result.walls()) {
				var local = SheetProfile.transform(cut.tool, plane, developed.wall(target.id).plane);
				var regions = OutlineBoolean.regions(target.outline, local, cut.owner, cut.origin);
				if (regions.length == 0) throw new SheetDiagnosis("sheet.cut-empty", "Cut removes an entire wall", ["wall:" + target.id]);
				if (regions.length != 1) throw new SheetDiagnosis("sheet.cut-disconnected", "Cut splits a wall into separate regions", ["wall:" + target.id]);
				if (regions[0].area() < target.outline.area() - 1e-9) { var changed = result.updateWallMaterial(target.id, regions[0]); }
			}
			for (bend in result.bends()) {
				var zone = developed.bend(bend.id); var local = SheetProfile.transform(cut.tool, plane, zone.frame.plane);
				var before = bend.materialRegions(zone.allowance); var area = 0.0; var remaining = 0.0;
				for (region in before) {
					area += region.area(); for (part in OutlineBoolean.regions(region, local, cut.owner, cut.origin)) remaining += part.area();
				}
				if (remaining < area - 1e-9) { var changed = result.cutBend(bend.id, local, cut.owner, cut.origin); }
			}
		}
		var verified = result.evaluate(result.folded()); return result;
	}
}
