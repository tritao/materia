package cadkit.sheetmetal;

/** One authored blank profile trims every affected developed domain before folding. */
class SheetCutGeometry {
	public static function apply(source:Sheet, wall:Int, tool:WallOutline, owner:String, ?origin:BoundaryOrigin):Sheet {
		var developed = source.evaluate(source.flat()); var plane = developed.wall(wall).plane;
		var result = source.copy();
		for (target in source.walls()) {
			var local = SheetProfile.transform(tool, plane, developed.wall(target.id).plane);
			var regions = OutlineBoolean.regions(target.outline, local, owner, origin);
			if (regions.length == 0) throw new SheetDiagnosis("sheet.cut-empty", "Cut removes an entire wall", ["wall:" + target.id]);
			if (regions.length != 1) throw new SheetDiagnosis("sheet.cut-disconnected", "Cut splits a wall into separate regions", ["wall:" + target.id]);
			if (regions[0].area() < target.outline.area() - 1e-9) { var changed = result.updateWallMaterial(target.id, regions[0]); }
		}
		for (bend in source.bends()) {
			var zone = developed.bend(bend.id); var local = SheetProfile.transform(tool, plane, zone.frame.plane);
			var before = bend.materialRegions(zone.allowance); var area = 0.0; var remaining = 0.0;
			for (region in before) {
				area += region.area(); for (part in OutlineBoolean.regions(region, local, owner, origin)) remaining += part.area();
			}
			if (remaining < area - 1e-9) { var changed = result.cutBend(bend.id, local, owner, origin); }
		}
		var verified = result.evaluate(result.folded()); return result;
	}
}
