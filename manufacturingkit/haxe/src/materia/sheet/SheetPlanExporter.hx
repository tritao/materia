package materia.sheet;

/** CSV cut list and SVG planning drawing. Exports require a currently valid cut plan. */
class SheetPlanExporter {
	public static function csv(plan:SheetCutPlan, stock:SheetStockSpec,
			requirements:Array<SheetPartRequirement>, validation:SheetPlanValidation,
			?source:StockPiece):String {
		ensureValid(validation);
		var byId = new Map<String, SheetPartRequirement>();
		for (item in requirements) byId.set(item.id, item);
		var lines:Array<String> = ["record_type,sequence,id,part_id,part_revision,material,thickness_mm,width_mm,height_mm,rotation_deg,x_mm,y_mm,cut_axis,source_piece_id"];
		var sequence = 1;
		for (placement in plan.placements) {
			var found:Null<SheetPartRequirement> = byId.get(placement.requirementId);
			if (found == null) throw 'Missing requirement "${placement.requirementId}" during export';
			var requirement:SheetPartRequirement = cast found;
			lines.push(row(["blank", sequence++, placement.id, requirement.partId,
				requirement.partRevision, requirement.materialId,
				SheetPlanValidator.millimetres(requirement.thickness, requirement.lengthUnit),
				SheetPlanValidator.millimetres(placement.width, plan.lengthUnit),
				SheetPlanValidator.millimetres(placement.height, plan.lengthUnit),
				placement.rotation, SheetPlanValidator.millimetres(placement.x, plan.lengthUnit),
				SheetPlanValidator.millimetres(placement.y, plan.lengthUnit), "", source == null ? "" : source.id]));
		}
		for (cut in plan.cuts) {
			var line:Null<SheetCutLine> = null;
			for (item in validation.cutLines) if (item.id == cut.id) line = item;
			if (line != null) lines.push(row(["cut", sequence++, cut.id, "", "", stock.materialId,
				SheetPlanValidator.millimetres(stock.thickness, stock.lengthUnit), "", "", "",
				line.x1Mm, line.y1Mm, cut.axis, source == null ? "" : source.id]));
		}
		for (region in validation.regions) if (region.retained)
			lines.push(row(["remnant", sequence++, region.id, "", "", stock.materialId,
				SheetPlanValidator.millimetres(source == null ? stock.thickness : source.thickness,
					source == null ? stock.lengthUnit : source.lengthUnit), region.widthMm, region.heightMm,
				"", region.xMm, region.yMm, "", source == null ? "" : source.id]));
		return lines.join("\n") + "\n";
	}

	public static function svg(plan:SheetCutPlan, stock:SheetStockSpec,
			requirements:Array<SheetPartRequirement>, validation:SheetPlanValidation,
			?source:StockPiece):String {
		ensureValid(validation);
		var scale = source == null ? SheetPlanValidator.millimetres(1, stock.lengthUnit)
			: SheetPlanValidator.millimetres(1, source.lengthUnit);
		var width = source == null ? stock.width * scale : source.width * scale;
		var height = source == null ? stock.height * scale : source.height * scale;
		var margin = SheetPlanValidator.millimetres(plan.edgeMargin, plan.lengthUnit);
		var byId = new Map<String, SheetPartRequirement>();
		for (item in requirements) byId.set(item.id, item);
		var out = new StringBuf();
		out.add('<?xml version="1.0" encoding="UTF-8"?>\n');
		out.add('<svg xmlns="http://www.w3.org/2000/svg" width="${fmt(width + 40)}mm" height="${fmt(height + 86)}mm" viewBox="0 0 ${fmt(width + 40)} ${fmt(height + 86)}">\n');
		out.add('<rect width="100%" height="100%" fill="#fff"/>\n');
		out.add('<text x="20" y="24" font-family="sans-serif" font-size="16" font-weight="bold">Planning drawing · ${xml(plan.id)}</text>\n');
		out.add('<text x="20" y="45" font-family="sans-serif" font-size="12">${source == null ? "Input: Nominal stock · " : "Input: Physical sheet " + xml(source.id) + " · "}Stock ${fmt(width)} × ${fmt(height)} mm · ${xml(stock.materialId)} · ${fmt(SheetPlanValidator.millimetres(source == null ? stock.thickness : source.thickness, source == null ? stock.lengthUnit : source.lengthUnit))} mm · Origin: lower left</text>\n');
		out.add('<g transform="translate(20,${fmt(height + 58)}) scale(1,-1)">\n');
		out.add('<rect x="0" y="0" width="${fmt(width)}" height="${fmt(height)}" fill="#e4dfd5" stroke="#253241" stroke-width="2"/>\n');
		out.add('<rect x="${fmt(margin)}" y="${fmt(margin)}" width="${fmt(width - 2 * margin)}" height="${fmt(height - 2 * margin)}" fill="none" stroke="#6f7882" stroke-dasharray="12 8" stroke-width="2"/>\n');
		for (line in validation.cutLines) {
			if (line.axis == "vertical") out.add('<rect x="${fmt(line.x1Mm)}" y="${fmt(line.y1Mm)}" width="${fmt(line.kerfMm)}" height="${fmt(line.y2Mm - line.y1Mm)}" fill="#d97265" fill-opacity="0.65"/>\n');
			else out.add('<rect x="${fmt(line.x1Mm)}" y="${fmt(line.y1Mm)}" width="${fmt(line.x2Mm - line.x1Mm)}" height="${fmt(line.kerfMm)}" fill="#d97265" fill-opacity="0.65"/>\n');
		}
		for (region in validation.regions) if (region.retained) {
			var y = region.yMm;
			out.add('<rect x="${fmt(region.xMm)}" y="${fmt(y)}" width="${fmt(region.widthMm)}" height="${fmt(region.heightMm)}" fill="#bfe4c5" stroke="#26733c" stroke-width="2"/>\n');
			var label = 'Remnant ${region.id} · ${fmt(region.widthMm)} × ${fmt(region.heightMm)}';
			textAt(out, region.xMm + region.widthMm / 2, y + region.heightMm / 2, label);
		}
		for (placement in plan.placements) {
			var x = SheetPlanValidator.millimetres(placement.x, plan.lengthUnit);
			var yBottom = SheetPlanValidator.millimetres(placement.y, plan.lengthUnit);
			var w = SheetPlanValidator.millimetres(placement.width, plan.lengthUnit);
			var h = SheetPlanValidator.millimetres(placement.height, plan.lengthUnit);
			var y = yBottom;
			out.add('<rect x="${fmt(x)}" y="${fmt(y)}" width="${fmt(w)}" height="${fmt(h)}" fill="#bed9ee" stroke="#174c76" stroke-width="2"/>\n');
			var found:Null<SheetPartRequirement> = byId.get(placement.requirementId);
			if (found == null) throw 'Missing requirement "${placement.requirementId}" during export';
			var requirement:SheetPartRequirement = cast found;
			var label = '${requirement.partId} · ${placement.id} · ${fmt(w)} × ${fmt(h)} mm';
			textAt(out, x + w / 2, y + h / 2, label);
		}
		out.add('</g>\n');
		out.add('<text x="20" y="${fmt(height + 76)}" font-family="sans-serif" font-size="11" fill="#8d261b">Planning drawing · kerf-aware layout · not a machine toolpath</text>\n');
		out.add('</svg>\n');
		return out.toString();
	}

	static function textAt(out:StringBuf, x:Float, y:Float, label:String):Void {
		out.add('<text x="${fmt(x)}" y="${fmt(y)}" transform="translate(0,${fmt(2 * y)}) scale(1,-1)" text-anchor="middle" dominant-baseline="middle" font-family="sans-serif" font-size="24" fill="#17202a">${xml(label)}</text>\n');
	}

	static function row(values:Array<Dynamic>):String return [for (value in values) csvValue(value)].join(",");
	static function csvValue(value:Dynamic):String {
		var text = value == null ? "" : Std.string(value);
		if (text.indexOf(",") >= 0 || text.indexOf("\"") >= 0 || text.indexOf("\n") >= 0)
			return '"' + StringTools.replace(text, '"', '""') + '"';
		return text;
	}
	static function xml(value:String):String return StringTools.replace(StringTools.replace(
		StringTools.replace(value, "&", "&amp;"), "<", "&lt;"), "\"", "&quot;");
	static function fmt(value:Float):String return Std.string(Math.round(value * 1000) / 1000);
	static function ensureValid(validation:SheetPlanValidation):Void {
		if (validation == null || !validation.valid)
			throw "Cannot export an invalid sheet cut plan" +
				(validation == null || validation.messages == null ? "" : ": " + validation.messages.join("; "));
	}
}
