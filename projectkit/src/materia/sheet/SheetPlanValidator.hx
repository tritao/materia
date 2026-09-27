package materia.sheet;

import haxe.crypto.Sha256;
import haxe.Json;
import materia.project.MaterialLibrary;
import materia.project.MaterialDef;
import materia.units.LengthUnit;

private typedef Rect = {var x:Float; var y:Float; var width:Float; var height:Float;}

/** Pure rectangular sheet-plan validation and guillotine-cut geometry. All result dimensions
 * are normalized to millimetres. Plan coordinates use a lower-left origin on the full sheet.
 * The numerical acceptance tolerance is 1e-7 mm for coordinates and max(1e-6 mm²,
 * sourceArea × 1e-9) for area balance.
 */
class SheetPlanValidator {
	public static inline var VERSION:Int = 1;
	public static inline var COORDINATE_TOLERANCE_MM:Float = 0.0000001;

	public static function millimetres(value:Float, unit:String):Float
		return value * (LengthUnit.metresPerUnit(unit) * 1000.0);

	public static function stockFingerprint(stock:SheetStockSpec):String {
		if (stock == null) return "";
		return Sha256.encode(Json.stringify({id: stock.id, revision: stock.revision,
			materialId: stock.materialId, lengthUnit: stock.lengthUnit, width: stock.width,
			height: stock.height, thickness: stock.thickness,
			supplier: stock.supplier, supplierCode: stock.supplierCode}));
	}

	public static function requirementFingerprint(requirement:SheetPartRequirement):String {
		if (requirement == null) return "";
		return Sha256.encode(Json.stringify({id: requirement.id, revision: requirement.revision,
			partId: requirement.partId, partRevision: requirement.partRevision,
			quantity: requirement.quantity, materialId: requirement.materialId,
			lengthUnit: requirement.lengthUnit, thickness: requirement.thickness,
			thicknessToleranceMinus: requirement.thicknessToleranceMinus,
			thicknessTolerancePlus: requirement.thicknessTolerancePlus,
			blankWidth: requirement.blankWidth, blankHeight: requirement.blankHeight,
			widthToleranceMinus: requirement.widthToleranceMinus,
			widthTolerancePlus: requirement.widthTolerancePlus,
			heightToleranceMinus: requirement.heightToleranceMinus,
			heightTolerancePlus: requirement.heightTolerancePlus,
			allowedRotations: requirement.allowedRotations,
			directionConstraint: requirement.directionConstraint}));
	}

	public static function captureRequirements(items:Array<SheetPartRequirement>):Array<SheetRequirementReference> {
		return [for (item in items) {id: item.id, revision: item.revision,
			fingerprint: requirementFingerprint(item)}];
	}

	public static function validate(plan:SheetCutPlan, stock:SheetStockSpec,
			requirements:Array<SheetPartRequirement>, ?source:StockPiece,
			?customMaterials:Array<MaterialDef>):SheetPlanValidation {
		var messages:Array<String> = [], regions:Array<SheetCutRegion> = [], cutLines:Array<SheetCutLine> = [];
		var output:SheetPlanValidation = {valid: false, messages: messages, regions: regions,
			cutLines: cutLines,
			kerfLossAreaMm2: 0, marginWasteAreaMm2: 0, discardedAreaMm2: 0,
			retainedAreaMm2: 0, partAreaMm2: 0, sourceAreaMm2: 0,
			areaBalanceErrorMm2: 0};
		if (plan == null) return fail(output, "Cut plan is missing");
		if (stock == null) return fail(output, "Referenced sheet stock specification is missing");
		if (plan.schemaVersion != VERSION || !validText(plan.id) || plan.revision < 1)
			messages.push("Cut plan has an invalid version, ID, or revision");
		if (!validText(stock.id) || stock.revision < 1 || !validMaterial(stock.materialId, customMaterials))
			messages.push("Sheet stock specification has an invalid identity or material");
		var planScale = 1.0, stockScale = 1.0;
		try planScale = LengthUnit.metresPerUnit(plan.lengthUnit) * 1000
		catch (_:Dynamic) messages.push('Unsupported cut-plan length unit "${plan.lengthUnit}"');
		try stockScale = LengthUnit.metresPerUnit(stock.lengthUnit) * 1000
		catch (_:Dynamic) messages.push('Unsupported stock length unit "${stock.lengthUnit}"');
		if (!positiveFinite(stock.width) || !positiveFinite(stock.height) || !positiveFinite(stock.thickness))
			messages.push("Stock dimensions and thickness must be positive and finite");
		if (plan.stockSpecId != stock.id || plan.stockSpecRevision != stock.revision ||
			plan.stockSpecFingerprint != stockFingerprint(stock))
			messages.push("Cut plan is stale: its sheet stock specification changed");
		if (plan.requirements == null || plan.placements == null || plan.cuts == null ||
			plan.retainedRegionIds == null || requirements == null)
			return fail(output, "Cut plan is missing requirements, placements, cuts, or remnant selections");
		if (!positiveOrZeroFinite(plan.kerf) || !positiveOrZeroFinite(plan.edgeMargin))
			messages.push("Kerf and edge margin must be finite and non-negative");

		var requirementsById:Map<String, SheetPartRequirement> = new Map();
		for (requirement in requirements) {
			if (!validRequirement(requirement, messages, customMaterials)) continue;
			if (requirementsById.exists(requirement.id)) messages.push('Duplicate requirement "${requirement.id}"');
			else requirementsById.set(requirement.id, requirement);
		}
		var refsSeen:Map<String, Bool> = new Map();
		if (plan.requirements.length != requirements.length)
			messages.push("Cut plan requirement references do not match the current requirements");
		for (reference in plan.requirements) {
			if (reference == null || !validText(reference.id) || refsSeen.exists(reference.id)) {
				messages.push("Cut plan has a missing or duplicate requirement reference");
				continue;
			}
			refsSeen.set(reference.id, true);
			var requirement = requirementsById.get(reference.id);
			if (requirement == null || reference.revision != requirement.revision ||
				reference.fingerprint != requirementFingerprint(requirement))
				messages.push('Cut plan is stale: requirement "${reference.id}" changed');
		}
		for (requirement in requirements) if (!refsSeen.exists(requirement.id))
			messages.push('Cut plan does not reference requirement "${requirement.id}"');

		var widthMm = stock.width * stockScale, heightMm = stock.height * stockScale;
		var thicknessMm = stock.thickness * stockScale;
		if (source != null) {
			if (source.stockSpecId != stock.id || source.materialId != stock.materialId)
				messages.push("Physical sheet does not match the cut plan stock specification");
			try {
				var sourceScale = LengthUnit.metresPerUnit(source.lengthUnit) * 1000;
				if (!positiveFinite(source.width) || !positiveFinite(source.height) || !positiveFinite(source.thickness))
					messages.push("Physical sheet has invalid measured dimensions");
				widthMm = source.width * sourceScale;
				heightMm = source.height * sourceScale;
				thicknessMm = source.thickness * sourceScale;
			} catch (_:Dynamic) messages.push("Physical sheet has an unsupported length unit");
		}
		output.sourceAreaMm2 = widthMm * heightMm;
		var kerfMm = plan.kerf * planScale, marginMm = plan.edgeMargin * planScale;
		if (!positiveFinite(widthMm) || !positiveFinite(heightMm) ||
			marginMm * 2 >= widthMm || marginMm * 2 >= heightMm) {
			messages.push("Edge margins leave no usable sheet area");
			return finish(output, messages);
		}
		var root:Rect = {x: marginMm, y: marginMm, width: widthMm - 2 * marginMm,
			height: heightMm - 2 * marginMm};
		output.marginWasteAreaMm2 = output.sourceAreaMm2 - root.width * root.height;

		var active:Map<String, Rect> = new Map();
		var allRegionIds:Map<String, Bool> = new Map();
		var cutIds:Map<String, Bool> = new Map();
		active.set("stock", root); allRegionIds.set("stock", true);
		for (cut in plan.cuts) {
			if (cut == null || !validText(cut.id) || !validText(cut.regionId) ||
				!validText(cut.firstRegionId) || !validText(cut.secondRegionId) ||
				(cut.axis != "vertical" && cut.axis != "horizontal") || !positiveFinite(cut.position)) {
				messages.push("Cut sequence contains an invalid cut");
				continue;
			}
			if (cutIds.exists(cut.id)) {
				messages.push('Cut sequence contains duplicate identity "${cut.id}"');
				continue;
			}
			cutIds.set(cut.id, true);
			var parent = active.get(cut.regionId);
			if (parent == null) {
				messages.push('Cut "${cut.id}" refers to a region that is not available at that step');
				continue;
			}
			if (allRegionIds.exists(cut.firstRegionId) || allRegionIds.exists(cut.secondRegionId) ||
				cut.firstRegionId == cut.secondRegionId) {
				messages.push('Cut "${cut.id}" reuses a region identity');
				continue;
			}
			var split = cut.position * planScale;
			var extent = cut.axis == "vertical" ? parent.width : parent.height;
			if (split <= 0 || split + kerfMm >= extent - COORDINATE_TOLERANCE_MM) {
				messages.push('Cut "${cut.id}" and its kerf do not fit inside region "${cut.regionId}"');
				continue;
			}
			var first:Rect, second:Rect, cutLength:Float;
			if (cut.axis == "vertical") {
				first = {x: parent.x, y: parent.y, width: split, height: parent.height};
				second = {x: parent.x + split + kerfMm, y: parent.y,
					width: parent.width - split - kerfMm, height: parent.height};
				cutLength = parent.height;
				cutLines.push({id: cut.id, axis: cut.axis, x1Mm: parent.x + split,
					y1Mm: parent.y, x2Mm: parent.x + split, y2Mm: parent.y + parent.height,
					kerfMm: kerfMm});
			} else {
				first = {x: parent.x, y: parent.y, width: parent.width, height: split};
				second = {x: parent.x, y: parent.y + split + kerfMm,
					width: parent.width, height: parent.height - split - kerfMm};
				cutLength = parent.width;
				cutLines.push({id: cut.id, axis: cut.axis, x1Mm: parent.x,
					y1Mm: parent.y + split, x2Mm: parent.x + parent.width,
					y2Mm: parent.y + split, kerfMm: kerfMm});
			}
			active.remove(cut.regionId);
			active.set(cut.firstRegionId, first); active.set(cut.secondRegionId, second);
			allRegionIds.set(cut.firstRegionId, true); allRegionIds.set(cut.secondRegionId, true);
			output.kerfLossAreaMm2 += kerfMm * cutLength;
		}

		var placementIds:Map<String, Bool> = new Map(), regionPlacements:Map<String, SheetPartPlacement> = new Map();
		var rectangles:Array<{placement:SheetPartPlacement, rect:Rect}> = [];
		for (placement in plan.placements) {
			if (placement == null || !validText(placement.id) || placementIds.exists(placement.id) ||
				!positiveFinite(placement.width) || !positiveFinite(placement.height) ||
				!Math.isFinite(placement.x) || !Math.isFinite(placement.y) || !validText(placement.regionId)) {
				messages.push("Cut plan contains a placement with invalid or duplicate identity or dimensions");
				continue;
			}
			placementIds.set(placement.id, true);
			var requirement = requirementsById.get(placement.requirementId);
			if (requirement == null) {
				messages.push('Placement "${placement.id}" refers to an unknown requirement');
				continue;
			}
			if (placement.rotation != 0 && placement.rotation != 90) {
				messages.push('Placement "${placement.id}" rotation must be 0 or 90 degrees');
				continue;
			}
			if (requirement.allowedRotations.indexOf(placement.rotation) < 0 ||
				(requirement.directionConstraint == "width-aligned" && placement.rotation != 0))
				messages.push('Placement "${placement.id}" uses a rotation that requirement "${requirement.id}" does not allow');
			var reqScale = 1.0;
			try reqScale = LengthUnit.metresPerUnit(requirement.lengthUnit) * 1000
			catch (_:Dynamic) reqScale = 0;
			if (reqScale == 0) continue;
			if (requirement.materialId != stock.materialId)
				messages.push('Requirement "${requirement.id}" material does not match the sheet');
			var nominalThickness = requirement.thickness * reqScale;
			var minThickness = nominalThickness - requirement.thicknessToleranceMinus * reqScale;
			var maxThickness = nominalThickness + requirement.thicknessTolerancePlus * reqScale;
			if (thicknessMm < minThickness - COORDINATE_TOLERANCE_MM ||
				thicknessMm > maxThickness + COORDINATE_TOLERANCE_MM)
				messages.push('Requirement "${requirement.id}" thickness is incompatible with the sheet');
			var expectedWidth = (placement.rotation == 0 ? requirement.blankWidth : requirement.blankHeight) * reqScale;
			var expectedHeight = (placement.rotation == 0 ? requirement.blankHeight : requirement.blankWidth) * reqScale;
			var minWidth = (placement.rotation == 0 ? requirement.blankWidth - requirement.widthToleranceMinus
				: requirement.blankHeight - requirement.heightToleranceMinus) * reqScale;
			var maxWidth = (placement.rotation == 0 ? requirement.blankWidth + requirement.widthTolerancePlus
				: requirement.blankHeight + requirement.heightTolerancePlus) * reqScale;
			var minHeight = (placement.rotation == 0 ? requirement.blankHeight - requirement.heightToleranceMinus
				: requirement.blankWidth - requirement.widthToleranceMinus) * reqScale;
			var maxHeight = (placement.rotation == 0 ? requirement.blankHeight + requirement.heightTolerancePlus
				: requirement.blankWidth + requirement.widthTolerancePlus) * reqScale;
			var placedWidth = placement.width * planScale, placedHeight = placement.height * planScale;
			if (placedWidth < minWidth - COORDINATE_TOLERANCE_MM || placedWidth > maxWidth + COORDINATE_TOLERANCE_MM ||
				placedHeight < minHeight - COORDINATE_TOLERANCE_MM || placedHeight > maxHeight + COORDINATE_TOLERANCE_MM)
				messages.push('Placement "${placement.id}" dimensions are outside requirement "${requirement.id}" tolerances');
			// Keep expected values referenced here so NaN requirements cannot silently validate.
			if (!positiveFinite(expectedWidth) || !positiveFinite(expectedHeight))
				messages.push('Requirement "${requirement.id}" blank dimensions are invalid');
			var rect:Rect = {x: placement.x * planScale, y: placement.y * planScale,
				width: placedWidth, height: placedHeight};
			if (rect.x < root.x - COORDINATE_TOLERANCE_MM || rect.y < root.y - COORDINATE_TOLERANCE_MM ||
				rect.x + rect.width > root.x + root.width + COORDINATE_TOLERANCE_MM ||
				rect.y + rect.height > root.y + root.height + COORDINATE_TOLERANCE_MM)
				messages.push('Placement "${placement.id}" is outside the usable sheet boundaries');
			if (regionPlacements.exists(placement.regionId))
				messages.push('More than one placement uses cut region "${placement.regionId}"');
			else regionPlacements.set(placement.regionId, placement);
			rectangles.push({placement: placement, rect: rect});
			output.partAreaMm2 += rect.width * rect.height;
		}
		for (i in 0...rectangles.length) for (j in (i + 1)...rectangles.length) {
			var a = rectangles[i], b = rectangles[j];
			if (overlaps(a.rect, b.rect)) messages.push('Placements "${a.placement.id}" and "${b.placement.id}" overlap');
		}

		var retainedIds:Map<String, Bool> = new Map();
		for (id in plan.retainedRegionIds) {
			if (!validText(id) || retainedIds.exists(id)) messages.push("Cut plan has a missing or duplicate remnant region");
			else retainedIds.set(id, true);
			if (!active.exists(id)) messages.push('Retained region "$id" is not a leaf after the ordered cuts');
		}
		var leafArea = 0.0, unretainedArea = 0.0;
		for (id in active.keys()) {
			var rect = active.get(id), placement = regionPlacements.get(id), retained = retainedIds.exists(id);
			leafArea += rect.width * rect.height;
			if (placement != null) {
				if (retained) messages.push('Placed part region "$id" cannot also be marked as a remnant');
				if (Math.abs(rect.x - placement.x * planScale) > COORDINATE_TOLERANCE_MM ||
					Math.abs(rect.y - placement.y * planScale) > COORDINATE_TOLERANCE_MM ||
					Math.abs(rect.width - placement.width * planScale) > COORDINATE_TOLERANCE_MM ||
					Math.abs(rect.height - placement.height * planScale) > COORDINATE_TOLERANCE_MM)
					messages.push('Placement "${placement.id}" does not exactly fill cut region "$id"');
			} else if (retained) output.retainedAreaMm2 += rect.width * rect.height;
			else unretainedArea += rect.width * rect.height;
			regions.push({id: id, xMm: rect.x, yMm: rect.y,
				widthMm: rect.width, heightMm: rect.height, retained: retained,
				placementId: placement == null ? null : placement.id});
		}
		for (placement in plan.placements) if (placement != null && !active.exists(placement.regionId))
			messages.push('Placement "${placement.id}" does not refer to a final cut region');
		var actualCounts:Map<String, Int> = new Map();
		for (placement in plan.placements) if (placement != null && validText(placement.requirementId)) {
			var count = actualCounts.get(placement.requirementId);
			actualCounts.set(placement.requirementId, count == null ? 1 : count + 1);
		}
		for (requirement in requirements) if (requirement != null) {
			var actual = actualCounts.get(requirement.id);
			if (actual != requirement.quantity)
				messages.push('Requirement "${requirement.id}" needs ${requirement.quantity} placements; found ${actual == null ? 0 : actual}');
		}
		output.discardedAreaMm2 = output.marginWasteAreaMm2 + unretainedArea;
		output.areaBalanceErrorMm2 = output.sourceAreaMm2 - (output.partAreaMm2 +
			output.retainedAreaMm2 + output.kerfLossAreaMm2 + output.discardedAreaMm2);
		var areaTolerance = Math.max(0.000001, output.sourceAreaMm2 * 0.000000001);
		if (Math.abs(output.areaBalanceErrorMm2) > areaTolerance)
			messages.push('Cut plan area balance is outside tolerance (${output.areaBalanceErrorMm2} mm²)');
		return finish(output, messages);
	}

	static function validRequirement(item:SheetPartRequirement, messages:Array<String>,
			customMaterials:Array<MaterialDef>):Bool {
		if (item == null || !validText(item.id) || item.revision < 1 || !validText(item.partId) ||
			item.partRevision < 1 || item.quantity < 1 || !validMaterial(item.materialId, customMaterials) ||
			item.allowedRotations == null || item.allowedRotations.length == 0 ||
			(item.directionConstraint != "free" && item.directionConstraint != "width-aligned")) {
			messages.push("A part requirement has an invalid identity, quantity, material, or orientation rule");
			return false;
		}
		try LengthUnit.metresPerUnit(item.lengthUnit)
		catch (_:Dynamic) { messages.push('Requirement "${item.id}" uses an unsupported length unit'); return false; }
		for (value in [item.thickness, item.blankWidth, item.blankHeight]) if (!positiveFinite(value)) {
			messages.push('Requirement "${item.id}" dimensions must be positive and finite'); return false;
		}
		for (value in [item.thicknessToleranceMinus, item.thicknessTolerancePlus,
			item.widthToleranceMinus, item.widthTolerancePlus,
			item.heightToleranceMinus, item.heightTolerancePlus]) if (!positiveOrZeroFinite(value)) {
			messages.push('Requirement "${item.id}" tolerances must be finite and non-negative'); return false;
		}
		if (item.thicknessToleranceMinus >= item.thickness ||
			item.widthToleranceMinus >= item.blankWidth ||
			item.heightToleranceMinus >= item.blankHeight) {
			messages.push('Requirement "${item.id}" lower tolerances consume a nominal dimension');
			return false;
		}
		for (rotation in item.allowedRotations) if (rotation != 0 && rotation != 90) {
			messages.push('Requirement "${item.id}" has an unsupported rotation'); return false;
		}
		if (item.allowedRotations.indexOf(0) < 0 && item.allowedRotations.indexOf(90) < 0) {
			messages.push('Requirement "${item.id}" has no supported rotation'); return false;
		}
		if (item.directionConstraint == "width-aligned" && item.allowedRotations.indexOf(0) < 0)
			messages.push('Requirement "${item.id}" width direction conflicts with its allowed rotations');
		return true;
	}

	static function validMaterial(id:String, customMaterials:Array<MaterialDef>):Bool {
		if (!validText(id)) return false;
		try {
			if (MaterialLibrary.get(id) != null) return true;
		} catch (_:Dynamic) {}
		if (customMaterials != null) for (item in customMaterials) if (item.id == id) return true;
		return false;
	}

	static function overlaps(a:Rect, b:Rect):Bool
		return a.x < b.x + b.width - COORDINATE_TOLERANCE_MM &&
			b.x < a.x + a.width - COORDINATE_TOLERANCE_MM &&
			a.y < b.y + b.height - COORDINATE_TOLERANCE_MM &&
			b.y < a.y + a.height - COORDINATE_TOLERANCE_MM;

	static function finish(result:SheetPlanValidation, messages:Array<String>):SheetPlanValidation {
		result.messages = messages;
		result.valid = messages.length == 0;
		return result;
	}

	static function fail(result:SheetPlanValidation, message:String):SheetPlanValidation
		return finish(result, [message]);

	static function validText(value:String):Bool return value != null && StringTools.trim(value).length > 0;
	static function positiveFinite(value:Float):Bool return Math.isFinite(value) && value > 0;
	static function positiveOrZeroFinite(value:Float):Bool return Math.isFinite(value) && value >= 0;
}
