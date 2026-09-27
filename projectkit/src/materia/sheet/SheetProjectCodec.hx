package materia.sheet;

import haxe.Json;
import materia.project.MaterialDef;
import materia.project.MaterialLibrary;
import materia.units.LengthUnit;

/** Versioned JSON codec for authored sheet plans and separate physical inventory history. */
class SheetProjectCodec {
	public static inline var VERSION:Int = 1;
	public static inline var FORMAT:String = "materia.sheet-project";

	public static function empty():SheetProjectRecord return {schemaVersion: VERSION,
		stockSpecs: [], requirements: [], plans: [], stockPieces: [], executions: []};

	public static function encode(record:SheetProjectRecord):String {
		validate(record);
		return Json.stringify({format: FORMAT, version: VERSION, project: record}, null, "\t") + "\n";
	}

	public static function decode(text:String):SheetProjectRecord {
		var root:Dynamic;
		try root = Json.parse(text) catch (error:Dynamic)
			throw "Could not parse sheet project: " + Std.string(error);
		if (textField(root, "format") != FORMAT || intField(root, "version") != VERSION)
			throw "Unsupported sheet project format or version";
		var raw = field(root, "project");
		var customMaterials:Array<MaterialDef> = null;
		if (Reflect.hasField(raw, "customMaterials") && Reflect.field(raw, "customMaterials") != null)
			customMaterials = [for (item in arrayField(raw, "customMaterials")) decodeMaterial(item)];
		var record:SheetProjectRecord = {schemaVersion: intField(raw, "schemaVersion"),
			stockSpecs: [for (item in arrayField(raw, "stockSpecs")) decodeStock(item)],
			requirements: [for (item in arrayField(raw, "requirements")) decodeRequirement(item)],
			plans: [for (item in arrayField(raw, "plans")) decodePlan(item)],
			stockPieces: [for (item in arrayField(raw, "stockPieces")) decodePiece(item)],
			executions: [for (item in arrayField(raw, "executions")) decodeExecution(item)]};
		if (customMaterials != null) record.customMaterials = customMaterials;
		validate(record);
		return record;
	}

	static function decodeStock(value:Dynamic):SheetStockSpec {
		var result:SheetStockSpec = {id: textField(value, "id"), revision: intField(value, "revision"),
			materialId: textField(value, "materialId"), lengthUnit: textField(value, "lengthUnit"),
			width: numberField(value, "width"), height: numberField(value, "height"),
			thickness: numberField(value, "thickness")};
		var supplier = optionalText(value, "supplier"), supplierCode = optionalText(value, "supplierCode");
		if (supplier != null) result.supplier = supplier;
		if (supplierCode != null) result.supplierCode = supplierCode;
		return result;
	}

	static function decodeRequirement(value:Dynamic):SheetPartRequirement {
		return {id: textField(value, "id"), revision: intField(value, "revision"),
			partId: textField(value, "partId"), partRevision: intField(value, "partRevision"),
			quantity: intField(value, "quantity"), materialId: textField(value, "materialId"),
			lengthUnit: textField(value, "lengthUnit"), thickness: numberField(value, "thickness"),
			thicknessToleranceMinus: numberField(value, "thicknessToleranceMinus"),
			thicknessTolerancePlus: numberField(value, "thicknessTolerancePlus"),
			blankWidth: numberField(value, "blankWidth"), blankHeight: numberField(value, "blankHeight"),
			widthToleranceMinus: numberField(value, "widthToleranceMinus"),
			widthTolerancePlus: numberField(value, "widthTolerancePlus"),
			heightToleranceMinus: numberField(value, "heightToleranceMinus"),
			heightTolerancePlus: numberField(value, "heightTolerancePlus"),
			allowedRotations: [for (item in arrayField(value, "allowedRotations")) integerValue(item, "allowedRotations")],
			directionConstraint: textField(value, "directionConstraint")};
	}

	static function decodePlan(value:Dynamic):SheetCutPlan {
		return {schemaVersion: intField(value, "schemaVersion"), id: textField(value, "id"),
			revision: intField(value, "revision"), stockSpecId: textField(value, "stockSpecId"),
			stockSpecRevision: intField(value, "stockSpecRevision"),
			stockSpecFingerprint: textField(value, "stockSpecFingerprint"),
			lengthUnit: textField(value, "lengthUnit"), kerf: numberField(value, "kerf"),
			edgeMargin: numberField(value, "edgeMargin"),
			requirements: [for (item in arrayField(value, "requirements"))
				{id: textField(item, "id"), revision: intField(item, "revision"), fingerprint: textField(item, "fingerprint")}],
			placements: [for (item in arrayField(value, "placements"))
				{id: textField(item, "id"), requirementId: textField(item, "requirementId"),
					x: numberField(item, "x"), y: numberField(item, "y"),
					width: numberField(item, "width"), height: numberField(item, "height"),
					rotation: intField(item, "rotation"), regionId: textField(item, "regionId")}],
			cuts: [for (item in arrayField(value, "cuts"))
				{id: textField(item, "id"), regionId: textField(item, "regionId"),
					axis: textField(item, "axis"), position: numberField(item, "position"),
					firstRegionId: textField(item, "firstRegionId"),
					secondRegionId: textField(item, "secondRegionId")}],
			retainedRegionIds: [for (item in arrayField(value, "retainedRegionIds")) textValue(item, "retainedRegionIds")]};
	}

	static function decodePiece(value:Dynamic):StockPiece {
		var result:StockPiece = {id: textField(value, "id"), stockSpecId: textField(value, "stockSpecId"),
			materialId: textField(value, "materialId"), lengthUnit: textField(value, "lengthUnit"),
			width: numberField(value, "width"), height: numberField(value, "height"),
			thickness: numberField(value, "thickness"), state: textField(value, "state")};
		for (name in ["batch", "location", "parentPieceId", "sourceExecutionId", "allocationPlanId", "allocationFingerprint"]) {
			var optional = optionalText(value, name);
			if (optional != null) Reflect.setField(result, name, optional);
		}
		return result;
	}

	static function decodeExecution(value:Dynamic):SheetCutExecution {
		return {id: textField(value, "id"), planId: textField(value, "planId"),
			planRevision: intField(value, "planRevision"),
			planFingerprint: textField(value, "planFingerprint"),
			consumedPieceId: textField(value, "consumedPieceId"),
			blanks: [for (item in arrayField(value, "blanks"))
				{id: textField(item, "id"), placementId: textField(item, "placementId"),
					requirementId: textField(item, "requirementId"), partId: textField(item, "partId"),
					partRevision: intField(item, "partRevision"), sourcePieceId: textField(item, "sourcePieceId"),
					sourceStockSpecId: textField(item, "sourceStockSpecId"),
					widthMm: numberField(item, "widthMm"), heightMm: numberField(item, "heightMm"),
					thicknessMm: numberField(item, "thicknessMm"), materialId: textField(item, "materialId")}],
			remnantPieceIds: [for (item in arrayField(value, "remnantPieceIds")) textValue(item, "remnantPieceIds")],
			kerfLossAreaMm2: numberField(value, "kerfLossAreaMm2"),
			marginWasteAreaMm2: numberField(value, "marginWasteAreaMm2"),
			discardedAreaMm2: numberField(value, "discardedAreaMm2"),
			completedAtUnixMilliseconds: numberField(value, "completedAtUnixMilliseconds")};
	}

	static function decodeMaterial(value:Dynamic):MaterialDef {
		var visual = field(value, "visual"), physical = field(value, "physical");
		var result:MaterialDef = {id: textField(value, "id"), name: textField(value, "name"),
			visual: {baseColor: floatArray(field(visual, "baseColor")),
				metallic: numberField(visual, "metallic"), roughness: numberField(visual, "roughness")},
			physical: {density: numberField(physical, "density"), spec: textField(physical, "spec")}};
		if (Reflect.hasField(visual, "emissive") && Reflect.field(visual, "emissive") != null)
			result.visual.emissive = floatArray(Reflect.field(visual, "emissive"));
		if (Reflect.hasField(visual, "alpha") && Reflect.field(visual, "alpha") != null)
			result.visual.alpha = numberField(visual, "alpha");
		return result;
	}

	public static function validate(record:SheetProjectRecord):Void {
		if (record == null || record.schemaVersion != VERSION || record.stockSpecs == null ||
			record.requirements == null || record.plans == null || record.stockPieces == null ||
			record.executions == null || record.stockSpecs.length > 10000 ||
			record.requirements.length > 100000 || record.plans.length > 10000 ||
			record.stockPieces.length > 1000000 || record.executions.length > 1000000)
			throw "Sheet project has an invalid schema or record count";
		MaterialLibrary.validateCustom(record.customMaterials == null ? [] : record.customMaterials);
		var specs:Map<String, SheetStockSpec> = new Map();
		for (item in record.stockSpecs) {
			if (item == null || !validText(item.id) || item.revision < 1 || specs.exists(item.id) ||
				!validMaterial(item.materialId, record.customMaterials) || !positive(item.width) || !positive(item.height) ||
				!positive(item.thickness)) throw "Sheet project has an invalid or duplicate stock specification";
			checkUnit(item.lengthUnit);
			if (item.supplier != null && !validText(item.supplier)) throw "Invalid sheet supplier reference";
			specs.set(item.id, item);
		}
		var requirements:Map<String, SheetPartRequirement> = new Map();
		for (item in record.requirements) {
			if (item == null || !validText(item.id) || item.revision < 1 || requirements.exists(item.id) ||
				!validText(item.partId) || item.partRevision < 1 || item.quantity < 1 ||
				!validMaterial(item.materialId, record.customMaterials) || !positive(item.thickness) ||
				!positive(item.blankWidth) || !positive(item.blankHeight) ||
				item.allowedRotations == null || item.allowedRotations.length == 0)
				throw "Sheet project has an invalid or duplicate part requirement";
			checkUnit(item.lengthUnit);
			for (value in [item.thicknessToleranceMinus, item.thicknessTolerancePlus,
				item.widthToleranceMinus, item.widthTolerancePlus,
				item.heightToleranceMinus, item.heightTolerancePlus]) if (!nonNegative(value))
				throw "Sheet project has invalid dimensional tolerances";
			if (item.thicknessToleranceMinus >= item.thickness ||
				item.widthToleranceMinus >= item.blankWidth ||
				item.heightToleranceMinus >= item.blankHeight)
				throw "Sheet project has a lower tolerance that consumes a nominal dimension";
			for (rotation in item.allowedRotations) if (rotation != 0 && rotation != 90)
				throw "Sheet project has an unsupported allowed rotation";
			if (item.directionConstraint != "free" && item.directionConstraint != "width-aligned")
				throw "Sheet project has an unsupported direction constraint";
			requirements.set(item.id, item);
		}
		var plans:Map<String, SheetCutPlan> = new Map();
		for (plan in record.plans) {
			if (plan == null || plan.schemaVersion != SheetPlanValidator.VERSION || !validText(plan.id) ||
				plan.revision < 1 || plan.stockSpecRevision < 1 || !validText(plan.stockSpecFingerprint) ||
				plans.exists(plan.id) || !specs.exists(plan.stockSpecId) ||
				plan.requirements == null || plan.placements == null || plan.cuts == null ||
				plan.retainedRegionIds == null || !nonNegative(plan.kerf) || !nonNegative(plan.edgeMargin))
				throw "Sheet project has an invalid or duplicate cut plan";
			checkUnit(plan.lengthUnit);
			var planReferences:Map<String, Bool> = new Map();
			for (reference in plan.requirements) if (reference == null || !requirements.exists(reference.id) ||
				!validText(reference.fingerprint) || reference.revision < 1 || planReferences.exists(reference.id))
				throw "Cut plan has an invalid requirement reference";
			else planReferences.set(reference.id, true);
			plans.set(plan.id, plan);
		}
		var pieces:Map<String, StockPiece> = new Map();
		for (piece in record.stockPieces) {
			if (piece == null || !validText(piece.id) || pieces.exists(piece.id) ||
				!specs.exists(piece.stockSpecId) || !validMaterial(piece.materialId, record.customMaterials) ||
				!positive(piece.width) || !positive(piece.height) || !positive(piece.thickness) ||
				(piece.state != "available" && piece.state != "allocated" && piece.state != "consumed"))
				throw "Sheet project has an invalid or duplicate stock piece";
			checkUnit(piece.lengthUnit);
			if (piece.state == "allocated" && !validText(piece.allocationPlanId))
				throw "Allocated sheet is missing its plan reference";
			if (piece.state == "allocated" && !validText(piece.allocationFingerprint))
				throw "Allocated sheet is missing its physical snapshot";
			if (piece.state != "allocated" &&
				(piece.allocationPlanId != null || piece.allocationFingerprint != null))
				throw "Unavailable sheet contains a stale allocation reference";
			if (piece.parentPieceId != null && !validText(piece.parentPieceId))
				throw "Stock remnant has an invalid parent reference";
			pieces.set(piece.id, piece);
		}
		for (piece in record.stockPieces) if (piece.parentPieceId != null && !pieces.exists(piece.parentPieceId))
			throw "Stock remnant refers to a missing parent piece";
		var executionIds:Map<String, Bool> = new Map();
		for (execution in record.executions) {
			if (execution == null || !validText(execution.id) || executionIds.exists(execution.id) ||
				!plans.exists(execution.planId) || execution.planRevision < 1 ||
				!validText(execution.planFingerprint) || !pieces.exists(execution.consumedPieceId) ||
				execution.blanks == null || execution.remnantPieceIds == null ||
				!nonNegative(execution.kerfLossAreaMm2) || !nonNegative(execution.marginWasteAreaMm2) ||
				!nonNegative(execution.discardedAreaMm2) || !positive(execution.completedAtUnixMilliseconds))
				throw "Sheet project has an invalid or duplicate cut execution";
			executionIds.set(execution.id, true);
			var consumedPiece = pieces.get(execution.consumedPieceId);
			if (consumedPiece == null || consumedPiece.state != "consumed")
				throw "Executed input sheet is not marked consumed";
			for (blank in execution.blanks) if (blank == null || !validText(blank.id) ||
				!validText(blank.placementId) || !requirements.exists(blank.requirementId) ||
				!validText(blank.partId) || blank.partRevision < 1 ||
				blank.sourcePieceId != execution.consumedPieceId ||
				blank.sourceStockSpecId != consumedPiece.stockSpecId ||
				!positive(blank.widthMm) || !positive(blank.heightMm) || !positive(blank.thicknessMm) ||
				!validMaterial(blank.materialId, record.customMaterials)) throw "Sheet project has an invalid produced blank";
			for (pieceId in execution.remnantPieceIds) {
				var remnant = pieces.get(pieceId);
				if (remnant == null || remnant.parentPieceId != execution.consumedPieceId ||
					remnant.sourceExecutionId != execution.id)
					throw "Sheet execution has a missing remnant record";
			}
		}
	}

	static function field(value:Dynamic, name:String):Dynamic {
		if (value == null || !Reflect.hasField(value, name)) throw 'Sheet record is missing "$name"';
		return Reflect.field(value, name);
	}
	static function arrayField(value:Dynamic, name:String):Array<Dynamic> {
		var result = field(value, name);
		if (!Std.isOfType(result, Array)) throw 'Sheet record field "$name" must be an array';
		return cast result;
	}
	static function textField(value:Dynamic, name:String):String {
		return textValue(field(value, name), name);
	}
	static function intField(value:Dynamic, name:String):Int {
		return integerValue(field(value, name), name);
	}
	static function textValue(value:Dynamic, name:String):String {
		if (!Std.isOfType(value, String)) throw 'Sheet record field "$name" must be text';
		return value;
	}
	static function integerValue(value:Dynamic, name:String):Int {
		if (!Std.isOfType(value, Int)) throw 'Sheet record field "$name" must be an integer';
		return value;
	}
	static function numberField(value:Dynamic, name:String):Float {
		var result = field(value, name);
		if (!Std.isOfType(result, Int) && !Std.isOfType(result, Float))
			throw 'Sheet record field "$name" must be a number';
		var number:Float = result;
		if (!Math.isFinite(number)) throw 'Sheet record field "$name" must be finite';
		return number;
	}
	static function optionalText(value:Dynamic, name:String):Null<String> {
		if (!Reflect.hasField(value, name) || Reflect.field(value, name) == null) return null;
		return textValue(Reflect.field(value, name), name);
	}
	static function floatArray(value:Dynamic):Array<Float> {
		if (!Std.isOfType(value, Array)) throw "Sheet material color must be an array";
		var result:Array<Float> = [];
		var raw:Array<Dynamic> = cast value;
		for (item in raw) {
			if (!Std.isOfType(item, Int) && !Std.isOfType(item, Float))
				throw "Sheet material color channels must be numbers";
			var channel:Float = item;
			result.push(channel);
		}
		return result;
	}
	static function checkUnit(unit:String):Void try LengthUnit.metresPerUnit(unit)
		catch (_:Dynamic) throw 'Unsupported sheet length unit "$unit"';
	static function validMaterial(id:String, custom:Array<materia.project.MaterialDef>):Bool {
		if (id == null) return false;
		if (MaterialLibrary.get(id) != null) return true;
		if (custom != null) for (item in custom) if (item.id == id) return true;
		return false;
	}
	static function validText(value:String):Bool return value != null && StringTools.trim(value).length > 0;
	static function positive(value:Float):Bool return Math.isFinite(value) && value > 0;
	static function nonNegative(value:Float):Bool return Math.isFinite(value) && value >= 0;
}
