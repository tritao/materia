package materia.sheet;

import haxe.crypto.Sha256;
import haxe.Json;
import haxe.io.Path as InventoryPath;
import sys.FileSystem;
import sys.io.AtomicFile;
import sys.io.File;
import materia.sheet.SheetPlanValidator;
import materia.sheet.SheetProjectCodec;

/** Local, atomic inventory workflow for registering, allocating, previewing and executing cuts. */
class SheetInventory {
	public var record(default, null):SheetProjectRecord;
	public var path(default, null):Null<String>;

	public function new(record:SheetProjectRecord, path:Null<String>) {
		this.record = SheetProjectCodec.decode(SheetProjectCodec.encode(record));
		this.path = path == null ? null : absolutePath(path);
	}

	public static function open(path:String):SheetInventory
		return new SheetInventory(SheetProjectCodec.decode(File.getContent(path)), path);

	public function save(?destination:String):Void {
		var target = destination == null ? path : absolutePath(destination);
		if (target == null) throw "Sheet inventory has no save path";
		AtomicFile.write(target, SheetProjectCodec.encode(record));
		path = target;
	}

	public function stockSpec(id:String):SheetStockSpec {
		for (item in record.stockSpecs) if (item.id == id) return item;
		throw 'Unknown sheet stock specification "$id"';
	}

	public function requirement(id:String):SheetPartRequirement {
		for (item in record.requirements) if (item.id == id) return item;
		throw 'Unknown sheet part requirement "$id"';
	}

	public function plan(id:String):SheetCutPlan {
		for (item in record.plans) if (item.id == id) return item;
		throw 'Unknown sheet cut plan "$id"';
	}

	public function piece(id:String):StockPiece {
		for (item in record.stockPieces) if (item.id == id) return item;
		throw 'Unknown physical stock piece "$id"';
	}

	/** Registration changes inventory only after the complete candidate record validates. */
	public function registerSheet(id:String, stockSpecId:String, ?width:Float, ?height:Float,
			?thickness:Float, ?lengthUnit:String, ?batch:String, ?location:String):StockPiece {
		var spec = stockSpec(stockSpecId);
		for (item in record.stockPieces) if (item.id == id) throw 'Duplicate physical stock ID "$id"';
		var created:StockPiece = {id: id, stockSpecId: spec.id, materialId: spec.materialId,
			lengthUnit: lengthUnit == null ? spec.lengthUnit : lengthUnit,
			width: width == null ? spec.width : width, height: height == null ? spec.height : height,
			thickness: thickness == null ? spec.thickness : thickness, state: "available",
			batch: batch, location: location};
		var next = cloneRecord(record);
		next.stockPieces.push(created);
		commit(next);
		return piece(id);
	}

	public function allocate(pieceId:String, planId:String):Void {
		var source = piece(pieceId), targetPlan = plan(planId);
		if (source.stockSpecId != targetPlan.stockSpecId)
			throw 'Stock piece "$pieceId" does not match plan "$planId"';
		if (source.state == "allocated" && source.allocationPlanId == targetPlan.id) return;
		if (source.state != "available") throw 'Stock piece "$pieceId" is not available';
		var next = cloneRecord(record), candidate = findPiece(next, pieceId);
		candidate.state = "allocated";
		candidate.allocationPlanId = targetPlan.id;
		candidate.allocationFingerprint = sourceFingerprint(source);
		commit(next);
	}

	public function cancelAllocation(pieceId:String, planId:String):Void {
		var source = piece(pieceId);
		if (source.state != "allocated" || source.allocationPlanId != planId)
			throw 'Stock piece "$pieceId" is not allocated to plan "$planId"';
		var next = cloneRecord(record), candidate = findPiece(next, pieceId);
		candidate.state = "available";
		candidate.allocationPlanId = null;
		candidate.allocationFingerprint = null;
		commit(next);
	}

	/** Persist a manual layout edit and advance its plan revision. */
	public function editPlan(planId:String, edit:SheetCutPlan->Void):Void {
		var next = cloneRecord(record), target:Null<SheetCutPlan> = null;
		for (item in next.plans) if (item.id == planId) target = item;
		if (target == null) throw 'Unknown sheet cut plan "$planId"';
		var selected:SheetCutPlan = cast target;
		edit(selected);
		selected.revision++;
		commit(next);
	}

	/** Persist changed part requirements. Existing plan fingerprints then report as stale. */
	public function editRequirement(requirementId:String, edit:SheetPartRequirement->Void):Void {
		var next = cloneRecord(record), target:Null<SheetPartRequirement> = null;
		for (item in next.requirements) if (item.id == requirementId) target = item;
		if (target == null) throw 'Unknown sheet part requirement "$requirementId"';
		var selected:SheetPartRequirement = cast target;
		edit(selected);
		selected.revision++;
		commit(next);
	}

	/** Accept current stock and part revisions as the new basis for a cut plan. */
	public function revalidatePlan(planId:String):Void {
		editPlan(planId, function(target) {
			var stock = stockSpec(target.stockSpecId);
			target.stockSpecRevision = stock.revision;
			target.stockSpecFingerprint = SheetPlanValidator.stockFingerprint(stock);
			var current:Array<SheetPartRequirement> = [];
			for (reference in target.requirements) current.push(requirement(reference.id));
			target.requirements = SheetPlanValidator.captureRequirements(current);
		});
	}

	/** Preview is side-effect free and recomputes every material, dimension, and cut check. */
	public function preview(planId:String, ?pieceId:Null<String>):SheetPlanValidation {
		var targetPlan = plan(planId), stock = stockSpec(targetPlan.stockSpecId);
		var requirements = [for (reference in targetPlan.requirements) requirement(reference.id)];
		var source:Null<StockPiece> = pieceId == null || pieceId == "" ? null : piece(pieceId);
		var result = SheetPlanValidator.validate(targetPlan, stock, requirements, source,
			record.customMaterials);
		if (source != null && (source.state == "consumed" ||
			(source.state == "allocated" && source.allocationPlanId != planId))) {
			result.valid = false;
			result.messages.push('Stock piece "${source.id}" is unavailable for this plan');
		}
		return result;
	}

	/** Confirmed execution consumes the allocated source and creates all outputs in one atomic save. */
	public function execute(planId:String, pieceId:String, executionId:String, confirmed:Bool):SheetCutExecution {
		if (!confirmed) throw "Cut completion needs explicit confirmation";
		if (executionId == null || StringTools.trim(executionId).length == 0)
			throw "Cut execution needs a unique ID";
		var targetPlan = plan(planId), source = piece(pieceId);
		var validation = preview(planId, pieceId);
		var currentRequirements = [for (reference in targetPlan.requirements) requirement(reference.id)];
		var fingerprint = executionFingerprint(targetPlan, source, currentRequirements);
		for (existing in record.executions) if (existing.id == executionId) {
			if (existing.planId != planId || existing.consumedPieceId != pieceId ||
				existing.planFingerprint != fingerprint)
				throw 'Execution ID "$executionId" was already used for different cutting details';
			return existing;
		}
		if (source.state != "allocated" || source.allocationPlanId != targetPlan.id)
			throw 'Stock piece "$pieceId" must be allocated to plan "$planId" before execution';
		if (source.allocationFingerprint != sourceFingerprint(source))
			throw 'Stock piece "$pieceId" changed after it was allocated';
		if (!validation.valid) throw "Cut plan failed validation: " + validation.messages.join("; ");

		var next = cloneRecord(record), nextSource = findPiece(next, pieceId);
		nextSource.state = "consumed";
		nextSource.allocationPlanId = null;
		nextSource.allocationFingerprint = null;
		var requirements = new Map<String, SheetPartRequirement>();
		for (item in currentRequirements) requirements.set(item.id, item);
		var blanks:Array<SheetBlankOutput> = [];
		for (placement in targetPlan.placements) {
			var requirement = requirements.get(placement.requirementId);
			if (requirement == null) throw 'Missing requirement "${placement.requirementId}" during execution';
			blanks.push({id: '$executionId/${placement.id}', placementId: placement.id,
				requirementId: requirement.id, partId: requirement.partId,
				partRevision: requirement.partRevision, sourcePieceId: pieceId,
				sourceStockSpecId: source.stockSpecId,
				widthMm: SheetPlanValidator.millimetres(placement.width, targetPlan.lengthUnit),
				heightMm: SheetPlanValidator.millimetres(placement.height, targetPlan.lengthUnit),
				thicknessMm: SheetPlanValidator.millimetres(source.thickness, source.lengthUnit),
				materialId: requirement.materialId});
		}
		var remnantIds:Array<String> = [];
		for (region in validation.regions) if (region.retained) {
			var remnantId = '$executionId/remnant/${region.id}';
			remnantIds.push(remnantId);
			next.stockPieces.push({id: remnantId, stockSpecId: source.stockSpecId,
				materialId: source.materialId, lengthUnit: "mm", width: region.widthMm,
				height: region.heightMm,
				thickness: SheetPlanValidator.millimetres(source.thickness, source.lengthUnit),
				state: "available", location: source.location, parentPieceId: source.id,
				sourceExecutionId: executionId});
		}
		var execution:SheetCutExecution = {id: executionId, planId: targetPlan.id,
			planRevision: targetPlan.revision, planFingerprint: fingerprint,
			consumedPieceId: pieceId, blanks: blanks, remnantPieceIds: remnantIds,
			kerfLossAreaMm2: validation.kerfLossAreaMm2,
			marginWasteAreaMm2: validation.marginWasteAreaMm2,
			discardedAreaMm2: Math.max(0, validation.discardedAreaMm2 - validation.marginWasteAreaMm2),
			completedAtUnixMilliseconds: Date.now().getTime()};
		next.executions.push(execution);
		commit(next);
		return execution;
	}

	function commit(next:SheetProjectRecord):Void {
		SheetProjectCodec.validate(next);
		if (path != null) AtomicFile.write(path, SheetProjectCodec.encode(next));
		record = next;
	}

	static function cloneRecord(value:SheetProjectRecord):SheetProjectRecord
		return SheetProjectCodec.decode(SheetProjectCodec.encode(value));

	static function absolutePath(value:String):String
		return InventoryPath.normalize(InventoryPath.isAbsolute(value) ? value : InventoryPath.join([Sys.getCwd(), value]));

	static function findPiece(value:SheetProjectRecord, id:String):StockPiece {
		for (item in value.stockPieces) if (item.id == id) return item;
		throw 'Unknown physical stock piece "$id"';
	}

	static function executionFingerprint(plan:SheetCutPlan, source:StockPiece,
			requirements:Array<SheetPartRequirement>):String {
	var requirementFingerprints:Array<String> = [];
	for (item in requirements) requirementFingerprints.push(SheetPlanValidator.requirementFingerprint(item));
		var stableSource = sourceFingerprint(source);
		return Sha256.encode(Json.stringify({plan: plan, source: stableSource,
			requirements: requirementFingerprints}));
	}

	static function sourceFingerprint(source:StockPiece):String
		return Sha256.encode(Json.stringify({id: source.id, stockSpecId: source.stockSpecId,
			materialId: source.materialId, lengthUnit: source.lengthUnit, width: source.width,
			height: source.height, thickness: source.thickness, batch: source.batch,
			location: source.location, parentPieceId: source.parentPieceId,
			sourceExecutionId: source.sourceExecutionId}));

}
