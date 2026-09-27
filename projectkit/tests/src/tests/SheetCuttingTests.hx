package tests;

import materia.sheet.SheetCut;
import materia.sheet.SheetCutExecution;
import materia.sheet.SheetCutPlan;
import materia.sheet.SheetInventory;
import materia.sheet.SheetPartPlacement;
import materia.sheet.SheetPartRequirement;
import materia.sheet.SheetPlanExporter;
import materia.sheet.SheetPlanValidation;
import materia.sheet.SheetPlanValidator;
import materia.sheet.SheetProjectCodec;
import materia.sheet.SheetProjectRecord;
import materia.sheet.SheetRequirementReference;
import materia.sheet.SheetStockSpec;
import materia.project.MaterialDef;
import sys.FileSystem;

class SheetCuttingTests {
	static var assertions = 0;

	static function main():Int {
		try {
			codecUnitsAndAreaBalance();
			invalidPlansAndStaleness();
			inventoryAtomicityAndRemnantReuse();
			Sys.println('Sheet cutting tests passed ($assertions assertions)');
			return 0;
		} catch (error:Dynamic) {
			Sys.stderr().writeString("Sheet cutting tests failed: " + Std.string(error) + "\n");
			return 1;
		}
	}

	static function codecUnitsAndAreaBalance():Void {
		var record = fixture(), roundTrip = SheetProjectCodec.decode(SheetProjectCodec.encode(record));
		check(roundTrip.schemaVersion == 1 && roundTrip.plans.length == 2 &&
			roundTrip.requirements.length == 2, "versioned project records roundtrip");
		near(SheetPlanValidator.millimetres(1, "in"), 25.4, "inch conversion uses shared project units");
		near(SheetPlanValidator.millimetres(10, "cm"), 100, "centimetres convert to millimetres");
		var plan = record.plans[0], stock = record.stockSpecs[0], requirement = record.requirements[0];
		var result = SheetPlanValidator.validate(plan, stock, [requirement]);
		check(result.valid, "hand-authored plan validates after unit conversion: " + result.messages.join("; "));
		near(result.sourceAreaMm2, 10000, "source sheet area is normalized to square millimetres");
		near(result.kerfLossAreaMm2, 240, "all ordered cuts account for 2 mm kerf");
		near(result.marginWasteAreaMm2, 1900, "edge margins are recorded as discarded area");
		near(result.areaBalanceErrorMm2, 0, "parts, remnants, kerf and edge waste balance");
		var right:Null<materia.sheet.SheetCutRegion> = null;
		for (region in result.regions) if (region.id == "right") right = region;
		check(right != null, "selected stock tail is an explicit reusable remnant");
		var retainedRight:materia.sheet.SheetCutRegion = cast right;
		check(retainedRight.retained, "selected stock tail is an explicit reusable remnant");
		near(retainedRight.widthMm, 58, "first remnant width includes kerf geometry");
		near(retainedRight.heightMm, 90, "first remnant height follows the cut tree");
		var csv = SheetPlanExporter.csv(plan, stock, [requirement], result);
		var svg = SheetPlanExporter.svg(plan, stock, [requirement], result);
		check(csv.indexOf("picking-part") >= 0 && csv.indexOf("cut") >= 0,
			"cut-list export includes part and cut identities");
		check(svg.indexOf("30 × 20 mm") >= 0 && svg.indexOf("not a machine toolpath") >= 0,
			"SVG labels dimensions and its planning-drawing status");

		var inchRecord = fixture();
		inchRecord.stockSpecs[0].lengthUnit = "m";
		inchRecord.stockSpecs[0].width = 0.1;
		inchRecord.stockSpecs[0].height = 0.1;
		inchRecord.stockSpecs[0].thickness = 0.018;
		inchRecord.plans[0].stockSpecFingerprint = SheetPlanValidator.stockFingerprint(inchRecord.stockSpecs[0]);
		var inchResult = SheetPlanValidator.validate(inchRecord.plans[0], inchRecord.stockSpecs[0],
			[inchRecord.requirements[0]]);
		check(inchResult.valid, "stock specifications may use a different supported unit");

		var customRecord = fixture();
		customRecord.customMaterials = [{id: "shop-plywood", name: "Shop plywood",
			visual: {baseColor: [0.7, 0.55, 0.34], metallic: 0, roughness: 0.7},
			physical: {density: 650, spec: "custom shop plywood"}}];
		customRecord.stockSpecs[0].materialId = "shop-plywood";
		customRecord.requirements[0].materialId = "shop-plywood";
		customRecord.plans[0].stockSpecFingerprint = SheetPlanValidator.stockFingerprint(customRecord.stockSpecs[0]);
		customRecord.plans[0].requirements = SheetPlanValidator.captureRequirements([customRecord.requirements[0]]);
		var customRoundTrip = SheetProjectCodec.decode(SheetProjectCodec.encode(customRecord));
		var customMaterials:Array<MaterialDef> = cast customRoundTrip.customMaterials;
		var customResult = SheetPlanValidator.validate(customRoundTrip.plans[0], customRoundTrip.stockSpecs[0],
			[customRoundTrip.requirements[0]], null, customRoundTrip.customMaterials);
		check(customResult.valid && customMaterials.length == 1,
			"project-local MaterialDef records survive persistence and resolve requirements");
	}

	static function invalidPlansAndStaleness():Void {
		var record = fixture(), plan = record.plans[0], stock = record.stockSpecs[0], req = record.requirements[0];
		var bad = clone(record);
		bad.requirements[0].materialId = "aluminium";
		bad.plans[0].requirements = SheetPlanValidator.captureRequirements([bad.requirements[0]]);
		var result = SheetPlanValidator.validate(bad.plans[0], bad.stockSpecs[0], [bad.requirements[0]]);
		check(!result.valid && contains(result.messages, "material"), "incompatible sheet and part materials fail");

		bad = clone(record);
		bad.stockSpecs[0].thickness = 1.2;
		bad.plans[0].stockSpecFingerprint = SheetPlanValidator.stockFingerprint(bad.stockSpecs[0]);
		result = SheetPlanValidator.validate(bad.plans[0], bad.stockSpecs[0], [bad.requirements[0]]);
		check(!result.valid && contains(result.messages, "thickness"), "incompatible stock thickness fails");

		bad = clone(record);
		bad.plans[0].placements[0].x = -1;
		result = validateFirst(bad);
		check(!result.valid && contains(result.messages, "outside the usable"), "parts cannot cross the stock boundary");

		bad = clone(record);
		var original = bad.plans[0].placements[0];
		var duplicate:SheetPartPlacement = {id: "overlap-copy", requirementId: original.requirementId,
			x: original.x, y: original.y, width: original.width, height: original.height,
			rotation: original.rotation, regionId: original.regionId};
		bad.plans[0].placements.push(duplicate);
		result = validateFirst(bad);
		check(!result.valid && contains(result.messages, "overlap"), "overlapping placements fail validation");

		bad = clone(record);
		bad.requirements[0].allowedRotations = [0];
		bad.plans[0].requirements = SheetPlanValidator.captureRequirements([bad.requirements[0]]);
		bad.plans[0].placements[0].rotation = 90;
		result = validateFirst(bad);
		check(!result.valid && contains(result.messages, "rotation"), "disallowed part rotation fails validation");

		bad = clone(record);
		bad.plans[0].cuts[1].regionId = "left-lower-before-it-is-cut";
		result = validateFirst(bad);
		check(!result.valid && contains(result.messages, "not available at that step"),
			"a layout needs a supported ordered cut sequence");

		bad = clone(record);
		bad.requirements[0].blankWidth = 3.1;
		result = validateFirst(bad);
		check(!result.valid && contains(result.messages, "stale"),
			"changing part dimensions invalidates the prior requirement fingerprint");
	}

	static function inventoryAtomicityAndRemnantReuse():Void {
		var path = "/tmp/materia-sheet-cut-" + Sys.getPid() + ".json";
		if (FileSystem.exists(path)) FileSystem.deleteFile(path);
		try {
			var inventory = new SheetInventory(fixture(), path);
			inventory.registerSheet("sheet-a", "small-sheet");
			inventory.allocate("sheet-a", "plan-one");
			var savedBeforePreview = SheetProjectCodec.encode(inventory.record);
			var preview = inventory.preview("plan-one", "sheet-a");
			check(preview.valid && SheetProjectCodec.encode(inventory.record) == savedBeforePreview,
				"preview validates without changing inventory");
			var unchanged = SheetProjectCodec.encode(inventory.record);
			expectThrow(function() inventory.execute("plan-one", "sheet-a", "exec-1", false), "confirmation");
			check(SheetProjectCodec.encode(inventory.record) == unchanged,
				"unconfirmed cutting leaves inventory unchanged");
			var first:SheetCutExecution = inventory.execute("plan-one", "sheet-a", "exec-1", true);
			check(first.blanks.length == 1 && first.remnantPieceIds.length == 2,
				"confirmed cut records one blank and two physical remnants");
			check(inventory.piece("sheet-a").state == "consumed", "source sheet is consumed on success");
			var reopened = SheetInventory.open(path);
			check(reopened.record.executions.length == 1 &&
				reopened.piece("sheet-a").state == "consumed", "input and outputs persist together");
			var replay = reopened.execute("plan-one", "sheet-a", "exec-1", true);
			check(replay.id == "exec-1" && reopened.record.executions.length == 1,
				"repeating an execution ID is idempotent");
			expectThrow(function() reopened.execute("plan-two", "sheet-a", "exec-1", true), "already used");

			var remnantId = "exec-1/remnant/right";
			reopened.allocate(remnantId, "plan-two");
			var secondPreview = reopened.preview("plan-two", remnantId);
			check(secondPreview.valid, "recorded remnant fits a second plan");
			var second = reopened.execute("plan-two", remnantId, "exec-2", true);
			check(second.blanks.length == 1 && reopened.piece(remnantId).state == "consumed",
				"remnant can be allocated and consumed by a second plan");
			reopened = SheetInventory.open(path);
			check(reopened.record.executions.length == 2 && reopened.record.stockPieces.length == 5,
				"reopened project retains the complete two-step stock history");
			reopened.allocate("exec-2/remnant/right-b", "plan-two");
			reopened.cancelAllocation("exec-2/remnant/right-b", "plan-two");
			check(reopened.piece("exec-2/remnant/right-b").state == "available",
				"cancelling an allocation releases the remnant");
		} catch (error:Dynamic) {
			if (FileSystem.exists(path)) FileSystem.deleteFile(path);
			throw error;
		}
		if (FileSystem.exists(path)) FileSystem.deleteFile(path);

		var changed = new SheetInventory(fixture(), null);
		changed.registerSheet("sheet-changed", "small-sheet");
		changed.allocate("sheet-changed", "plan-one");
		changed.piece("sheet-changed").width = 9;
		expectThrow(function() changed.execute("plan-one", "sheet-changed", "exec-changed", true), "changed after it was allocated");
		check(changed.record.executions.length == 0 && changed.piece("sheet-changed").state == "allocated",
			"a changed allocated source is rejected without creating outputs");

		var moved = new SheetInventory(fixture(), null);
		moved.registerSheet("sheet-moved", "small-sheet", null, null, null, null, null, "rack-a");
		moved.allocate("sheet-moved", "plan-one");
		moved.piece("sheet-moved").location = "rack-b";
		expectThrow(function() moved.execute("plan-one", "sheet-moved", "exec-moved", true), "changed after it was allocated");
		check(moved.record.executions.length == 0 && moved.piece("sheet-moved").state == "allocated",
			"moving stock after allocation also rejects execution atomically");

		var invalid = new SheetInventory(fixture(), null);
		invalid.registerSheet("sheet-invalid", "small-sheet");
		invalid.allocate("sheet-invalid", "plan-one");
		invalid.plan("plan-one").placements[0].x = -1;
		var before = SheetProjectCodec.encode(invalid.record);
		expectThrow(function() invalid.execute("plan-one", "sheet-invalid", "exec-invalid", true), "failed validation");
		check(SheetProjectCodec.encode(invalid.record) == before,
			"failed validation leaves the complete inventory record unchanged");
	}

	static function validateFirst(record:SheetProjectRecord):SheetPlanValidation
		return SheetPlanValidator.validate(record.plans[0], record.stockSpecs[0], [record.requirements[0]]);

	static function fixture():SheetProjectRecord {
		var stock:SheetStockSpec = {id: "small-sheet", revision: 1, materialId: "plywood-birch",
			lengthUnit: "cm", width: 10, height: 10, thickness: 1.8};
		var first:SheetPartRequirement = {id: "req-a", revision: 1, partId: "picking-part",
			partRevision: 1, quantity: 1, materialId: "plywood-birch", lengthUnit: "cm",
			thickness: 1.8, thicknessToleranceMinus: 0.03, thicknessTolerancePlus: 0.03,
			blankWidth: 3, blankHeight: 2, widthToleranceMinus: 0.05, widthTolerancePlus: 0.05,
		heightToleranceMinus: 0.05, heightTolerancePlus: 0.05,
			allowedRotations: [0, 90], directionConstraint: "free"};
		var second:SheetPartRequirement = {id: "req-b", revision: 1, partId: "remnant-part",
			partRevision: 1, quantity: 1, materialId: "plywood-birch", lengthUnit: "mm",
			thickness: 18, thicknessToleranceMinus: 0.3, thicknessTolerancePlus: 0.3,
			blankWidth: 20, blankHeight: 20, widthToleranceMinus: 0.5, widthTolerancePlus: 0.5,
		heightToleranceMinus: 0.5, heightTolerancePlus: 0.5,
			allowedRotations: [0], directionConstraint: "free"};
		var firstPlan:SheetCutPlan = {schemaVersion: 1, id: "plan-one", revision: 1,
			stockSpecId: stock.id, stockSpecRevision: stock.revision,
			stockSpecFingerprint: SheetPlanValidator.stockFingerprint(stock),
			lengthUnit: "mm", kerf: 2, edgeMargin: 5,
			requirements: SheetPlanValidator.captureRequirements([first]),
			placements: [{id: "blank-a", requirementId: first.id, x: 5, y: 5,
				width: 30, height: 20, rotation: 0, regionId: "part-a"}],
			cuts: [
				{id: "cut-1", regionId: "stock", axis: "vertical", position: 30,
					firstRegionId: "left", secondRegionId: "right"},
				{id: "cut-2", regionId: "left", axis: "horizontal", position: 20,
					firstRegionId: "part-a", secondRegionId: "drop-a"}],
			retainedRegionIds: ["drop-a", "right"]};
		var secondPlan:SheetCutPlan = {schemaVersion: 1, id: "plan-two", revision: 1,
			stockSpecId: stock.id, stockSpecRevision: stock.revision,
			stockSpecFingerprint: SheetPlanValidator.stockFingerprint(stock),
			lengthUnit: "mm", kerf: 2, edgeMargin: 2,
			requirements: SheetPlanValidator.captureRequirements([second]),
			placements: [{id: "blank-b", requirementId: second.id, x: 2, y: 2,
				width: 20, height: 20, rotation: 0, regionId: "part-b"}],
			cuts: [
				{id: "cut-b1", regionId: "stock", axis: "vertical", position: 20,
					firstRegionId: "column", secondRegionId: "right-b"},
				{id: "cut-b2", regionId: "column", axis: "horizontal", position: 20,
					firstRegionId: "part-b", secondRegionId: "drop-b"}],
			retainedRegionIds: ["drop-b", "right-b"]};
		return {schemaVersion: 1, stockSpecs: [stock], requirements: [first, second],
			plans: [firstPlan, secondPlan], stockPieces: [], executions: []};
	}

	static function clone(record:SheetProjectRecord):SheetProjectRecord
		return SheetProjectCodec.decode(SheetProjectCodec.encode(record));
	static function check(value:Bool, message:String):Void {
		assertions++;
		if (!value) throw message;
	}
	static function near(value:Float, expected:Float, message:String):Void
		check(Math.abs(value - expected) < 0.00001, '$message (expected $expected, got $value)');
	static function contains(items:Array<String>, needle:String):Bool {
		for (item in items) if (item.indexOf(needle) >= 0) return true;
		return false;
	}
	static function expectThrow(action:Void->Void, text:String):Void {
		var thrown = false;
		try {
			action();
		} catch (error:Dynamic) {
			thrown = true;
			if (Std.string(error).indexOf(text) < 0) throw 'Expected "$text", got: ' + Std.string(error);
		}
		check(thrown, 'operation throws "$text"');
	}
}
