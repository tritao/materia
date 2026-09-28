import pickingstation.PickingStation;
import pickingstation.PickingStationConfig;
import pickingstation.PickingScenario;
import pickingstation.PickingScenario.PickingAction;
import pickingstation.PickingScenario.PickingScenarioStatus;
import pickingstation.ShelfAssembly;
import materia.sheet.SheetPlanValidator;

/** Focused deterministic checks for the virtual station example. */
class PickingStationChecks {
	static function check(value:Bool, message:String):Void if (!value) throw message;

	public static function run():Void {
		var config = PickingStationConfig.defaults();
		var station = new PickingStation(config);
		var positions = station.storagePositions(), ids = new Map<String, Bool>();
		check(positions.length == 6, "Default station should expose six positions");
		for (position in positions) {
			check(!ids.exists(position.id), "Storage position IDs must be unique");
			ids.set(position.id, true);
			check(position.indicatorId == position.id + "/indicator", "Indicator reference should be stable");
			check(position.allowedContainerWidth > 0 && position.allowedContainerHeight > 0,
				"Container envelope should fit inside bin");
		}
		check(config.binX(2) - config.binX(1) >= config.binWidth + PickingStationConfig.BIN_GAP,
			"Bins should not overlap");
		check(config.binY() - config.binDepth / 2 >= -config.shelfDepth / 2 +
			ShelfAssembly.RETAINING_LIP_THICKNESS + 2,
			"Bins should clear the retaining lip");
		check(station.instances().length == 20, "Assembly occurrence count should match parts");
		var custom = new PickingStationConfig(240, 300, 180, 3, 3, 3, 300, 400, 1000, 900);
		check(custom.rackWidth > config.rackWidth && custom.shelfWidth > config.shelfWidth,
			"Parameterized width should propagate into the rack and shelf");
		check(new PickingStation(custom).storagePositions().length == 9,
			"Shelf and bin counts should propagate into addressed positions");

		var record = PickingStationDesign.initialRecord();
		check(record.requirements.length == 3 && record.plans.length == 1,
			"Default station should have sheet requirements and one authored plan");
		var validation = SheetPlanValidator.validate(record.plans[0], record.stockSpecs[0], record.requirements);
		check(validation.valid, "Authored sheet plan should validate: " + validation.messages.join("; "));
		var bom = PickingStationDesign.finishedBom();
		for (item in record.requirements) {
			var panel = PickingStationDesign.panelByPartId(item.partId);
			check(bom.quantity(panel.designation) == item.quantity,
				"Finished BOM quantity must match sheet requirement for " + item.id);
		}
		check(ShelfAssembly.RETAINING_LIP_THICKNESS == record.requirements[2].thickness,
			"Retaining lip geometry and blank gauge should match");
		var changedRequirements = PickingStationDesign.requirements(custom);
		var stale = SheetPlanValidator.validate(record.plans[0], record.stockSpecs[0], changedRequirements);
		check(!stale.valid && stale.messages.join(";").indexOf("stale") >= 0,
			"Dimension changes should make the authored plan stale");

		var scenario = new PickingScenario(station);
		var source = scenario.orderLines[0].sourcePositionId, wrong = positions[1].id;
		var initial = scenario.quantityAt(source);
		check(scenario.apply(StartOrder).accepted && scenario.status == Running, "Order should start");
		check(!scenario.apply(SelectBin(wrong)).accepted && scenario.quantityAt(source) == initial,
			"Wrong-bin selection should preserve quantity");
		check(!scenario.apply(ConfirmPick).accepted && scenario.quantityAt(source) == initial,
			"Wrong-bin confirmation should preserve quantity");
		check(scenario.apply(SelectBin(source)).accepted && scenario.apply(ConfirmPick).accepted,
			"Correct bin should confirm one pick");
		check(scenario.quantityAt(source) == initial - 2 && scenario.totalPickedQuantity() == 2,
			"Confirmation should decrement exactly once");
		for (index in 1...scenario.orderLines.length) {
			var line = scenario.orderLines[index];
			check(scenario.apply(SelectBin(line.sourcePositionId)).accepted &&
				scenario.apply(ConfirmPick).accepted, "Remaining order line should complete");
		}
		check(scenario.status == Complete && scenario.totalPickedQuantity() == 6,
			"Sample order should complete with six units");
		var completed = scenario.stateSignature();
		check(!scenario.apply(ConfirmPick).accepted && scenario.stateSignature() == completed,
			"Repeated confirmation should be idempotent");
		var actions:Array<PickingAction> = [StartOrder,
			SelectBin(scenario.orderLines[0].sourcePositionId), ConfirmPick,
			SelectBin(scenario.orderLines[1].sourcePositionId), ConfirmPick,
			SelectBin(scenario.orderLines[2].sourcePositionId), ConfirmPick];
		check(scenario.replay(actions) == completed && scenario.replay(actions) == completed,
			"Replay should be deterministic");
		check(scenario.apply(ResetFixture).accepted && scenario.quantityAt(source) == initial &&
			scenario.totalPickedQuantity() == 0, "Reset should restore authored fixture");
		scenario.setAvailableQuantity(source, 1);
		scenario.apply(StartOrder);
		check(scenario.status == Shortage && scenario.quantityAt(source) == 1 &&
			scenario.totalPickedQuantity() == 0, "Shortage should be clear and leave stock untouched");
		check(scenario.apply(ResetFixture).accepted && scenario.quantityAt(source) == initial,
			"Reset should recover from shortage");
		var request:Dynamic = haxe.Json.parse(haxe.Json.stringify({protocol: "materia.project-ui.v1", actions: [
			{kind: "action", id: "start-order"},
			{kind: "select", id: "project:" + wrong},
			{kind: "action", id: "confirm-pick"}
		]}));
		var projectUi:Dynamic = PickingStationUiExtension.render(request);
		var uiRows:Array<Dynamic> = Reflect.field(Reflect.field(projectUi, "panel"), "rows");
		check(Reflect.field(uiRows[5], "text") == "Wrong bin. Quantities unchanged." &&
			Reflect.field(uiRows[4], "text") == "Progress: 0/3 lines · 0 units",
			"Project UI extension should report wrong-bin feedback without a pick");
		var uiColours:Array<Dynamic> = Reflect.field(projectUi, "colours");
		var green:Float = Reflect.field(uiColours[1], "g");
		check(uiColours.length == positions.length * 2 &&
			green > 0.9,
			"Project UI extension should illuminate the active indicator");
		var completedRequest:Dynamic = haxe.Json.parse(haxe.Json.stringify({protocol: "materia.project-ui.v1", actions: [
			{kind: "action", id: "start-order"},
			{kind: "select", id: "project:" + scenario.orderLines[0].sourcePositionId},
			{kind: "action", id: "confirm-pick"},
			{kind: "select", id: "project:" + scenario.orderLines[1].sourcePositionId},
			{kind: "action", id: "confirm-pick"},
			{kind: "select", id: "project:" + scenario.orderLines[2].sourcePositionId},
			{kind: "action", id: "confirm-pick"},
			{kind: "action", id: "confirm-pick"}
		]}));
		var completedUi:Dynamic = PickingStationUiExtension.render(completedRequest);
		var completedRows:Array<Dynamic> = Reflect.field(Reflect.field(completedUi, "panel"), "rows");
		check(Reflect.field(completedRows[0], "text") == "Status: Complete" &&
			Reflect.field(completedRows[4], "text") == "Progress: 3/3 lines · 6 units" &&
			haxe.Json.stringify(completedUi) == haxe.Json.stringify(PickingStationUiExtension.render(completedRequest)),
			"Project UI extension should complete and replay deterministically");
		var completeActions:Array<Dynamic> = Reflect.field(completedRequest, "actions");
		completeActions.push({kind: "action", id: "reset-fixture"});
		var resetUi:Dynamic = PickingStationUiExtension.render(completedRequest);
		var resetRows:Array<Dynamic> = Reflect.field(Reflect.field(resetUi, "panel"), "rows");
		check(Reflect.field(resetRows[0], "text") == "Status: Idle" &&
			Reflect.field(resetRows[4], "text") == "Progress: 0/3 lines · 0 units",
			"Project UI extension reset should restore the fixture");
		Sys.println("Picking-station checks passed");
	}
}
