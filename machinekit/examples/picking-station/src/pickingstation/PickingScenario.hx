package pickingstation;

import machinekit.picking.PickingStation;
import machinekit.picking.StorageRack.StoragePosition;

enum PickingScenarioStatus {
	Idle;
	Running;
	Shortage;
	Complete;
}

enum PickingAction {
	StartOrder;
	SelectBin(positionId:String);
	ConfirmPick;
	ResetFixture;
}

typedef PickingActionResult = {
	var accepted:Bool;
	var message:String;
	var status:PickingScenarioStatus;
	var availableQuantity:Int;
	var pickedQuantity:Int;
}

typedef PickingScenarioState = {
	var status:PickingScenarioStatus;
	var currentSku:Null<String>;
	var requestedQuantity:Int;
	var sourcePositionId:Null<String>;
	var selectedPositionId:Null<String>;
	var highlightedPositionId:Null<String>;
	var illuminatedIndicatorId:Null<String>;
	var completedLines:Int;
	var totalLines:Int;
	var feedback:String;
}

/** Example-only inventory and order state machine. Geometry remains in PickingStation. */
class PickingScenario {
	public final station:PickingStation;
	public final orderLines:Array<PickingOrderLine>;
	public var status(default, null):PickingScenarioStatus = Idle;
	public var feedback(default, null):String = "Fixture loaded. Start the sample order.";
	public var currentLineIndex(default, null):Int = 0;
	public var selectedPositionId(default, null):Null<String> = null;
	final positions:Map<String, StoragePosition> = new Map();
	final inventory:Map<String, BinContents> = new Map();
	final initialQuantities:Map<String, Int> = new Map();
	final pickedByLine:Map<String, Int> = new Map();

	public function new(?station:PickingStation) {
		this.station = station == null ? new PickingStation() : station;
		for (position in this.station.storagePositions()) positions.set(position.id, position);
		var skuIndex = 0;
		var skuNames = ["BEARING-608", "SCREW-M5", "WASHER-M5", "NUT-M5", "SPACER-10"];
		for (position in this.station.storagePositions()) {
			var sku = skuNames[skuIndex % skuNames.length];
			var available = 8 + (skuIndex % 3) * 2;
			inventory.set(position.id, {containerId: "container-" + (skuIndex + 1), sku: sku, availableQuantity: available});
			initialQuantities.set(position.id, available);
			skuIndex++;
		}
		var storage = this.station.storagePositions();
		if (storage.length < 3) throw "Picking example needs at least three storage positions";
		var third = storage.length > 3 ? storage[3] : storage[2];
		orderLines = [
			new PickingOrderLine("order-01", "line-01", storage[0].id, contentsAt(storage[0].id).sku, 2),
			new PickingOrderLine("order-01", "line-02", storage[1].id, contentsAt(storage[1].id).sku, 3),
			new PickingOrderLine("order-01", "line-03", third.id, contentsAt(third.id).sku, 1)
		];
	}

	public function start():PickingActionResult {
		if (status == Complete) return result(false, "The sample order is already complete. Reset to replay it.");
		if (status == Shortage) return result(false, feedback);
		if (status == Running) return result(true, "Order is already running.");
		status = Running;
		selectedPositionId = null;
		feedback = activateTask();
		return result(true, feedback);
	}

	public function selectBin(positionId:String):PickingActionResult {
		if (!positions.exists(positionId)) return result(false, 'Unknown storage position "$positionId".');
		selectedPositionId = positionId;
		if (status != Running) {
			feedback = "Start the sample order before selecting a bin.";
			return result(false, feedback);
		}
		if (positionId != current().sourcePositionId) {
			var sourceId = current().sourcePositionId;
			feedback = 'Wrong bin. Source is $sourceId; quantities are unchanged.';
			return result(false, feedback);
		}
		var requested = current().requestedQuantity, sku = current().sku;
		feedback = 'Correct bin selected. Confirm $requested × $sku.';
		return result(true, feedback);
	}

	public function confirmPick():PickingActionResult {
		if (status == Complete) return result(false, "Order is already complete; this confirmation was ignored.");
		if (status == Shortage) return result(false, feedback);
		if (status != Running) return result(false, "Start the sample order before confirming a pick.");
		if (selectedPositionId == null) return result(false, "Select the highlighted source bin first.");
		if (selectedPositionId != current().sourcePositionId) {
			feedback = "Wrong bin. Nothing was picked.";
			return result(false, feedback);
		}
		var source = contentsAt(current().sourcePositionId);
		if (source.availableQuantity < current().requestedQuantity)
			return result(false, activateTask());
		source.availableQuantity -= current().requestedQuantity;
		pickedByLine.set(current().id, current().requestedQuantity);
		currentLineIndex++;
		selectedPositionId = null;
		if (currentLineIndex >= orderLines.length) {
			status = Complete;
			feedback = "Sample order complete.";
		} else {
			feedback = activateTask();
		}
		return result(true, feedback);
	}

	public function reset():PickingActionResult {
		for (positionId in initialQuantities.keys()) {
			var initial = initialQuantities.get(positionId);
			if (initial == null) throw 'Missing initial quantity for "$positionId"';
			contentsAt(positionId).availableQuantity = initial;
		}
		pickedByLine.clear();
		status = Idle;
		currentLineIndex = 0;
		selectedPositionId = null;
		feedback = "Fixture reset. Start the sample order.";
		return result(true, feedback);
	}

	public function apply(action:PickingAction):PickingActionResult return switch (action) {
		case StartOrder: start();
		case SelectBin(positionId): selectBin(positionId);
		case ConfirmPick: confirmPick();
		case ResetFixture: reset();
	};

	/** Replays discrete actions from the initial fixture and returns a deterministic state key. */
	public function replay(actions:Array<PickingAction>):String {
		reset();
		for (action in actions) apply(action);
		return stateSignature();
	}

	public function currentLine():Null<PickingOrderLine>
		return status == Idle || status == Complete || currentLineIndex >= orderLines.length ?
			null : orderLines[currentLineIndex];

	public function currentAvailableQuantity():Int {
		var line = currentLine();
		return line == null ? 0 : contentsAt(line.sourcePositionId).availableQuantity;
	}

	public function quantityAt(positionId:String):Int {
		return contentsAt(positionId).availableQuantity;
	}

	/** Example fixture override for scripted shortage cases; reset restores the authored stock. */
	public function setAvailableQuantity(positionId:String, quantity:Int):Void {
		if (status != Idle || quantity < 0) throw "Fixture quantities can only be set before starting";
		contentsAt(positionId).availableQuantity = quantity;
	}

	public function containerAt(positionId:String):String return contentsAt(positionId).containerId;

	public function skuAt(positionId:String):String {
		return contentsAt(positionId).sku;
	}

	public function highlightedPositionId():Null<String>
		return status == Running || status == Shortage ? current().sourcePositionId : null;

	public function illuminatedIndicatorId():Null<String> {
		var source = highlightedPositionId();
		if (source == null) return null;
		var position = positions.get(source);
		return position == null ? null : position.indicatorId;
	}

	public function state():PickingScenarioState {
		var line = currentLine();
		return {status: status, currentSku: line == null ? null : line.sku,
			requestedQuantity: line == null ? 0 : line.requestedQuantity,
			sourcePositionId: line == null ? null : line.sourcePositionId,
			selectedPositionId: selectedPositionId, highlightedPositionId: highlightedPositionId(),
			illuminatedIndicatorId: illuminatedIndicatorId(), completedLines: currentLineIndex,
			totalLines: orderLines.length, feedback: feedback};
	}

	public function stateSignature():String {
		var quantities:Array<String> = [];
		for (position in station.storagePositions()) quantities.push(position.id + "=" + quantityAt(position.id));
		var picked:Array<String> = [];
		for (line in orderLines) picked.push(line.id + "=" + (pickedByLine.exists(line.id) ? pickedByLine.get(line.id) : 0));
		return Std.string(status) + "|line=" + currentLineIndex + "|" + quantities.join(",") + "|" + picked.join(",");
	}

	function current():PickingOrderLine return orderLines[currentLineIndex];
	function taskMessage():String {
		var line = current();
		return 'Pick ${line.sku} · quantity ${line.requestedQuantity} from ${line.sourcePositionId}.';
	}

	function activateTask():String {
		var line = current(), available = contentsAt(line.sourcePositionId).availableQuantity;
		if (available < line.requestedQuantity) {
			status = Shortage;
			return 'Shortage: ${line.sku} has $available available; ${line.requestedQuantity} requested.';
		}
		return taskMessage();
	}
	function contentsAt(positionId:String):BinContents {
		var value = inventory.get(positionId);
		if (value == null) throw 'Unknown storage position "$positionId"';
		return value;
	}
	function result(accepted:Bool, message:String):PickingActionResult return {
		accepted: accepted, message: message, status: status,
		availableQuantity: currentAvailableQuantity(), pickedQuantity: totalPickedQuantity()};

	public function totalPickedQuantity():Int {
		var result = 0;
		for (line in orderLines) if (pickedByLine.exists(line.id)) result += pickedByLine.get(line.id);
		return result;
	}
}

class PickingOrderLine {
	public final orderId:String;
	public final id:String;
	public final sourcePositionId:String;
	public final sku:String;
	public final requestedQuantity:Int;

	public function new(orderId:String, id:String, sourcePositionId:String, sku:String, requestedQuantity:Int) {
		if (orderId == null || id == null || sourcePositionId == null || sku == null || requestedQuantity <= 0)
			throw "Picking order lines need identity, source, SKU, and positive quantity";
		this.orderId = orderId;
		this.id = id;
		this.sourcePositionId = sourcePositionId;
		this.sku = sku;
		this.requestedQuantity = requestedQuantity;
	}
}

private typedef BinContents = {
	var containerId:String;
	var sku:String;
	var availableQuantity:Int;
}
