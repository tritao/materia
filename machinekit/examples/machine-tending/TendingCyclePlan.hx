import machinekit.robotics.DualArmGripperTool;

/** Production operations before CAD frames and certified paths are resolved. */
/**
 * The robot never reports itself clear: the machine's robot zone measures it. The door is asked
 * to close as the arm leaves in the legacy pneumatic sequence. The electric sequence returns
 * to park, clamps with the door open, and waits for full door closure before moving again.
 */
enum TendingOperation {
	Collect(slot:Int, tool:String, fromTool:String, initial:Bool);
	OpenDoor;
	RequestDoorClose;
	DoorClosed;
	Unclamp;
	Clamp;
	/** Places a blank; legacy mode also clamps here. Electric mode uses explicit Clamp after withdrawal. */
	Load(tool:String, fromTool:Null<String>);
	/** Leaves the vise after loading, through the loading route. */
	LeaveVise(tool:String);
	StartCycle;
	CycleReady;
	Unload(slot:Int, tool:String);
	Retreat(tool:String);
	Deliver(slot:Int, tool:String, fromTool:String, returnToStage:Bool);
}

class TendingCyclePlanChecks {
	public static function run():Void {
		var a = DualArmGripperTool.TOOL_CHANNEL_A, b = DualArmGripperTool.TOOL_CHANNEL_B;
		var plan = TendingCyclePlan.bearingBlocks(), requests = plan.transferRequests();
		var closed = [for (request in requests) if (request.role == TendingTransferRole.Inventory) request];
		var expected = [
			{station: "infeed-0-0", tool: a, fromTool: a},
			{station: "infeed-0-1", tool: b, fromTool: a},
			{station: "outfeed-0-0", tool: a, fromTool: b},
			{station: "infeed-0-2", tool: a, fromTool: b},
			{station: "outfeed-0-1", tool: b, fromTool: a},
			{station: "infeed-1-0", tool: b, fromTool: a},
			{station: "outfeed-0-2", tool: a, fromTool: b},
			{station: "outfeed-1-0", tool: b, fromTool: b}
		];
		if (requests.length != 23 || closed.length != expected.length)
			throw "Four-part production needs inventory, tray approach, startup and vise routes";
		for (index in 0...expected.length) {
			var actual = closed[index], fixture = expected[index];
			if (actual.station != fixture.station || actual.tool != fixture.tool || actual.fromTool != fixture.fromTool)
				throw 'Wrong tray inventory transition $index: ${actual.station}/${actual.tool}/${actual.fromTool}';
		}
		var cycles = 0, unloads = 0, deliveries = 0, exchanges = 0;
		for (operation in plan.operations()) switch operation {
			case StartCycle: cycles++;
			case Unload(_, _): unloads++;
			case Deliver(_, _, _, _): deliveries++;
			case Load(_, fromTool): if (fromTool != null) exchanges++;
			case _: {}
		}
		if (cycles != 4 || unloads != 4 || deliveries != 4 || exchanges != 3)
			throw "Production sequence lost a cut, unload, delivery or dual-gripper exchange";
		if (plan.exchangeCount(a, b) != 2 || plan.exchangeCount(b, a) != 1)
			throw "Exchange ranking lost the production frequencies";
		if (plan.retreatsAfterUnload(a) || !plan.retreatsAfterUnload(b))
			throw "Final retreat payload state does not follow the production sequence";
		var closing = false, deliveriesWhileClosing = 0, leavesWhileClosing = 0;
		for (operation in plan.operations()) switch operation {
			case RequestDoorClose:
				if (closing) throw "Door close requests cannot overlap";
				closing = true;
			case LeaveVise(_) | Retreat(_):
				if (!closing) throw "The arm leaves the vise only after asking for the door to close";
				leavesWhileClosing++;
			case DoorClosed:
				if (!closing) throw "Door confirmation needs its owning request";
				closing = false;
			case StartCycle:
				if (closing) throw "A cut cannot start before door-closed confirmation";
			case Deliver(_, _, _, returnToStage):
				if (closing) {
					if (returnToStage) throw "Only final delivery overlaps door closure";
					deliveriesWhileClosing++;
				}
			case _: {}
		}
		if (closing || deliveriesWhileClosing != 1)
			throw "Final delivery overlaps closure and still confirms the closed door";
		if (leavesWhileClosing != 5) throw "Every vise exit overlaps its measured door closure";
		var reordered = new TendingCyclePlan(2, 3, [5, 1]);
		var alternate = [for (request in reordered.transferRequests()) if (request.role == TendingTransferRole.Inventory) request];
		var stations = ["infeed-1-2", "infeed-0-1", "outfeed-1-2", "outfeed-0-1"];
		if (reordered.firstSlot != 5 || alternate.length != stations.length) throw "Reordered job lost its selected slots";
		for (index in 0...stations.length) if (alternate[index].station != stations[index])
			throw "Production routes do not follow the selected row-major slot addresses";
		reject(() -> new TendingCyclePlan(2, 3, []));
		reject(() -> new TendingCyclePlan(2, 3, [1, 1]));
		reject(() -> new TendingCyclePlan(2, 3, [6]));
		var approaches = [for (request in requests) if (request.role == TendingTransferRole.TrayApproach) request];
		var starts = [for (request in requests) if (request.role == TendingTransferRole.Startup) request];
		if (approaches.length != 8 || starts.length != 1 || starts[0].fromPose != "ready" ||
			starts[0].toPose != "infeed-0-0-clearance") throw "Production lost its approach or startup certificates";
		for (index in 0...approaches.length) {
			var route = approaches[index], fixture = expected[index];
			if (route.fromPose != fixture.station + "-clearance" || route.toPose != fixture.station + "-approach" ||
				route.tool != fixture.tool || route.fromTool != fixture.tool || route.door != TendingTransferDoor.Closed)
				throw "Tray approach certificate does not match its actual tool and endpoints";
		}
		trace("Tending production plan passed: 23 requested transfers, 17 closed-door routes");
	}

	static function reject(make:Void->TendingCyclePlan):Void {
		var refused = false;
		try { var rejected = make(); } catch (_:Dynamic) refused = true;
		if (!refused) throw "Invalid tending production was accepted";
	}
}

enum TendingTransferDoor {
	Open;
	Closed;
}
enum TendingTransferRole {
	Inventory;
	TrayApproach;
	Startup;
	ViseEntry;
	Exchange;
}

typedef TendingTransferRequest = {
	var station:String;
	var tool:String;
	var fromTool:String;
	var fromPose:String;
	var toPose:String;
	var door:TendingTransferDoor;
	var role:TendingTransferRole;
}

/** One production sequence owns both mission ordering and the paths it needs. */
class TendingCyclePlan {
	public final electricProcess:Bool;
	public final rows:Int;
	public final columns:Int;
	public final firstSlot:Int;
	final sequence:Array<TendingOperation>;

	public static function bearingBlocks():TendingCyclePlan
		return new TendingCyclePlan(2, 3, [0, 1, 2, 3]);

	public static function electricBearingBlocks():TendingCyclePlan
		return new TendingCyclePlan(2, 3, [0, 1, 2, 3], true);

	public function new(rows:Int, columns:Int, slots:Array<Int>, electricProcess:Bool=false) {
		if (rows < 1 || columns < 1 || slots == null || slots.length == 0)
			throw "Tending production needs a tray grid and at least one infeed slot";
		this.electricProcess = electricProcess;
		this.rows = rows;
		this.columns = columns;
		firstSlot = slots[0];
		var seen = new Map<Int, Bool>();
		for (slot in slots) {
			if (slot < 0 || slot >= rows * columns || seen.exists(slot))
				throw 'Invalid or repeated tending slot $slot';
			seen.set(slot, true);
		}
		if(electricProcess) {
			sequence=[Collect(firstSlot,toolFor(0),toolFor(0),true),OpenDoor,Unclamp,Load(toolFor(0),null),
				LeaveVise(toolFor(0)),Clamp,RequestDoorClose,DoorClosed,StartCycle];
			for(cycle in 0...slots.length) {
				var slot=slots[cycle],tool=toolFor(cycle),hasNext=cycle+1<slots.length;
				var nextTool=hasNext ? toolFor(cycle+1) : tool;
				if(hasNext)sequence.push(Collect(slots[cycle+1],nextTool,tool,false));
				sequence.push(CycleReady);sequence.push(OpenDoor);sequence.push(Unload(slot,tool));
				sequence.push(Retreat(tool)); // Return through the certified common park; no unverified exchange path.
				if(hasNext){sequence.push(Load(nextTool,null));sequence.push(LeaveVise(nextTool));sequence.push(Clamp);}
				sequence.push(RequestDoorClose);sequence.push(DoorClosed);
				if(hasNext)sequence.push(StartCycle);
				sequence.push(Deliver(slot,tool,nextTool,true));
			}
			return;
		}
		sequence = [Collect(firstSlot, toolFor(0), toolFor(0), true), OpenDoor,
			Unclamp, Load(toolFor(0), null), RequestDoorClose, LeaveVise(toolFor(0)), DoorClosed, StartCycle];
		for (cycle in 0...slots.length) {
			var slot = slots[cycle], tool = toolFor(cycle), hasNext = cycle + 1 < slots.length;
			var nextTool = hasNext ? toolFor(cycle + 1) : tool;
			if (hasNext) sequence.push(Collect(slots[cycle + 1], nextTool, tool, false));
			sequence.push(CycleReady);
			sequence.push(OpenDoor);
			sequence.push(Unload(slot, tool));
			if (hasNext) sequence.push(Load(nextTool, tool));
			sequence.push(RequestDoorClose);
			sequence.push(hasNext ? LeaveVise(nextTool) : Retreat(tool));
			if (hasNext) {
				sequence.push(DoorClosed);
				sequence.push(StartCycle);
			}
			sequence.push(Deliver(slot, tool, nextTool, hasNext));
			if (!hasNext) sequence.push(DoorClosed);
		}
	}

	public function operations():Array<TendingOperation> return sequence.copy();

	/** Frequency comes from the authored sequence, rather than a pose-ranking constant. */
	public function exchangeCount(fromTool:String, tool:String):Int {
		var count = 0;
		for (operation in sequence) switch operation {
			case Load(destination, source): if (source == fromTool && destination == tool) count++;
			case _: {}
		}
		return count;
	}

	/** Only the final unload retreats carrying its finish instead of exchanging it. */
	public function retreatsAfterUnload(tool:String):Bool {
		for (operation in sequence) switch operation {
			case Retreat(heldTool): if (heldTool == tool) return true;
			case _: {}
		}
		return false;
	}
	public function rowOf(slot:Int):Int return Std.int(slot / columns);
	public function columnOf(slot:Int):Int return slot % columns;

	static function toolFor(cycle:Int):String return cycle % 2 == 0
		? DualArmGripperTool.TOOL_CHANNEL_A : DualArmGripperTool.TOOL_CHANNEL_B;

	function trayStation(slot:Int, outfeed:Bool):String
		return (outfeed ? "outfeed-" : "infeed-") + rowOf(slot) + "-" + columnOf(slot);
	public static function approachRoute(station:String):String return "approach/" + station;
	public static function startupRoute(station:String):String return "startup/" + station;

	/** Every exported move consumes one of these complete, conservatively checked paths. */
	public function transferRequests():Array<TendingTransferRequest> {
		var result:Array<TendingTransferRequest> = [];
		for (operation in sequence) switch operation {
			case Collect(slot, tool, fromTool, initial):
				var station = trayStation(slot, false);
				add(result, {station: station, tool: tool, fromTool: fromTool, fromPose: "outside staging",
					toPose: station + "-clearance", door: Closed, role: Inventory});
				trayApproach(result, station, tool);
				if (initial) add(result, {station: startupRoute(station), tool: tool, fromTool: tool,
					fromPose: "ready", toPose: station + "-clearance", door: Closed, role: Startup});
			case Load(tool, fromTool):
				// Loading always leaves through this route, including after an exchange.
				vise(result, "vise-load", tool);
				if (fromTool != null) add(result, {station: "exchange", tool: tool, fromTool: fromTool,
					fromPose: "vise-approach", toPose: "vise-load-approach", door: Open, role: Exchange});
			case LeaveVise(tool): vise(result, "vise-load", tool);
			case Unload(_, tool): vise(result, "vise", tool);
			case Retreat(tool): vise(result, "vise", tool);
			case Deliver(slot, tool, fromTool, _):
				var station = trayStation(slot, true);
				add(result, {station: station, tool: tool, fromTool: fromTool, fromPose: "outside staging",
					toPose: station + "-clearance", door: Closed, role: Inventory});
				trayApproach(result, station, tool);
			case _: {}
		}
		return result;
	}

	static function vise(result:Array<TendingTransferRequest>, station:String, tool:String):Void
		add(result, {station: station, tool: tool, fromTool: tool, fromPose: "outside staging",
			toPose: station + "-approach", door: Open, role: ViseEntry});
	static function trayApproach(result:Array<TendingTransferRequest>, station:String, tool:String):Void
		add(result, {station: approachRoute(station), tool: tool, fromTool: tool,
			fromPose: station + "-clearance", toPose: station + "-approach", door: Closed, role: TrayApproach});

	static function add(result:Array<TendingTransferRequest>, request:TendingTransferRequest):Void {
		for (previous in result)
			if (previous.station == request.station && previous.tool == request.tool && previous.fromTool == request.fromTool) {
				if (previous.fromPose != request.fromPose || previous.toPose != request.toPose ||
					previous.door != request.door || previous.role != request.role)
					throw 'Conflicting tending certificate for ${request.station}';
				return;
			}
		result.push(request);
	}
}
