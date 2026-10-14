package robotkit.tool;

import haxe.Int64;

/** Confirmed stages of one physical tool exchange. */
enum ToolChangePhase {
	Idle;
	UnlockRequested;
	OutgoingUnlocked;
	OutgoingDetached;
	LockRequested;
	Complete;
}

/**
 * Coordinates lock feedback and runtime selection for one changer exchange.
 * The mission owns dock motion and CAD-body attachment; it may advance this
 * transaction only after those physical stages have completed.
 */
class ToolChangeTransaction {
	final selection:ToolRuntimeSelection;
	final lock:ChangerLock;
	/** Commissioned source-clock freshness bound for lock and tool-identity observations. */
	final maximumObservationAgeNs:Int64;
	final tools:Map<String, ToolRuntime> = new Map();
	public var phase(default, null):ToolChangePhase = Idle;
	public var outgoingToolId(default, null):Null<String>;
	public var incomingToolId(default, null):Null<String>;
	var clockId:Null<String>;
	var previousObservationNs:Null<Int64>;
	var previousEventNs:Null<Int64>;

	public function new(selection:ToolRuntimeSelection, runtimes:Array<ToolRuntime>,
		maximumObservationAgeNs:Int64) {
		if (selection == null || runtimes == null || runtimes.length < 2)
			throw "Tool change needs one selection and at least two changer runtimes";
		if (maximumObservationAgeNs == null ||
			Int64.compare(maximumObservationAgeNs, Int64.ofInt(0)) <= 0)
			throw "Tool change needs a positive commissioned lock-observation age limit";
		this.selection = selection;
		this.maximumObservationAgeNs = maximumObservationAgeNs;
		var shared:Null<ChangerLock> = null;
		for (candidate in runtimes) {
			if (candidate == null) throw "Tool change runtimes need a physical changer lock and tool identity";
			var runtime:ToolRuntime = cast candidate;
			var runtimeLock = runtime.changerLock;
			var runtimeId = runtime.changerToolId;
			if (runtimeLock == null || runtimeId == null || runtimeId.length == 0)
				throw "Tool change runtimes need a physical changer lock and tool identity";
			var checkedLock:ChangerLock = cast runtimeLock;
			var checkedId:String = cast runtimeId;
			if (shared == null) shared = checkedLock;
			else if (shared != checkedLock)
				throw "Tool change runtimes must share one physical changer lock";
			if (tools.exists(checkedId))
				throw 'Tool change repeats physical identity "$checkedId"';
			tools.set(checkedId, runtime);
		}
		lock = cast shared;
	}

	/**
	 * Start after the arm has reached the outgoing tool's dock and its process
	 * lease has ended. The selected runtime is safed before unlock is requested.
	 */
	public function begin(targetToolId:String, timestampNs:Int64,
		?sourceClockId:String = "robotkit.monotonic"):Void {
		if (phase != Idle && phase != Complete)
			throw 'Tool change from "$outgoingToolId" to "$incomingToolId" is already in progress';
		if (targetToolId == null || targetToolId.length == 0)
			throw "Tool change needs a non-empty incoming physical tool identity";
		if (sourceClockId == null || sourceClockId.length == 0)
			throw "Tool change needs a source clock identity";
		// A completed exchange may be followed by a new clock domain.
		previousObservationNs = null;
		previousEventNs = null;
		validateEventTime(timestampNs);
		if (selection.activeProcessOwner() != null)
			throw 'Tool change cannot interrupt process "${selection.activeProcessOwner()}"';
		var selected = selection.active();
		if (selected == null) throw "Tool change needs a currently selected changer tool";
		var current:ToolRuntime = cast selected;
		var selectedToolId = current.changerToolId;
		if (selectedToolId == null) throw "Tool change needs a currently selected changer tool";
		var currentToolId:String = cast selectedToolId;
		if (tools.get(currentToolId) != current)
			throw 'Selected physical tool "$currentToolId" is not managed by this changer transaction';
		var incoming = tools.get(targetToolId);
		if (incoming == null) throw 'Tool change requested unknown physical tool "$targetToolId"';
		if (incoming == current) throw 'Tool change already has physical tool "$targetToolId" selected';
		if (current.changerLock != lock || incoming.changerLock != lock)
			throw "Tool change lost the shared physical changer lock";
		var observed = observationAt(timestampNs, sourceClockId);
		previousEventNs = timestampNs;
		if (!observed.locked || observed.toolId != currentToolId)
			throw 'Tool change expected locked tool "$currentToolId"; feedback is ' +
				'${observed.locked ? "locked" : "unlocked"} with identity "${observed.toolId}"';
		var unsafe = current.unsafeChannels();
		if (unsafe.length != 0)
			throw 'Tool change cannot begin with unsafe outgoing channels: ${unsafe.join(", ")}';

		selection.select(null, timestampNs, sourceClockId);
		outgoingToolId = currentToolId;
		incomingToolId = targetToolId;
		clockId = sourceClockId;
		phase = UnlockRequested;
		requestUnlock(timestampNs);
	}

	/** Retry the unlock command after an I/O interruption without replaying dock motion. */
	public function requestUnlock(timestampNs:Int64):Void {
		require(UnlockRequested, "unlock request");
		var observed = currentObservation(timestampNs);
		if (observed.toolId != outgoingToolId)
			throw 'Tool change cannot unlock with measured tool "${observed.toolId}"; expected "$outgoingToolId"';
		if (observed.locked) lock.unlock(timestampNs);
	}

	/** Accept measured unlocked feedback while the outgoing tool identity is still present. */
	public function confirmUnlocked(timestampNs:Int64):Void {
		require(UnlockRequested, "unlocked confirmation");
		var observed = currentObservation(timestampNs);
		if (observed.locked || observed.toolId != outgoingToolId)
			throw 'Tool change expected unlocked outgoing tool "$outgoingToolId" before detachment';
		phase = OutgoingUnlocked;
	}

	/** Accept detachment only after the outgoing tool has been placed at its dock. */
	public function confirmDetached(timestampNs:Int64):Void {
		require(OutgoingUnlocked, "detachment confirmation");
		var observed = currentObservation(timestampNs);
		if (observed.locked || observed.toolId != null)
			throw 'Tool change expected an empty unlocked changer after removing "$outgoingToolId"';
		phase = OutgoingDetached;
	}

	/** Request locking only after the incoming CAD tool has been seated at the changer. */
	public function confirmAttached(timestampNs:Int64):Void {
		require(OutgoingDetached, "attachment confirmation");
		var target = incomingToolId;
		if (target == null) throw "Tool change lost its incoming physical identity";
		var observed = currentObservation(timestampNs);
		if (observed.locked || observed.toolId != target)
			throw 'Tool change expected unlocked incoming tool "$target" before locking';
		phase = LockRequested;
		requestLock(timestampNs);
	}

	/** Retry the lock command after an I/O interruption without replaying dock motion. */
	public function requestLock(timestampNs:Int64):Void {
		require(LockRequested, "lock request");
		var target = incomingToolId;
		if (target == null) throw "Tool change lost its incoming physical identity";
		var observed = currentObservation(timestampNs);
		if (observed.toolId != target)
			throw 'Tool change cannot lock measured tool "${observed.toolId}"; expected "$target"';
		if (!observed.locked) lock.lock(timestampNs);
	}

	/** Select the new TCP and capabilities only after measured lock and identity agree. */
	public function confirmLocked(timestampNs:Int64):Void {
		require(LockRequested, "lock confirmation");
		var observed = currentObservation(timestampNs);
		var target = incomingToolId;
		if (target == null || !observed.locked || observed.toolId != target)
			throw 'Tool change expected locked incoming tool "$target" before runtime selection';
		var runtime = tools.get(target);
		if (runtime == null) throw 'Tool change lost runtime for physical tool "$target"';
		var selectedClock = clockId;
		if (selectedClock == null) throw "Tool change lost its source clock identity";
		selection.select(runtime, timestampNs, selectedClock);
		phase = Complete;
	}

	function currentObservation(timestampNs:Int64):ChangerLockObservation {
		var currentClock = clockId;
		if (currentClock == null) throw "Tool change has no active source clock";
		validateEventTime(timestampNs);
		var observed = observationAt(timestampNs, currentClock);
		previousEventNs = timestampNs;
		return observed;
	}

	function validateEventTime(timestampNs:Int64):Void {
		if (Int64.compare(timestampNs, Int64.ofInt(0)) < 0)
			throw "Tool change event time must be non-negative";
		var previous = previousEventNs;
		if (previous != null && Int64.compare(timestampNs, cast(previous, Int64)) < 0)
			throw "Tool change event time moved backwards";
	}

	function observationAt(timestampNs:Int64, expectedClock:String):ChangerLockObservation {
		if (Int64.compare(timestampNs, Int64.ofInt(0)) < 0)
			throw "Tool change event time must be non-negative";
		var observed = lock.observation();
		if (observed == null) throw "Changer returned no lock/tool observation";
		if (observed.sourceClockId == null || observed.sourceClockId.length == 0)
			throw "Changer feedback has no source clock identity";
		if (observed.sourceClockId != expectedClock)
			throw 'Changer feedback clock "${observed.sourceClockId}" does not match tool-change clock "$expectedClock"';
		if (observed.toolId != null && observed.toolId.length == 0)
			throw "Changer feedback has an empty tool identity";
		if (Int64.compare(observed.sourceTimestampNs, Int64.ofInt(0)) < 0)
			throw "Changer feedback time must be non-negative";
		if (Int64.compare(observed.sourceTimestampNs, timestampNs) > 0)
			throw "Changer feedback is newer than the tool-change event";
		if (Int64.compare(Int64.sub(timestampNs, observed.sourceTimestampNs), maximumObservationAgeNs) > 0)
			throw "Changer lock/tool identity feedback is stale";
		var previous = previousObservationNs;
		if (previous != null && Int64.compare(observed.sourceTimestampNs, cast(previous, Int64)) < 0)
			throw "Changer feedback time moved backwards during tool change";
		previousObservationNs = observed.sourceTimestampNs;
		return observed;
	}

	function require(expected:ToolChangePhase, operation:String):Void {
		if (phase != expected)
			throw 'Tool change cannot accept $operation during phase "$phase"';
	}
}
