package humankit;

/** Ordered actions for one body. A failed action stops the job. */
class HumanJob {
	final actions:Array<HumanAction> = [];
	var worker:Null<HumanBody>;
	var index:Int = 0;
	var started:Bool = false;
	var cancelled:Bool = false;
	var error:Null<String> = null;

	public function new(?worker:HumanBody)
		this.worker = worker;

	public function add(action:HumanAction):HumanJob {
		if (started) throw "Cannot add actions after a job starts";
		actions.push(action);
		return this;
	}

	/** Binds an unstarted job to a simulated body. */
	public function bind(worker:HumanBody):Void {
		if (started) throw "A running job cannot change bodies";
		this.worker = worker;
	}

	public function currentIndex():Int
		return index;

	/** Read-only copy of the ordered actions, for physics bindings. */
	public function orderedActions():Array<HumanAction>
		return actions.copy();

	public function currentAction():Null<HumanAction>
		return index < actions.length ? actions[index] : null;

	public function isDone():Bool
		return cancelled || error != null || index >= actions.length;

	public function failure():Null<String>
		return error;

	/** Stops the job with a failure found outside its actions, such as by a simulation layer. */
	public function abort(reason:String):Void {
		if (isDone()) return;
		error = 'Action $index: $reason';
		if (worker != null) worker.cancel();
	}

	public function cancel():Void {
		cancelled = true;
		if (worker != null) worker.cancel();
	}

	public function advance(seconds:Float):Void {
		if (seconds < 0.0) throw "A job cannot advance backwards";
		if (isDone()) {
			if (worker != null && error == null && !cancelled) worker.advance(seconds);
			return;
		}
		if (worker == null) throw "A job needs a human body before advancing";
		if (!started) {
			started = true;
			startReady();
		}
		if (isDone()) return;
		var current = actions[index];
		current.advance(seconds);
		worker.advance(seconds);
		if (current.failure() != null) {
			error = 'Action $index: ${current.failure()}';
			worker.cancel();
			return;
		}
		if (current.isDone()) {
			index++;
			startReady();
		}
	}

	function startReady():Void {
		while (index < actions.length) {
			var current = actions[index];
			current.start(worker);
			if (current.failure() != null) {
				error = 'Action $index: ${current.failure()}';
				worker.cancel();
				return;
			}
			if (!current.isDone()) return;
			index++;
		}
	}
}
