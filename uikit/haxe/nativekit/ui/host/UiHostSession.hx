package nativekit.ui.host;

enum abstract UiHostLifecycle(Int) from Int to Int {
	var Starting = 0;
	var LoadingResources = 1;
	var Running = 2;
	var Stopping = 3;
	var Stopped = 4;
	var Failed = 5;
}

/** Structured host error retained after asynchronous startup or frame failure. */
class UiHostError {
	public final stage:String;
	public final message:String;
	public final cause:Dynamic;

	public function new(stage:String, cause:Dynamic) {
		this.stage = stage;
		this.cause = cause;
		this.message = Std.string(cause);
	}

	public function toString():String return stage + ": " + message;
}

/** Nonblocking handle for a hosted application's complete lifetime. */
class UiHostSession {
	public var state(default, null):UiHostLifecycle = UiHostLifecycle.Starting;
	public var error(default, null):Null<UiHostError> = null;
	final stopCallback:Void->Void;
	var disposed:Bool = false;

	public function new(stopCallback:Void->Void) this.stopCallback = stopCallback;

	@:allow(nativekit.ui.host.UiHostRuntime)
	@:allow(nativekit.ui.host.BrowserUiHost)
	function transition(next:UiHostLifecycle):Void state = next;

	@:allow(nativekit.ui.host.UiHostRuntime)
	@:allow(nativekit.ui.host.BrowserUiHost)
	function fail(stage:String, cause:Dynamic):Void {
		if (error == null) error = new UiHostError(stage, cause);
		state = UiHostLifecycle.Failed;
	}

	public function isActive():Bool return state != UiHostLifecycle.Stopped && state != UiHostLifecycle.Failed;

	/** Stops the hosted application. Browser hosts do not claim to close the tab. */
	public function stop():Void {
		if (disposed || state == UiHostLifecycle.Stopped || state == UiHostLifecycle.Failed) return;
		state = UiHostLifecycle.Stopping;
		stopCallback();
	}

	public function dispose():Void {
		if (disposed) return;
		disposed = true;
		if (state != UiHostLifecycle.Stopped && state != UiHostLifecycle.Failed)
			state = UiHostLifecycle.Stopping;
		stopCallback();
	}
}
