package nativekit.ui.host;

/** Window, pacing, and deterministic-capture policy for DesktopUiHost. */
class DesktopUiHostOptions extends UiHostOptions {
	public var targetFps:Float = 60.0;
	public var captureDirectory:Null<String> = null;
	public var frameLimit:Int = 0;
	/** Capture after this wall-clock interval while preserving normal frame scheduling. */
	public var captureSeconds:Float = 0.0;
	/** True while application state changes without input events, such as live simulation. */
	public var continuousFrames:Null<Void->Bool> = null;
	public var eventHistoryLimit:Int = 100;

	public function new() super();

	public function validate():Void {
		if (title == null || title.length == 0 || width <= 0 || height <= 0 ||
			eventQueueCapacity <= 0 || targetFps <= 0.0 || frameLimit < 0 ||
			captureSeconds < 0.0 || !Math.isFinite(captureSeconds) ||
			eventHistoryLimit < 0)
			throw "Desktop UI host options are invalid";
		if (captureDirectory != null && captureDirectory.length == 0)
			throw "Desktop UI capture directory cannot be empty";
		if (captureDirectory != null && frameLimit == 0 && captureSeconds == 0.0)
			frameLimit = 3;
		if (captureSeconds > 0.0 && (captureDirectory == null || frameLimit > 0))
			throw "Timed desktop captures require a directory and no frame limit";
	}
}
