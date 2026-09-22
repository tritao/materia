package nativekit.ui.host;

/** Window, pacing, and deterministic-capture policy for DesktopUiHost. */
class DesktopUiHostOptions {
	public var title:String = "NativeKit UI";
	public var width:Int = 1280;
	public var height:Int = 800;
	public var targetFps:Float = 60.0;
	public var eventQueueCapacity:Int = 256;
	public var captureDirectory:Null<String> = null;
	public var frameLimit:Int = 0;
	public var eventHistoryLimit:Int = 100;

	public function new() {}

	public function validate():Void {
		if (title == null || title.length == 0 || width <= 0 || height <= 0 ||
			targetFps <= 0.0 || eventQueueCapacity <= 0 || frameLimit < 0 ||
			eventHistoryLimit < 0)
			throw "Desktop UI host options are invalid";
		if (captureDirectory != null && captureDirectory.length == 0)
			throw "Desktop UI capture directory cannot be empty";
		if (captureDirectory != null && frameLimit == 0)
			frameLimit = 3;
	}
}
