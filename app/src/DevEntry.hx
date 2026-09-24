package app;

import nativekit.ui.host.DesktopUiHostSession;

/** Short-call entry points for a development host that owns the loaded module. */
class DevEntry {
	static var session:Null<DesktopUiHostSession>;

	public static function main():Void {}

	public static function start(arguments:String):Void {
		if (session != null) throw "Materia development session is already active";
		var values:Array<Dynamic> = haxe.Json.parse(arguments);
		var parsed:Array<String> = [];
		for (value in values) {
			if (!Std.isOfType(value, String)) throw "Live arguments must be strings";
			parsed.push(Std.string(value));
		}
		session = Main.open(parsed);
	}

	public static function tick():Int {
		var current = session;
		if (current == null) return 0;
		if (current.tick()) return 1;
		return 0;
	}

	public static function saveState():String return Main.liveState();

	public static function restoreState(value:String):Void Main.restoreLiveState(value);

	public static function close():Int {
		var current = session;
		session = null;
		Main.clearLiveEditor();
		return current == null ? 0 : current.close();
	}
}
