package nativekit.ui.core;

import haxe.io.Bytes;

/** Length-prefixed key path with deterministic UTF-8 hashing. */
class KeyScope {
	final path:String;

	public function new(path:String = "") {
		this.path = path;
	}

	public function child(key:Key):KeyScope {
		if (key == null)
			throw "A key scope requires a key";
		return new KeyScope(path + key.value.length + ":" + key.value + "|");
	}

	public function widgetId(localKey:String):WidgetId {
		return widgetIdForPath(widgetPath(localKey));
	}

	/** Reuses the exact scoped path for hashing and diagnostic identity. */
	public function widgetPath(localKey:String):String {
		if (localKey == null || localKey.length == 0)
			throw "Local widget keys must not be empty";
		return path + localKey.length + ":" + localKey;
	}

	public static function widgetIdForPath(fullPath:String):WidgetId {
		var bytes = Bytes.ofString(fullPath);
		var hash = -2128831035;
		for (index in 0...bytes.length)
			hash = (hash ^ bytes.get(index)) * 16777619;
		var value = hash & 0x7fffffff;
		return new WidgetId(value == 0 ? 1 : value);
	}

	public inline function pathValue():String
		return path;
}
