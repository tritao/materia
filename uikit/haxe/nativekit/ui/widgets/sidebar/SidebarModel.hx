package nativekit.ui.widgets.sidebar;

import nativekit.ui.core.View;

/** View-independent sidebar registration and validated persistent presentation. */
class SidebarModel {
	public final modes:Array<SidebarMode> = [];
	public var activeId(default, null):String = "";
	public var visible(default, null):Bool = true;
	public var onChange:Void->Void = function() {};
	var savedActive:String = "";
	var sequence:Int = 0;
	static final widthPattern = ~/^[0-9]+(\.[0-9]+)?([eE][+-]?[0-9]+)?$/;
	var savedWidths:Map<String, Float> = [];
	var savedVisible:Map<String, Bool> = [];
	public function new() {}

	public function register(id:String, provider:Void->View, options:SidebarModeOptions):Void {
		if (!validId(id) || provider == null || options == null || options.label.length == 0 ||
			!validWidth(options.width) || find(id) != null) throw "Invalid or duplicate sidebar mode";
		var mode = new SidebarMode(id, provider, options, sequence++);
		if (savedWidths.exists(id)) mode.width = savedWidths.get(id);
		if (savedVisible.exists(id)) mode.visible = savedVisible.get(id);
		modes.push(mode);
		modes.sort(function(a, b) return a.order == b.order ? a.sequence - b.sequence : a.order - b.order);
		if (mode.visible && (activeId == "" || savedActive == id)) activeId = id;
		onChange();
	}

	public function unregister(id:String):Bool {
		var mode = find(id);
		if (mode == null) return false;
		savedWidths.set(id, mode.width); savedVisible.set(id, mode.visible);
		modes.remove(mode);
		var active = selected(); activeId = active == null ? "" : active.id; savedActive = activeId;
		onChange(); return true;
	}
	public function find(id:String):Null<SidebarMode> {
		for (mode in modes) if (mode.id == id) return mode;
		return null;
	}
	public function selected():Null<SidebarMode> {
		var active = find(activeId);
		if (active != null && active.visible) return active;
		for (mode in modes) if (mode.visible) return mode;
		return null;
	}
	public function select(id:String):Bool {
		var mode = find(id);
		if (mode == null || !mode.visible) return false;
		activeId = id; savedActive = id; visible = true; onChange(); return true;
	}
	public function setVisible(value:Bool):Void {
		if (visible == value) return;
		visible = value; onChange();
	}
	public function rememberWidth(id:String, width:Float):Void {
		var mode = find(id);
		if (mode != null && validWidth(width)) { mode.width = width; savedWidths.set(id, width); }
	}
	public function setModeVisible(id:String, value:Bool):Bool {
		var mode = find(id);
		if (mode == null) return false;
		mode.visible = value; savedVisible.set(id, value);
		var selected = selected(); activeId = selected == null ? "" : selected.id; savedActive = activeId;
		onChange(); return true;
	}
	public function encode():String {
		var values:Array<String> = [];
		for (mode in modes) values.push(mode.id + "," + mode.width + "," + (mode.visible ? "1" : "0"));
		for (id in savedWidths.keys()) if (find(id) == null)
			values.push(id + "," + savedWidths.get(id) + "," + (savedVisible.get(id) == false ? "0" : "1"));
		values.sort(Reflect.compare);
		return "1|" + (savedActive == "" ? activeId : savedActive) + "|" + (visible ? "1" : "0") + "|" + values.join(";");
	}
	/** Reject malformed input atomically; unknown providers can register later. */
	public function restore(value:String):Bool {
		if (value.length > 65536) return false;
		var parts = value.split("|");
		if (parts.length != 4 || parts[0] != "1" || (parts[1] != "" && !validId(parts[1])) ||
			(parts[2] != "0" && parts[2] != "1")) return false;
		var widths:Map<String, Float> = [], shown:Map<String, Bool> = [];
		if (parts[3] != "") for (row in parts[3].split(";")) {
			var fields = row.split(",");
			if (fields.length != 3 || !validId(fields[0]) || widths.exists(fields[0]) ||
				(fields[2] != "0" && fields[2] != "1")) return false;
			if (!widthPattern.match(fields[1])) return false;
			var width = Std.parseFloat(fields[1]);
			if (!validWidth(width)) return false;
			widths.set(fields[0], width); shown.set(fields[0], fields[2] == "1");
		}
		savedActive = parts[1]; savedWidths = widths; savedVisible = shown;
		visible = parts[2] == "1";
		for (mode in modes) {
			if (widths.exists(mode.id)) mode.width = widths.get(mode.id);
			if (shown.exists(mode.id)) mode.visible = shown.get(mode.id);
		}
		activeId = savedActive;
		var active = selected(); activeId = active == null ? "" : active.id;
		onChange(); return true;
	}
	static function validWidth(width:Float):Bool return (width == width && width - width == 0.0) && width >= 80.0 && width <= 4096.0;
	static function validId(id:String):Bool {
		if (id.length == 0 || id.length > 128) return false;
		for (i in 0...id.length) {
			var c = id.charCodeAt(i);
			if (!(c >= 97 && c <= 122 || c >= 65 && c <= 90 || c >= 48 && c <= 57 || c == 45 || c == 95)) return false;
		}
		return true;
	}
}
