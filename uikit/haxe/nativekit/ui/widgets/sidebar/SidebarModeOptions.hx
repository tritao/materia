package nativekit.ui.widgets.sidebar;

/** Registration metadata; width is in layout pixels. */
class SidebarModeOptions {
	public final label:String;
	public final order:Int;
	public final visible:Bool;
	public final width:Float;
	public function new(label:String, order:Int = 0, visible:Bool = true, width:Float = 240.0) {
		this.label = label; this.order = order; this.visible = visible; this.width = width;
	}
}
