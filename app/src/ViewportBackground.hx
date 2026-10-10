package app;

import haxeon.editor.ViewportLook;
import haxeon.ui.Color;

/** Neutral viewport gradient shared by the editor's 2D and 3D views (haxeon.editor.ViewportLook). */
class ViewportBackground {
	public static function top():Color
		return color(ViewportLook.backgroundTop());

	public static function bottom():Color
		return color(ViewportLook.backgroundBottom());

	static function color(rgb:Array<Float>):Color
		return Color.rgba(rgb[0], rgb[1], rgb[2], 1.0);
}
