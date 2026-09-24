package app;

import Color;

/** Neutral viewport gradient shared by the editor's 2D and 3D views. */
class ViewportBackground {
	public static function top():Color
		return Color.rgba(0.894, 0.910, 0.922, 1.0);

	public static function bottom():Color
		return Color.rgba(0.847, 0.867, 0.882, 1.0);
}
