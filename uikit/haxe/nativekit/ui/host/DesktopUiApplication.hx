package nativekit.ui.host;

import LayoutFrame;
import nativekit.ui.core.RenderNode;
import nativekit.ui.core.UiContext;

/** Application surface consumed by the reusable NativeKit desktop host. */
interface DesktopUiApplication {
	public function context():UiContext;
	public function submit(frame:LayoutFrame):RenderNode;
	public function diagnosticState():Dynamic;
	public function dispose():Void;
}
