package nativekit.ui.host;

import FontCollection;
import NativeKitEvents;

/** Live services available while constructing a hosted desktop application. */
class DesktopUiHostContext {
	public final fonts:FontCollection;
	public final events:NativeKitEvents;

	@:allow(nativekit.ui.host.DesktopUiHost)
	private function new(fonts:FontCollection, events:NativeKitEvents) {
		this.fonts = fonts;
		this.events = events;
	}
}
