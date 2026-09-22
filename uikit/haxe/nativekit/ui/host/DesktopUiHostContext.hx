package nativekit.ui.host;

import FontCollection;
import NativeKitEvents;
import nativekit.ffi.NativeKitTypes;

/** Live services available while constructing a hosted desktop application. */
class DesktopUiHostContext {
	public final fonts:FontCollection;
	public final events:NativeKitEvents;
	/** Borrowed window handle for application-owned native dialogs. */
	public final window:WindowHandle;
	/** An application may defer closing while it asks to save a document. */
	public var onCloseRequested:Null<(Void->Void)->Void> = null;
	final close:Void->Void;

	@:allow(nativekit.ui.host.DesktopUiHost)
	private function new(fonts:FontCollection, events:NativeKitEvents, window:WindowHandle, close:Void->Void) {
		this.fonts = fonts;
		this.events = events;
		this.window = window;
		this.close = close;
	}
	public function requestClose():Void {
		if (onCloseRequested == null) close();
		else onCloseRequested(close);
	}
}
