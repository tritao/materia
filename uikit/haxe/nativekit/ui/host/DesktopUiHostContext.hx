package nativekit.ui.host;

import FontCollection;
import NativeKitEvents;
import nativekit.ffi.NativeKitTypes;

/** Live services available while constructing a hosted desktop application. */
class DesktopUiHostContext extends UiHostContext {
	/** Borrowed window handle for application-owned native dialogs. */
	public final window:WindowHandle;
	/** An application may defer closing while it asks to save a document. */
	@:allow(nativekit.ui.host.DesktopUiHost)
	private function new(fonts:FontCollection, events:NativeKitEvents, window:WindowHandle,
			close:Void->Void, scheduleFrame:Void->Void) {
		super(fonts, events, close, scheduleFrame);
		this.window = window;
	}
}
