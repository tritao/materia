package nativekit.ui.host;

import FontCollection;
import NativeKitEvents;
import haxe.Int64;
import nativekit.ffi.NativeKit;

/** Live, platform-neutral services available to a hosted application. */
class UiHostContext {
	public final fonts:FontCollection;
	public final events:NativeKitEvents;
	/** An application may defer closing while it asks to save or confirm. */
	public var onCloseRequested:Null<(Void->Void)->Void> = null;
	final close:Void->Void;
	final scheduleFrame:Void->Void;

	public function new(fonts:FontCollection, events:NativeKitEvents,
			close:Void->Void, scheduleFrame:Void->Void) {
		this.fonts = fonts;
		this.events = events;
		this.close = close;
		this.scheduleFrame = scheduleFrame;
	}

	public function supports(capability:Int64):Bool {
		return Int64.compare(Int64.and(NativeKit.nk_get_capabilities(), capability),
			Int64.ofInt(0)) != 0;
	}

	/** Requests another frame without prescribing how the host schedules it. */
	public function requestFrame():Void scheduleFrame();

	/** Begins the application-defined close-confirmation flow. */
	public function requestClose():Void {
		if (onCloseRequested == null) close();
		else onCloseRequested(close);
	}
}
