package nativekit.ui.host;

import FrameInfo;
import LayoutFrame;
import NativeKitEvents;
import NativeKitSurface;
import Renderer;
import Surface;
import nativekit.ffi.NativeKitTypes;
import nativekit.ui.core.NativeInputAdapter;

/** Shared application, input, frame, and disposal ownership used by host adapters. */
class UiHostRuntime {
	public final session:UiHostSession;
	public var logicalWidth(get, never):Float;
	public var logicalHeight(get, never):Float;
	public var framebufferWidth(get, never):Int;
	public var framebufferHeight(get, never):Int;
	public var scale(get, never):Float;
	public var rendered(default, null):Int = 0;
	public var surfaceReady(get, never):Bool;

	final window:WindowHandle;
	final surface:SurfaceHandle;
	final context:UiHostContext;
	var application:Null<UiApplication> = null;
	var input:Null<NativeInputAdapter> = null;
	var nativeSurface:Null<NativeKitSurface> = null;
	var renderer:Null<Renderer> = null;
	var frame:LayoutFrame;
	var frameInfo:FrameInfo;
	final frameState:UiHostFrameState;
	var disposed:Bool = false;
	var started:Bool = false;
	var callbackDepth:Int = 0;
	var disposeRequested:Bool = false;

	public function new(session:UiHostSession, context:UiHostContext, window:WindowHandle,
			surface:SurfaceHandle, width:Int, height:Int) {
		this.session = session;
		this.context = context;
		this.window = window;
		this.surface = surface;
		frameState = new UiHostFrameState(width, height);
		frame = new LayoutFrame(width, height);
		frameInfo = new FrameInfo(width, height, width, height, 1.0);
	}

	function get_logicalWidth():Float return frameState.logicalWidth;
	function get_logicalHeight():Float return frameState.logicalHeight;
	function get_framebufferWidth():Int return frameState.framebufferWidth;
	function get_framebufferHeight():Int return frameState.framebufferHeight;
	function get_scale():Float return frameState.scale;
	function get_surfaceReady():Bool return frameState.surfaceAvailable;

	public function start(create:UiHostContext->UiApplication):Void {
		if (started || disposed || session.state == UiHostLifecycle.Failed ||
			session.state == UiHostLifecycle.Stopping || session.state == UiHostLifecycle.Stopped) {
			fail("application-start", "UI host runtime cannot be started more than once or after termination");
			return;
		}
		started = true;
		callbackDepth++;
		try {
			var created = create(context);
			if (created == null) throw "UI application factory returned null";
			if (disposeRequested || session.state != UiHostLifecycle.Starting &&
				session.state != UiHostLifecycle.LoadingResources) {
				try created.dispose() catch (error:Dynamic) session.cleanupFailed("application-dispose", error);
			} else {
				application = created;
				renderer = Renderer.create();
				nativeSurface = NativeKitSurface.borrowNativeHandle(surface);
				created.context().attachPlatformSurface(nativeSurface);
				created.context().attachPlatformWindow(window);
				input = new NativeInputAdapter(created.context(), new Handle(window.rawValue()),
					new Handle(surface.rawValue()));
				input.attach(context.events);
				session.transition(UiHostLifecycle.Running);
			}
		} catch (error:Dynamic) {
			fail("application-start", error);
		}
		callbackDepth--;
		if (callbackDepth == 0 && disposeRequested) disposeNow();
	}

	public function resize(width:Float, height:Float, framebufferWidth:Int,
			framebufferHeight:Int):Void {
		frameState.resize(width, height, framebufferWidth, framebufferHeight);
	}

	public function setScale(value:Float):Void frameState.setScale(value);
	public function setSurfaceReady(value:Bool):Void frameState.setSurfaceAvailable(value);

	/** Renders at most once for the adapter's scheduling callback. */
	public function render(timeSeconds:Float):Bool {
		if (disposed || session.state != UiHostLifecycle.Running || !frameState.canRender() ||
			application == null || renderer == null)
			return false;
		try {
			callbackDepth++;
			frame.setViewport(logicalWidth, logicalHeight);
			frame.deltaSeconds = frameState.nextDelta(timeSeconds);
			frameInfo.set(logicalWidth, logicalHeight, framebufferWidth, framebufferHeight, scale);
			application.submit(frame);
			if (session.state != UiHostLifecycle.Running || disposeRequested) {
				callbackDepth--;
				if (callbackDepth == 0 && disposeRequested) disposeNow();
				return false;
			}
			application.context().render(renderer, Surface.fromNativeHandle(surface), frameInfo);
			rendered++;
			callbackDepth--;
			if (callbackDepth == 0 && disposeRequested) disposeNow();
			return true;
		} catch (error:Dynamic) {
			if (callbackDepth > 0) callbackDepth--;
			fail("frame", error);
			if (callbackDepth == 0 && disposeRequested) disposeNow();
			return false;
		}
	}

	public function app():Null<UiApplication> return application;
	public function frameRenderer():Null<Renderer> return renderer;

	public function fail(stage:String, error:Dynamic):Void {
		session.fail(stage, error);
		dispose();
	}

	public function dispose():Void {
		if (disposed || disposeRequested) return;
		disposeRequested = true;
		if (callbackDepth > 0) return;
		disposeNow();
	}

	public function isCallbackActive():Bool return callbackDepth > 0;

	function disposeNow():Void {
		if (disposed) return;
		disposed = true;
		var ownedInput = input;
		input = null;
		if (ownedInput != null)
			try ownedInput.detach() catch (error:Dynamic) session.cleanupFailed("input-detach", error);
		var ownedApplication = application;
		application = null;
		if (ownedApplication != null)
			try ownedApplication.dispose() catch (error:Dynamic) session.cleanupFailed("application-dispose", error);
		var ownedRenderer = renderer;
		renderer = null;
		if (ownedRenderer != null)
			try ownedRenderer.dispose() catch (error:Dynamic) session.cleanupFailed("renderer-dispose", error);
		var ownedSurface = nativeSurface;
		nativeSurface = null;
		if (ownedSurface != null)
			try ownedSurface.releaseBorrowed() catch (error:Dynamic) session.cleanupFailed("surface-release", error);
	}
}
