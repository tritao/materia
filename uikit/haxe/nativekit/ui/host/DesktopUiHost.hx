package nativekit.ui.host;

import FontCollection;
import FrameInfo;
import LayoutFrame;
import nativekit.ffi.NativeKit;
import nativekit.ffi.NativeKitTypes;
import NativeKitEventValue;
import NativeKitEvents;
import NativeKitEvents.NativeKitEventSubscription;
import nativekit.ffi.NativeKitGpu;
import NativeKitSurface;
import NativeKitSurface.NativeKitSurfaceFrameSubscription;
import Renderer;
import Surface;
import haxe.Json;
import nativekit.ui.core.NativeInputAdapter;
import sys.FileSystem;
import sys.io.File;

private class DesktopUiFrameState {
	public var running:Bool = true;
	public var ready:Bool = false;
	public var logicalWidth:Float;
	public var logicalHeight:Float;
	public var framebufferWidth:Int;
	public var framebufferHeight:Int;
	public var scale:Float = 1.0;
	public var rendered:Int = 0;
	public var callbackError:Null<Dynamic> = null;

	public function new(width:Int, height:Int) {
		logicalWidth = width;
		logicalHeight = height;
		framebufferWidth = width;
		framebufferHeight = height;
	}
}

/** Owns the complete NativeKit desktop lifecycle for one UIKit application. */
class DesktopUiHost {
	public static function run(options:DesktopUiHostOptions,
			create:DesktopUiHostContext->DesktopUiApplication):Int {
		if (options == null || create == null)
			throw "Desktop UI host requires options and an application factory";
		options.validate();
		var initialized = false;
		var window = WindowHandle.invalid();
		var surface = SurfaceHandle.invalid();
		var events:Null<NativeKitEvents> = null;
		var eventSubscription:Null<NativeKitEventSubscription> = null;
		var nativeSurface:Null<NativeKitSurface> = null;
		var frameSubscription:Null<NativeKitSurfaceFrameSubscription> = null;
		var input:Null<NativeInputAdapter> = null;
		var fonts:Null<FontCollection> = null;
		var renderer:Null<Renderer> = null;
		var application:Null<DesktopUiApplication> = null;
		var eventHistory:Array<String> = [];
		var result = 0;

		try {
			var init = new InitOptions();
			init.set_api_version(NativeKit.nk_api_version());
			init.set_event_queue_capacity(options.eventQueueCapacity);
			if (NativeKit.nk_init(init) != Result.Ok)
				throw "NativeKit initialization failed: " + NativeKit.nk_last_error();
			initialized = true;

			var windowOptions = new WindowOptions();
			windowOptions.set_width(options.width);
			windowOptions.set_height(options.height);
			windowOptions.set_title(options.title);
			windowOptions.set_flags(WindowFlags.Resizable);
			windowOptions.set_owner(WindowHandle.invalid());
			windowOptions.set_kind(WindowKind.Normal);
			var createdWindow = NativeKit.nk_window_create(windowOptions);
			if (createdWindow.status != Result.Ok)
				throw "Window creation failed: " + NativeKit.nk_last_error();
			window = createdWindow.out_window.borrow();

			var graphicsApi:GraphicsApi = NativeKitGpu.nkgpu_default_graphics_api();
			var surfaceOptions = new SurfaceOptions();
			var surfaceFlags = SurfaceFlags.Stencil;
			if (graphicsApi == GraphicsApi.Opengl) {
				surfaceFlags = SurfaceFlags.ForwardCompatible | SurfaceFlags.Stencil;
				surfaceOptions.set_major_version(3);
				surfaceOptions.set_minor_version(3);
			}
			surfaceOptions.set_flags(surfaceFlags);
			surfaceOptions.set_api(graphicsApi);
			surfaceOptions.set_width(options.width);
			surfaceOptions.set_height(options.height);
			var createdSurface = NativeKit.nk_surface_create(new Handle(window.rawValue()), surfaceOptions);
			if (createdSurface.status != Result.Ok)
				throw "Surface creation failed: " + NativeKit.nk_last_error();
			surface = createdSurface.out_surface.borrow();

			var state = new DesktopUiFrameState(options.width, options.height);
			var layoutFrame = new LayoutFrame(options.width, options.height);
			var frameInfo = new FrameInfo(options.width, options.height,
				options.width, options.height, 1.0);
			var previousTime = Sys.time();
			var pump = new NativeKitEvents();
			events = pump;
			fonts = FontCollection.create();
			fonts.addSystemFallbacks();
			renderer = Renderer.create();
			application = create(new DesktopUiHostContext(fonts, pump));
			if (application == null)
				throw "Desktop UI application factory returned null";
			var borrowedSurface = NativeKitSurface.borrowNativeHandle(surface);
			nativeSurface = borrowedSurface;
			application.context().attachPlatformSurface(borrowedSurface);
			application.context().attachPlatformWindow(window);
			input = new NativeInputAdapter(application.context(), new Handle(window.rawValue()),
				new Handle(surface.rawValue()));
			input.attach(pump);

			var renderFrame = function(framebufferWidth:Int, framebufferHeight:Int) {
				if (!state.running || !state.ready || framebufferWidth <= 0 || framebufferHeight <= 0)
					return;
				state.framebufferWidth = framebufferWidth;
				state.framebufferHeight = framebufferHeight;
				var now = Sys.time();
				layoutFrame.setViewport(state.logicalWidth, state.logicalHeight);
				layoutFrame.deltaSeconds = Math.max(0.0, Math.min(0.1, now - previousTime));
				previousTime = now;
				frameInfo.set(state.logicalWidth, state.logicalHeight, framebufferWidth,
					framebufferHeight, state.scale);
				try application.submit(layoutFrame)
				catch (error:Dynamic) throw "UI submit failed: " + Std.string(error);
				try application.context().render(renderer, Surface.fromNativeHandle(surface), frameInfo)
				catch (error:Dynamic) throw "UI render failed: " + Std.string(error);
				state.rendered++;
				if (options.captureDirectory != null && state.rendered >= options.frameLimit) {
					writeDiagnostics(options, application, renderer, state, eventHistory);
					state.running = false;
				}
			};

			eventSubscription = pump.listen(function(value) {
				if (options.eventHistoryLimit > 0) {
					eventHistory.push(Std.string(value));
					while (eventHistory.length > options.eventHistoryLimit) eventHistory.shift();
				}
				switch (value) {
					case WindowClose(source) if (source.rawValue() == window.rawValue()):
						state.running = false;
					case WindowResize(source, width, height) if (source.rawValue() == window.rawValue()):
						if (NativeKit.nk_surface_set_bounds(surface, 0, 0, width, height) != Result.Ok)
							throw "Surface resize failed";
						state.logicalWidth = width;
						state.logicalHeight = height;
					case WindowScaleChanged(source, scale) if (source.rawValue() == window.rawValue()):
						state.scale = scale;
					case SurfaceReady(source) if (source.rawValue() == surface.rawValue()):
						state.ready = true;
						var size = NativeKit.nk_surface_get_framebuffer_size(surface);
						if (size.status != Result.Ok) throw "Framebuffer size query failed";
						state.framebufferWidth = size.out_width;
						state.framebufferHeight = size.out_height;
						var scale = NativeKit.nk_window_get_scale(window);
						if (scale.status != Result.Ok) throw "Window scale query failed";
						state.scale = scale.out_scale;
						if (frameSubscription == null)
							frameSubscription = borrowedSurface.onFrame(function(width, height) {
								try {
									renderFrame(width, height);
									if (state.running && NativeKit.nk_surface_request_frame(surface) != Result.Ok)
										throw "Surface frame request failed";
								} catch (error:Dynamic) {
									state.callbackError = error;
									state.running = false;
								}
							});
						if (NativeKit.nk_surface_request_frame(surface) != Result.Ok)
							throw "Initial surface frame request failed";
					case SurfaceResize(source, width, height, framebufferWidth, framebufferHeight)
							if (source.rawValue() == surface.rawValue()):
						state.logicalWidth = width;
						state.logicalHeight = height;
						state.framebufferWidth = framebufferWidth;
						state.framebufferHeight = framebufferHeight;
					case SurfaceLost(source) if (source.rawValue() == surface.rawValue()):
						state.ready = false;
					case _:
				}
			});

			while (state.running) {
				var hadEvent = pump.poll();
				if (state.callbackError != null) throw state.callbackError;
				if (state.running && !hadEvent) pump.wait(1.0 / options.targetFps);
			}
		} catch (error:Dynamic) {
			Sys.println(options.title + ": " + Std.string(error));
			var stack = haxe.CallStack.toString(haxe.CallStack.exceptionStack());
			if (stack.length > 0) Sys.println(stack);
			result = 1;
		}

		if (input != null) input.detach();
		if (frameSubscription != null) frameSubscription.dispose();
		if (application != null) application.dispose();
		if (renderer != null) renderer.dispose();
		if (fonts != null) fonts.dispose();
		if (eventSubscription != null) eventSubscription.dispose();
		if (nativeSurface != null) nativeSurface.releaseBorrowed();
		if (surface.isValid()) NativeKit.nk_surface_destroy(surface);
		if (window.isValid()) NativeKit.nk_window_destroy(window);
		if (initialized) NativeKit.nk_shutdown();
		if (events != null) events.runtimeShutdown();
		return result;
	}

	static function writeDiagnostics(options:DesktopUiHostOptions,
			application:DesktopUiApplication, renderer:Renderer, state:DesktopUiFrameState,
			events:Array<String>):Void {
		var directory:String = cast options.captureDirectory;
		createDirectories(directory);
		File.saveContent(directory + "/ui-tree.txt", application.context().dumpTree() + "\n");
		File.saveContent(directory + "/layout.json",
			Json.stringify(application.context().inspect(), null, "  ") + "\n");
		File.saveContent(directory + "/app-state.json",
			Json.stringify(application.diagnosticState(), null, "  ") + "\n");
		var metrics = application.context().frameMetrics;
		var stats = renderer.stats();
		var frameData:Dynamic = {
			logicalWidth: state.logicalWidth,
			logicalHeight: state.logicalHeight,
			framebufferWidth: state.framebufferWidth,
			framebufferHeight: state.framebufferHeight,
			pixelScale: state.scale,
			renderedFrames: state.rendered,
			ui: metrics == null ? null : {
				frameNumber: metrics.frameNumber, nodeCount: metrics.nodeCount,
				submitSeconds: metrics.submitSeconds, renderSeconds: metrics.renderSeconds,
				totalSeconds: metrics.totalSeconds, paintedNodes: metrics.paintedNodes,
				paintSkippedNodes: metrics.paintSkippedNodes, emptyPaintNodes: metrics.emptyPaintNodes,
				nativeLayoutSubmitted: metrics.nativeLayoutSubmitted,
				nativeLayoutReused: metrics.nativeLayoutReused
			},
			renderer: {
				pathPreparations: Std.string(stats.pathPreparations),
				pathCacheHits: Std.string(stats.pathCacheHits),
				pathCacheMisses: Std.string(stats.pathCacheMisses),
				rasterCacheHits: Std.string(stats.rasterCacheHits),
				rasterCacheMisses: Std.string(stats.rasterCacheMisses)
			}
		};
		File.saveContent(directory + "/frame-metrics.json",
			Json.stringify(frameData, null, "  ") + "\n");
		File.saveContent(directory + "/events.jsonl",
			[for (event in events) Json.stringify({event: event})].join("\n") + "\n");
		var screenshot = directory + "/frame.png";
		var screenshotResult = Sys.command("import", ["-window", options.title, screenshot]);
		if (screenshotResult != 0)
			File.saveContent(directory + "/screenshot-error.txt",
				"ImageMagick import could not capture the application window (exit " +
				screenshotResult + ").\n");
		Sys.println("diagnostics captured in " + directory);
	}

	static function createDirectories(path:String):Void {
		if (path == null || path.length == 0 || FileSystem.exists(path)) return;
		var slash = path.lastIndexOf("/");
		if (slash > 0) createDirectories(path.substr(0, slash));
		if (!FileSystem.exists(path)) FileSystem.createDirectory(path);
	}
}
