package nativekit.ui.host;

import FontCollection;
import nativekit.ffi.NativeKit;
import nativekit.ffi.NativeKitTypes;
import NativeKitEventValue;
import NativeKitEvents;
import NativeKitEvents.NativeKitEventSubscription;
import nativekit.ffi.NativeKitGpu;
import NativeKitSurface;
import NativeKitSurface.NativeKitSurfaceFrameSubscription;
import Renderer;
import haxe.Json;
import sys.FileSystem;
import sys.io.File;

/** Owns the complete NativeKit desktop lifecycle for one UIKit application. */
class DesktopUiHost {
	public static function run(options:DesktopUiHostOptions,
			create:DesktopUiHostContext->DesktopUiApplication):Int {
		var host = open(options, create);
		while (host.tick()) {}
		return host.close();
	}

	public static function open(options:DesktopUiHostOptions,
			create:DesktopUiHostContext->DesktopUiApplication):DesktopUiHostSession {
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
		var fonts:Null<FontCollection> = null;
		var runtime:Null<UiHostRuntime> = null;
		var session:Null<UiHostSession> = null;
		var eventHistory:Array<String> = [];
		var frameHistory:Array<Dynamic> = [];
		var captureState = {startedAt: -1.0};
		var result = 0;
		var step:Void->Bool = function() return false;

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

			var pump = new NativeKitEvents();
			events = pump;
			fonts = FontCollection.create();
			fonts.addSystemFallbacks();
			var active = true;
			var surfaceAvailable = false;
			var framePending = false;
			var frameRequested = true;
			var scheduleFrame = function() {
				frameRequested = true;
			};
			session = new UiHostSession(function() active = false);
			var hostContext = new DesktopUiHostContext(fonts, pump, window, surface,
				function() session.stop(), scheduleFrame);
			runtime = new UiHostRuntime(session, hostContext, window, surface,
				options.width, options.height);
			runtime.start(function(_) return create(hostContext));
			if (session.state == UiHostLifecycle.Failed) throw session.error;
			if (runtime.app() != null)
				runtime.app().context().onAnimationFrameRequested = scheduleFrame;
			var borrowedSurface = NativeKitSurface.borrowNativeHandle(surface);
			nativeSurface = borrowedSurface;

			eventSubscription = pump.listen(function(value) {
				if (options.eventHistoryLimit > 0) {
					eventHistory.push(Std.string(value));
					while (eventHistory.length > options.eventHistoryLimit) eventHistory.shift();
				}
				switch (value) {
					case WindowClose(source) if (source.rawValue() == window.rawValue()):
						hostContext.requestClose();
						scheduleFrame();
					case WindowResize(source, width, height) if (source.rawValue() == window.rawValue()):
						if (NativeKit.nk_surface_set_bounds(surface, 0, 0, width, height) != Result.Ok)
							throw "Surface resize failed";
						runtime.resize(width, height, runtime.framebufferWidth, runtime.framebufferHeight);
						scheduleFrame();
					case WindowScaleChanged(source, scale) if (source.rawValue() == window.rawValue()):
						runtime.setScale(scale);
						scheduleFrame();
					case SurfaceReady(source) if (source.rawValue() == surface.rawValue()):
						surfaceAvailable = true;
						var size = NativeKit.nk_surface_get_framebuffer_size(surface);
						if (size.status != Result.Ok) throw "Framebuffer size query failed";
						var scale = NativeKit.nk_window_get_scale(window);
						if (scale.status != Result.Ok) throw "Window scale query failed";
						runtime.setScale(scale.out_scale);
						runtime.resize(runtime.logicalWidth, runtime.logicalHeight,
							size.out_width, size.out_height);
						runtime.setSurfaceReady(size.out_width > 0 && size.out_height > 0);
						if (frameSubscription == null)
							frameSubscription = borrowedSurface.onFrame(function(width, height) {
								try {
									framePending = false;
									if (!active || !surfaceAvailable || !frameRequested) return;
									frameRequested = false;
									runtime.resize(runtime.logicalWidth, runtime.logicalHeight, width, height);
									var frameStartedAt = Sys.time();
									runtime.render(Sys.time());
									if (options.captureSeconds > 0.0 && captureState.startedAt < 0.0 && runtime.rendered > 0)
										captureState.startedAt = frameStartedAt;
									if (options.captureDirectory != null) {
										var metrics = runtime.app() == null ? null : runtime.app().context().frameMetrics;
										frameHistory.push({
											frame: runtime.rendered,
											startedAtSeconds: frameStartedAt,
											frameSeconds: Sys.time() - frameStartedAt,
											submitSeconds: metrics == null ? null : metrics.submitSeconds,
											styleResolutions: metrics == null ? null : metrics.styleResolutions,
											styleCacheHits: metrics == null ? null : metrics.styleCacheHits,
											styleCacheMisses: metrics == null ? null : metrics.styleCacheMisses,
											styleChangedNodes: metrics == null ? null : metrics.styleChangedNodes,
											viewSeconds: metrics == null ? null : metrics.viewSeconds,
											treeAndStyleSeconds: metrics == null ? null : metrics.treeAndStyleSeconds,
											nativeLayoutSeconds: metrics == null ? null : metrics.nativeLayoutSeconds,
											reconcileSeconds: metrics == null ? null : metrics.reconcileSeconds,
											renderSeconds: metrics == null ? null : metrics.renderSeconds,
											customPaintSeconds: metrics == null ? null : metrics.customPaintSeconds,
											nativeRenderSeconds: metrics == null ? null : metrics.nativeRenderSeconds,
											nodeCount: metrics == null ? null : metrics.nodeCount
										});
									}
									if (session.state == UiHostLifecycle.Failed) active = false;
									if (options.captureDirectory != null && options.frameLimit > 0 && runtime.rendered >= options.frameLimit) {
										writeDiagnostics(options, cast runtime.app(), cast runtime.frameRenderer(), runtime, eventHistory, frameHistory);
										session.stop();
									}
									var continueFrames = options.continuousFrames;
									if (active && session.state == UiHostLifecycle.Running &&
										(options.captureDirectory != null && options.frameLimit > 0 ||
											runtime.app().context().needsAnimationFrame ||
										(continueFrames != null && continueFrames()))) {
										scheduleFrame();
									}
								} catch (error:Dynamic) {
									runtime.fail("frame-callback", error);
									active = false;
								}
							});
						scheduleFrame();
					case SurfaceResize(source, width, height, framebufferWidth, framebufferHeight)
						if (source.rawValue() == surface.rawValue()):
						runtime.resize(width, height, framebufferWidth, framebufferHeight);
						scheduleFrame();
					case SurfaceLost(source) if (source.rawValue() == surface.rawValue()):
						surfaceAvailable = false;
						framePending = false;
						runtime.setSurfaceReady(false);
					case _:
						scheduleFrame();
				}
			});

			step = function() {
				if (!active) return false;
				try {
				var hadEvent = pump.poll();
				if (session.state == UiHostLifecycle.Failed) throw session.error;
				if (active && captureState.startedAt >= 0.0 && options.captureSeconds > 0.0 &&
					Sys.time() - captureState.startedAt >= options.captureSeconds) {
					writeDiagnostics(options, cast runtime.app(), cast runtime.frameRenderer(), runtime,
						eventHistory, frameHistory);
					session.stop();
				}
				if (active && surfaceAvailable && frameRequested && !framePending) {
					if (NativeKit.nk_surface_request_frame(surface) != Result.Ok)
						throw "Surface frame request failed";
					framePending = true;
				}
				if (active && !hadEvent) pump.wait(1.0 / options.targetFps);
					if (session.state == UiHostLifecycle.Failed) throw session.error;
				} catch (error:Dynamic) {
					session.fail("desktop-host", error);
					Sys.println(options.title + ": " + Std.string(error));
					if (session.error != null && session.error.stack.length > 0)
						Sys.println(session.error.stack);
					active = false;
					result = 1;
				}
				return active;
			};
		} catch (error:Dynamic) {
			if (session != null && session.state != UiHostLifecycle.Failed)
				session.fail("desktop-host", error);
			var detail = Std.string(error);
			if (Std.isOfType(error, UiHostError)) {
				var hostError:UiHostError = cast error;
				detail = hostError.toString();
				var message = Reflect.field(hostError.cause, "message");
				if (message != null) detail = hostError.stage + ": " + Std.string(message);
			}
			Sys.println(options.title + ": " + detail);
			var stack = session != null && session.error != null && session.error.stack.length > 0
				? session.error.stack : haxe.CallStack.toString(haxe.CallStack.exceptionStack());
			if (stack.length > 0) Sys.println(stack);
			result = 1;
		}

		var finish = function() {
		var ownedFrameSubscription = frameSubscription;
		frameSubscription = null;
		if (ownedFrameSubscription != null)
			try ownedFrameSubscription.dispose() catch (error:Dynamic) if (session != null) session.cleanupFailed("frame-subscription", error);
		var ownedEventSubscription = eventSubscription;
		eventSubscription = null;
		if (ownedEventSubscription != null)
			try ownedEventSubscription.dispose() catch (error:Dynamic) if (session != null) session.cleanupFailed("event-subscription", error);
		var ownedRuntime = runtime;
		runtime = null;
		if (ownedRuntime != null) ownedRuntime.dispose();
		var ownedFonts = fonts;
		fonts = null;
		if (ownedFonts != null)
			try ownedFonts.dispose() catch (error:Dynamic) if (session != null) session.cleanupFailed("fonts-dispose", error);
		var ownedNativeSurface = nativeSurface;
		nativeSurface = null;
		if (ownedNativeSurface != null)
			try ownedNativeSurface.releaseBorrowed() catch (error:Dynamic) if (session != null) session.cleanupFailed("frame-surface-release", error);
		var ownedSurface = surface;
		surface = SurfaceHandle.invalid();
		if (ownedSurface.isValid())
			try NativeKit.nk_surface_destroy(ownedSurface) catch (error:Dynamic) if (session != null) session.cleanupFailed("surface-destroy", error);
		var ownedWindow = window;
		window = WindowHandle.invalid();
		if (ownedWindow.isValid())
			try NativeKit.nk_window_destroy(ownedWindow) catch (error:Dynamic) if (session != null) session.cleanupFailed("window-destroy", error);
		var wasInitialized = initialized;
		initialized = false;
		if (wasInitialized)
			try NativeKit.nk_shutdown() catch (error:Dynamic) if (session != null) session.cleanupFailed("nativekit-shutdown", error);
		var ownedEvents = events;
		events = null;
		if (ownedEvents != null)
			try ownedEvents.runtimeShutdown() catch (error:Dynamic) if (session != null) session.cleanupFailed("events-shutdown", error);
		if (session != null && session.state != UiHostLifecycle.Failed &&
			session.state != UiHostLifecycle.Stopped) {
			if (session.state != UiHostLifecycle.Stopping) session.transition(UiHostLifecycle.Stopping);
			session.transition(UiHostLifecycle.Stopped);
		}
		return result;
		};
		return new DesktopUiHostSession(step, finish);
	}

	static function writeDiagnostics(options:DesktopUiHostOptions,
			application:DesktopUiApplication, renderer:Renderer, state:UiHostRuntime,
			events:Array<String>, frames:Array<Dynamic>):Void {
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
		File.saveContent(directory + "/frame-timeline.jsonl",
			[for (frame in frames) Json.stringify(frame)].join("\n") + "\n");
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
