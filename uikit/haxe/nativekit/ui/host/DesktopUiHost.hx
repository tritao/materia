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
import haxe.io.Bytes;
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
		var eventCounts:Map<String, Int> = new Map();
		var frameRequestCounts:Map<String, Int> = new Map();
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
			if (options.icons != null) {
				var icons = options.icons;
				var pixels = Bytes.alloc(icons.byteCount);
				var images:Array<IconImage> = [];
				var offset = 0;
				for (icon in icons.imageList()) {
					pixels.blit(offset, icon.pixels, 0, icon.pixels.length);
					var image = new IconImage();
					image.set_offset(offset);
					image.set_width(icon.width);
					image.set_height(icon.height);
					image.set_stride(icon.stride);
					images.push(image);
					offset += icon.pixels.length;
				}
				if (NativeKit.nk_window_set_icons(window, pixels, images) != Result.Ok)
					throw "Window icons failed: " + NativeKit.nk_last_error();
			}

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
			var frameRequested = false;
			var frameRequestedAt = -1.0;
			var frameRequestReason = "none";
			var frameRequestSerial = 0;
			var incrementCount = function(counts:Map<String, Int>, key:String):Void {
				var previous = counts.get(key);
				counts.set(key, previous == null ? 1 : previous + 1);
			};
			var scheduleFrameWithReason = function(reason:String):Void {
				frameRequested = true;
				frameRequestedAt = Sys.time();
				frameRequestReason = reason;
				frameRequestSerial++;
				incrementCount(frameRequestCounts, reason);
			};
			var scheduleFrame = function():Void scheduleFrameWithReason("api");
			scheduleFrameWithReason("startup");
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
				var eventName = eventKind(value);
				incrementCount(eventCounts, eventName);
				if (options.eventHistoryLimit > 0) {
					eventHistory.push(Std.string(value));
					while (eventHistory.length > options.eventHistoryLimit) eventHistory.shift();
				}
				switch (value) {
					case WindowClose(source) if (source.rawValue() == window.rawValue()):
						hostContext.requestClose();
						scheduleFrameWithReason("window-close");
					case WindowResize(source, width, height) if (source.rawValue() == window.rawValue()):
						if (NativeKit.nk_surface_set_bounds(surface, 0, 0, width, height) != Result.Ok)
							throw "Surface resize failed";
						runtime.resize(width, height, runtime.framebufferWidth, runtime.framebufferHeight);
						scheduleFrameWithReason("window-resize");
					case WindowScaleChanged(source, scale) if (source.rawValue() == window.rawValue()):
						runtime.setScale(scale);
						scheduleFrameWithReason("window-scale");
					case WindowMove(source, _, _) if (source.rawValue() == window.rawValue()):
						// Moving the top-level window does not change the surface contents.
						// On X11 this event is emitted for every position update while the
						// window is dragged; redrawing the complete Haxe UI here competes
						// with the compositor and makes the drag less responsive.
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
									var requestedAt = frameRequestedAt;
									var requestReason = frameRequestReason;
									var requestSerial = frameRequestSerial;
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
											requestReason: requestReason,
											requestSerial: requestSerial,
											requestAgeSeconds: requestedAt < 0.0 ? null : frameStartedAt - requestedAt,
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
										writeDiagnostics(options, cast runtime.app(), cast runtime.frameRenderer(), runtime,
											eventHistory, frameHistory, eventCounts, frameRequestCounts);
										session.stop();
									}
									var continueFrames = options.continuousFrames;
									var needsAnimationFrame = runtime.app().context().needsAnimationFrame;
									var wantsContinuousFrames = !needsAnimationFrame &&
										continueFrames != null && continueFrames();
									if (active && session.state == UiHostLifecycle.Running &&
										(options.captureDirectory != null && options.frameLimit > 0 ||
											needsAnimationFrame || wantsContinuousFrames)) {
										if (options.captureDirectory != null && options.frameLimit > 0)
											scheduleFrameWithReason("capture");
										else if (needsAnimationFrame)
											scheduleFrameWithReason("animation");
										else
											scheduleFrameWithReason("continuous");
									}
								} catch (error:Dynamic) {
									runtime.fail("frame-callback", error);
									active = false;
								}
							});
						scheduleFrameWithReason("surface-ready");
					case SurfaceResize(source, width, height, framebufferWidth, framebufferHeight)
						if (source.rawValue() == surface.rawValue()):
						runtime.resize(width, height, framebufferWidth, framebufferHeight);
						scheduleFrameWithReason("surface-resize");
					case SurfaceLost(source) if (source.rawValue() == surface.rawValue()):
						surfaceAvailable = false;
						framePending = false;
						runtime.setSurfaceReady(false);
					case _:
						scheduleFrameWithReason("event:" + eventName);
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
						eventHistory, frameHistory, eventCounts, frameRequestCounts);
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
			events:Array<String>, frames:Array<Dynamic>, eventCounts:Map<String, Int>,
			frameRequestCounts:Map<String, Int>):Void {
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
			},
			scheduling: {
				eventCounts: countsObject(eventCounts),
				frameRequestCounts: countsObject(frameRequestCounts)
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

	static function countsObject(counts:Map<String, Int>):Dynamic {
		var result:Dynamic = {};
		for (key in counts.keys()) {
			var value = counts.get(key);
			Reflect.setField(result, key, value == null ? 0 : value);
		}
		return result;
	}

	static function eventKind(value:NativeKitEventValue):String {
		return switch (value) {
			case WindowClose(_): "WindowClose";
			case WindowResize(_, _, _): "WindowResize";
			case WindowMove(_, _, _): "WindowMove";
			case WindowFramebufferResize(_, _, _): "WindowFramebufferResize";
			case WindowScaleChanged(_, _): "WindowScaleChanged";
			case WindowStateChanged(_, _): "WindowStateChanged";
			case Key(_, _, _, _, _): "Key";
			case TextInput(_, _): "TextInput";
			case TextEdit(_, _): "TextEdit";
			case PointerMove(_, _, _): "PointerMove";
			case PointerButton(_, _, _, _, _, _): "PointerButton";
			case PointerScroll(_, _, _): "PointerScroll";
			case PointerEnter(_, _): "PointerEnter";
			case Touch(_, _, _, _, _, _, _, _, _, _): "Touch";
			case SurfaceReady(_): "SurfaceReady";
			case SurfaceResize(_, _, _, _, _): "SurfaceResize";
			case SurfaceLost(_): "SurfaceLost";
			case None: "None";
			case _: "other";
		};
	}

	static function createDirectories(path:String):Void {
		if (path == null || path.length == 0 || FileSystem.exists(path)) return;
		var slash = path.lastIndexOf("/");
		if (slash > 0) createDirectories(path.substr(0, slash));
		if (!FileSystem.exists(path)) FileSystem.createDirectory(path);
	}
}
