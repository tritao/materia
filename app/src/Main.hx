package app;

import Canvas;
import Color;
import LayoutAxis;
import LayoutDirection;
import LayoutFrame;
import LayoutStyle;
import Insets;
import Rect;
import sys.FileSystem;
import sys.io.File;
import haxe.Json;
import nativekit.ui.core.Command;
import nativekit.ui.core.CommandContext;
import nativekit.ui.core.CommandRegistry;
import nativekit.ui.core.CommandResult;
import nativekit.ui.core.DockNode;
import nativekit.ui.core.DockPanelDescriptor;
import nativekit.ui.core.DockSplitAxis;
import nativekit.ui.core.DockWorkspaceModel;
import nativekit.ui.core.DockWorkspacePersistence;
import nativekit.ui.core.PlotModel;
import nativekit.ui.core.PlotPoint;
import nativekit.ui.core.PlotSeries;
import nativekit.ui.core.PropertyDescriptor;
import nativekit.ui.core.PropertyDescriptorOptions;
import nativekit.ui.core.PropertyInspectorSection;
import nativekit.ui.core.PropertyType;
import nativekit.ui.core.PropertyValue;
import nativekit.ui.core.RenderNode;
import nativekit.ui.core.Shortcut;
import nativekit.ui.core.UiContext;
import nativekit.ui.core.UiEvent;
import nativekit.ui.core.UiEventKind;
import nativekit.ui.core.UiKey;
import nativekit.ui.core.UiModifier;
import nativekit.ui.core.View;
import nativekit.ui.core.ViewportCamera;
import nativekit.ui.core.ViewportContent;
import nativekit.ui.widgets.Align;
import nativekit.ui.widgets.AppShell;
import nativekit.ui.widgets.Button;
import nativekit.ui.widgets.ButtonVariant;
import nativekit.ui.widgets.Column;
import nativekit.ui.widgets.CommandButton;
import nativekit.ui.widgets.CommandMenu;
import nativekit.ui.widgets.CommandPalette;
import nativekit.ui.widgets.DockWorkspace;
import nativekit.ui.widgets.GpuViewport;
import nativekit.ui.widgets.Icon;
import nativekit.ui.widgets.IconButton;
import nativekit.ui.widgets.KeyedView;
import nativekit.ui.widgets.ListView;
import nativekit.ui.widgets.ListViewModel;
import nativekit.ui.widgets.Menu;
import nativekit.ui.widgets.MenuItem;
import nativekit.ui.widgets.Popup;
import nativekit.ui.widgets.PlotView;
import nativekit.ui.widgets.PropertyInspector;
import nativekit.ui.widgets.Row;
import nativekit.ui.widgets.ScrollController;
import nativekit.ui.widgets.ScrollView;
import nativekit.ui.widgets.SearchField;
import nativekit.ui.widgets.SizedBox;
import nativekit.ui.widgets.Stack;
import nativekit.ui.widgets.StackChild;
import nativekit.ui.widgets.Spacer;
import nativekit.ui.widgets.SplitOrientation;
import nativekit.ui.widgets.SplitSide;
import nativekit.ui.widgets.SplitView;
import nativekit.ui.widgets.SplitViewOptions;
import nativekit.ui.widgets.TabItem;
import nativekit.ui.widgets.Tabs;
import nativekit.ui.widgets.Text;
import nativekit.ui.widgets.TextField;
import nativekit.ui.widgets.Toolbar;
import nativekit.ui.widgets.TreeRootMetadata;
import nativekit.ui.widgets.TreeView;
import nativekit.ui.widgets.TreeViewModel;
import FontCollection;
import FrameInfo;
import NativeKit;
import NativeKit.GraphicsApi;
import NativeKit.Handle;
import NativeKit.InitOptions;
import NativeKit.Result;
import NativeKit.SurfaceFlags;
import NativeKit.SurfaceHandle;
import NativeKit.SurfaceOptions;
import NativeKit.WindowFlags;
import NativeKit.WindowHandle;
import NativeKit.WindowKind;
import NativeKit.WindowOptions;
import NativeKitEventValue;
import NativeKitEvents;
import NativeKitEvents.NativeKitEventSubscription;
import NativeKitGpu;
import NativeKitSurface;
import NativeKitSurface.NativeKitSurfaceFrameSubscription;
import Renderer;
import Surface;
import robotkit.world.RemoteRobot;
import robotkit.world.RobotWorld;
import nativekit.ui.core.NativeInputAdapter;
import nativekit.ui.lab.ComponentLab;
import nativekit.ui.theme.Theme;

/**
	Small executable driver for the shared reference-editor shell.

	The native host owns the window, surface, renderer, and input adapter. It
	creates `ReferenceEditorApp`, submits `app.view()` every frame, and renders
	the returned tree through the normal NativeKit UI path.
*/
class Main {
  public static function main():Int {
    var args = Sys.args();
    for (arg in args)
      if (arg != "--reset-workspace" && arg != "--snapshot" &&
          arg != "--lab" && arg != "--dark" && arg.indexOf("--story=") != 0 &&
          arg.indexOf("--capture-dir=") != 0 && arg.indexOf("--frames=") != 0 &&
          arg.indexOf("--robot=") != 0) {
        Sys.println("Usage: materia [--reset-workspace] [--snapshot] " +
          "[--lab] [--dark] [--story=ID] [--capture-dir=PATH] [--frames=N] " +
          "[--robot=HOST:PORT]");
        return 2;
      }

    if (args.indexOf("--snapshot") >= 0) {
      var editor = new ReferenceEditorApp();
      if (args.indexOf("--reset-workspace") >= 0) editor.resetWorkspace();
      Sys.println(editor.workspace.snapshotJson());
      editor.dispose();
      return 0;
    }
    var diagnostics = ReferenceEditorDiagnostics.fromArgs(args);
    if (diagnostics == null) return 2;
    return ReferenceEditorHost.run(args.indexOf("--reset-workspace") >= 0, diagnostics);
  }
}

private class ReferenceEditorDiagnostics {
  public final captureDirectory:Null<String>;
  public final frameLimit:Int;
  public final componentLab:Bool;
  public final storyId:Null<String>;
  public final darkTheme:Bool;
  public final robotHost:Null<String>;
  public final robotPort:Int;
  public final events:Array<String> = [];

  public function new(captureDirectory:Null<String>, frameLimit:Int,
      componentLab:Bool, storyId:Null<String>, darkTheme:Bool,
      robotHost:Null<String>, robotPort:Int) {
    this.captureDirectory = captureDirectory;
    this.frameLimit = frameLimit;
    this.componentLab = componentLab;
    this.storyId = storyId;
    this.darkTheme = darkTheme;
    this.robotHost = robotHost;
    this.robotPort = robotPort;
  }

  public static function fromArgs(args:Array<String>):Null<ReferenceEditorDiagnostics> {
    var directory:Null<String> = null;
    var frames = 0;
    var lab = args.indexOf("--lab") >= 0;
    var story:Null<String> = null;
    var robotEndpoint:Null<String> = null;
    for (arg in args) {
      if (arg.indexOf("--capture-dir=") == 0)
        directory = arg.substr(14);
      else if (arg.indexOf("--frames=") == 0) {
        var parsed = Std.parseInt(arg.substr(9));
        if (parsed == null || parsed <= 0) {
          Sys.println("materia: --frames requires a positive integer");
          return null;
        }
        frames = parsed;
      } else if (arg.indexOf("--story=") == 0) {
        story = arg.substr(8);
        lab = true;
      } else if (arg.indexOf("--robot=") == 0) {
        robotEndpoint = arg.substr(8);
      }
    }
    if (directory != null && directory.length == 0) {
      Sys.println("materia: --capture-dir requires a path");
      return null;
    }
    if (directory != null && frames == 0) frames = 3;
    if (story != null && story.length == 0) {
      Sys.println("materia: --story requires an ID");
      return null;
    }
    var robotHost:Null<String> = null;
    var robotPort = 0;
    if (robotEndpoint != null) {
      var separator = robotEndpoint.lastIndexOf(":");
      if (separator <= 0 || separator == robotEndpoint.length - 1) {
        Sys.println("materia: --robot requires HOST:PORT");
        return null;
      }
      robotHost = robotEndpoint.substr(0, separator);
      var parsedPort = Std.parseInt(robotEndpoint.substr(separator + 1));
      if (parsedPort == null || parsedPort <= 0 || parsedPort > 65535) {
        Sys.println("materia: --robot port must be an integer from 1 to 65535");
        return null;
      }
      robotPort = parsedPort;
    }
    return new ReferenceEditorDiagnostics(directory, frames, lab, story,
      args.indexOf("--dark") >= 0, robotHost, robotPort);
  }

  public function record(event:NativeKitEventValue):Void {
    events.push(Std.string(event));
    while (events.length > 100) events.shift();
  }

  public function enabled():Bool return captureDirectory != null;
}

private class ReferenceEditorFrameState {
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

/** NativeKit desktop owner for the reference editor's window and frame loop. */
private class ReferenceEditorHost {
  static inline var INITIAL_WIDTH:Int = 1320;
  static inline var INITIAL_HEIGHT:Int = 900;
  static inline var TARGET_FPS:Float = 60.0;

  public static function run(resetWorkspace:Bool, diagnostics:ReferenceEditorDiagnostics):Int {
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
    var editor:Null<ReferenceEditorApp> = null;
    var world:Null<RobotWorld> = null;
    var result = 0;

    try {
      var init = new InitOptions();
      init.set_api_version(NativeKit.nk_api_version());
      init.set_event_queue_capacity(256);
      if (NativeKit.nk_init(init) != Result.Ok)
        throw "NativeKit initialization failed: " + NativeKit.nk_last_error();
      initialized = true;

      var windowOptions = new WindowOptions();
      windowOptions.set_width(INITIAL_WIDTH);
      windowOptions.set_height(INITIAL_HEIGHT);
      windowOptions.set_title("Materia Reference Editor");
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
      surfaceOptions.set_width(INITIAL_WIDTH);
      surfaceOptions.set_height(INITIAL_HEIGHT);
      var createdSurface = NativeKit.nk_surface_create(new Handle(window.rawValue()), surfaceOptions);
      if (createdSurface.status != Result.Ok)
        throw "Surface creation failed: " + NativeKit.nk_last_error();
      surface = createdSurface.out_surface.borrow();

      fonts = FontCollection.create();
      fonts.addSystemFallbacks();
      renderer = Renderer.create();
      var activeTheme = Theme.light();
      if (diagnostics.darkTheme) activeTheme = Theme.dark();
      var pump = new NativeKitEvents();
      events = pump;
      var robotHost = diagnostics.robotHost;
      if (robotHost != null) {
        world = new RobotWorld();
        var remote = new RemoteRobot("warehouse/forklift-17");
        world.attach(remote);
        remote.connect(robotHost, diagnostics.robotPort, pump);
      }
      editor = new ReferenceEditorApp(fonts, null, activeTheme, world);
      if (diagnostics.componentLab) editor.enableComponentLab(diagnostics.storyId);
      if (resetWorkspace) editor.resetWorkspace();

      var state = new ReferenceEditorFrameState(INITIAL_WIDTH, INITIAL_HEIGHT);
      var layoutFrame = new LayoutFrame(INITIAL_WIDTH, INITIAL_HEIGHT);
      var frameInfo = new FrameInfo(INITIAL_WIDTH, INITIAL_HEIGHT, INITIAL_WIDTH, INITIAL_HEIGHT, 1.0);
      var previousTime = Sys.time();
      var borrowedSurface = NativeKitSurface.borrowNativeHandle(surface);
      nativeSurface = borrowedSurface;
      editor.ui.attachPlatformSurface(borrowedSurface);
      editor.ui.attachPlatformWindow(window);
      input = new NativeInputAdapter(editor.ui, new Handle(window.rawValue()),
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
        try editor.submit(layoutFrame)
        catch (error:Dynamic) throw "UI submit failed: " + Std.string(error);
        try editor.ui.render(renderer, Surface.fromNativeHandle(surface), frameInfo)
        catch (error:Dynamic) throw "UI render failed: " + Std.string(error);
        state.rendered++;
        if (diagnostics.enabled() && state.rendered >= diagnostics.frameLimit) {
          writeDiagnostics(diagnostics, editor, renderer, state);
          state.running = false;
        }
      };

      eventSubscription = pump.listen(function(value) {
        diagnostics.record(value);
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
            if (size.status != Result.Ok)
              throw "Framebuffer size query failed";
            state.framebufferWidth = size.out_width;
            state.framebufferHeight = size.out_height;
            var scale = NativeKit.nk_window_get_scale(window);
            if (scale.status != Result.Ok)
              throw "Window scale query failed";
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
        if (state.callbackError != null)
          throw state.callbackError;
        if (state.running && !hadEvent)
          pump.wait(1.0 / TARGET_FPS);
      }
    } catch (error:Dynamic) {
      Sys.println("materia: " + Std.string(error));
      var stack = haxe.CallStack.toString(haxe.CallStack.exceptionStack());
      if (stack.length > 0) Sys.println(stack);
      result = 1;
    }

    if (input != null) input.detach();
    if (frameSubscription != null) frameSubscription.dispose();
    if (world != null) world.close();
    if (editor != null) editor.dispose();
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

  static function writeDiagnostics(diagnostics:ReferenceEditorDiagnostics,
      editor:ReferenceEditorApp, renderer:Renderer, state:ReferenceEditorFrameState):Void {
    var directory:String = cast diagnostics.captureDirectory;
    createDirectories(directory);
    File.saveContent(directory + "/ui-tree.txt", editor.ui.dumpTree() + "\n");
    File.saveContent(directory + "/layout.json",
      Json.stringify(editor.ui.inspect(), null, "  ") + "\n");
    File.saveContent(directory + "/app-state.json",
      Json.stringify(editor.diagnosticState(), null, "  ") + "\n");
    var metrics = editor.ui.frameMetrics;
    var stats = renderer.stats();
    var frameData:Dynamic = {
      logicalWidth: state.logicalWidth,
      logicalHeight: state.logicalHeight,
      framebufferWidth: state.framebufferWidth,
      framebufferHeight: state.framebufferHeight,
      pixelScale: state.scale,
      renderedFrames: state.rendered,
      ui: metrics == null ? null : {
        frameNumber: metrics.frameNumber,
        nodeCount: metrics.nodeCount,
        submitSeconds: metrics.submitSeconds,
        renderSeconds: metrics.renderSeconds,
        totalSeconds: metrics.totalSeconds,
        paintedNodes: metrics.paintedNodes,
        paintSkippedNodes: metrics.paintSkippedNodes,
        emptyPaintNodes: metrics.emptyPaintNodes,
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
    var eventLines = [for (event in diagnostics.events) Json.stringify({event: event})];
    File.saveContent(directory + "/events.jsonl", eventLines.join("\n") + "\n");

    var screenshot = directory + "/frame.png";
    var screenshotResult = Sys.command("import",
      ["-window", "Materia Reference Editor", screenshot]);
    if (screenshotResult != 0)
      File.saveContent(directory + "/screenshot-error.txt",
        "ImageMagick import could not capture the application window (exit " +
        screenshotResult + ").\n");
    Sys.println("materia: diagnostics captured in " + directory);
  }

  static function createDirectories(path:String):Void {
    if (path == null || path.length == 0 || FileSystem.exists(path)) return;
    var slash = path.lastIndexOf("/");
    if (slash > 0) createDirectories(path.substr(0, slash));
    if (!FileSystem.exists(path)) FileSystem.createDirectory(path);
  }

}

/** Shared/app integration object passed to a platform frame loop. */
class ReferenceEditorApp {
  public static inline var WORKSPACE_KEY:String = "reference-editor";

  public final ui:UiContext;
  public final commands:CommandRegistry;
  public final workspace:DockWorkspaceModel;
  public final workspacePath:String;
  public final world:Null<RobotWorld>;

  final storage:FileDockWorkspacePersistence;
  final treeModel:ReferenceSceneTreeModel;
  final viewportCamera:ViewportCamera;
  final viewportContent:ReferenceViewportContent;
  final telemetry:PlotModel;
  final properties:Array<PropertyDescriptor>;
  final logLines:Array<String>;
  var selectedNode:String;
  var nodeVisible:Bool;
  var nodeMass:Float;
  var nodeMode:String;
  var gridVisible:Bool;
  var paletteVisible:Bool;
  var contextMenuVisible:Bool;
  var contextMenuX:Float;
  var contextMenuY:Float;
  var componentLab:Null<ComponentLab>;

  public function new(? fonts:FontCollection, ? workspaceFile:String, ?theme:Theme,
      ?world:RobotWorld) {
    ui = new UiContext(null, fonts, theme == null ? Theme.light() : theme);
    commands = ui.commands;
    this.world = world;
    workspacePath = workspaceFile == null || workspaceFile.length == 0 ? defaultWorkspacePath() : workspaceFile;
    storage = new FileDockWorkspacePersistence(workspacePath);
    treeModel = new ReferenceSceneTreeModel();
    viewportCamera = new ViewportCamera();
    viewportContent = new ReferenceViewportContent();
    telemetry = makeTelemetry();
    logLines = ["Console connected", "GPU viewport ready", "Workspace restored"];
    selectedNode = "body";
    nodeVisible = true;
    nodeMass = 2.5;
    nodeMode = "solid";
    gridVisible = true;
    paletteVisible = false;
    contextMenuVisible = false;
    contextMenuX = 0.0;
    contextMenuY = 0.0;
    componentLab = null;
    updateCommandContext();

    properties = makeProperties();
    workspace = makeWorkspace();
    workspace.restoreFromOrDefault(storage, WORKSPACE_KEY);
    workspace.listen(function() saveWorkspace());
    installCommands();
  }

  /** Build the shared view tree for one host frame. */
  public function view():View {
    if (componentLab != null) return componentLab.view(ui);
    var workspaceView = new DockWorkspace("reference-workspace", workspace);
    var layers:Array<StackChild> = [new StackChild(
      "workspace",
      workspaceView,
      0.0,
      0.0,
      0,
      LayoutAxis.grow(),
      LayoutAxis.grow()
    )];
    if (contextMenuVisible) {
      var menu = new CommandMenu("scene-context-menu", [
        "scene.frame-selected",
        "scene.toggle-grid",
        "context.delete-selection",
        "workspace.reset"
      ], contextMenuX, contextMenuY, commands, ui.commandContext, function() {
        contextMenuVisible = false;
        commands.refresh();
      }
      );
      layers.push(new StackChild("context-menu", menu, 0.0, 0.0, 20));
    }
    if (paletteVisible) {
      var palette = new CommandPalette("reference-command-palette",
        commands, ui.commandContext, 260.0, 96.0, "", function() {
        paletteVisible = false;
        commands.refresh();
      }, function(_) {
        log("Command executed from palette");
        commands.refresh();
      }
      );
      layers.push(new StackChild("command-palette", palette, 0.0, 0.0, 30));
    }

    var shellStyle = fillStyle();
    shellStyle.background = Color.rgba(0.93, 0.95, 0.98, 1.0);
    return new AppShell("reference-editor-shell", new Stack("overlay-host", layers), topBar(), null, null, shellStyle);
  }

  /** Convenience entry point for a NativeKit host's layout phase. */
  public function submit(frame:LayoutFrame):RenderNode return ui.submit(view(), frame);

  public function dispose():Void ui.dispose();

  public function enableComponentLab(?storyId:String):Void {
    componentLab = new ComponentLab(storyId);
    commands.refresh();
  }

  /** Machine-readable application state paired with diagnostic frame captures. */
  public function diagnosticState():Dynamic return componentLab == null ? {
    selectedNode: selectedNode,
    selection: ui.commandContext.selection.copy(),
    nodeVisible: nodeVisible,
    nodeMass: nodeMass,
    nodeMode: nodeMode,
    gridVisible: gridVisible,
    paletteVisible: paletteVisible,
    contextMenuVisible: contextMenuVisible,
    camera: {
      panX: viewportCamera.panX,
      panY: viewportCamera.panY,
      zoom: viewportCamera.zoom
    },
    robot: robotDiagnosticState(),
    panels: workspace.panelIds(),
    workspace: Json.parse(workspace.snapshotJson()),
    recentLog: logLines.copy()
  } : {
    mode: "component-lab",
    lab: componentLab.diagnosticState()
  };

  public function resetWorkspace():Void {
    workspace.reset();
    saveWorkspace();
    log("Workspace reset");
    commands.refresh();
  }

  function robotDiagnosticState():Dynamic {
    var currentWorld = world;
    if (currentWorld == null)
      return null;
    var snapshot = currentWorld.snapshot();
    var robots:Array<Dynamic> = [];
    for (id in snapshot.robotIds()) {
      var instance = currentWorld.robots.get(id);
      var state = snapshot.robot(id);
      var fault = instance == null ? null : instance.fault();
      robots.push({
        id: id,
        status: instance == null ? "missing" : Std.string(instance.status()),
        state: state == null ? null : {
          robotId: state.id,
          sequence: Std.string(state.sourceSequence),
          timestampNs: Std.string(state.timestampNs),
          q: state.positions.copy(),
          dq: state.velocities.copy(),
          effort: state.efforts.copy(),
          mode: state.mode,
          fault: state.faultCode
        },
        fault: fault == null ? null : {
          robotId: fault.id,
          code: fault.code,
          message: fault.message,
          fatal: fault.fatal
        }
      });
    }
    return {
      status: Std.string(currentWorld.status()),
      sequence: snapshot.sequence,
      topologyRevision: snapshot.topologyRevision,
      timestampNs: Std.string(snapshot.timestampNs),
      robots: robots
    };
  }

  function topBar():View {
    var barStyle = fillStyle();
    barStyle.height = LayoutAxis.fixed(48.0);
    barStyle.direction = LayoutDirection.LeftToRight;
    barStyle.childGap = 14.0;
    barStyle.padding = new Insets(14.0, 8.0, 14.0, 8.0);
    barStyle.background = Color.rgba(0.98, 0.99, 1.0, 1.0);

    var titleStyle = new LayoutStyle();
    titleStyle.width = LayoutAxis.fixed(190.0);
    var hintStyle = new LayoutStyle();
    hintStyle.width = LayoutAxis.grow();
    var robotLabel = world == null ? "World: offline"
      : "World: " + Std.string(world.status());
    var hint = new Text("Reference Editor  ·  Ctrl+K command palette  ·  " + robotLabel,
      hintStyle, Color.rgba(0.32, 0.38, 0.47, 1.0));
    return new Row(
      "editor-toolbar-row",
      [
        new KeyedView(
          "title",
          new Text(
            "MATERIA / EDITOR",
            titleStyle,
            Color.rgba(
              0.10,
              0.14,
              0.21,
              1.0
            )
          )
        ),
        new KeyedView(
          "commands",
          new Toolbar(
            "editor-toolbar",
            [
              "editor.save",
              "scene.frame-selected",
              "editor.command-palette",
              "workspace.reset"
            ],
            commands
          )
        ),
        new KeyedView(
          "hint",
          hint
        )
      ],
      barStyle
    );
  }

  function makeWorkspace():DockWorkspaceModel {
    var result = new DockWorkspaceModel();
    result.register(new DockPanelDescriptor("hierarchy", "Hierarchy", function(_) {
      return hierarchyPanel();
    }, false));
    result.register(new DockPanelDescriptor("viewport", "Viewport", function(_) {
      return viewportPanel();
    }, false));
    result.register(new DockPanelDescriptor("inspector", "Inspector", function(_) {
      return inspectorPanel();
    }, false));
    result.register(new DockPanelDescriptor("console", "Console", function(_) {
      return consolePanel();
    }
    ));
    result.register(new DockPanelDescriptor("telemetry", "Telemetry", function(_) {
      return telemetryPanel();
    }
    ));

    var centerTabs = DockNode.Tabs(["viewport", "console", "telemetry"], "viewport");
    var editorArea = DockNode.Split(DockSplitAxis.Horizontal, 0.76, centerTabs, DockNode.Panel("inspector"));
    result.setDefaultLayout(DockNode.Split(DockSplitAxis.Horizontal, 0.22, DockNode.Panel("hierarchy"), editorArea));
    return result;
  }

  function hierarchyPanel():View {
    var treeStyle = fillStyle();
    treeStyle.padding = new Insets(8.0, 8.0, 8.0, 8.0);
    treeStyle.background = Color.rgba(0.98, 0.99, 1.0, 1.0);
    var treeViewport = fillStyle();
    var tree = new TreeView("scene-hierarchy",
      treeModel, treeViewport, null, 420.0, selectedNode, ["scene", "body"], function(id) {
      selectedNode = id;
      log("Selected " + id);
      updateCommandContext();
      commands.refresh();
    }, function(id) {
      selectedNode = id;
      log("Activated " + id);
      updateCommandContext();
    }, null, null);
    return new Column(
      "hierarchy-panel",
      [
        new KeyedView("heading", sectionHeading("SCENE HIERARCHY")),
        new KeyedView(
          "tree",
          tree
        )
      ],
      treeStyle
    );
  }

  function viewportPanel():View {
    var viewportStyle = fillStyle();
    viewportStyle.background = Color.rgba(0.025, 0.035, 0.055, 1.0);
    var viewport = new GpuViewport(
      "scene-gpu-viewport",
      viewportContent,
      viewportCamera,
      viewportStyle,
      "Scene GPU viewport"
    );
    viewport.setAppearance(Color.rgba(0.025, 0.035, 0.055, 1.0), Color.rgba(0.16, 0.24, 0.36, 0.75), 32.0, gridVisible);
    viewport.on(UiEventKind.PointerDown, function(event:UiEvent) {
      if (event.button != 2) return;
      contextMenuX = event.x;
      contextMenuY = event.y;
      contextMenuVisible = true;
      event.preventDefault();
      event.stopPropagation();
      commands.refresh();
    }
    );
    return viewport;
  }

  function inspectorPanel():View {
    var style = fillStyle();
    style.padding = new Insets(10.0, 10.0, 10.0, 10.0);
    style.background = Color.rgba(0.98, 0.99, 1.0, 1.0);
    var inspector = new PropertyInspector(
      "scene-inspector",
      properties,
      style,
      null,
      [
        new PropertyInspectorSection(
          "transform",
          "Transform",
          [
            properties[0],
            properties[1]
          ]
        ),
        new PropertyInspectorSection(
          "rendering",
          "Rendering",
          [
            properties[2],
            properties[3],
            properties[4]
          ]
        )
      ],
      null,
      "Scene inspector"
    );
    return new Column(
      "inspector-panel",
      [
        new KeyedView("heading", sectionHeading("INSPECTOR")),
        new KeyedView(
          "properties",
          inspector
        )
      ],
      style
    );
  }

  function consolePanel():View {
    var style = fillStyle();
    style.padding = new Insets(12.0, 12.0, 12.0, 12.0);
    style.background = Color.rgba(0.95, 0.97, 0.99, 1.0);
    var rows:Array<KeyedView> = [];
    for (index in 0...logLines.length) rows.push(new KeyedView(
      "log:" + index,
      new Text(
        "> " + logLines[index],
        null,
        Color.rgba(
          0.22,
          0.38,
          0.44,
          1.0
        )
      )
    ));
    return new Column(
      "console-panel",
      [
        new KeyedView("heading", sectionHeading("CONSOLE")),
        new KeyedView(
          "logs",
          new Column(
            "console-lines",
            rows,
            fillStyle()
          )
        )
      ],
      style
    );
  }

  function telemetryPanel():View {
    var style = fillStyle();
    style.padding = new Insets(12.0, 12.0, 12.0, 12.0);
    style.background = Color.rgba(0.96, 0.97, 0.99, 1.0);
    var plotStyle = fillStyle();
    plotStyle.height = LayoutAxis.grow();
    var plot = new PlotView("frame-telemetry", telemetry, plotStyle, "Frame telemetry");
    return new Column(
      "telemetry-panel",
      [
        new KeyedView("heading", sectionHeading("TELEMETRY")),
        new KeyedView(
          "plot",
          plot
        ),
        new KeyedView(
          "caption",
          new Text("Frame time · GPU submission · layout cost")
        )
      ],
      style
    );
  }

  function sectionHeading(label:String):Text {
    return new Text(label, null, Color.rgba(0.24, 0.39, 0.61, 1.0));
  }

  function installCommands():Void {
    workspace.installCommands(commands, "workspace");
    commands.register(new Command("editor.save", "Save workspace", function() {
      saveWorkspace();
      log("Workspace saved");
    }, new Shortcut(UiKey.S, UiModifier.Control)));
    commands.register(new Command("editor.command-palette", "Open command palette", function() {
      paletteVisible = true;
      contextMenuVisible = false;
      commands.refresh();
    }, new Shortcut(UiKey.K, UiModifier.Control)));
    commands.register(new Command("scene.frame-selected", "Frame selected", function() {
      viewportCamera.setPan(0.0, 0.0);
      viewportCamera.setZoom(1.0);
      log("Framed " + selectedNode);
    }
    ));
    commands.register(new Command("scene.toggle-grid", "Toggle grid", function() {
      gridVisible = !gridVisible;
      log(gridVisible ? "Grid enabled" : "Grid disabled");
    }, null, null, function() return gridVisible));
    commands.register(Command.contextual("context.delete-selection", "Delete selected node", function(context) {
      var id = context.selection.length == 0 ? selectedNode : context.selection[0];
      log("Delete requested for " + id);
      return CommandResult.executed();
    }, null, function(context) return context.hasSelection && selectedNode != "scene"));
  }

  function updateCommandContext():Void {
    ui.setCommandContext(new CommandContext(null, [selectedNode], "scene-viewport", null, "reference-editor"));
  }

  function makeProperties():Array < PropertyDescriptor > {
    var xSettings = new PropertyDescriptorOptions();
    xSettings.category = "Transform";
    xSettings.unit = "m";
    xSettings.step = 0.1;
    var ySettings = new PropertyDescriptorOptions();
    ySettings.category = "Transform";
    ySettings.unit = "m";
    ySettings.step = 0.1;
    var visibleSettings = new PropertyDescriptorOptions();
    visibleSettings.category = "Rendering";
    var massSettings = new PropertyDescriptorOptions();
    massSettings.category = "Physics";
    massSettings.minimum = 0.0;
    massSettings.maximum = 100.0;
    massSettings.step = 0.1;
    massSettings.unit = "kg";
    var modeSettings = new PropertyDescriptorOptions();
    modeSettings.category = "Rendering";
    modeSettings.options = [
      new nativekit.ui.core.PropertyOption("solid", "Solid"),
      new nativekit.ui.core.PropertyOption(
        "wire",
        "Wire"
      )
    ];
    return [new PropertyDescriptor("position-x",
      "Position X", PropertyType.Float, function(_) return PropertyValue.Float(viewportCamera.panX), function(
        _,
        value
      ) {
      switch (value) {
        case PropertyValue.Float(next):
          viewportCamera.setPan(next, viewportCamera.panY);
        case PropertyValue.Int(next):
          viewportCamera.setPan(next, viewportCamera.panY);
        default:
          throw "Position X requires a number";
      }
    }, xSettings), new PropertyDescriptor("position-y",
      "Position Y", PropertyType.Float, function(_) return PropertyValue.Float(viewportCamera.panY), function(
        _,
        value
      ) {
      switch (value) {
        case PropertyValue.Float(next):
          viewportCamera.setPan(viewportCamera.panX, next);
        case PropertyValue.Int(next):
          viewportCamera.setPan(viewportCamera.panX, next);
        default:
          throw "Position Y requires a number";
      }
    }, ySettings), new PropertyDescriptor("mass",
      "Mass", PropertyType.Float, function(_) return PropertyValue.Float(nodeMass), function(
        _,
        value
      ) {
      switch (value) {
        case PropertyValue.Float(next):
          nodeMass = next;
        case PropertyValue.Int(next):
          nodeMass = next;
        default:
          throw "Mass requires a number";
      }
    }, massSettings), new PropertyDescriptor("visible",
      "Visible", PropertyType.Bool, function(_) return PropertyValue.Bool(nodeVisible), function(
        _,
        value
      ) {
      switch (value) {
        case PropertyValue.Bool(next):
          nodeVisible = next;
        default:
          throw "Visible requires a boolean";
      }
    }, visibleSettings), new PropertyDescriptor("mode",
      "Mode", PropertyType.Enum, function(_) return PropertyValue.Enum(nodeMode), function(
        _,
        value
      ) {
      switch (value) {
        case PropertyValue.Enum(next):
          nodeMode = next;
        default:
          throw "Mode requires an enum value";
      }
    }, modeSettings)];
  }

  function makeTelemetry():PlotModel {
    var model = new PlotModel();
    var frameTime = new PlotSeries("frame-time", "Frame time", Color.rgba(0.28, 0.75, 0.98, 1.0), 2.0);
    var gpuTime = new PlotSeries("gpu-time", "GPU submission", Color.rgba(0.78, 0.45, 0.98, 1.0), 2.0);
    for (index in 0...64) {
      frameTime.add(new PlotPoint(index, 10.0 + Math.sin(index * 0.24) * 2.2));
      gpuTime.add(new PlotPoint(index, 4.0 + Math.cos(index * 0.19) * 1.2));
    }
    model.addSeries(frameTime);
    model.addSeries(gpuTime);
    return model;
  }

  function saveWorkspace():Void {
    try workspace.saveTo(storage, WORKSPACE_KEY);
    catch (error:Dynamic) log("Workspace save failed: " + Std.string(error));
  }

  function log(message:String):Void {
    if (logLines == null) return;
    logLines.push(message);
    while (logLines.length > 8) logLines.shift();
  }

  static function fillStyle():LayoutStyle {
    var style = new LayoutStyle();
    style.width = LayoutAxis.grow();
    style.height = LayoutAxis.grow();
    return style;
  }

  static function defaultWorkspacePath():String {
    var configured = Sys.getEnv("REFERENCE_EDITOR_WORKSPACE");
    return configured == null || configured.length == 0 ? "build/reference-editor-workspace.json" : configured;
  }
}

private class ReferenceSceneTreeModel implements TreeViewModel {
  public function new() {
  }

  public function rootCount():Int return 1;

  public function rootRange(start:Int, count:Int):Array < TreeRootMetadata > {
    return start <= 0 && count > 0 ?[new TreeRootMetadata("scene", true)] :[];
  }

  public function rootKeyAt(index:Int):String return "scene";

  public function childCount(parentKey:String):Int {
    return switch (parentKey) {
      case "scene":
        3;
      case "body":
        2;
      default:
        0;
    };
  }

  public function childKeyAt(parentKey:String, index:Int):String {
    return switch (parentKey) {
      case "scene":
        ["camera", "body", "key-light"][index];
      case "body":
        ["mesh", "material"][index];
      default:
        parentKey + ":child:" + index;
    };
  }

  public function initiallyExpanded(key:String):Bool return key == "scene" || key == "body";

  public function estimatedExtent():Float return 28.0;

  public function extentIsUniform():Bool return true;

  public function extentAt(key:String):Float return 28.0;

  public function buildItem(key:String):View {
    var labels:Map<String, String> = [
      "scene" = > "Scene",
      "camera" = > "Camera",
      "body" = > "Body",
      "mesh" = > "Mesh",
      "material" = > "Material",
      "key-light" = > "Key Light"
    ];
    return new Text(labels.exists(key) ? labels.get(key) : key);
  }

  public function revision():Int return 1;
}

private class ReferenceViewportContent implements ViewportContent {
  public function new() {
  }

  public function width():Float return 960.0;

  public function height():Float return 640.0;

  public function revision():Int return 1;

  public function paint(canvas:Canvas, destination:Rect):Void {
    canvas.fillRect(destination, Color.rgba(0.06, 0.09, 0.14, 1.0));
    for (x in 0...16) canvas.fillRect(new Rect(x * 64.0, 0.0, 1.0, 640.0), Color.rgba(0.13, 0.19, 0.28, 0.55));
    for (y in 0...11) canvas.fillRect(new Rect(0.0, y * 64.0, 960.0, 1.0), Color.rgba(0.13, 0.19, 0.28, 0.55));
    canvas.fillRect(new Rect(284.0, 166.0, 392.0, 250.0), Color.rgba(0.18, 0.38, 0.58, 0.92));
    canvas.fillRect(new Rect(316.0, 136.0, 328.0, 30.0), Color.rgba(0.28, 0.58, 0.84, 0.9));
    canvas.fillRect(new Rect(340.0, 416.0, 280.0, 28.0), Color.rgba(0.12, 0.22, 0.34, 1.0));
  }
}

private class FileDockWorkspacePersistence implements DockWorkspacePersistence {
  final path:String;

  public function new(path:String) this.path = path;

  public function load(key:String):Null < String > {
    return FileSystem.exists(path) ? File.getContent(path) : null;
  }

  public function save(key:String, value:String):Void {
    var slash = path.lastIndexOf("/");
    var backslash = path.lastIndexOf("\\");
    var separator = slash > backslash ? slash : backslash;
    var directory = separator < 0 ? "" : path.substr(0, separator);
    if (directory != null
      && directory.length > 0 && !FileSystem.exists(directory)) FileSystem.createDirectory(directory);
    File.saveContent(path, value);
  }
}
