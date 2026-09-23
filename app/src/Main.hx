package app;

import Canvas;
import Color;
import LayoutAxis;
import LayoutAlignmentY;
import LayoutDirection;
import LayoutFrame;
import LayoutStyle;
import LayoutWrapMode;
import Insets;
import Rect;
import PathBuilder;
import Point;
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
import nativekit.ui.core.TextStyleOverride;
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
import nativekit.ui.widgets.TreeRootMetadata;
import nativekit.ui.widgets.TreeView;
import nativekit.ui.widgets.TreeViewModel;
import FontCollection;
import cadkit.parametric.ParametricError;
import robotkit.world.RemoteRobot;
import robotkit.world.RobotWorld;
import nativekit.ui.lab.ComponentLab;
import nativekit.ui.theme.Theme;
import nativekit.ui.icons.IconName;
import nativekit.ui.host.DesktopUiApplication;
import nativekit.ui.host.DesktopUiHost;
import nativekit.ui.host.DesktopUiHostOptions;
import nativekit.ui.host.DesktopUiHostContext;
import nativekit.ui.widgets.Dialog;

/**
	Small executable driver for the shared reference-editor shell.

	The reusable UIKit desktop host owns the window, rendering, input, and
	diagnostics lifecycle; this executable only configures the editor.
*/
class Main {
  public static function main():Int {
    var args = Sys.args();
    for (arg in args)
      if (arg != "--reset-workspace" && arg != "--snapshot" &&
          arg != "--lab" && arg != "--dark" && arg != "--perspective" &&
          arg.indexOf("--story=") != 0 &&
          arg.indexOf("--width=") != 0 && arg.indexOf("--height=") != 0 &&
          arg.indexOf("--capture-dir=") != 0 && arg.indexOf("--frames=") != 0 &&
          arg.indexOf("--capture-seconds=") != 0 &&
          arg.indexOf("--robot=") != 0 && arg.indexOf("--setup-script=") != 0) {
        Sys.println("Usage: materia [--reset-workspace] [--snapshot] " +
          "[--lab] [--dark] [--perspective] [--story=ID] [--width=PX] [--height=PX] " +
          "[--capture-dir=PATH] [--frames=N|--capture-seconds=N] " +
          "[--robot=HOST:PORT] [--setup-script=REFERENCE]");
        return 2;
      }

    if (args.indexOf("--snapshot") >= 0) {
      var editor = new ReferenceEditorApp();
      if (args.indexOf("--reset-workspace") >= 0) editor.resetWorkspace();
      Sys.println(editor.workspace.snapshotJson());
      editor.dispose();
      return 0;
    }
    var diagnostics = ReferenceEditorLaunchOptions.fromArgs(args);
    if (diagnostics == null) return 2;
    var host = new DesktopUiHostOptions();
    host.title = "Materia Reference Editor";
    host.width = diagnostics.windowWidth;
    host.height = diagnostics.windowHeight;
    host.captureDirectory = diagnostics.captureDirectory;
    host.frameLimit = diagnostics.frameLimit;
    host.captureSeconds = diagnostics.captureSeconds;
    var activeEditor:Null<ReferenceEditorApp> = null;
    host.continuousFrames = function() return activeEditor != null &&
      (activeEditor.simulation.isRunning() || diagnostics.robotHost != null);
    return DesktopUiHost.run(host, function(context) {
      var activeTheme = Theme.light();
      if (diagnostics.darkTheme) activeTheme = Theme.dark();
      var world:Null<RobotWorld> = null;
      var robotHost = diagnostics.robotHost;
      if (robotHost != null) {
        world = new RobotWorld();
        var remote = new RemoteRobot("warehouse/forklift-17");
        world.attach(remote);
        remote.connect(robotHost, diagnostics.robotPort, context.events);
      }
      var editor = new ReferenceEditorApp(context.fonts, null, activeTheme, world, context,
        diagnostics.setupScript);
      activeEditor = editor;
      if (diagnostics.componentLab) editor.enableComponentLab(diagnostics.storyId);
      if (args.indexOf("--reset-workspace") >= 0) editor.resetWorkspace();
      if (args.indexOf("--perspective") >= 0) editor.workspace.activate("perspective");
      return editor;
    });
  }
}

private class ReferenceEditorLaunchOptions {
  public final captureDirectory:Null<String>;
  public final frameLimit:Int;
  public final captureSeconds:Float;
  public final componentLab:Bool;
  public final storyId:Null<String>;
  public final darkTheme:Bool;
  public final robotHost:Null<String>;
  public final robotPort:Int;
  public final setupScript:Null<String>;
  public final windowWidth:Int;
  public final windowHeight:Int;
  public function new(captureDirectory:Null<String>, frameLimit:Int, captureSeconds:Float,
      componentLab:Bool, storyId:Null<String>, darkTheme:Bool,
      robotHost:Null<String>, robotPort:Int,setupScript:Null<String>,
      windowWidth:Int, windowHeight:Int) {
    this.captureDirectory = captureDirectory;
    this.frameLimit = frameLimit;
    this.captureSeconds = captureSeconds;
    this.componentLab = componentLab;
    this.storyId = storyId;
    this.darkTheme = darkTheme;
    this.robotHost = robotHost;
    this.robotPort = robotPort;
    this.setupScript=setupScript;
    this.windowWidth = windowWidth;
    this.windowHeight = windowHeight;
  }

  public static function fromArgs(args:Array<String>):Null<ReferenceEditorLaunchOptions> {
    var directory:Null<String> = null;
    var frames = 0;
    var captureSeconds = 0.0;
    var lab = args.indexOf("--lab") >= 0;
    var story:Null<String> = null;
    var robotEndpoint:Null<String> = null;
    var setupScript:Null<String> = null;
    var windowWidth = 1320;
    var windowHeight = 900;
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
      } else if (arg.indexOf("--capture-seconds=") == 0) {
        var parsed = Std.parseFloat(arg.substr(18));
        if (!Math.isFinite(parsed) || parsed <= 0.0) {
          Sys.println("materia: --capture-seconds requires a positive finite number");
          return null;
        }
        captureSeconds = parsed;
      } else if (arg.indexOf("--story=") == 0) {
        story = arg.substr(8);
        lab = true;
      } else if (arg.indexOf("--width=") == 0) {
        var parsed = Std.parseInt(arg.substr(8));
        if (parsed == null || parsed < 480) {
          Sys.println("materia: --width requires at least 480 pixels");
          return null;
        }
        windowWidth = parsed;
      } else if (arg.indexOf("--height=") == 0) {
        var parsed = Std.parseInt(arg.substr(9));
        if (parsed == null || parsed < 360) {
          Sys.println("materia: --height requires at least 360 pixels");
          return null;
        }
        windowHeight = parsed;
      } else if (arg.indexOf("--robot=") == 0) {
        robotEndpoint = arg.substr(8);
      } else if(arg.indexOf("--setup-script=")==0) {
        setupScript=arg.substr(15);
      }
    }
    if (directory != null && directory.length == 0) {
      Sys.println("materia: --capture-dir requires a path");
      return null;
    }
    if(setupScript!=null&&StringTools.trim(setupScript).length==0){
      Sys.println("materia: --setup-script requires a registered reference");return null;
    }
    if (captureSeconds > 0.0 && (directory == null || frames > 0)) {
      Sys.println("materia: --capture-seconds requires --capture-dir and excludes --frames");
      return null;
    }
    if (directory != null && frames == 0 && captureSeconds == 0.0) frames = 3;
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
    return new ReferenceEditorLaunchOptions(directory, frames, captureSeconds, lab, story,
      args.indexOf("--dark") >= 0, robotHost, robotPort,setupScript,
      windowWidth, windowHeight);
  }
}

/** Shared/app integration object passed to a platform frame loop. */
class ReferenceEditorApp implements DesktopUiApplication {
  public static inline var WORKSPACE_KEY:String = "reference-editor";

  public final ui:UiContext;
  final appearance:EditorAppearance;
  public final commands:CommandRegistry;
  public final workspace:DockWorkspaceModel;
  public final workspacePath:String;
  public final world:RobotWorld;
  public final simulation:ApplicationSimulation;

  final storage:FileDockWorkspacePersistence;
  public final session:SceneDocumentSession;
  public final documents:SceneDocumentController;
  public var scene(get, never):EditorScene;
  function get_scene():EditorScene return session.scene;
  final files:Null<SceneFileDialogs>;
  var sceneGeneration:Int = 0;
  var treeModel:EditorSceneTree;
  final viewportCamera:ViewportCamera;
  var viewportContent:EditorSceneViewport;
  final telemetry:PlotModel;
  final logLines:Array<String>;
  var gridVisible:Bool;
  var gridSnapEnabled:Bool;
  var gridSpacing:Float;
  var paletteVisible:Bool;
  var toolbarMenuVisible:Bool;
  var viewportWidth:Float = 1320.0;
  var contextMenuVisible:Bool;
  var contextMenuX:Float;
  var contextMenuY:Float;
  var componentLab:Null<ComponentLab>;
  var sceneViewport:Null<GpuViewport> = null;
  var perspectiveViewport:Null<EditorPerspectiveViewport> = null;
  final hostContext:Null<DesktopUiHostContext>;
  var dragPointer:Null<Int> = null;
  var dragPointerX:Float = 0.0;
  var dragPointerY:Float = 0.0;
  var sceneInspector:Null<PropertyInspector> = null;
  var inspectorSelectionRevision:Int = -1;
  public var sensors(get, never):SensorConfiguration;
  function get_sensors():SensorConfiguration return session.sensors;

  public function new(? fonts:FontCollection, ? workspaceFile:String, ?theme:Theme,
      ?world:RobotWorld, ?hostContext:DesktopUiHostContext,?setupScript:String) {
    this.hostContext = hostContext;
    appearance = new EditorAppearance(theme);
    ui = new UiContext(null, fonts, appearance.theme);
    commands = ui.commands;
    this.world = world == null ? new RobotWorld() : world;
    simulation = new ApplicationSimulation(this.world,ApplicationSimulation.MUJOCO);
    workspacePath = workspaceFile == null || workspaceFile.length == 0 ? defaultWorkspacePath() : workspaceFile;
    storage = new FileDockWorkspacePersistence(workspacePath);
    session = new SceneDocumentSession();
    session.beforeReplace=simulation.clear;
    if(setupScript!=null){var scripted=session.openScript(setupScript);
      simulation.setBackend(scripted.backend);simulation.setTimestep(scripted.timestep);}
    files = hostContext == null ? null : new SceneFileDialogs(hostContext);
    documents = new SceneDocumentController(session, function(save, path, complete) {
      var chooser = files;
      if (chooser == null) complete(null, "File dialogs require the desktop host");
      else chooser.choose(save, path, complete);
    }, documentChanged, commitActiveDrag, cancelActiveDrag);
    if (hostContext != null) hostContext.onCloseRequested = function(close) documents.requestClose(close);
    treeModel = new EditorSceneTree(scene);
    viewportCamera = new ViewportCamera();
    viewportContent = new EditorSceneViewport(scene);
    if (hostContext != null) {
      perspectiveViewport = new EditorPerspectiveViewport("scene-perspective", scene,
        hostContext);
    }
    telemetry = makeTelemetry();
    logLines = ["Scene ready: two editable objects", "Select a box; edit position or visibility", "Middle-drag to pan; scroll to zoom"];
    gridVisible = true;
    gridSnapEnabled = false;
    gridSpacing = EditorSceneViewport.GRID_STEP;
    paletteVisible = false;
    toolbarMenuVisible = false;
    contextMenuVisible = false;
    contextMenuX = 0.0;
    contextMenuY = 0.0;
    componentLab = null;
    updateCommandContext();

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
        "scene.create",
        "scene.create-plate",
        "scene.add-face-hole",
        "scene.duplicate",
        "scene.delete",
        "scene.frame-selected",
        "scene.toggle-grid",
        "scene.toggle-grid-snap",
        "scene.grid-spacing-0.1",
        "scene.grid-spacing-0.2",
        "scene.grid-spacing-0.5",
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
    if (toolbarMenuVisible) {
      var toolbarMenu = new CommandMenu("editor-more-menu", [
        "editor.save-as", "scene.export-step", "editor.undo", "editor.redo",
        "scene.frame-selected", "scene.reset-perspective", "scene.show-perspective",
        "scene.toggle-grid", "editor.command-palette", "workspace.reset"
      ], Math.max(8.0, viewportWidth - 228.0), 44.0, commands, ui.commandContext,
        function() { toolbarMenuVisible = false; commands.refresh(); },
        function(_) { toolbarMenuVisible = false; commands.refresh(); });
      layers.push(new StackChild("editor-more-menu", toolbarMenu, 0.0, 0.0, 25));
    }

    var documentDialog = makeDocumentDialog();
    if (documentDialog != null) layers.push(new StackChild("document-dialog", documentDialog,
      0.0, 0.0, 100, LayoutAxis.grow(), LayoutAxis.grow()));
    var shellStyle = fillStyle();
    shellStyle.background = appearance.canvas;
    return new AppShell("reference-editor-shell", new Stack("overlay-host", layers), topBar(), null, null, shellStyle);
  }

  /** Convenience entry point for a NativeKit host's layout phase. */
  public function submit(frame:LayoutFrame):RenderNode {
    viewportWidth = frame.width;
    return ui.submit(view(), frame);
  }

  public function context():UiContext return ui;

  public function dispose():Void {
    simulation.dispose();
    world.close();
    if (files != null) files.dispose();
    if (perspectiveViewport != null) perspectiveViewport.dispose();
    session.dispose();
    ui.dispose();
  }

  public function enableComponentLab(?storyId:String):Void {
    componentLab = new ComponentLab(storyId);
    commands.refresh();
  }

  /** Machine-readable application state paired with diagnostic frame captures. */
  public function diagnosticState():Dynamic return componentLab == null ? {
    selectedNode: scene.selectedId,
    scene: scene.diagnosticState(),
    document: {path: session.path, label: session.label(), dirty: session.isDirty(),
      confirmation: documents.needsConfirmation(), choosing: documents.choosing, error: documents.error},
    selection: ui.commandContext.selection.copy(),
    gridVisible: gridVisible,
    gridSnapEnabled: gridSnapEnabled,
    gridSpacing: gridSpacing,
    paletteVisible: paletteVisible,
    contextMenuVisible: contextMenuVisible,
    perspective: perspectiveViewport == null ? null : perspectiveViewport.diagnosticState(),
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
      var instance = currentWorld.robot(id);
      var state = snapshot.robot(id);
      var fault = instance == null ? null : instance.fault();
      robots.push({
        id: id,
        status: instance == null ? "missing" : Std.string(instance.status()),
        state: state == null ? null : {
          robotId: state.id,
          sequence: Std.string(state.sourceSequence),
          timestampNs: Std.string(state.timestampNs),
          q: state.positions.toArray(),
          dq: state.velocities.toArray(),
          effort: state.efforts.toArray(),
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
    var compact = viewportWidth < 1040.0;
    var minimal = viewportWidth < 800.0;
    var barStyle = fillStyle();
    barStyle.height = LayoutAxis.fixed(44.0);
    barStyle.direction = LayoutDirection.LeftToRight;
    barStyle.childAlignY = LayoutAlignmentY.Center;
    barStyle.childGap = compact ? 6.0 : 10.0;
    barStyle.padding = new Insets(10.0, 5.0, 10.0, 5.0);
    barStyle.background = appearance.toolbar;

    var titleStyle = new LayoutStyle();
    titleStyle.width = LayoutAxis.fixed(compact ? 78.0 : 92.0);
    var fileActions = new Row("toolbar-file", [
      new KeyedView("new", toolbarAction("toolbar-new", "editor.new", "New", IconName.NewFile, compact)),
      new KeyedView("open", toolbarAction("toolbar-open", "editor.open", "Open", IconName.FolderOpen, compact)),
      new KeyedView("save", toolbarAction("toolbar-save", "editor.save", "Save", IconName.Save, compact, true))
    ], toolbarGroupStyle());
    var items:Array<KeyedView> = [
      new KeyedView("brand", new Text("MATERIA", titleStyle, appearance.theme.text,
        TextStyleOverride.text(13.0, 0.5))),
      new KeyedView("file", toolbarGroup("file-group", "FILE", fileActions, compact))
    ];
    if (!minimal) {
      items.push(new KeyedView("file-edit-divider", toolbarDivider("file-edit-divider")));
      var editActions = new Row("toolbar-edit", [
        new KeyedView("undo", toolbarAction("toolbar-undo", "editor.undo", "Undo", IconName.Undo, compact)),
        new KeyedView("redo", toolbarAction("toolbar-redo", "editor.redo", "Redo", IconName.Redo, compact))
      ], toolbarGroupStyle());
      items.push(new KeyedView("edit", toolbarGroup("edit-group", "EDIT", editActions, compact)));
      items.push(new KeyedView("edit-view-divider", toolbarDivider("edit-view-divider")));
      var viewActions = new Row("toolbar-view", [
        new KeyedView("frame", toolbarAction("toolbar-frame", "scene.frame-selected",
          "Frame", IconName.Inspect, compact))
      ], toolbarGroupStyle());
      items.push(new KeyedView("view", toolbarGroup("view-group", "VIEW", viewActions, compact)));
    }
    items.push(new KeyedView("space", new Spacer("toolbar-space", LayoutAxis.grow(),
      LayoutAxis.fixed(1.0))));
    var documentLabel = shortenLabel(session.label(), compact ? 18 : 30);
    var status = minimal ? documentLabel : documentLabel + "  ·  World: " +
      (world == null ? "offline" : Std.string(world.status()));
    items.push(new KeyedView("status", new Text(status, null, appearance.muted,
      TextStyleOverride.text(12.0))));
    var more = new Button(compact ? "" : "More", null, function() {
      toolbarMenuVisible = !toolbarMenuVisible;
      commands.refresh();
    }, "toolbar-more");
    more.variant = ButtonVariant.Navigation;
    more.leadingIcon = IconName.ChevronDown;
    more.accessibilityLabel = "More editor actions";
    more.selected = toolbarMenuVisible;
    items.push(new KeyedView("more", more));
    return new Row("editor-toolbar-row", items, barStyle);
  }

  function toolbarAction(key:String, commandId:String, label:String, icon:IconName,
      compact:Bool, primary:Bool = false):CommandButton {
    var action = new CommandButton(key, commandId, commands);
    action.displayLabel = compact ? "" : label;
    action.leadingIcon = icon;
    action.variant = primary ? ButtonVariant.Primary : ButtonVariant.Navigation;
    return action;
  }

  function toolbarGroup(key:String, label:String, actions:View, compact:Bool):View {
    if (compact) return actions;
    return new Row(key, [
      new KeyedView("label", new Text(label, null, appearance.heading,
        TextStyleOverride.text(10.0, 0.8))),
      new KeyedView("actions", actions)
    ], toolbarGroupStyle());
  }

  static function toolbarGroupStyle():LayoutStyle {
    var style = new LayoutStyle();
    style.childAlignY = LayoutAlignmentY.Center;
    style.childGap = 3.0;
    return style;
  }

  function toolbarDivider(key:String):View {
    var divider = new Spacer(key, LayoutAxis.fixed(1.0), LayoutAxis.fixed(20.0));
    divider.style.background = appearance.divider;
    return divider;
  }

  static function shortenLabel(value:String, maximum:Int):String
    return value.length <= maximum ? value : value.substr(0, maximum - 3) + "...";

  function makeWorkspace():DockWorkspaceModel {
    var result = new DockWorkspaceModel();
    result.register(new DockPanelDescriptor("hierarchy", "Hierarchy", function(_) {
      return hierarchyPanel();
    }, false));
    result.register(new DockPanelDescriptor("viewport", "Viewport", function(_) {
      return viewportPanel();
    }, false));
    result.register(new DockPanelDescriptor("perspective", "Perspective", function(_) {
      return perspectivePanel();
    }, false));
    result.register(new DockPanelDescriptor("inspector", "Inspector", function(_) {
      return inspectorPanel();
    }, false));
    result.register(new DockPanelDescriptor("sensors", "Sensors", function(_) {
      return sensorPanel();
    }, false));
    result.register(new DockPanelDescriptor("console", "Console", function(_) {
      return consolePanel();
    }
    ));
    result.register(new DockPanelDescriptor("telemetry", "Telemetry", function(_) {
      return telemetryPanel();
    }
    ));

    var centerTabs = DockNode.Tabs(["viewport", "perspective", "console", "telemetry"], "viewport");
    var editorArea = DockNode.Split(DockSplitAxis.Horizontal, 0.68, centerTabs, DockNode.Panel("inspector"));
    result.setDefaultLayout(DockNode.Split(DockSplitAxis.Horizontal, 0.25,
      DockNode.Tabs(["hierarchy", "sensors"], "hierarchy"), editorArea));
    return result;
  }

  function sensorPanel():View {
    var style=fillStyle();style.padding=new Insets(12.0,12.0,12.0,12.0);
    style.background=appearance.surface;
    var robotRows:Array<KeyedView> = [];
    var attachedIds = world.robotIds();
    var worldIds = attachedIds.copy();
    for(id in sensors.configuredRobotIds())if(worldIds.indexOf(id)<0)worldIds.push(id);
    worldIds.sort(Reflect.compare);
    var simulatedIds = simulation.simulatedRobotIds();
    sensors.setReadOnlyRobots([for(id in attachedIds) if(simulatedIds.indexOf(id)<0) id]);
    for (id in worldIds) {
      var robotButton = new Button(id,null,function(){sensors.selectRobot(id);commands.refresh();},"sensor-robot:"+id);
      robotButton.selected = id == sensors.robotId;
      robotButton.enabled = simulatedIds.indexOf(id)>=0 || sensors.configuredRobotIds().indexOf(id)>=0;
      robotRows.push(new KeyedView("robot:"+id,robotButton));
    }
    if (robotRows.length == 0) robotRows.push(new KeyedView("robot-id",new Text("Robot: "+sensors.robotId)));
    var rows:Array<KeyedView> = [];
    for(index in 0...sensors.model.sensors.length) {
      var sensor=sensors.model.sensors[index];
      var button=new Button(sensor.name+" · "+sensor.kind,null,function(){sensors.select(index);commands.refresh();},"sensor:"+sensor.id);
      button.selected=index==sensors.selectedIndex;rows.push(new KeyedView("sensor:"+sensor.id,button));
    }
    var ownership=session.scriptOwnership;
    var addLidar=new Button("LiDAR",null,function(){sensors.add("lidar");commands.refresh();},"sensor-add-lidar");
    var addImu=new Button("IMU",null,function(){sensors.add("imu");commands.refresh();},"sensor-add-imu");
    var removeSensor=new Button("Remove",null,function(){sensors.removeSelected();commands.refresh();},"sensor-remove");
    addLidar.leadingIcon=IconName.Plus;addImu.leadingIcon=IconName.Plus;
    removeSensor.leadingIcon=IconName.Trash;
    addLidar.enabled=ownership==null;addImu.enabled=ownership==null;removeSensor.enabled=ownership==null;
    var actions=new Row("sensor-actions",[
      new KeyedView("add-lidar",addLidar),new KeyedView("add-imu",addImu),
      new KeyedView("remove",removeSensor)
    ],actionRowStyle());
    var runtimeActions=new Column("sensor-runtime-actions",[
      new KeyedView("configuration",new Row("sensor-configuration-actions",[
      new KeyedView("undo",new Button("Undo",null,function(){
        if(ownership==null)sensors.document.undo();else {ownership.document.undo();refreshScriptMaterialization("Override undone");}
        commands.refresh();},"sensor-undo")),
      new KeyedView("redo",new Button("Redo",null,function(){
        if(ownership==null)sensors.document.redo();else {ownership.document.redo();refreshScriptMaterialization("Override redone");}
        commands.refresh();},"sensor-redo")),
      new KeyedView("apply",new Button(simulation.appliedRevision == 0 ? "Apply" : "Rebuild",null,function(){
        log(simulation.rebuild(sensors,scene) ? "Shared simulation configuration applied" : "Simulation rebuild rejected: "+simulation.error);
        commands.refresh();
      },"sensor-apply"))],actionRowStyle())),
      new KeyedView("playback",new Row("sensor-playback-actions",[
      new KeyedView("run",new Button("Run",null,function(){
        try {simulation.start();log("Simulation running");} catch(error:Dynamic){log("Run rejected: "+Std.string(error));}
        commands.refresh();
      },"sensor-run")),
      new KeyedView("pause",new Button("Pause",null,function(){
        simulation.stop();log("Simulation paused");commands.refresh();
      },"sensor-pause")),
      new KeyedView("reset",new Button("Reset",null,function(){
        log(simulation.reset() ? "Shared simulation reset" : "No simulation to reset");
        commands.refresh();
      },"sensor-reset")),
      new KeyedView("design",new Button("Design",null,function(){
        simulation.clear();log("Returned to design mode");commands.refresh();
      },"sensor-design"))
      ],actionRowStyle()))
    ],actionColumnStyle());
    var backendActions=new Row("sensor-backend-actions",[
      new KeyedView("deterministic",new Button("Test backend",null,function(){
        if(ownership==null)simulation.setBackend(ApplicationSimulation.DETERMINISTIC);else try {
          ownership.setOverride(ScriptOwnership.SIMULATION_TARGET,"backend","integer",ApplicationSimulation.DETERMINISTIC);
          refreshScriptMaterialization("Physics backend override changed");
        } catch(error:Dynamic)log("Override rejected: "+Std.string(error));commands.refresh();
      },"sensor-backend-deterministic")),
      new KeyedView("mujoco",new Button("MuJoCo",null,function(){
        if(ownership==null)simulation.setBackend(ApplicationSimulation.MUJOCO);else try {
          ownership.setOverride(ScriptOwnership.SIMULATION_TARGET,"backend","integer",ApplicationSimulation.MUJOCO);
          refreshScriptMaterialization("Physics backend override changed");
        } catch(error:Dynamic)log("Override rejected: "+Std.string(error));commands.refresh();
      },"sensor-backend-mujoco"))],actionRowStyle());
    var content:Array<KeyedView> = [new KeyedView("heading",sectionHeading("SENSORS")),
      new KeyedView("apply-state",new Text(simulation.pending(sensors,scene)
        ? "Pending changes · rebuild required"
        : "Configuration applied",null,appearance.muted,TextStyleOverride.text(12.0))),
      new KeyedView("simulation-mode",new Text("Mode: "+(simulation.isActive()
        ? (simulation.isRunning()?"Running":"Paused") : "Design"))),
      new KeyedView("ownership",ownership==null?new Text("Origin: document"):
        textLines("script-origin",["Origin: script",ownership.reference,
          'configuration v${ownership.configurationVersion}'])),
      new KeyedView("robots-heading",sectionHeading("ROBOTS")),
      new KeyedView("robots",new Column("sensor-robots",robotRows)),
      new KeyedView("backend-heading",sectionHeading("PHYSICS · "+simulation.userBackendName().toUpperCase())),
      new KeyedView("backend-actions",backendActions),
      new KeyedView("devices-heading",sectionHeading("DEVICES")),
      new KeyedView("actions",actions),
      new KeyedView("list",new Column("sensor-list",rows)),
      new KeyedView("runtime-heading",sectionHeading("SIMULATION")),
      new KeyedView("runtime-actions",runtimeActions)];
    if(ownership!=null){
      var overrideLabel=ownership.overridesEnabled?"Disable overrides":"Enable overrides";
      content.insert(4,new KeyedView("script-actions",new Column("script-actions",[
        new KeyedView("source",new Row("script-source-actions",[
        new KeyedView("reload",new Button("Reload script",null,function(){
          try {var result=session.reloadScript();simulation.setBackend(result.backend);
            simulation.setTimestep(result.timestep);documentChanged();log("Script reloaded; Apply/Rebuild restarts simulation");}
          catch(error:Dynamic)log("Script reload rejected; active simulation unchanged: "+Std.string(error));
          commands.refresh();},"script-reload")),
        new KeyedView("overrides",new Button(overrideLabel,null,function(){
          ownership.setOverridesEnabled(!ownership.overridesEnabled);
          refreshScriptMaterialization(ownership.overridesEnabled?"Overrides enabled":"Overrides disabled");
        },"script-overrides"))])),
        new KeyedView("revert",new Row("script-revert-actions",[
        new KeyedView("revert-simulation",new Button("Revert simulation",null,function(){
          if(ownership.revertTarget(ScriptOwnership.SIMULATION_TARGET))
            refreshScriptMaterialization("Simulation settings reverted to script values");
        },"script-revert-simulation")),
        new KeyedView("remove-stale",new Button("Remove stale",null,function(){
          if(ownership.removeStaleOverrides())refreshScriptMaterialization("Stale overrides removed");
          },"script-remove-stale"))
        ]))
      ])));
      content.insert(5,new KeyedView("simulation-origins",textLines("script-simulation-origins",
        ownership.propertyOrigins(ScriptOwnership.SIMULATION_TARGET,["backend","timestep"]))));
      content.insert(6,new KeyedView("robot-origins",textLines("script-robot-origins",
        ["Robot pose"].concat(ownership.propertyOrigins(sensors.robotId,["position","rotation"])))));
    }
    var selected=sensors.selected();
    if(!sensors.isEditable())content.push(new KeyedView("read-only",new Text("Remote robot configuration is read-only")));
    if(selected!=null){
      if(ownership!=null){
        var sensorTarget=sensors.robotId+"/"+selected.id;
        var sensorProperties=["updateRate","noiseStddev","noiseSeed","mount.frameId","mount.position","mount.rotation"];
        if(selected.kind=="lidar"){sensorProperties.push("rayCount");sensorProperties.push("maxRange");}
        content.push(new KeyedView("sensor-origin",textLines("script-sensor-origins",
          ["Value origins"].concat(ownership.propertyOrigins(sensorTarget,sensorProperties)))));
        var decreaseRate=new Button("Rate -1 Hz",null,function(){
            try {ownership.setSensorRate(sensors.robotId,selected.id,Math.max(0,selected.updateRate-1));
              refreshScriptMaterialization("Sensor rate override changed");}
            catch(error:Dynamic)log("Override rejected: "+Std.string(error));
          },"script-rate-decrease");
        var increaseRate=new Button("Rate +1 Hz",null,function(){
            try {ownership.setSensorRate(sensors.robotId,selected.id,selected.updateRate+1);
              refreshScriptMaterialization("Sensor rate override changed");}
            catch(error:Dynamic)log("Override rejected: "+Std.string(error));
          },"script-rate-increase");
        var revertRate=new Button("Revert rate",null,function(){
            if(ownership.revertSensorRate(sensors.robotId,selected.id))
              refreshScriptMaterialization("Sensor rate reverted to script value");
          },"script-rate-revert");
        decreaseRate.enabled=ownership.overridesEnabled;
        increaseRate.enabled=ownership.overridesEnabled;
        revertRate.enabled=ownership.overridesEnabled;
        var rateActions=new Row("script-rate-actions",[
          new KeyedView("decrease",decreaseRate),new KeyedView("increase",increaseRate),
          new KeyedView("revert",revertRate)]);
        content.push(new KeyedView("rate-actions",rateActions));
        var revertSensor=new Button("Revert selected sensor",null,function(){
          if(ownership.revertTarget(sensorTarget))refreshScriptMaterialization("Sensor overrides reverted");
        },"script-sensor-revert-all");
        revertSensor.enabled=ownership.overridesEnabled;
        content.push(new KeyedView("sensor-revert",revertSensor));
      }
      var sensorInspector=new PropertyInspector("sensor-inspector:"+selected.id,
        sensors.properties(),null,null,null,null,"Sensor configuration");
      sensorInspector.labelWidth=viewportWidth < 820.0 ? 76.0 : 100.0;
      sensorInspector.enabled=ownership==null;
      content.push(new KeyedView("properties",sensorInspector));
    }
    var diagnostics=sensors.diagnostics();
    if(diagnostics.length>0)content.push(new KeyedView("diagnostics",new Text(
      diagnostics[0].code+": "+diagnostics[0].message)));
    if(ownership!=null&&ownership.diagnostics.length>0)
      content.push(new KeyedView("script-diagnostics",textLines("script-diagnostic-lines",ownership.diagnostics)));
    var contentStyle=new LayoutStyle();contentStyle.width=LayoutAxis.stretch();
    contentStyle.height=LayoutAxis.fit();contentStyle.padding=new Insets(8.0,8.0,8.0,8.0);
    return new ScrollView("sensor-scroll",new Column("sensor-panel",content,contentStyle),style);
  }

  function hierarchyPanel():View {
    var treeStyle = fillStyle();
    treeStyle.padding = new Insets(10.0, 10.0, 10.0, 10.0);
    treeStyle.background = appearance.surface;
    treeStyle.childGap = 6.0;
    var treeViewport = fillStyle();
    var tree = new TreeView("scene-hierarchy",
      treeModel, treeViewport, null, 420.0, scene.treeSelectionKey(), ["scene"], function(id) {
      scene.selectTreeKey(id);
      log("Selected " + id);
      updateCommandContext();
      commands.refresh();
    }, function(id) {
      scene.selectTreeKey(id);
      log("Activated " + id);
      updateCommandContext();
    }, null, null);
    return new Column(
      "hierarchy-panel",
      [
        new KeyedView("heading", sectionHeading("SCENE")),
        new KeyedView("actions", new Column("scene-object-actions", [
          new KeyedView("create", new Row("scene-create-actions", [
            new KeyedView("rectangle", sceneAction("scene-create", "scene.create", "Rectangle", IconName.Plus)),
            new KeyedView("plate", sceneAction("scene-create-plate", "scene.create-plate", "Plate", IconName.Plus)),
            new KeyedView("hole", sceneAction("scene-add-face-hole", "scene.add-face-hole", "Hole", IconName.Plus))
          ], actionRowStyle())),
          new KeyedView("edit", new Row("scene-edit-actions", [
            new KeyedView("duplicate", sceneAction("scene-duplicate", "scene.duplicate", "Duplicate", IconName.Copy)),
            new KeyedView("delete", sceneAction("scene-delete", "scene.delete", "Delete", IconName.Trash))
          ], actionRowStyle()))
        ], actionColumnStyle())),
        new KeyedView(
          "tree",
          tree
        )
      ],
      treeStyle
    );
  }

  function sceneAction(key:String, commandId:String, label:String, icon:IconName):CommandButton {
    var action = new CommandButton(key, commandId, commands);
    action.displayLabel = label;
    action.leadingIcon = icon;
    return action;
  }

  static function textLines(key:String, lines:Array<String>):View return new Column(key,
    [for (index in 0...lines.length) new KeyedView("line:"+index,new Text(lines[index]))]);

  static function actionRowStyle():LayoutStyle {
    var style = new LayoutStyle();
    style.width = LayoutAxis.grow();
    style.childGap = 4.0;
    style.wrapMode = LayoutWrapMode.Wrap;
    style.rowGap = 4.0;
    return style;
  }

  static function actionColumnStyle():LayoutStyle {
    var style = new LayoutStyle();
    style.width = LayoutAxis.grow();
    style.childGap = 4.0;
    return style;
  }

  function viewportPanel():View {
    viewportContent.setSimulationState(simulation.isActive(),simulation.environmentVisualState(),
      simulation.visualRevision());
    if (sceneViewport != null) {
      sceneViewport.setAppearance(Color.rgba(0.025, 0.035, 0.055, 1.0),
        Color.rgba(0.16, 0.24, 0.36, 0.75), viewportContent.gridStep * EditorSceneViewport.SCALE, gridVisible);
      return sceneViewport;
    }
    var viewportStyle = fillStyle();
    viewportStyle.background = Color.rgba(0.025, 0.035, 0.055, 1.0);
    var viewport = new GpuViewport(
      "scene-gpu-viewport",
      viewportContent,
      viewportCamera,
      viewportStyle,
      "Scene XY view: drag objects, middle-drag to pan, scroll to zoom"
    );
    viewport.panButton = 2; // NativeKit middle button.
    viewport.setOverlay(function(canvas, geometry) {
      viewportContent.viewportWidth = geometry.width;
      viewportContent.viewportHeight = geometry.height;
      paintSimulationOverlay(canvas);
    });
    viewport.setAppearance(Color.rgba(0.025, 0.035, 0.055, 1.0), Color.rgba(0.16, 0.24, 0.36, 0.75),
      viewportContent.gridStep * EditorSceneViewport.SCALE, gridVisible);
    viewport.on(UiEventKind.PointerDown, function(event:UiEvent) {
      if (event.button == 0) {
        var hit = viewportContent.pick(viewportCamera, event.localX, event.localY);
        viewportContent.selectAt(viewportCamera,event.localX,event.localY);
        if (hit != "scene" && viewportContent.beginDrag(viewportCamera,
            event.localX, event.localY, gridSnapEnabled)) {
          dragPointer = event.pointerId;
          dragPointerX = event.x;
          dragPointerY = event.y;
          event.capturePointer();
        }
        updateCommandContext();
        commands.refresh();
        event.preventDefault();
        event.stopPropagation();
        return;
      }
      if (event.button != 1) return; // NativeKit right button.
      contextMenuX = event.x;
      contextMenuY = event.y;
      contextMenuVisible = true;
      event.preventDefault();
      event.stopPropagation();
      commands.refresh();
    }
    );
    viewport.on(UiEventKind.PointerMove, function(event:UiEvent) {
      if (dragPointer == null || event.pointerId != dragPointer) return;
      dragPointerX = event.x;
      dragPointerY = event.y;
      if (viewportContent.updateDrag(viewportCamera, event.localX, event.localY,
          (event.modifiers & UiModifier.Shift) != 0)) commands.refresh();
      event.preventDefault();
      event.stopPropagation();
    });
    viewport.on(UiEventKind.PointerUp, function(event:UiEvent) {
      if (dragPointer == null || event.pointerId != dragPointer) return;
      viewportContent.commitDrag();
      dragPointer = null;
      event.releasePointer();
      updateCommandContext();
      commands.refresh();
      event.preventDefault();
      event.stopPropagation();
    });
    viewport.on(UiEventKind.PointerCancel, function(event:UiEvent) {
      if (dragPointer == null || event.pointerId != dragPointer) return;
      viewportContent.cancelDrag();
      dragPointer = null;
      event.releasePointer();
      updateCommandContext();
      commands.refresh();
    });
    sceneViewport = viewport;
    return viewport;
  }

  function paintSimulationOverlay(canvas:Canvas):Void {
    for(robot in simulation.visualState()) {
      var base=simulationPoint(robot.position[0],robot.position[1]);
      canvas.fillRect(new Rect(base.x-5,base.y-5,10,10),Color.rgba(0.3,1.0,0.65,0.95));
      for(link in robot.links){var linkPoint=simulationPoint(link.position[0],link.position[1]);
        canvas.fillRect(new Rect(linkPoint.x-4,linkPoint.y-4,8,8),Color.rgba(0.2,0.85,0.55,0.9));}
      for(sensor in robot.sensors) {
        var linkPosition=robot.position,linkRotation=robot.rotation;
        for(link in robot.links)if(link.id==sensor.linkId){linkPosition=link.position;linkRotation=link.rotation;break;}
        var offset=rotateVector(linkRotation,sensor.mountPosition.toArray());
        var origin=[linkPosition[0]+offset[0],linkPosition[1]+offset[1],linkPosition[2]+offset[2]];
        var mount=simulationPoint(origin[0],origin[1]);
        canvas.fillRect(new Rect(mount.x-3,mount.y-3,6,6),Color.rgba(1.0,0.75,0.2,0.95));
        if(sensor.kind!="lidar"||sensor.values.length==0)continue;
        var rotation=multiplyQuaternion(linkRotation,sensor.mountRotation.toArray());
        var rays=new PathBuilder();var values=sensor.values.toArray();
        for(index in 0...values.length){
          var angle=index*6.283185307179586/values.length;
          var direction=rotateVector(rotation,[Math.cos(angle),Math.sin(angle),0.0]);
          var hit=simulationPoint(origin[0]+direction[0]*values[index],origin[1]+direction[1]*values[index]);
          rays.moveTo(mount.x,mount.y).lineTo(hit.x,hit.y);
        }
        canvas.strokeTransient(rays.build(),Color.rgba(0.25,0.8,1.0,0.55),1.0);
      }
    }
  }
  function simulationPoint(x:Float,y:Float):Point return viewportCamera.worldToViewport(
    EditorSceneViewport.ORIGIN_X+x*EditorSceneViewport.SCALE,
    EditorSceneViewport.ORIGIN_Y-y*EditorSceneViewport.SCALE);
  static function rotateVector(q:Array<Float>,v:Array<Float>):Array<Float> {
    var x=q[0],y=q[1],z=q[2],w=q[3];
    var tx=2*(y*v[2]-z*v[1]),ty=2*(z*v[0]-x*v[2]),tz=2*(x*v[1]-y*v[0]);
    return [v[0]+w*tx+y*tz-z*ty,v[1]+w*ty+z*tx-x*tz,v[2]+w*tz+x*ty-y*tx];
  }
  static function multiplyQuaternion(a:Array<Float>,b:Array<Float>):Array<Float> return [
    a[3]*b[0]+a[0]*b[3]+a[1]*b[2]-a[2]*b[1],
    a[3]*b[1]-a[0]*b[2]+a[1]*b[3]+a[2]*b[0],
    a[3]*b[2]+a[0]*b[1]-a[1]*b[0]+a[2]*b[3],
    a[3]*b[3]-a[0]*b[0]-a[1]*b[1]-a[2]*b[2]];

  function perspectivePanel():View {
    if (perspectiveViewport != null) {
      perspectiveViewport.setPlacementOptions(gridSnapEnabled, gridSpacing);
      perspectiveViewport.setSimulationState(simulation.isActive(),simulation.environmentVisualState(),
        simulation.visualRevision(),simulation.visualState());
    }
    return perspectiveViewport == null
      ? new Text("Perspective rendering requires the desktop GPU host.")
      : perspectiveViewport;
  }

  function inspectorPanel():View {
    var style = fillStyle();
    style.padding = new Insets(10.0, 10.0, 10.0, 10.0);
    style.background = appearance.surface;
    var selected = scene.object(scene.selectedId);
    if (selected == null)
      return new Column("inspector-empty", [
        new KeyedView("heading", sectionHeading("INSPECTOR")),
        new KeyedView("hint", new Text("Select an object to edit its properties."))
      ], style);
    if (sceneInspector == null || inspectorSelectionRevision != scene.selectionRevision) {
      sceneInspector = new PropertyInspector("scene-inspector:" + scene.selectedId,
        scene.properties(), style, null, null, null, "Selected object inspector");
      inspectorSelectionRevision = scene.selectionRevision;
    }
    var inspector = sceneInspector;
    inspector.labelWidth = viewportWidth < 820.0 ? 76.0 : 116.0;
    var ownership=session.scriptOwnership;
    inspector.enabled = ownership==null&&!simulation.isActive() && !viewportContent.dragging() &&
      (perspectiveViewport == null || !perspectiveViewport.dragging());
    var rows:Array<KeyedView> = [new KeyedView("heading",sectionHeading(selected.label))];
    if(scene.isCadPart(selected.id))rows.push(new KeyedView("face-selection",
      new Text(scene.selectedCadFaceIndex<0?"Click a CAD face to select it":
        selected.kind=="cad-plate"
          ?"Selected face "+(scene.selectedCadFaceIndex+1)+" · Add hole uses the picked location"
          :"Selected face "+(scene.selectedCadFaceIndex+1))));
    if(ownership!=null) {
      rows.push(new KeyedView("origin",textLines("script-object-origins",
        ["Script-owned"].concat(ownership.propertyOrigins(selected.id,["position","dimensions",
          "mass","collisionEnabled","dynamicBody","visible"])))));
      rows.push(new KeyedView("revert",new Button("Revert object overrides",null,function(){
        if(ownership.revertTarget(selected.id))refreshScriptMaterialization("Object overrides reverted");
      },"script-object-revert")));
    }
    rows.push(new KeyedView("properties",inspector));
    return new Column(
      "inspector-panel",
      rows,
      style
    );
  }

  function consolePanel():View {
    var style = fillStyle();
    style.padding = new Insets(12.0, 12.0, 12.0, 12.0);
    style.background = appearance.surface;
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
    style.background = appearance.surface;
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
    return new Text(label, null, appearance.heading, TextStyleOverride.text(11.0, 0.8));
  }

  function runSceneEdit(label:String, action:Void->Bool):Void {
    try {
      action();
      updateCommandContext();
    } catch (error:ParametricError) {
      log(label + ": " + error.message);
    }
    commands.refresh();
  }

  function installCommands():Void {
    workspace.installCommands(commands, "workspace");
    commands.register(new Command("scene.create", "Add rectangle", function() {
      scene.createRectangle();
      updateCommandContext();
      commands.refresh();
    }, null, function() return canEditObjects() && scene.canCreate()));
    commands.register(new Command("scene.create-plate", "Add mounting plate", function() {
      runSceneEdit("Could not add mounting plate", function() scene.createMountingPlate());
    }, null, function() return canEditObjects() && scene.canCreate()));
    commands.register(new Command("scene.create-bracket", "Add L bracket", function() {
      runSceneEdit("Could not add L bracket", function() scene.createBracket());
    }, null, function() return canEditObjects() && scene.canCreate()));
    commands.register(new Command("scene.add-face-hole", "Add hole on selected face", function() {
      runSceneEdit("Could not add hole", function() scene.addHoleOnSelectedFace());
    },null,function() return canEditObjects()&&scene.canAddHoleOnSelectedFace()));
    commands.register(new Command("scene.export-step", "Export STEP", function() {
      var chooser=files;
      if(chooser==null)return;
      chooser.chooseExport("Export selected CAD part","CAD part.step",function(path,error) {
        if(error!=null){log(error);return;}
        if(path==null)return;
        try { scene.exportSelectedCad(path); log("Exported STEP: "+path); }
        catch(failure:Dynamic) log("STEP export failed: "+Std.string(failure));
        commands.refresh();
      });
    },null,function() {
      var selected=scene.object(scene.selectedId);
      return !documents.blocked()&&files!=null&&selected!=null&&scene.isCadPart(selected.id);
    }));
    commands.register(new Command("scene.duplicate", "Duplicate", function() {
      runSceneEdit("Could not duplicate object", function() scene.duplicateSelected());
    }, new Shortcut(68, UiModifier.Control), function() return canEditObjects()
      && scene.canCreate() && scene.object(scene.selectedId) != null));
    commands.register(new Command("scene.delete", "Delete", function() {
      scene.deleteSelected();
      updateCommandContext();
      commands.refresh();
    }, null, function() return canEditObjects() && scene.object(scene.selectedId) != null));
    commands.register(new Command("editor.undo", "Undo", function() {
      runSceneEdit("Could not undo", function() scene.document.undo());
    }, new Shortcut(UiKey.Z, UiModifier.Control), function() return canEditObjects() && scene.document.canUndo));
    commands.register(new Command("editor.redo", "Redo", function() {
      runSceneEdit("Could not redo", function() scene.document.redo());
    }, new Shortcut(UiKey.Z, UiModifier.Control | UiModifier.Shift), function() return canEditObjects() && scene.document.canRedo));
    commands.register(new Command("editor.new", "New", function() documents.requestNew(),
      new Shortcut(78, UiModifier.Control), function() return !documents.blocked()));
    commands.register(new Command("editor.open", "Open", function() documents.requestOpen(),
      new Shortcut(79, UiModifier.Control), function() return !documents.blocked()));
    commands.register(new Command("editor.save", "Save", function() documents.save(),
      new Shortcut(UiKey.S, UiModifier.Control), function() return !documents.blocked()));
    commands.register(new Command("editor.save-as", "Save As", function() documents.save(true),
      new Shortcut(UiKey.S, UiModifier.Control | UiModifier.Shift), function() return !documents.blocked()));
    commands.register(new Command("workspace.save", "Save workspace", function() {
      saveWorkspace();
      log("Workspace saved");
    }));
    commands.register(new Command("editor.command-palette", "Open command palette", function() {
      paletteVisible = true;
      contextMenuVisible = false;
      commands.refresh();
    }, new Shortcut(UiKey.K, UiModifier.Control), function() return !documents.blocked()));
    commands.register(new Command("scene.frame-selected", "Frame selected", function() {
      if (workspace.activePanelId == "perspective" && perspectiveViewport != null)
        perspectiveViewport.frameSelected();
      else
        viewportContent.frameSelected(viewportCamera);
      log("Framed " + scene.selectedId);
    }, null, function() return !documents.blocked() && scene.items().length > 0));
    commands.register(new Command("scene.reset-perspective", "Reset perspective view", function() {
      if (perspectiveViewport != null) perspectiveViewport.resetView();
      log("Perspective view reset");
    }, null, function() return !documents.blocked() && perspectiveViewport != null));
    commands.register(new Command("scene.show-perspective", "Show perspective view", function() {
      workspace.open("perspective", "viewport");
      commands.refresh();
    }, null, function() return perspectiveViewport != null));
    commands.register(new Command("scene.toggle-grid", "Toggle grid", function() {
      gridVisible = !gridVisible;
      log(gridVisible ? "Grid enabled" : "Grid disabled");
    }, null, null, function() return gridVisible));
    commands.register(new Command("scene.toggle-grid-snap", "Toggle grid snapping", function() {
      gridSnapEnabled = !gridSnapEnabled;
      log(gridSnapEnabled ? "Grid snapping enabled" : "Grid snapping disabled");
    }, null, null, function() return gridSnapEnabled));
    registerGridSpacing("scene.grid-spacing-0.1", "Grid spacing: 0.1 m", 0.1);
    registerGridSpacing("scene.grid-spacing-0.2", "Grid spacing: 0.2 m", 0.2);
    registerGridSpacing("scene.grid-spacing-0.5", "Grid spacing: 0.5 m", 0.5);
    registerNudgeCommand("scene.nudge-left", "Nudge left", UiKey.Left, 0, -0.1, 0.0);
    registerNudgeCommand("scene.nudge-right", "Nudge right", UiKey.Right, 0, 0.1, 0.0);
    registerNudgeCommand("scene.nudge-up", "Nudge up", UiKey.Up, 0, 0.0, 0.1);
    registerNudgeCommand("scene.nudge-down", "Nudge down", UiKey.Down, 0, 0.0, -0.1);
    registerNudgeCommand("scene.nudge-left-large", "Nudge left (large)", UiKey.Left,
      UiModifier.Shift, -1.0, 0.0);
    registerNudgeCommand("scene.nudge-right-large", "Nudge right (large)", UiKey.Right,
      UiModifier.Shift, 1.0, 0.0);
    registerNudgeCommand("scene.nudge-up-large", "Nudge up (large)", UiKey.Up,
      UiModifier.Shift, 0.0, 1.0);
    registerNudgeCommand("scene.nudge-down-large", "Nudge down (large)", UiKey.Down,
      UiModifier.Shift, 0.0, -1.0);
    commands.register(new Command("scene.cancel-drag", "Cancel object drag", function() {
      cancelActiveDrag();
      updateCommandContext();
      commands.refresh();
    }, new Shortcut(UiKey.Escape), function() return viewportContent.dragging() ||
      (perspectiveViewport != null && perspectiveViewport.dragging())));
  }

  function registerGridSpacing(id:String, label:String, spacing:Float):Void {
    commands.register(new Command(id, label, function() {
      viewportContent.setGridStep(spacing);
      gridSpacing = spacing;
      log("Grid spacing set to " + spacing + " m");
      commands.refresh();
    }, null, null, function() return gridSpacing == spacing));
  }

  function registerNudgeCommand(id:String, label:String, key:Int, modifiers:Int,
      deltaX:Float, deltaY:Float):Void {
    commands.register(new Command(id, label, function() {
      scene.nudgeSelected(deltaX, deltaY);
      updateCommandContext();
      commands.refresh();
    }, new Shortcut(key, modifiers), function() return canEditObjects() &&
      scene.object(scene.selectedId) != null));
  }

  function documentChanged():Void {
    cancelActiveDrag();
    if (sceneGeneration != session.generation) {
      var ownership=session.scriptOwnership;
      if(ownership!=null){simulation.setBackend(ownership.backend());simulation.setTimestep(ownership.timestep());}
      log("Document configuration replaced");
      sceneGeneration = session.generation;
      treeModel = new EditorSceneTree(scene);
      viewportContent = new EditorSceneViewport(scene);
      viewportContent.setGridStep(gridSpacing);
      if (perspectiveViewport != null) perspectiveViewport.dispose();
      perspectiveViewport = hostContext == null ? null :
        new EditorPerspectiveViewport("scene-perspective", scene, hostContext);
      sceneViewport = null;
      sceneInspector = null;
      inspectorSelectionRevision = -1;
      viewportCamera.setPan(0, 0);
      viewportCamera.setZoom(1);
      updateCommandContext();
    }
    paletteVisible = false;
    contextMenuVisible = false;
    commands.refresh();
  }

  function refreshScriptMaterialization(message:String):Void {
    try {var result=session.refreshScriptOverrides();simulation.setBackend(result.backend);
      simulation.setTimestep(result.timestep);documentChanged();log(message+"; Apply/Rebuild restarts simulation");}
    catch(error:Dynamic)log("Script override rejected: "+Std.string(error));
    commands.refresh();
  }

  function canEditObjects():Bool return session.scriptOwnership==null && !documents.blocked() && !simulation.isActive() && !viewportContent.dragging() &&
    (perspectiveViewport == null || !perspectiveViewport.dragging());

  function commitActiveDrag():Void {
    var changed = false;
    if (viewportContent.dragging()) {
      viewportContent.commitDrag();
      releaseDragPointer();
      changed = true;
    }
    if (perspectiveViewport != null && perspectiveViewport.dragging()) {
      var pointer = perspectiveViewport.commitDrag();
      if (pointer != null) ui.pointerCancel(pointer.id, pointer.x, pointer.y);
      changed = true;
    }
    if (!changed) return;
    updateCommandContext();
    commands.refresh();
  }

  function cancelActiveDrag():Void {
    if (viewportContent.dragging()) viewportContent.cancelDrag();
    releaseDragPointer();
    if (perspectiveViewport != null && perspectiveViewport.dragging()) {
      var pointer = perspectiveViewport.cancelDrag();
      if (pointer != null) ui.pointerCancel(pointer.id, pointer.x, pointer.y);
    }
    updateCommandContext();
    commands.refresh();
  }

  function releaseDragPointer():Void {
    var pointer = dragPointer;
    if (pointer == null) return;
    dragPointer = null;
    ui.pointerCancel(pointer, dragPointerX, dragPointerY);
  }

  function makeDocumentDialog():Null<View> {
    if (!documents.needsConfirmation() && documents.error == null) return null;
    var style = new LayoutStyle();
    style.width = LayoutAxis.grow();
    style.childGap = 12.0;
    style.padding = new Insets(16, 16, 16, 16);
    var buttons:Array<KeyedView> = [];
    var title = "Unsaved changes";
    var message = "Save changes to " + session.label() + " before continuing?";
    var dismiss = function() documents.resolve("cancel");
    if (documents.error != null) {
      title = "Scene document error";
      message = documents.error;
      dismiss = documents.dismissError;
      buttons.push(new KeyedView("close", new Button("Close", null, documents.dismissError, "document-error-close")));
    } else {
      buttons.push(new KeyedView("save", new Button("Save", null, function() documents.resolve("save"), "document-confirm-save")));
      buttons.push(new KeyedView("discard", new Button("Discard", null, function() documents.resolve("discard"), "document-confirm-discard")));
      buttons.push(new KeyedView("cancel", new Button("Cancel", null, dismiss, "document-confirm-cancel")));
    }
    var content = new Column("document-message", [
      new KeyedView("message", new Text(message)),
      new KeyedView("buttons", new Row("document-actions", buttons))
    ], style);
    var dialog = new Dialog("document-confirmation", title, content, dismiss, 520.0);
    dialog.dismissOnOutside = false;
    return dialog;
  }

  function updateCommandContext():Void {
    ui.setCommandContext(scene.context());
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
