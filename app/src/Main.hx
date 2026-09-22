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
import robotkit.world.RemoteRobot;
import robotkit.world.RobotWorld;
import nativekit.ui.lab.ComponentLab;
import nativekit.ui.theme.Theme;
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
    var diagnostics = ReferenceEditorLaunchOptions.fromArgs(args);
    if (diagnostics == null) return 2;
    var host = new DesktopUiHostOptions();
    host.title = "Materia Reference Editor";
    host.width = 1320;
    host.height = 900;
    host.captureDirectory = diagnostics.captureDirectory;
    host.frameLimit = diagnostics.frameLimit;
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
      var editor = new ReferenceEditorApp(context.fonts, null, activeTheme, world, context);
      if (diagnostics.componentLab) editor.enableComponentLab(diagnostics.storyId);
      if (args.indexOf("--reset-workspace") >= 0) editor.resetWorkspace();
      return editor;
    });
  }
}

private class ReferenceEditorLaunchOptions {
  public final captureDirectory:Null<String>;
  public final frameLimit:Int;
  public final componentLab:Bool;
  public final storyId:Null<String>;
  public final darkTheme:Bool;
  public final robotHost:Null<String>;
  public final robotPort:Int;
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

  public static function fromArgs(args:Array<String>):Null<ReferenceEditorLaunchOptions> {
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
    return new ReferenceEditorLaunchOptions(directory, frames, lab, story,
      args.indexOf("--dark") >= 0, robotHost, robotPort);
  }
}

/** Shared/app integration object passed to a platform frame loop. */
class ReferenceEditorApp implements DesktopUiApplication {
  public static inline var WORKSPACE_KEY:String = "reference-editor";

  public final ui:UiContext;
  public final commands:CommandRegistry;
  public final workspace:DockWorkspaceModel;
  public final workspacePath:String;
  public final world:Null<RobotWorld>;

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
      ?world:RobotWorld, ?hostContext:DesktopUiHostContext) {
    this.hostContext = hostContext;
    ui = new UiContext(null, fonts, theme == null ? Theme.light() : theme);
    commands = ui.commands;
    this.world = world;
    workspacePath = workspaceFile == null || workspaceFile.length == 0 ? defaultWorkspacePath() : workspaceFile;
    storage = new FileDockWorkspacePersistence(workspacePath);
    session = new SceneDocumentSession();
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
        hostContext.surface);
    }
    telemetry = makeTelemetry();
    logLines = ["Scene ready: two editable objects", "Select a box; edit position or visibility", "Middle-drag to pan; scroll to zoom"];
    gridVisible = true;
    gridSnapEnabled = false;
    gridSpacing = EditorSceneViewport.GRID_STEP;
    paletteVisible = false;
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

    var documentDialog = makeDocumentDialog();
    if (documentDialog != null) layers.push(new StackChild("document-dialog", documentDialog,
      0.0, 0.0, 100, LayoutAxis.grow(), LayoutAxis.grow()));
    var shellStyle = fillStyle();
    shellStyle.background = Color.rgba(0.93, 0.95, 0.98, 1.0);
    return new AppShell("reference-editor-shell", new Stack("overlay-host", layers), topBar(), null, null, shellStyle);
  }

  /** Convenience entry point for a NativeKit host's layout phase. */
  public function submit(frame:LayoutFrame):RenderNode return ui.submit(view(), frame);

  public function context():UiContext return ui;

  public function dispose():Void {
    if (world != null) world.close();
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
    var hint = new Text(session.label() + "  ·  " + robotLabel,
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
              "editor.new",
              "editor.open",
              "editor.save",
              "editor.save-as",
              "editor.undo",
              "editor.redo",
              "scene.frame-selected"
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
    var editorArea = DockNode.Split(DockSplitAxis.Horizontal, 0.76, centerTabs, DockNode.Panel("inspector"));
    result.setDefaultLayout(DockNode.Split(DockSplitAxis.Horizontal, 0.22,
      DockNode.Tabs(["hierarchy", "sensors"], "hierarchy"), editorArea));
    return result;
  }

  function sensorPanel():View {
    var style=fillStyle();style.padding=new Insets(8.0,8.0,8.0,8.0);
    style.background=Color.rgba(0.98,0.99,1.0,1.0);
    var robotRows:Array<KeyedView> = [];
    if (world != null) for (id in world.snapshot().robotIds()) {
      var robotButton = new Button(id,null,function(){sensors.selectRobot(id);commands.refresh();},"sensor-robot:"+id);
      robotButton.selected = id == sensors.robotId;
      robotRows.push(new KeyedView("robot:"+id,robotButton));
    }
    if (robotRows.length == 0) robotRows.push(new KeyedView("robot-id",new Text("Robot: "+sensors.robotId)));
    var rows:Array<KeyedView> = [];
    for(index in 0...sensors.model.sensors.length) {
      var sensor=sensors.model.sensors[index];
      var button=new Button(sensor.name+" · "+sensor.kind,null,function(){sensors.select(index);commands.refresh();},"sensor:"+sensor.id);
      button.selected=index==sensors.selectedIndex;rows.push(new KeyedView("sensor:"+sensor.id,button));
    }
    var actions=new Row("sensor-actions",[
      new KeyedView("add-lidar",new Button("+ LiDAR",null,function(){sensors.add("lidar");commands.refresh();},"sensor-add-lidar")),
      new KeyedView("add-imu",new Button("+ IMU",null,function(){sensors.add("imu");commands.refresh();},"sensor-add-imu")),
      new KeyedView("remove",new Button("Remove",null,function(){sensors.removeSelected();commands.refresh();},"sensor-remove"))
    ]);
    var runtimeActions=new Row("sensor-runtime-actions",[
      new KeyedView("undo",new Button("Undo",null,function(){sensors.document.undo();commands.refresh();},"sensor-undo")),
      new KeyedView("redo",new Button("Redo",null,function(){sensors.document.redo();commands.refresh();},"sensor-redo")),
      new KeyedView("apply",new Button(sensors.appliedRevision == 0 ? "Apply" : "Rebuild",null,function(){
        log(sensors.apply() ? "Sensor simulation configuration applied" : "Sensor configuration rejected");
        commands.refresh();
      },"sensor-apply")),
      new KeyedView("reset",new Button("Reset",null,function(){
        log(sensors.reset() ? "Sensor simulation reset" : "No sensor simulation to reset");
        commands.refresh();
      },"sensor-reset"))
    ]);
    var content:Array<KeyedView> = [new KeyedView("heading",sectionHeading("SENSORS")),
      new KeyedView("robots",new Column("sensor-robots",robotRows)),
      new KeyedView("actions",actions),new KeyedView("runtime-actions",runtimeActions),
      new KeyedView("list",new Column("sensor-list",rows))];
    var selected=sensors.selected();
    if(selected!=null)content.push(new KeyedView("properties",new PropertyInspector(
      "sensor-inspector:"+selected.id,sensors.properties(),null,null,null,null,"Sensor configuration")));
    var diagnostics=sensors.diagnostics();
    if(diagnostics.length>0)content.push(new KeyedView("diagnostics",new Text(
      diagnostics[0].code+": "+diagnostics[0].message)));
    return new ScrollView("sensor-scroll",new Column("sensor-panel",content,style),style);
  }

  function hierarchyPanel():View {
    var treeStyle = fillStyle();
    treeStyle.padding = new Insets(8.0, 8.0, 8.0, 8.0);
    treeStyle.background = Color.rgba(0.98, 0.99, 1.0, 1.0);
    var treeViewport = fillStyle();
    var tree = new TreeView("scene-hierarchy",
      treeModel, treeViewport, null, 420.0, scene.selectedId, ["scene"], function(id) {
      scene.select(id);
      log("Selected " + id);
      updateCommandContext();
      commands.refresh();
    }, function(id) {
      scene.select(id);
      log("Activated " + id);
      updateCommandContext();
    }, null, null);
    return new Column(
      "hierarchy-panel",
      [
        new KeyedView("heading", sectionHeading("SCENE HIERARCHY")),
        new KeyedView("actions", new Column("scene-object-actions", [
          new KeyedView("create", new CommandButton("scene-create", "scene.create", commands)),
          new KeyedView("duplicate", new CommandButton("scene-duplicate", "scene.duplicate", commands)),
          new KeyedView("delete", new CommandButton("scene-delete", "scene.delete", commands))
        ])),
        new KeyedView(
          "tree",
          tree
        )
      ],
      treeStyle
    );
  }

  function viewportPanel():View {
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
    viewport.setOverlay(function(_, geometry) {
      viewportContent.viewportWidth = geometry.width;
      viewportContent.viewportHeight = geometry.height;
    });
    viewport.setAppearance(Color.rgba(0.025, 0.035, 0.055, 1.0), Color.rgba(0.16, 0.24, 0.36, 0.75),
      viewportContent.gridStep * EditorSceneViewport.SCALE, gridVisible);
    viewport.on(UiEventKind.PointerDown, function(event:UiEvent) {
      if (event.button == 0) {
        var hit = viewportContent.pick(viewportCamera, event.localX, event.localY);
        scene.select(hit);
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

  function perspectivePanel():View {
    return perspectiveViewport == null
      ? new Text("Perspective rendering requires the desktop GPU host.")
      : perspectiveViewport;
  }

  function inspectorPanel():View {
    var style = fillStyle();
    style.padding = new Insets(10.0, 10.0, 10.0, 10.0);
    style.background = Color.rgba(0.98, 0.99, 1.0, 1.0);
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
    inspector.enabled = !viewportContent.dragging();
    return new Column(
      "inspector-panel",
      [
        new KeyedView("heading", sectionHeading(selected.label)),
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
    commands.register(new Command("scene.create", "Add rectangle", function() {
      scene.createRectangle();
      updateCommandContext();
      commands.refresh();
    }, null, function() return canEditObjects() && scene.canCreate()));
    commands.register(new Command("scene.duplicate", "Duplicate", function() {
      scene.duplicateSelected();
      updateCommandContext();
      commands.refresh();
    }, new Shortcut(68, UiModifier.Control), function() return canEditObjects()
      && scene.canCreate() && scene.object(scene.selectedId) != null));
    commands.register(new Command("scene.delete", "Delete", function() {
      scene.deleteSelected();
      updateCommandContext();
      commands.refresh();
    }, null, function() return canEditObjects() && scene.object(scene.selectedId) != null));
    commands.register(new Command("editor.undo", "Undo", function() {
      scene.document.undo();
      updateCommandContext();
      commands.refresh();
    }, new Shortcut(UiKey.Z, UiModifier.Control), function() return canEditObjects() && scene.document.canUndo));
    commands.register(new Command("editor.redo", "Redo", function() {
      scene.document.redo();
      updateCommandContext();
      commands.refresh();
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
      viewportContent.frameSelected(viewportCamera);
      log("Framed " + scene.selectedId);
    }, null, function() return !documents.blocked() && scene.object(scene.selectedId) != null));
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
    }, new Shortcut(UiKey.Escape), function() return viewportContent.dragging()));
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
      sceneGeneration = session.generation;
      treeModel = new EditorSceneTree(scene);
      viewportContent = new EditorSceneViewport(scene);
      viewportContent.setGridStep(gridSpacing);
      if (perspectiveViewport != null) perspectiveViewport.dispose();
      perspectiveViewport = hostContext == null ? null :
        new EditorPerspectiveViewport("scene-perspective", scene, hostContext.surface);
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

  function canEditObjects():Bool return !documents.blocked() && !viewportContent.dragging();

  function commitActiveDrag():Void {
    if (!viewportContent.dragging()) return;
    viewportContent.commitDrag();
    releaseDragPointer();
    updateCommandContext();
    commands.refresh();
  }

  function cancelActiveDrag():Void {
    if (viewportContent.dragging()) viewportContent.cancelDrag();
    releaseDragPointer();
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
