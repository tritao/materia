package app;

import nativekit.ui.widgets.controls.Select;
import nativekit.ui.widgets.controls.Toggle;


import app.EditorToolbarLayout.EditorToolbarDensity;
import app.editor.CharacterPreview;
import app.editor.ObjectKindRegistry;
import app.editor.TelemetryPanel;
import app.editor.SensorPanel;
import app.editor.HierarchyPanel;
import app.editor.InspectorPanel;
import app.editor.ProjectUiExtension;
import app.editor.EditorDocumentCommands;
import app.editor.SceneObjectCommands;
import app.editor.AssemblyCommands;
import app.editor.SceneViewCommands;
import app.editor.SimulationCommands;
import app.editor.ExampleCatalog;
import app.editor.ExampleCatalog.ExampleEntry;
import app.ProjectLoadJob;
import app.editor.StartPanel;
import app.editor.EditorGrid;
import Color;
import LayoutAxis;
import LayoutDistribution;
import LayoutAlignmentY;
import LayoutDirection;
import LayoutFrame;
import LayoutStyle;
import LayoutWrapMode;
import Insets;
import Rect;
import sys.FileSystem;
import sys.io.File;
import haxe.Json;
import materia.sheet.SheetInventory;
import nativekit.ui.core.Command;
import nativekit.ui.core.CommandContext;
import nativekit.ui.core.CommandRegistry;
import nativekit.ui.core.CommandResult;
import nativekit.ui.docking.DockPanelDescriptor;
import nativekit.ui.docking.DockWorkspaceCommands;
import nativekit.ui.docking.DockWorkspaceModel;
import nativekit.ui.docking.DockNodeTools;
import nativekit.ui.docking.DockWorkspaceSnapshot;
import nativekit.ui.docking.DockWorkspacePersistence;
import nativekit.ui.docking.DockWorkspaceStorage;
import nativekit.ui.properties.PropertyDescriptor;
import nativekit.ui.properties.PropertyDescriptorOptions;
import nativekit.ui.properties.PropertyInspectorSection;
import nativekit.ui.properties.PropertyType;
import nativekit.ui.properties.PropertyValue;
import nativekit.ui.core.RenderNode;
import nativekit.ui.core.RetainedView;
import nativekit.ui.core.Shortcut;
import nativekit.ui.core.UiContext;
import nativekit.ui.core.UiEvent;
import nativekit.ui.core.UiKey;
import nativekit.ui.core.UiModifier;
import nativekit.ui.core.TextStyleOverride;
import nativekit.ui.core.View;
import nativekit.ui.widgets.layout.Align;
import nativekit.ui.widgets.layout.AppShell;
import robotkit.world.RobotStatus;
import nativekit.ui.widgets.controls.Button;
import nativekit.ui.widgets.controls.ButtonVariant;
import nativekit.ui.widgets.layout.Column;
import nativekit.ui.widgets.commands.CommandButton;
import nativekit.ui.widgets.commands.CommandMenu;
import nativekit.ui.widgets.commands.CommandPalette;
import nativekit.ui.widgets.docking.DockWorkspace;
import nativekit.ui.widgets.docking.DockPanelContent;
import nativekit.ui.widgets.Icon;
import nativekit.ui.widgets.controls.IconButton;
import nativekit.ui.widgets.KeyedView;
import nativekit.ui.widgets.collections.ListView;
import nativekit.ui.widgets.collections.ListViewModel;
import nativekit.ui.widgets.overlays.Menu;
import nativekit.ui.widgets.overlays.MenuItem;
import nativekit.ui.widgets.overlays.Popup;
import nativekit.ui.widgets.overlays.Tooltip;
import nativekit.ui.widgets.properties.PropertyInspector;
import nativekit.ui.widgets.layout.Row;
import nativekit.ui.widgets.scroll.ScrollController;
import nativekit.ui.widgets.scroll.ScrollView;
import nativekit.ui.widgets.controls.SearchField;
import nativekit.ui.widgets.layout.SizedBox;
import nativekit.ui.widgets.layout.Stack;
import nativekit.ui.widgets.layout.StackChild;
import nativekit.ui.widgets.layout.Spacer;
import nativekit.ui.widgets.layout.SplitOrientation;
import nativekit.ui.widgets.layout.SplitSide;
import nativekit.ui.widgets.layout.SplitView;
import nativekit.ui.widgets.layout.SplitViewOptions;
import nativekit.ui.widgets.controls.TabItem;
import nativekit.ui.widgets.controls.Tabs;
import nativekit.ui.widgets.text.Text;
import nativekit.ui.widgets.text.TextArea;
import nativekit.editorkit.TextDocument;
import nativekit.ui.widgets.text.TextField;
import nativekit.ui.widgets.collections.TreeRootMetadata;
import nativekit.ui.widgets.collections.TreeView;
import nativekit.ui.widgets.collections.TreeViewModel;
import FontCollection;
import cadkit.parametric.ParametricError;
import bimkit.BimDocument;
import robotkit.world.RemoteRobot;
import robotkit.world.RobotWorld;
import nativekit.ui.lab.ComponentLab;
import nativekit.ui.theme.Theme;
import nativekit.ui.icons.IconName;
import nativekit.ui.host.DesktopUiApplication;
import nativekit.ui.host.DesktopUiHost;
import nativekit.ui.host.DesktopUiHostOptions;
import nativekit.ui.host.DesktopUiHostContext;
import nativekit.ui.host.DesktopUiHostSession;
import nativekit.ui.widgets.overlays.Dialog;
import nativekit.ui.settings.ShortcutBindings;
import nativekit.ui.widgets.controls.TabItem;
import nativekit.ui.widgets.controls.Tabs;
import nativekit.ui.widgets.controls.TabsSelectionMode;
import nativekit.ui.widgets.settings.SettingsPanel;
import nativekit.ui.widgets.settings.ShortcutsPanel;

/**
	Small executable driver for the shared reference-editor shell.

	The reusable UIKit desktop host owns the window, rendering, input, and
	diagnostics lifecycle; this executable only configures the editor.
*/
class Main {
  public static inline var DEFAULT_WINDOW_WIDTH:Int = 1600;
  public static inline var DEFAULT_WINDOW_HEIGHT:Int = 1000;
  static var liveEditor:Null<ReferenceEditorApp>;

  public static function liveState():String {
    var editor = liveEditor;
    return editor == null ? "" : editor.liveState();
  }

  public static function restoreLiveState(state:String):Void {
    var editor = liveEditor;
    if (editor != null && state.length > 0) editor.restoreLiveState(state);
  }

  public static function clearLiveEditor():Void liveEditor = null;

  public static function main():Int {
    var args = Sys.args();
    for (arg in args)
      if (arg != "--reset-workspace" && arg != "--snapshot" && arg != "--simulate" &&
          arg != "--lab" && arg != "--dark" && arg != "--perspective" && arg != "--demo" &&
          arg.indexOf("--story=") != 0 &&
          arg.indexOf("--width=") != 0 && arg.indexOf("--height=") != 0 &&
          arg.indexOf("--capture-dir=") != 0 && arg.indexOf("--frames=") != 0 &&
          arg.indexOf("--capture-seconds=") != 0 && arg.indexOf("--msaa=") != 0 &&
          arg.indexOf("--robot=") != 0 && arg.indexOf("--setup-script=") != 0 &&
          arg.indexOf("--project=") != 0 && arg.indexOf("--project-action=") != 0 &&
          arg.indexOf("--example=") != 0 && arg.indexOf("--example-settle=") != 0 && arg != "--example-play" &&
          arg != "--record" && arg.indexOf("--record=") != 0 &&
          arg.indexOf("--character=") != 0 && arg.indexOf("--character-clip=") != 0 &&
          arg.indexOf("--character-hold=") != 0 && arg.indexOf("--character-display=") != 0 &&
          arg.indexOf("--character-route=") != 0 && arg.indexOf("--character-facility-route=") != 0 &&
          arg.indexOf("--character-reach=") != 0 && arg.indexOf("--character-reach-clip=") != 0 &&
          arg != "--worker-demo=rack-to-table" && arg.indexOf("--worker-demo-step=") != 0) {
        Sys.println("Usage: materia [--reset-workspace] [--snapshot [--simulate]] [--demo] " +
          "[--lab] [--dark] [--perspective] [--story=ID] [--width=PX] [--height=PX] " +
          "[--capture-dir=PATH] [--frames=N|--capture-seconds=N] [--msaa=SAMPLES] " +
          "[--robot=HOST:PORT] [--setup-script=REFERENCE] [--project=PATH] " +
          "[--project-action=ID] [--record[=PATH]] [--character=GLTF [--character-clip=NAME] " +
          "[--character-hold=GLTF] [--character-display=mesh|capsules|skeleton] " +
          "[--character-route=X,Y;X,Y;... | --character-facility-route=FROM,TO] " +
          "[--character-reach=X,Y,Z [--character-reach-clip=NAME]]] " +
          "[--worker-demo=rack-to-table [--worker-demo-step=N]] " +
          "[--example=ID[,ID...] [--example-settle=SECONDS] [--example-play]]");
        return 2;
      }

    if (args.indexOf("--snapshot") >= 0) {
      var projectPath:String = "";
      for (arg in args) if (arg.indexOf("--project=") == 0) projectPath = arg.substr(10);
      var editor = args.indexOf("--demo") >= 0 ?
        new ReferenceEditorApp(null, null, null, null, null, null, null, null, true) :
        new ReferenceEditorApp();
      if (projectPath.length > 0) {
        var generated = MateriaProjectRunner.loadProject(projectPath);
        editor.session.openGeneratedScene(generated.objects, projectPath, generated.assembly,
          generated.geometryBySnapshot, generated.assemblyDefinition, generated.assemblyState,
          generated.localCentersByDefinition, generated.metresPerUnit,
          generated.physical, generated.recipeDocument, generated.robotMotions, generated.robotGrips,
          generated.faceDescriptorsByDefinition);
      }
      // Opens bundled examples in order, exactly as the Start page does, for headless checks.
      var settleSeconds = 0.0;
      for (arg in args) if (arg.indexOf("--example-settle=") == 0) settleSeconds = Std.parseFloat(arg.substr(17));
      for (arg in args) if (arg.indexOf("--example=") == 0)
        for (id in arg.substr(10).split(",")) {
          var entry = ExampleCatalog.find(id);
          if (entry == null) throw 'Unknown example "$id"';
          ExampleCatalog.open(editor, entry);
          Sys.println('Opened example ${entry.id}: document "${editor.session.label()}", ' +
            '${editor.scene.records().length} scene records');
          if (args.indexOf("--example-play") >= 0) {
            if (!editor.simulation.rebuild(editor.sensors, editor.scene, editor.session))
              Sys.println('Play rejected: ${editor.simulation.error}');
            else editor.simulation.start();
          }
          // Let a running simulation step, as the interactive editor does between frames.
          var until = Sys.time() + settleSeconds;
          while (Sys.time() < until) {
            editor.tick();
            Sys.sleep(0.016);
          }
        }
      if (args.indexOf("--reset-workspace") >= 0) editor.resetWorkspace();
      if (args.indexOf("--worker-demo=rack-to-table") >= 0) {
        editor.enableWorkerDemo(workerDemoSteps(args), false);
        var worker = editor.simulation.humanWorker("worker-demo");
        var signals = editor.simulation.humanSignals("worker-demo");
        var environment = editor.simulation.environmentVisualState();
        var part = [for (entry in environment) if (entry.id == "worker-demo-part") entry];
        Sys.println(haxe.Json.stringify({workerDemo:"rack-to-table", jobDone:worker != null &&
          worker.currentJobDone(), jobFailure:worker == null ? "missing worker" : worker.currentJobFailure(),
          part:part.length == 0 ? null : part[0].position,
          zones:signals == null ? [] : signals.zones,
          separation:signals == null ? null : signals.separation.get(editor.sensors.robotId)}));
        editor.dispose();
        return 0;
      }
      if (args.indexOf("--simulate") >= 0) {
        if (!editor.simulation.rebuild(editor.sensors, editor.scene, editor.session))
          throw "Snapshot simulation rebuild failed: " + editor.simulation.error;
        editor.simulation.start();
        var frame = editor.simulation.capturePresentationSnapshot();
        Sys.println(haxe.Json.stringify({workspace: haxe.Json.parse(editor.workspace.snapshotJson()),
          simulation: {running: editor.simulation.isRunning(), error: editor.simulation.error,
            generatedParts: frame.environment.length}}));
      } else Sys.println(editor.workspace.snapshotJson());
      editor.dispose();
      return 0;
    }
    var live = open(args);
    while (live.tick()) {}
    return live.close();
  }

  /** Open the same editor as main(), while letting an embedding host pump it. */
  public static function open(args:Array<String>):DesktopUiHostSession {
    var diagnostics = ReferenceEditorLaunchOptions.fromArgs(args);
    if (diagnostics == null) throw "Invalid editor launch options";
    var host = new DesktopUiHostOptions();
    // Captures screenshot their window by title, so give each capture run a title no other window shares.
    host.title = diagnostics.captureDirectory == null ? "Materia" : "Materia capture " + Sys.getPid();
    host.icons = MateriaIcon.create();
    host.width = diagnostics.windowWidth;
    host.height = diagnostics.windowHeight;
    host.captureDirectory = diagnostics.captureDirectory;
    host.recordPath = diagnostics.recordPath;
    host.recordMetadata = {arguments:args.copy(), cwd:Sys.getCwd()};
    host.frameLimit = diagnostics.frameLimit;
    host.captureSeconds = diagnostics.captureSeconds;
    var activeEditor:Null<ReferenceEditorApp> = null;
    // A launch project builds in the background; captures wait for it so they show the project.
    host.captureReady = function() return activeEditor == null || !activeEditor.openingProject();
    host.continuousFrames = function() return activeEditor != null &&
      (activeEditor.simulation.isRunning() || diagnostics.robotHost != null ||
        activeEditor.hasCharacterPreview());
    var hosted = DesktopUiHost.open(host, function(context) {
      // --dark overrides the saved color scheme for this run only.
      var activeTheme:Null<Theme> = diagnostics.darkTheme ? Theme.dark() : null;
      var world:Null<RobotWorld> = null;
      var robotHost = diagnostics.robotHost;
      if (robotHost != null) {
        world = new RobotWorld();
        var remote = new RemoteRobot("warehouse/forklift-17");
        world.attach(remote);
        remote.connect(robotHost, diagnostics.robotPort, context.events);
      }
      var editor = new ReferenceEditorApp(context.fonts, null, activeTheme, world, context,
        diagnostics.setupScript, diagnostics.projectPath, diagnostics.recordPath, diagnostics.demo);
      activeEditor = editor;
      liveEditor = editor;
      for (arg in args) if (arg.indexOf("--project-action=") == 0)
        editor.projectInitialAction = arg.substr(17);
      for (arg in args) if (arg.indexOf("--msaa=") == 0) {
        var samples = Std.parseInt(arg.substr(7));
        if (samples == null || samples < 1) throw "--msaa takes a sample count of 1 or more";
        editor.setAntialiasing(samples, false);
      }
      if (diagnostics.componentLab) editor.enableComponentLab(diagnostics.storyId);
      for (arg in args) if (arg.indexOf("--character=") == 0) {
        var clip:Null<String> = null;
        var hold:Null<String> = null;
        var display = humankit.HumanDisplay.Mesh;
        var route:Null<Array<Array<Float>>> = null;
        var facilityRoute = false;
        var reachTarget:Null<Array<Float>> = null;
        var reachClip:Null<String> = null;
        for (option in args) {
          if (option.indexOf("--character-clip=") == 0) clip = option.substr(17);
          if (option.indexOf("--character-hold=") == 0) hold = option.substr(17);
          if (option.indexOf("--character-display=") == 0)
            display = humankit.HumanDisplays.parse(option.substr(20));
          if (option.indexOf("--character-route=") == 0) {
            if (facilityRoute)
              throw "--character-route cannot be combined with --character-facility-route";
            route = [for (point in option.substr(18).split(";")) {
              var coordinates = point.split(",");
              if (coordinates.length != 2) throw "--character-route takes X,Y points separated by ;";
              [Std.parseFloat(coordinates[0]), Std.parseFloat(coordinates[1])];
            }];
          }
          if (option.indexOf("--character-facility-route=") == 0) {
            if (route != null)
              throw "--character-facility-route cannot be combined with --character-route";
            var stations = option.substr(option.indexOf("=") + 1).split(",");
            if (stations.length != 2)
              throw "--character-facility-route takes FROM,TO station IDs";
            route = app.editor.FacilityRouteDemo.route(stations[0], stations[1]);
            facilityRoute = true;
          }
          if (option.indexOf("--character-reach=") == 0) {
            var coordinates = option.substr(option.indexOf("=") + 1).split(",");
            if (coordinates.length != 3) throw "--character-reach takes an X,Y,Z model-space target";
            reachTarget = [for (value in coordinates) Std.parseFloat(value)];
          }
          if (option.indexOf("--character-reach-clip=") == 0)
            reachClip = option.substr(option.indexOf("=") + 1);
        }
        editor.enableCharacterPreview(arg.substr(12), clip, hold, display, route, reachTarget, reachClip);
      }
      if (args.indexOf("--reset-workspace") >= 0) editor.resetWorkspace();
      if (args.indexOf("--worker-demo=rack-to-table") >= 0)
        editor.enableWorkerDemo(workerDemoSteps(args), true);
      if (args.indexOf("--perspective") >= 0) editor.workspace.activate("perspective");
      // Start shows for a plain launch; explicit documents, demos, and previews go straight to the model.
      var explicitContent = diagnostics.projectPath != null || diagnostics.setupScript != null ||
        diagnostics.demo || diagnostics.componentLab || args.indexOf("--perspective") >= 0 ||
        args.indexOf("--worker-demo=rack-to-table") >= 0 ||
        Lambda.exists(args, function(arg) return arg.indexOf("--character=") == 0);
      if (!explicitContent && editor.preferences.showStartPage) editor.showStartPage();
      return editor;
    });
    return new DesktopUiHostSession(function() {
      var active = hosted.tick();
      if (activeEditor != null) activeEditor.tick();
      return active;
    }, function() return hosted.close());
  }

  static function workerDemoSteps(args:Array<String>):Int {
    for (arg in args) if (arg.indexOf("--worker-demo-step=") == 0) {
      var value = Std.parseInt(arg.substr(19));
      if (value == null || value < 0 || value > 3600) throw "--worker-demo-step needs 0 to 3600 ticks";
      return value;
    }
    return 0;
  }
}

private class ReferenceEditorLaunchOptions {
  public final captureDirectory:Null<String>;
  public final recordPath:Null<String>;
  public final frameLimit:Int;
  public final captureSeconds:Float;
  public final componentLab:Bool;
  public final storyId:Null<String>;
  public final darkTheme:Bool;
  public final demo:Bool;
  public final robotHost:Null<String>;
  public final robotPort:Int;
  public final setupScript:Null<String>;
  public final projectPath:Null<String>;
  public final windowWidth:Int;
  public final windowHeight:Int;
  public function new(captureDirectory:Null<String>, recordPath:Null<String>, frameLimit:Int, captureSeconds:Float,
      componentLab:Bool, storyId:Null<String>, darkTheme:Bool, demo:Bool,
      robotHost:Null<String>, robotPort:Int,setupScript:Null<String>,projectPath:Null<String>,
      windowWidth:Int, windowHeight:Int) {
    this.captureDirectory = captureDirectory;
    this.recordPath = recordPath;
    this.frameLimit = frameLimit;
    this.captureSeconds = captureSeconds;
    this.componentLab = componentLab;
    this.storyId = storyId;
    this.darkTheme = darkTheme;
    this.demo = demo;
    this.robotHost = robotHost;
    this.robotPort = robotPort;
    this.setupScript=setupScript;
    this.projectPath=projectPath;
    this.windowWidth = windowWidth;
    this.windowHeight = windowHeight;
  }

  public static function fromArgs(args:Array<String>):Null<ReferenceEditorLaunchOptions> {
    var directory:Null<String> = null;
    var recordPath:Null<String> = null;
    var frames = 0;
    var captureSeconds = 0.0;
    var lab = args.indexOf("--lab") >= 0;
    var story:Null<String> = null;
    var robotEndpoint:Null<String> = null;
    var setupScript:Null<String> = null;
    var projectPath:Null<String> = null;
    var windowWidth = Main.DEFAULT_WINDOW_WIDTH;
    var windowHeight = Main.DEFAULT_WINDOW_HEIGHT;
    for (arg in args) {
      if (arg.indexOf("--capture-dir=") == 0)
        directory = arg.substr(14);
      else if (arg == "--record")
        recordPath = "app/build/recordings/materia-" + StringTools.replace(Std.string(Sys.time()), ".", "-") +
          "-" + Sys.getPid() + ".jsonl";
      else if (arg.indexOf("--record=") == 0)
        recordPath = arg.substr(9);
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
      } else if (arg.indexOf("--project=") == 0) {
        projectPath = arg.substr(10);
      }
    }
    if (directory != null && directory.length == 0) {
      Sys.println("materia: --capture-dir requires a path");
      return null;
    }
    if (recordPath != null && recordPath.length == 0) {
      Sys.println("materia: --record requires a non-empty path");
      return null;
    }
    if(setupScript!=null&&StringTools.trim(setupScript).length==0){
      Sys.println("materia: --setup-script requires a registered reference");return null;
    }
    if(projectPath!=null&&StringTools.trim(projectPath).length==0){
      Sys.println("materia: --project requires a path");return null;
    }
    if(setupScript!=null&&projectPath!=null){
      Sys.println("materia: --project cannot be combined with --setup-script");return null;
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
    return new ReferenceEditorLaunchOptions(directory, recordPath, frames, captureSeconds, lab, story,
      args.indexOf("--dark") >= 0, args.indexOf("--demo") >= 0,
      robotHost, robotPort,setupScript,projectPath,
      windowWidth, windowHeight);
  }
}

/** Shared/app integration object passed to a platform frame loop. */
class ReferenceEditorApp implements DesktopUiApplication {
  public static inline var WORKSPACE_KEY:String = "reference-editor";
  static inline var FILE_BAR_HEIGHT:Float = 34.0;
  static inline var CONTEXT_BAR_HEIGHT:Float = 40.0;
  // File tier, one-pixel divider, and mode/transport tier.
  static inline var TOOLBAR_HEIGHT:Float = 75.0;
  static inline var STATUS_HEIGHT:Float = 28.0;

  public final ui:UiContext;
  var appearance:EditorAppearance;
  public final commands:CommandRegistry;
  public final workspace:DockWorkspaceModel;
  var workspacePanelContents:Array<DockPanelContent>;
  public final workspacePath:String;
  public final world:RobotWorld;
  public final simulation:ApplicationSimulation;
  public var bimModel(get, never):BimDocument;
  function get_bimModel():BimDocument return session.bim;
  var bimEditor:BimModelEditor;

  final storage:FileDockWorkspacePersistence;
  final workspaceSaves:WorkspaceSaveWorker;
  public final session:ProjectDocumentSession;
  public final documents:SceneDocumentController;
  public var scene(get, never):EditorScene;
  function get_scene():EditorScene return session.scene;
  final files:Null<SceneFileDialogs>;
  var sceneGeneration:Int = 0;
  var treeModel:EditorSceneTree;
  final telemetry:TelemetryPanel;
  static inline var MAX_LOG_LINES:Int = 1000;
  final logLines:Array<String>;
  // The console view edits nothing: it is a read-only text area over this document, so its text is
  // selectable and copyable. logLengths holds each entry's code-point length for trimming old lines.
  final consoleDocument:TextDocument = new TextDocument("");
  final logLengths:Array<Int> = [];
  var gridVisible:Bool;
  var gridSnapEnabled:Bool;
  var gridSpacing:Float;
  /** The viewport's lighting preset number, from the saved setting. */
  var lightingPreset:Int = 0;
  var paletteVisible:Bool;
  var toolbarMenuVisible:Bool;
  var viewportWidth:Float = Main.DEFAULT_WINDOW_WIDTH;
  var viewportHeight:Float = Main.DEFAULT_WINDOW_HEIGHT;
  var toolbarDensity:EditorToolbarDensity = Full;
  public var mode(default, null):EditorMode = EditorMode.Design;
  public var preferences(default, null):AppPreferences;
  /** The open Editor Settings dialog's state, or null while it is closed. */
  public var settingsPanel(default, null):Null<SettingsPanel> = null;
  public var shortcutsPanel(default, null):Null<ShortcutsPanel> = null;
  /** "general" or "shortcuts": the Editor Settings tab shown. */
  public var settingsTab(default, null):String = "general";
  /** The user's shortcut choices, applied to `commands` and saved with the settings. */
  public var shortcutBindings(default, null):ShortcutBindings;
  /** Samples per pixel the 3D viewport asks for; one is off. Kept when the viewport is replaced. */
  public var antialiasing(default, null):Int = AppPreferences.DEFAULT_ANTIALIASING;
  // Start page example that is queued to open; it runs a few frames later so "Opening..." is visible first.
  var startLoading:Null<ExampleEntry> = null;
  var startLoadDelay:Int = 0;
  // The worker-thread build behind startLoading, when the example is a project.
  var startJob:Null<ProjectLoadJob> = null;
  var startFailure:Null<String> = null;
  // The Start page was opened only to show a launch project's progress, so it closes once that opens.
  var closeStartAfterOpen:Bool = false;
  var lastRecordedPath:Null<String> = null;
  // Each mode keeps the layout it was left in; the mode set itself is never persisted.
  final modeSnapshots:Map<String, DockWorkspaceSnapshot> = new Map();
  // Mode to return to when the simulation stops after Play switched into Simulate.
  var modeBeforePlay:Null<EditorMode> = null;
  var contextMenuVisible:Bool;
  var hierarchyAddVisible:Bool = false;
  var hierarchySearch:String = "";
  var hierarchyExpansionRevision:Int = 0;
  var hierarchyMenuVisible:Bool = false;
  var hierarchyMenuX:Float = 0.0;
  var hierarchyMenuY:Float = 0.0;
  var hierarchyAddX:Float = 12.0;
  var hierarchyAddY:Float = 120.0;
  var renameId:Null<String> = null;
  var renameValue:String = "";
  var viewportOptionsVisible:Bool = false;
  var viewportOptionsX:Float = 0.0;
  var viewportOptionsY:Float = 0.0;
  var viewAngleMenuVisible:Bool = false;
  var viewAngleMenuX:Float = 0.0;
  var viewAngleMenuY:Float = 0.0;
  var contextMenuX:Float;
  var contextMenuY:Float;
  var componentLab:Null<ComponentLab>;
  var characterPreview:Null<CharacterPreview> = null;
  var perspectiveViewport:Null<EditorPerspectiveViewport> = null;
  final hostContext:Null<DesktopUiHostContext>;
  var sceneInspector:Null<PropertyInspector> = null;
  var inspectorSelectionRevision:Int = -1;
  /** Retained sensor inspector; its editors own drafts and validation state. */
  var sensorInspector:Null<PropertyInspector> = null;
  var sensorInspectorSensor:Dynamic = null;
  var sensorInspectorModel:Dynamic = null;
  /** Companion sheet-manufacturing record for projects that include one. */
  var sheetInventory:Null<SheetInventory> = null;
  var sheetInventoryPath:Null<String> = null;
  var sheetPlanSelection:String = "";
  var sheetPieceSelection:String = "";
  var sheetOperationsExpanded:Bool = false;
  var projectUiExtension:Null<ProjectUiExtension> = null;
  var projectUiReference:Null<String> = null;
  var projectUiError:Null<String> = null;
  public var projectInitialAction:Null<String> = null;
  var framePresentation:Null<ApplicationPresentationSnapshot> = null;
  var refinementFrameSubmitted:Bool = false;
  var cachedSubmitKey:String = "";
  var viewRevision:Int = 0;
  var cachedSubmitViewRevision:Int = -1;
  var cachedSubmitSceneGeneration:Int = -1;
  var cachedSubmitSceneRevision:Int = -1;
  var cachedSubmitSelectionRevision:Int = -1;
  var cachedSubmitEnvironmentRevision:Int = -1;
  var cachedSubmitSensorRevision:Int = -1;
  var cachedSubmitSimulationRevision:Int = -1;
  var cachedSubmitPerspectiveKey:String = "";
  final semanticRecordPath:Null<String>;
  var lastRecordedSelection:String = "";
  var lastSemanticAction:Dynamic = null;
  final externalWorldHasRobots:Bool;
  public var sensors(get, never):SensorConfiguration;
  function get_sensors():SensorConfiguration return session.sensors;

  public function new(? fonts:FontCollection, ? workspaceFile:String, ?theme:Theme,
      ?world:RobotWorld, ?hostContext:DesktopUiHostContext,?setupScript:String,?projectPath:String,
      ?recordPath:String, demo:Bool = false) {
    this.hostContext = hostContext;
    semanticRecordPath = recordPath;
    workspacePath = workspaceFile == null || workspaceFile.length == 0 ? defaultWorkspacePath() : workspaceFile;
    preferences = AppPreferences.besideWorkspace(workspacePath);
    appearance = new EditorAppearance(theme != null ? theme : savedTheme());
    ui = new UiContext(null, fonts, appearance.theme);
    commands = ui.commands;
    if (semanticRecordPath != null) commands.onInvoked = function(id, result) {
      recordSemantic("command", {id:id, status:Std.string(result.status),
        changed:result.changed, selected:scene.selectedId,
        documentPath:session.path, documentDirty:session.isDirty()});
    };
    this.world = world == null ? new RobotWorld() : world;
    externalWorldHasRobots = this.world.robotIds().length > 0;
    simulation = new ApplicationSimulation(this.world,ApplicationSimulation.MUJOCO);
    session = new ProjectDocumentSession(demo ? BimEditorDemo.create() : null, demo);
    attachSceneRecorder();
    storage = new FileDockWorkspacePersistence(workspacePath);
    antialiasing = preferences.antialiasing;
    // The settings dialog edits the store directly; apply what it changes.
    preferences.store.onChanged(AppSettings.ANTIALIASING, function(_) {
      if (preferences.antialiasing != antialiasing) setAntialiasing(preferences.antialiasing, false);
    });
    preferences.store.onChanged(AppSettings.COLOR_SCHEME, function(_) {
      var dark = preferences.store.getString(AppSettings.COLOR_SCHEME) == "dark";
      if (dark != appearance.dark) applyTheme(dark ? Theme.dark() : Theme.light());
    });
    preferences.store.onChanged("editors/3d", function(_) readViewSettings());
    preferences.store.onChanged("", function(_) { if (settingsPanel != null) invalidateView(); });
    session.beforeReplace=simulation.clear;
    if(setupScript!=null){var scripted=session.openScript(setupScript);
      simulation.setBackend(scripted.backend);simulation.setTimestep(scripted.timestep);}
    // A window must paint its first frame before a project builds, so a hosted editor opens the launch
    // project on a worker thread once it is up. Without a window there is nothing to keep responsive.
    var deferredProject:Null<String> = null;
    if(projectPath!=null){
      if(hostContext!=null) deferredProject=projectPath;
      else {
        var generated=MateriaProjectRunner.loadProject(projectPath);
        session.openGeneratedScene(generated.objects, projectPath, generated.assembly,
          generated.geometryBySnapshot, generated.assemblyDefinition, generated.assemblyState,
          generated.localCentersByDefinition, generated.metresPerUnit,
          generated.physical, generated.recipeDocument, generated.robotMotions, generated.robotGrips,
          generated.faceDescriptorsByDefinition);
      }
    }
    bimEditor = makeBimEditor();
    files = hostContext == null ? null : new SceneFileDialogs(hostContext);
    documents = new SceneDocumentController(session, function(save, path, complete) {
      var chooser = files;
      if (chooser == null) complete(null, "File dialogs require the desktop host");
      else chooser.choose(save, path, complete);
    }, documentChanged, commitActiveDrag, cancelActiveDrag);
    documents.busy = function() return startLoading != null;
    if (hostContext != null) hostContext.onCloseRequested = function(close) {
      // A running build must not keep the window from closing: cancel it and let the normal prompt run.
      if (startJob != null) {
        startJob.control.cancel();
        startJob = null;
        startLoading = null;
      }
      documents.requestClose(close);
    };
    treeModel = new EditorSceneTree(scene, session.projectAssemblyDefinition, session.generatedLabels());
    if (hostContext != null) {
      perspectiveViewport = new EditorPerspectiveViewport("scene-perspective", scene,
        hostContext);
      perspectiveViewport.setSampleCount(antialiasing);
      perspectiveViewport.setLightingPreset(lightingPreset);
    }
    telemetry = new TelemetryPanel(demo);
    logLines = [];
    for (line in demo ? ["Demo scene ready", "Select a box; edit position or visibility",
        "Middle-drag to pan; scroll to zoom"] : ["Scene ready", "Use Add to create an object"])
      log(line);
    readViewSettings();
    paletteVisible = false;
    toolbarMenuVisible = false;
    contextMenuVisible = false;
    contextMenuX = 0.0;
    contextMenuY = 0.0;
    componentLab = null;
    scene.onSelectionChanged = updateCommandContext;
    updateCommandContext();

    workspace = makeWorkspace();
    DockWorkspaceStorage.restoreOrDefault(workspace, storage, WORKSPACE_KEY);
    EditorWorkspaceLayout.migrateLegacyViewport(workspace);
    // Layouts saved before transient panels existed can still contain them; they open only on request.
    for (id in TRANSIENT_PANELS) if (workspace.isOpen(id)) workspace.close(id);
    if(deferredProject!=null) openProjectInBackground(deferredProject);
    else if(projectPath!=null&&perspectiveViewport!=null){
      workspace.activate("perspective");
      scene.select("scene");
      perspectiveViewport.frameSelected();
    }
    workspaceSaves = new WorkspaceSaveWorker(storage, WORKSPACE_KEY);
    workspace.listen(queueWorkspaceSave);
    installCommands();
    shortcutBindings = new ShortcutBindings(commands, preferences.store);
  }

  /** Build the shared view tree for one host frame. */
  public function view():View {
    framePresentation = null;
    if (componentLab != null) return componentLab.view(ui);
    framePresentation = simulation.capturePresentationSnapshot();
    var workspaceView = new DockWorkspace("reference-workspace", workspace,
      workspacePanelContents);
    workspaceView.availableHeight = Math.max(0.0, viewportHeight - TOOLBAR_HEIGHT - STATUS_HEIGHT);
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
        "assembly.mate-planar",
        "assembly.mate-coaxial",
        "assembly.mate-parallel",
        "assembly.mate-perpendicular",
        "scene.toggle-grid",
        "scene.toggle-grid-snap",
        "scene.grid-spacing-0.1",
        "scene.grid-spacing-0.2",
        "scene.grid-spacing-0.5",
        "workspace.reset"
      ], contextMenuX, contextMenuY, commands, ui.commandContext, function() {
        contextMenuVisible = false;
        invalidateView();
      }
      );
      layers.push(new StackChild("context-menu", menu, 0.0, 0.0, 20));
    }
    var documentDialog = makeDocumentDialog();
    if (documentDialog != null) layers.push(new StackChild("document-dialog", documentDialog,
      0.0, 0.0, 100, LayoutAxis.grow(), LayoutAxis.grow()));
    var shellStyle = fillStyle();
    shellStyle.background = appearance.canvas;
    var shell = new AppShell("reference-editor-shell", new Stack("overlay-host", layers),
      new RetainedView("editor-top-bar", function(_) return topBar(),
        function() return chromeRevisionKey()),
      null, null, shellStyle, null,
      new RetainedView("editor-status-bar", function(_) return statusBar(),
        function() return chromeRevisionKey()));
    var windowLayers:Array<StackChild> = [new StackChild("shell", shell, 0.0, 0.0, 0,
      LayoutAxis.grow(), LayoutAxis.grow())];
    if (toolbarMenuVisible && documentDialog == null) {
      var toolbarMenu = new CommandMenu("editor-more-menu", [
        "editor.save-as", "scene.export-step", "editor.undo", "editor.redo",
        "scene.frame-selected", "scene.reset-perspective",
        "scene.lighting-studio", "scene.lighting-soft", "scene.lighting-contrast",
        "scene.toggle-grid", "editor.toggle-dark-theme", "start.show", "editor.command-palette", "editor.settings",
        "workspace.reset"
      ], Math.max(8.0, viewportWidth - 228.0), FILE_BAR_HEIGHT, commands, ui.commandContext,
        function() { toolbarMenuVisible = false; invalidateView(); },
        function(_) { toolbarMenuVisible = false; invalidateView(); });
      windowLayers.push(new StackChild("editor-more-menu", toolbarMenu, 0.0, 0.0, 25));
    }
    if (hierarchyAddVisible && documentDialog == null) {
      var items:Array<MenuItem> = [];
      var addSection = function(name:String, ids:Array<String>) {
        items.push(new MenuItem("heading:" + name, name.toUpperCase(), null, false));
        for (id in ids) {
          var command = commands.get(id);
          if (command == null) continue;
          var selectedId = id;
          items.push(new MenuItem(id, command.label, function() {
            commands.executeContext(selectedId, ui.commandContext);
            invalidateView();
          }, command.isEnabled(ui.commandContext)));
        }
      };
      addSection("Primitive", ObjectKindRegistry.addMenuCommands("Primitive"));
      addSection("CAD", ObjectKindRegistry.addMenuCommands("CAD"));
      addSection("Machining", ObjectKindRegistry.addMenuCommands("Machining"));
      addSection("People", ObjectKindRegistry.addMenuCommands("People"));
      addSection("Feature", ["scene.create-sketch", "scene.create-face-sketch",
        "scene.create-extrusion", "scene.add-face-hole", "scene.create-pocket",
        "scene.create-vertical-fillet"]);
      addSection("Import", ObjectKindRegistry.addMenuCommands("Import"));
      var addMenu = new Menu("hierarchy-add-menu", items, hierarchyAddX, hierarchyAddY,
        function() { hierarchyAddVisible = false; invalidateView(); });
      windowLayers.push(new StackChild("hierarchy-add-menu", addMenu, 0.0, 0.0, 25));
    }
    if (hierarchyMenuVisible && documentDialog == null) {
      var objectMenu = new Menu("hierarchy-object-menu", [
        new MenuItem("rename", "Rename (F2)", function() startRename(scene.selectedId), canEditObjects()),
        new MenuItem("duplicate", "Duplicate", function() commands.execute("scene.duplicate"),
          commands.get("scene.duplicate").isEnabled(ui.commandContext)),
        new MenuItem("delete", "Delete", function() commands.execute("scene.delete"),
          commands.get("scene.delete").isEnabled(ui.commandContext)),
        new MenuItem("frame", "Frame selected", function() commands.execute("scene.frame-selected"))
      ], hierarchyMenuX, hierarchyMenuY, function() { hierarchyMenuVisible = false; invalidateView(); });
      windowLayers.push(new StackChild("hierarchy-object-menu", objectMenu, 0.0, 0.0, 26));
    }
    if (renameId != null && documentDialog == null) {
      windowLayers.push(new StackChild("hierarchy-rename-dialog", renameDialog(), 0.0, 0.0, 40,
        LayoutAxis.grow(), LayoutAxis.grow()));
    }
    if (viewportOptionsVisible && documentDialog == null) {
      var optionIds = ["scene.grid-spacing-0.1", "scene.grid-spacing-0.2", "scene.grid-spacing-0.5"];
      optionIds = optionIds.concat([
        "scene.lighting-studio", "scene.lighting-soft", "scene.lighting-contrast"]);
      optionIds = optionIds.concat([
        "scene.antialiasing-off", "scene.antialiasing-2x", "scene.antialiasing-4x"]);
      var options = new CommandMenu("viewport-options-menu", optionIds,
        viewportOptionsX, viewportOptionsY, commands, ui.commandContext,
        function() { viewportOptionsVisible = false; invalidateView(); },
        function(_) { viewportOptionsVisible = false; invalidateView(); });
      windowLayers.push(new StackChild("viewport-options-menu", options, 0.0, 0.0, 25));
    }
    if (viewAngleMenuVisible && documentDialog == null) {
      var angleMenu = new Menu("view-angle-menu", [
        new MenuItem("default", "Default perspective", function() {
          if (perspectiveViewport != null) perspectiveViewport.resetView();
        }),
        new MenuItem("top", "Top view", function() {
          if (perspectiveViewport != null) perspectiveViewport.setViewAngle(-Math.PI / 2, 1.48);
        }),
        new MenuItem("front", "Front view", function() {
          if (perspectiveViewport != null) perspectiveViewport.setViewAngle(-Math.PI / 2, 0.0);
        }),
        new MenuItem("right", "Right view", function() {
          if (perspectiveViewport != null) perspectiveViewport.setViewAngle(0.0, 0.0);
        })
      ], viewAngleMenuX, viewAngleMenuY, function() {
        viewAngleMenuVisible = false;
        invalidateView();
      });
      windowLayers.push(new StackChild("view-angle-menu", angleMenu, 0.0, 0.0, 25));
    }
    if (settingsPanel != null && documentDialog == null) {
      windowLayers.push(new StackChild("editor-settings-dialog", settingsDialog(settingsPanel), 0.0, 0.0, 45,
        LayoutAxis.grow(), LayoutAxis.grow()));
    }
    if (paletteVisible && documentDialog == null) {
      var palette = new CommandPalette("reference-command-palette",
        commands, ui.commandContext, 0.0, 0.0, "", function() {
          paletteVisible = false;
          invalidateView();
        }, function(_) {
          log("Command executed from palette");
          invalidateView();
        });
      palette.centered = true;
      windowLayers.push(new StackChild("command-palette", palette, 0.0, 0.0, 1000,
        LayoutAxis.grow(), LayoutAxis.grow()));
    }
    return new Stack("window-overlay-host", windowLayers);
  }

  /** Convenience entry point for a NativeKit host's layout phase. */
  public function submit(frame:LayoutFrame):RenderNode {
    var selectionKey = scene.treeSelectionKey();
    if (selectionKey != lastRecordedSelection) {
      lastRecordedSelection = selectionKey;
      recordSemantic("selection", {id:selectionKey});
    }
    var saveError = workspaceSaves.takeError();
    if (saveError != null) log("Workspace save failed: " + saveError);
    viewportWidth = frame.width;
    viewportHeight = frame.height;
    toolbarDensity = EditorToolbarLayout.forWidth(toolbarDensity, frame.width);
    updateReadOnlyRobots();
    refinementFrameSubmitted = true;
    // A stable editor frame does not need to reconstruct its declarative tree.
    // Keep live simulation, component stories, and externally populated worlds
    // on the normal path because their presentation can change independently
    // of the UI revision counters.
    if (componentLab != null || simulation.isActive() || externalWorldHasRobots)
      return ui.submit(view(), frame);
    return ui.submitCached(function() return view(), frame, editorSubmitKey());
  }

  /** Advance the character preview, then CAD preview refinement after a rendered frame. */
  public function tick():Void {
    var queued = startLoading;
    if (queued != null) {
      var job = startJob;
      if (job != null) {
        // A project is building on its worker thread: keep the frame loop going for the spinner and phase.
        if (!job.isFinished()) {
          if (hostContext != null) hostContext.requestFrame();
        } else {
          startJob = null;
          startLoading = null;
          if (job.wasCancelled()) log("Cancelled opening " + queued.title);
          else try {
            ExampleCatalog.finish(this, queued, job.take());
            if (closeStartAfterOpen) workspace.close("start");
          } catch (failure:Dynamic) {
            startFailure = "Could not open " + queued.title + ": " + Std.string(failure);
            log(startFailure);
          }
          closeStartAfterOpen = false;
          invalidateView();
        }
      } else if (startLoadDelay > 0) {
        startLoadDelay--;
        if (hostContext != null) hostContext.requestFrame();
      } else {
        try startJob = ExampleCatalog.begin(this, queued) catch (failure:Dynamic) {
          startFailure = "Could not open " + queued.title + ": " + Std.string(failure);
          log(startFailure);
        }
        // Quick examples opened inside begin(); only a project leaves a job to wait for.
        if (startJob == null) startLoading = null;
        invalidateView();
      }
    }
    if (simulation.isActive() && simulation.isRunning()) {
      simulation.pump();
      if (hostContext != null) hostContext.requestFrame();
    }
    if (characterPreview != null) {
      characterPreview.advance(scene);
      if (hostContext != null) hostContext.requestFrame();
    }
    if (!refinementFrameSubmitted) return;
    refinementFrameSubmitted = false;
    var before = scene.visualRevision;
    var needsFrame = scene.advanceCadMeshRefinement();
    if ((needsFrame || scene.visualRevision != before) && hostContext != null)
      hostContext.requestFrame();
  }

  /** True from launch until the project named on the command line has opened, failed, or been cancelled. */
  public function openingProject():Bool return startLoading != null;

  /**
   * Queues a project as if it had been chosen on the Start page, and shows that page so the build's
   * progress and Cancel button are visible while it runs. The build runs on a worker thread, so the
   * caller returns at once and the window keeps painting; `tick()` opens the result.
   */
  public function openProjectInBackground(path:String):Void {
    startLoading = ExampleCatalog.launchEntry(path);
    startLoadDelay = 3;
    closeStartAfterOpen = workspace.get("start") != null && !workspace.isOpen("start");
    showStartPage();
  }

  /** Opens the Start page beside the 3D view. */
  public function showStartPage():Void {
    if (workspace.get("start") == null) return;
    workspace.open("start", "perspective");
    invalidateView();
  }

  /** Queues a bundled example; the unsaved-change prompt runs first. */
  function requestExample(entry:ExampleEntry):Void {
    startFailure = null;
    documents.requestRun(function() {
      startLoading = entry;
      startLoadDelay = 3;
      invalidateView();
    });
  }

  /** Stops a project build that is running for the Start page. */
  function cancelExampleLoad():Void {
    var job = startJob;
    if (job == null) return;
    job.control.phase("Cancelling");
    job.control.cancel();
    invalidateView();
  }

  function requestOpenPath(path:String):Void {
    startFailure = null;
    documents.requestOpenPath(path);
  }

  function invalidateView():Void {
    viewRevision++;
    if (hostContext != null) hostContext.requestFrame();
  }

  function editorSubmitKey():String {
    var sceneRevision = scene.revision;
    var selectionRevision = scene.selectionRevision;
    var environmentRevision = scene.environmentRevision;
    var sensorRevision = session.document.revision;
    var simulationRevision = simulation.appliedRevision;
    var perspectiveKey = perspectiveViewport == null ? "" : perspectiveViewport.presentationKey();
    if (cachedSubmitViewRevision != viewRevision ||
        cachedSubmitSceneGeneration != sceneGeneration ||
        cachedSubmitSceneRevision != sceneRevision ||
        cachedSubmitSelectionRevision != selectionRevision ||
        cachedSubmitEnvironmentRevision != environmentRevision ||
        cachedSubmitSensorRevision != sensorRevision ||
        cachedSubmitSimulationRevision != simulationRevision ||
        cachedSubmitPerspectiveKey != perspectiveKey) {
      cachedSubmitViewRevision = viewRevision;
      cachedSubmitSceneGeneration = sceneGeneration;
      cachedSubmitSceneRevision = sceneRevision;
      cachedSubmitSelectionRevision = selectionRevision;
      cachedSubmitEnvironmentRevision = environmentRevision;
      cachedSubmitSensorRevision = sensorRevision;
      cachedSubmitSimulationRevision = simulationRevision;
      cachedSubmitPerspectiveKey = perspectiveKey;
      cachedSubmitKey = buildEditorSubmitKey(sceneGeneration, sceneRevision,
        environmentRevision, sensorRevision, simulationRevision, perspectiveKey,
        selectionRevision) + ":view:" + viewRevision;
    }
    return cachedSubmitKey;
  }

  static function buildEditorSubmitKey(sceneGeneration:Int, sceneRevision:Int,
      environmentRevision:Int, sensorRevision:Int, simulationRevision:Int,
      perspectiveKey:String, ?selectionRevision:Int = 0):String {
    return "editor:" + sceneGeneration + ":" + sceneRevision + ":" +
      environmentRevision + ":" + sensorRevision + ":" + simulationRevision +
      ":selection:" + selectionRevision + ":perspective:" + perspectiveKey;
  }

  public function context():UiContext return ui;

  public function dispose():Void {
    if (projectUiExtension != null) projectUiExtension.dispose();
    var saveError = workspaceSaves.close();
    if (saveError != null) log("Workspace save failed: " + saveError);
    simulation.dispose();
    world.close();
    if (files != null) files.dispose();
    if (perspectiveViewport != null) perspectiveViewport.dispose();
    if (characterPreview != null) characterPreview.dispose();
    session.dispose();
    ui.dispose();
  }

  public function liveState():String return Json.stringify({
    document: session.liveState(),
    selected: scene.selectedId,
    workspace: workspace.snapshotJson()
  });

  public function restoreLiveState(source:String):Void {
    var state:Dynamic = Json.parse(source);
    var document:String = Reflect.field(state, "document");
    var selected:String = Reflect.field(state, "selected");
    var layout:String = Reflect.field(state, "workspace");
    session.restoreLiveState(document);
    documentChanged();
    if (selected != null) scene.select(selected);
    if (layout != null) workspace.restoreJson(layout);
    if (hostContext != null) hostContext.requestFrame();
  }

  /** Shows an animated glTF character walking around the origin; it is not saved. */
  public function enableCharacterPreview(path:String, ?clipName:String, ?propPath:String,
      display:humankit.HumanDisplay = humankit.HumanDisplay.Mesh, ?route:Array<Array<Float>>,
      ?reachTarget:Array<Float>, ?reachClip:String):Void {
    releaseCharacterPreview();
    var preview = new CharacterPreview(path, clipName, propPath, display, route, reachTarget, reachClip);
    characterPreview = preview;
    // A humanoid preview walks through the shared simulation as a person.
    simulation.addParticipant(preview);
    if (hostContext != null) hostContext.requestFrame();
  }

  /** Opens the saved rack-to-table worker example and starts its simulation. */
  public function enableWorkerDemo(advanceTicks:Int = 0, realtime:Bool = false):Void {
    if (advanceTicks < 0) throw "Worker demo ticks must be non-negative";
    var path = app.editor.WorkerAssetPath.resolve("app/examples/worker-rack-to-table.materia");
    session.openExample(path);
    documentChanged();
    for (record in scene.records()) if (record.type == "human-worker") {
      scene.select(record.id);
      break;
    }
    simulation.setBackend(ApplicationSimulation.MUJOCO);
    if (!simulation.rebuild(sensors, scene, session))
      throw 'Worker demo simulation failed: ${simulation.error}';
    for (_ in 0...advanceTicks) simulation.step();
    if (realtime) simulation.start();
  }

  public function hasCharacterPreview():Bool return characterPreview != null;

  function releaseCharacterPreview():Void {
    var preview = characterPreview;
    if (preview == null) return;
    simulation.removeParticipant(preview);
    preview.dispose();
    characterPreview = null;
  }

  public function enableComponentLab(?storyId:String):Void {
    componentLab = new ComponentLab(storyId);
    invalidateView();
  }

  /** Machine-readable application state paired with diagnostic frame captures. */
  public function diagnosticState():Dynamic return componentLab == null ? {
    lastSemanticAction: lastSemanticAction,
    selectedNode: scene.selectedId,
    scene: scene.diagnosticState(),
    document: {path: session.path, label: session.label(), dirty: session.isDirty(),
      confirmation: documents.needsConfirmation(), choosing: documents.choosing, error: documents.error},
    selection: ui.commandContext.selection.copy(),
    gridVisible: gridVisible,
    gridSnapEnabled: gridSnapEnabled,
    gridSpacing: gridSpacing,
    paletteVisible: paletteVisible,
    settingsVisible: settingsPanel != null,
    contextMenuVisible: contextMenuVisible,
    perspective: perspectiveViewport == null ? null : perspectiveViewport.diagnosticState(),
    robot: robotDiagnosticState(),
    panels: workspace.panelIds(),
    workspace: Json.parse(workspace.snapshotJson()),
    recentLog: logLines.slice(Std.int(Math.max(0, logLines.length - 8)))
  } : {
    mode: "component-lab",
    lab: componentLab.diagnosticState()
  };

  public function resetWorkspace():Void {
    modeSnapshots.remove(mode.id);
    workspace.reset();
    log("Workspace reset");
    invalidateView();
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

  /** Two tiers: file and document actions above, mode and simulation controls below. */
  function topBar():View {
    var compact = toolbarDensity != Full;
    var stack = new LayoutStyle();
    stack.width = LayoutAxis.grow();
    stack.height = LayoutAxis.fixed(TOOLBAR_HEIGHT);
    stack.direction = LayoutDirection.TopToBottom;
    // The dock workspace is painted after the toolbar; without this its panels would cover the tooltips
    // that hang below the transport buttons.
    stack.zIndex = 20;
    var divider = new Spacer("toolbar-tier-divider", LayoutAxis.grow(), LayoutAxis.fixed(1.0));
    divider.style.background = appearance.theme.tokens.border;
    return new Column("editor-toolbar", [
      new KeyedView("file", fileBar(compact)),
      new KeyedView("divider", divider),
      new KeyedView("context", contextBar(compact))
    ], stack);
  }

  function toolbarRowStyle(height:Float, background:Color):LayoutStyle {
    var style = fillStyle();
    style.height = LayoutAxis.fixed(height);
    style.direction = LayoutDirection.LeftToRight;
    style.childAlignY = LayoutAlignmentY.Center;
    style.childGap = 6.0;
    style.padding = new Insets(10.0, 3.0, 10.0, 3.0);
    style.background = background;
    return style;
  }

  function fileBar(compact:Bool):View {
    var minimal = toolbarDensity == Minimal;
    var titleStyle = new LayoutStyle();
    titleStyle.width = LayoutAxis.fixed(compact ? 72.0 : 84.0);
    var items:Array<KeyedView> = [
      new KeyedView("brand", new Text("MATERIA", titleStyle, appearance.theme.text,
        TextStyleOverride.text(13.0, 0.5))),
      new KeyedView("new", toolbarAction("toolbar-new", "editor.new", "New", IconName.NewFile, compact)),
      new KeyedView("open", toolbarAction("toolbar-open", "editor.open", "Open", IconName.FolderOpen, compact)),
      new KeyedView("save", toolbarAction("toolbar-save", "editor.save", "Save", IconName.Save, compact, true))
    ];
    if (!minimal) {
      items.push(new KeyedView("undo", toolbarAction("toolbar-undo", "editor.undo", "Undo", IconName.Undo, compact)));
      items.push(new KeyedView("redo", toolbarAction("toolbar-redo", "editor.redo", "Redo", IconName.Redo, compact)));
    }
    items.push(new KeyedView("space", new Spacer("toolbar-space", LayoutAxis.grow(),
      LayoutAxis.fixed(1.0))));
    var documentLabel = shortenLabel(session.label(), compact ? 18 : 30);
    items.push(new KeyedView("status", new Text(documentLabel, null, appearance.theme.tokens.textSecondary,
      TextStyleOverride.text(12.0))));
    var more = new Button(compact ? "" : "More", null, function() {
      toolbarMenuVisible = !toolbarMenuVisible;
      invalidateView();
    }, "toolbar-more");
    more.variant = ButtonVariant.Secondary;
    more.leadingIcon = IconName.ChevronDown;
    more.accessibilityLabel = "More editor actions";
    more.selected = toolbarMenuVisible;
    items.push(new KeyedView("more", more));
    return new Row("editor-file-bar", items, toolbarRowStyle(FILE_BAR_HEIGHT, appearance.toolbar));
  }

  // The mode strip is tinted while a simulation is active, so the non-editing state is obvious.
  function contextBar(compact:Bool):View {
    var items:Array<KeyedView> = [
      new KeyedView("modes", modeSwitcher(compact)),
      new KeyedView("space", new Spacer("context-space", LayoutAxis.grow(), LayoutAxis.fixed(1.0))),
      new KeyedView("transport", transportGroup(compact)),
      new KeyedView("space-end", new Spacer("context-space-end", LayoutAxis.grow(), LayoutAxis.fixed(1.0))),
      new KeyedView("frame", toolbarAction("toolbar-frame", "scene.frame-selected",
        "Frame", IconName.Inspect, compact))
    ];
    return new Row("editor-context-bar", items, toolbarRowStyle(CONTEXT_BAR_HEIGHT,
      simulation.isActive() ? appearance.toolbarSimulating : appearance.toolbar));
  }

  /** Segmented mode buttons; the active mode reads as selected through its command's checked state. */
  function modeSwitcher(compact:Bool):View {
    var style = new LayoutStyle();
    style.direction = LayoutDirection.LeftToRight;
    style.childAlignY = LayoutAlignmentY.Center;
    style.childGap = 2.0;
    var buttons:Array<KeyedView> = [];
    for (candidate in EditorMode.all())
      buttons.push(new KeyedView(candidate.id, toolbarAction("toolbar-mode-" + candidate.id,
        "editor.mode." + candidate.id, candidate.label, candidate.icon, compact)));
    return new Row("editor-mode-switcher", buttons, style);
  }

  /** Play/Pause, Step, Reset, and return-to-design controls for the shared simulation. */
  function transportGroup(compact:Bool):View {
    var style = new LayoutStyle();
    style.direction = LayoutDirection.LeftToRight;
    style.childAlignY = LayoutAlignmentY.Center;
    style.childGap = 4.0;
    var running = simulation.isRunning();
    // Labels appear when the bar has room; every button always explains itself on hover.
    var items:Array<KeyedView> = [new KeyedView("play-pause", running
      ? transportButton("toolbar-sim-pause", "sim.pause", "Pause", IconName.Pause,
        "Pause (F5): freeze time, keeping the simulation", false, compact)
      : transportButton("toolbar-sim-play", "sim.play", "Play", IconName.Play,
        "Play (F5): run the simulation", true, compact))];
    items.push(new KeyedView("step", transportButton("toolbar-sim-step", "sim.step", "Step",
      IconName.StepForward, "Step (F10): advance the simulation by one step", false, compact)));
    items.push(new KeyedView("reset", transportButton("toolbar-sim-reset", "sim.reset", "Reset",
      IconName.Reset, "Reset (Shift+F5): return to the starting state and stay in Simulate", false, compact)));
    items.push(new KeyedView("stop", transportButton("toolbar-sim-stop", "sim.stop", "Stop",
      IconName.Stop, "Stop: discard the simulation and return to editing", false, compact)));
    if (!compact) items.push(new KeyedView("state", simulationStateChip()));
    return new Row("editor-transport", items, style);
  }

  /** A transport button that shows its label when there is room and always has a hover explanation. */
  function transportButton(key:String, commandId:String, label:String, icon:IconName, explanation:String,
      primary:Bool, compact:Bool):View {
    var action = toolbarAction(key, commandId, label, icon, compact, primary);
    return new Tooltip(key + "-tooltip", action, new Text(explanation), 0.0, 36.0);
  }

  /** Current simulation state as a chip that cannot be mistaken for a button label. */
  function simulationStateChip():View {
    var tokens = appearance.theme.tokens;
    var active = simulation.isActive();
    var text = simulation.isRunning() ? "Running" : active ? "Paused" : "Design";
    if (active && simulation.pending(sensors, scene)) text += " · rebuild pending";
    var dotColor = simulation.isRunning() ? tokens.success : active ? tokens.warning : tokens.textSecondary;
    var chip = new LayoutStyle();
    chip.direction = LayoutDirection.LeftToRight;
    chip.childAlignY = LayoutAlignmentY.Center;
    chip.childGap = 6.0;
    chip.padding = new Insets(10.0, 4.0, 10.0, 4.0);
    chip.background = tokens.surface;
    chip.radiusTopLeft = chip.radiusTopRight = chip.radiusBottomLeft = chip.radiusBottomRight = 11.0;
    // Drawn rather than typed: the bullet glyph is not in the UI font and shows as a missing-glyph box.
    var dot = new Spacer("simulation-state-dot", LayoutAxis.fixed(8.0), LayoutAxis.fixed(8.0));
    dot.style.background = dotColor;
    dot.style.radiusTopLeft = dot.style.radiusTopRight = dot.style.radiusBottomLeft =
      dot.style.radiusBottomRight = 4.0;
    return new Row("simulation-state", [
      new KeyedView("dot", dot),
      new KeyedView("text", new Text(text, null, tokens.text, TextStyleOverride.text(12.0)))
    ], chip);
  }

  function chromeRevisionKey():String {
    var presentationRevision = framePresentation == null ? -1 : framePresentation.revision;
    return "generation=" + sceneGeneration + ":scene=" + scene.revision +
      ":selection=" + scene.selectionRevision + ":simulation=" + simulation.appliedRevision +
      ":active=" + simulation.isActive() + ":running=" + simulation.isRunning() +
      ":error=" + (simulation.error == null ? "" : simulation.error) +
      ":world=" + Std.string(world.status()) + ":presentation=" + presentationRevision +
      ":grid=" + gridSpacing + ":snap=" + gridSnapEnabled +
      ":mode=" + mode.id + ":density=" + Std.string(toolbarDensity) + ":menu=" + toolbarMenuVisible +
      ":viewport=" + viewportWidth + "x" + viewportHeight + ":view=" + viewRevision +
      ":mates=" + Std.string(session.assemblyMateStatus());
  }

  function statusBar():View {
    var style = new LayoutStyle();
    style.width = LayoutAxis.grow();
    style.height = LayoutAxis.fixed(STATUS_HEIGHT);
    style.direction = LayoutDirection.LeftToRight;
    style.childAlignY = LayoutAlignmentY.Center;
    style.childGap = 12.0;
    style.padding = new Insets(10.0, 3.0, 10.0, 3.0);
    style.background = appearance.toolbar;

    var selected = scene.object(scene.selectedId);
    var left = simulation.error != null ? "Simulation error: " + simulation.error :
      selected == null ? "Ready" : selected.label + " selected";
    var staleCount = session.staleEdits().length;
    if (staleCount > 0) left = staleCount + " stale project edit" + (staleCount == 1 ? "" : "s");
    var mode = simulation.isRunning() ? "Running" : simulation.isActive() ? "Paused" : "Design";
    if (simulation.isActive() && simulation.pending(sensors, scene)) mode += " · Rebuild pending";
    var worldLabel = switch (world.status()) {
      case RobotStatus.Disconnected: "World Offline";
      case RobotStatus.Connecting: "World Connecting";
      case RobotStatus.Ready: "World Ready";
      case RobotStatus.Fault: "World Fault";
    };
    var right = mode + (viewportWidth >= 700.0 ? " · " + simulation.userBackendName() : "") +
      " · " + worldLabel;
    var items:Array<KeyedView> = [
      new KeyedView("selection", new Text(shortenLabel(left, viewportWidth < 700.0 ? 20 : 48),
        null, simulation.error == null ? appearance.theme.tokens.textSecondary :
          appearance.theme.tokens.danger, TextStyleOverride.text(12.0))),
      new KeyedView("space", new Spacer("status-space", LayoutAxis.grow(), LayoutAxis.fixed(1.0)))
    ];
    var mates = session.assemblyMateStatus();
    if (mates != null) {
      var trouble = StringTools.startsWith(mates, "Mates conflict") || session.assemblyMateProblem != null;
      items.push(new KeyedView("mates", new Text(shortenLabel(mates, viewportWidth < 900.0 ? 32 : 72), null,
        trouble ? appearance.theme.tokens.danger : appearance.theme.tokens.textSecondary, TextStyleOverride.text(12.0))));
    }
    if (viewportWidth >= 900.0)
      items.push(new KeyedView("grid", new Text("Grid " + gridSpacingLabel() +
        " m · Snap " + (gridSnapEnabled ? "On" : "Off"), null,
        appearance.theme.tokens.textSecondary, TextStyleOverride.text(12.0))));
    items.push(new KeyedView("runtime", new Text(right, null,
      appearance.theme.tokens.textSecondary, TextStyleOverride.text(12.0))));
    return new Row("editor-status-bar", items, style);
  }

  function toolbarAction(key:String, commandId:String, label:String, icon:IconName,
      compact:Bool, primary:Bool = false):CommandButton {
    var action = new CommandButton(key, commandId, commands);
    action.displayLabel = compact ? "" : label;
    action.leadingIcon = icon;
    action.variant = primary ? ButtonVariant.Primary : ButtonVariant.Secondary;
    return action;
  }

  static function shortenLabel(value:String, maximum:Int):String
    return value.length <= maximum ? value : value.substr(0, maximum - 3) + "...";

  function makeWorkspace():DockWorkspaceModel {
    var result = new DockWorkspaceModel();
    result.register(new DockPanelDescriptor("start", "Start", true, true, IconName.Grid));
    result.register(new DockPanelDescriptor("hierarchy", "Hierarchy", false, true, IconName.Hierarchy));
    result.register(new DockPanelDescriptor("bim", "BIM", false, true, IconName.Building));
    // Recognize legacy saved layouts; this panel is removed before the UI builds.
    result.register(new DockPanelDescriptor("viewport", "Viewport", false, true, IconName.Grid));
    result.register(new DockPanelDescriptor("perspective", "3D", false, true, IconName.Cube));
    result.register(new DockPanelDescriptor("inspector", "Inspector", false, true, IconName.Sliders));
    result.register(new DockPanelDescriptor("sensors", "Sensors", false, true, IconName.Radar));
    result.register(new DockPanelDescriptor("console", "Console", true, true, IconName.Terminal));
    result.register(new DockPanelDescriptor("telemetry", "Telemetry", true, true, IconName.Activity));

    workspacePanelContents = [
      new DockPanelContent("start", function(_) return StartPanel.build(this)),
      new DockPanelContent("hierarchy", function(_) return hierarchyPanel(), null,
        function() return "scene=" + scene.revision + ":selection=" + scene.selectionRevision +
          ":filter=" + hierarchySearch + ":expansion=" + hierarchyExpansionRevision +
          ":simulating=" + simulation.isActive()),
      new DockPanelContent("bim", function(_) return bimEditor),
      new DockPanelContent("perspective", function(_) return perspectivePanel(),
        function(_, width) return perspectivePanel(width),
        // perspectivePanel() pushes grid and simulation settings into the viewport, so a hit may only skip it when
        // none of its inputs changed: the chrome key covers grid, snap, simulation, presentation and size.
        function() return chromeRevisionKey() + ":gridVisible=" + gridVisible + ":options=" + viewportOptionsVisible),
      new DockPanelContent("inspector", function(_) return inspectorPanel(), null,
        function() return "scene=" + scene.revision + ":selection=" + scene.selectionRevision +
          ":simulation=" + simulation.appliedRevision + ":active=" + simulation.isActive() +
          ":content=" + (sceneInspector == null ? 0 : sceneInspector.contentRevision())),
      new DockPanelContent("sensors", function(_) return sensorPanel(), null,
        function() return "generation=" + session.generation + ":document=" + session.document.revision +
          ":sensors=" + sensors.revision() + ":robot=" + sensors.robotId +
          ":selected=" + sensors.selectedIndex + ":simulation=" + simulation.appliedRevision +
          ":active=" + simulation.isActive() + ":running=" + simulation.isRunning()),
      new DockPanelContent("console", function(_) return consolePanel(), null,
        function() return "log=" + consoleDocument.revision + ":stale=" + session.staleEdits().join("\n") +
          ":dark=" + appearance.dark),
      new DockPanelContent("telemetry", function(_) return telemetry.build(framePresentation, appearance.theme.tokens.surface,
        appearance.theme.tokens.textSecondary, simulation))
    ];

    result.setDefaultLayout(EditorWorkspaceLayout.defaultLayout());
    return result;
  }

  function sensorPanel():View return SensorPanel.build(this);

  function updateReadOnlyRobots():Void {
    var simulatedIds = simulation.simulatedRobotIds();
    sensors.setReadOnlyRobots([for (id in world.robotIds()) if (simulatedIds.indexOf(id) < 0) id]);
  }

  function hierarchyPanel():View return HierarchyPanel.build(this);

  function sceneAction(key:String, commandId:String, label:String, icon:Null<IconName>):CommandButton {
    var action = new CommandButton(key, commandId, commands);
    action.displayLabel = label;
    action.leadingIcon = icon;
    action.variant = commandId == "scene.apply-sketch" ? ButtonVariant.Primary : ButtonVariant.Secondary;
    return action;
  }

  function startRename(id:String):Void {
    var item = scene.object(id);
    if (item == null || !canEditObjects()) return;
    renameId = id;
    renameValue = item.label;
    hierarchyMenuVisible = false;
    invalidateView();
  }

  function finishRename():Void {
    var id = renameId;
    if (id == null) return;
    try {
      scene.setName(id, renameValue);
      renameId = null;
      invalidateView();
    } catch (error:Dynamic) log("Rename failed: " + Std.string(error));
  }

  function savedTheme():Theme
    return preferences.store.getString(AppSettings.COLOR_SCHEME) == "dark" ? Theme.dark() : Theme.light();

  function applyTheme(theme:Theme):Void {
    appearance = new EditorAppearance(theme);
    ui.setTheme(appearance.theme);
    commands.refresh();
    invalidateView();
    if (hostContext != null) hostContext.requestFrame();
  }

  /** Copies the saved grid and lighting settings into the fields the viewport and toolbar read. */
  function readViewSettings():Void {
    var store = preferences.store;
    gridVisible = store.getBool(AppSettings.GRID_VISIBLE);
    gridSnapEnabled = store.getBool(AppSettings.GRID_SNAP);
    gridSpacing = store.getFloat(AppSettings.GRID_SPACING);
    lightingPreset = Std.int(Math.max(0, AppSettings.LIGHTING_PRESETS.indexOf(store.getString(AppSettings.LIGHTING))));
    if (perspectiveViewport != null) perspectiveViewport.setLightingPreset(lightingPreset);
    if (commands != null) commands.refresh();
    invalidateView();
  }

  /** Opens the Editor Settings dialog on its first category. */
  public function openSettings():Void {
    if (settingsPanel == null) settingsPanel = new SettingsPanel("editor-settings", preferences.store, invalidateView);
    if (shortcutsPanel == null) shortcutsPanel = new ShortcutsPanel("editor-shortcuts", shortcutBindings, invalidateView);
    paletteVisible = false;
    contextMenuVisible = false;
    toolbarMenuVisible = false;
    invalidateView();
  }

  public function showSettingsTab(tab:String):Void {
    if (tab != "general" && tab != "shortcuts") throw 'Unknown settings tab: $tab';
    settingsTab = tab;
    invalidateView();
  }

  public function closeSettings():Void {
    if (settingsPanel == null) return;
    settingsPanel = null;
    shortcutsPanel = null;
    invalidateView();
  }

  function settingsDialog(panel:SettingsPanel):View {
    var body = new LayoutStyle();
    body.width = LayoutAxis.grow();
    body.height = LayoutAxis.fixed(Math.max(240.0, Math.min(640.0, viewportHeight - 200.0)));
    body.direction = LayoutDirection.TopToBottom;
    var close = new Button("Close", null, closeSettings, "editor-settings-close");
    var closeRow = new LayoutStyle();
    closeRow.width = LayoutAxis.grow();
    closeRow.childDistribution = LayoutDistribution.Center;
    var content = new Column("editor-settings-content", [
      new KeyedView("panel", new Column("editor-settings-body", [new KeyedView("tabs", new Tabs("editor-settings-tabs", [
        new TabItem("general", "General", panel),
        new TabItem("shortcuts", "Shortcuts", shortcutsPanel == null ? new Text("") : shortcutsPanel)
      ], settingsTab, showSettingsTab, fillStyle(), null, null, null, null, TabsSelectionMode.Controlled))], body)),
      new KeyedView("actions", new Row("editor-settings-actions", [new KeyedView("close", close)], closeRow))
    ], actionColumnStyle());
    return new Dialog("editor-settings", "Editor Settings", content, closeSettings,
      Math.max(360.0, Math.min(960.0, viewportWidth - 80.0)));
  }

  function renameDialog():View {
    var field = new TextField("rename-name", renameValue, function(value) renameValue = value);
    field.style.width = LayoutAxis.stretch();
    field.label = "Object name";
    field.onSubmit = function(_) finishRename();
    var confirm = new Button("Rename", null, finishRename, "rename-confirm");
    confirm.variant = ButtonVariant.Primary;
    var content = new Column("rename-content", [
      new KeyedView("name", field),
      new KeyedView("actions", new Row("rename-actions", [
        new KeyedView("save", confirm),
        new KeyedView("cancel", new Button("Cancel", null, function() renameId = null, "rename-cancel"))
      ], actionRowStyle()))
    ], actionColumnStyle());
    return new Dialog("rename-object", "Rename object", content, function() renameId = null, 360.0);
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

  function viewportWithControls(content:View, availableWidth:Float):View {
    var compact = availableWidth > 0.0 && availableWidth < 400.0;
    var tiny = availableWidth > 0.0 && availableWidth < 235.0;
    var options = new Button(gridSpacingLabel() + (tiny ? "" : " m"), null, null,
      "viewport-options");
    options.variant = ButtonVariant.Navigation;
    options.classes = ["viewport-tool"];
    options.accessibilityLabel = "Grid spacing: " + gridSpacingLabel() + " m";
    options.trailingIcon = IconName.ChevronDown;
    options.iconSize = 12.0;
    options.selected = viewportOptionsVisible;
    options.onClickEvent = function(event) {
      if (viewportOptionsVisible) {
        viewportOptionsVisible = false;
        invalidateView();
        return;
      }
      var bounds = menuTriggerBounds(event);
      viewportOptionsX = Math.max(8.0, Math.min(viewportWidth - 228.0, bounds.x));
      viewportOptionsY = Math.max(8.0, Math.min(viewportHeight - 245.0, bounds.y + bounds.height));
      viewportOptionsVisible = true;
      invalidateView();
    };
    var controls:Array<KeyedView> = [new KeyedView("frame",
      viewportToolbarAction("viewport-frame", "scene.frame-selected", "Frame", IconName.Inspect, compact))];
    if (!tiny) controls.push(new KeyedView("reset",
      viewportToolbarAction("viewport-reset", "scene.reset-perspective", "Reset", IconName.Cube, compact)));
    var groupDivider = new Spacer("viewport-toolbar-group-divider", LayoutAxis.fixed(1.0),
      LayoutAxis.fixed(20.0));
    groupDivider.style.background = appearance.theme.tokens.border;
    controls.push(new KeyedView("group-divider", groupDivider));
    controls.push(new KeyedView("grid", viewportToolbarAction("viewport-grid",
      "scene.toggle-grid", "Grid", IconName.Grid, compact)));
    controls.push(new KeyedView("snap", viewportToolbarAction("viewport-snap",
      "scene.toggle-grid-snap", "Snap", IconName.Magnet, compact)));
    controls.push(new KeyedView("options", options));
    // While a jointed part is dragged, say whether it follows the cursor and why not.
    var dragMessage = perspectiveViewport == null ? null : perspectiveViewport.assemblyDragMessage();
    var mateMessage = perspectiveViewport == null ? null : perspectiveViewport.matePickMessage();
    if (mateMessage != null) dragMessage = mateMessage;
    if (dragMessage != null)
      controls.push(new KeyedView("assembly-drag-status", new Text(dragMessage, null,
        appearance.theme.tokens.textSecondary, TextStyleOverride.text(12.0))));
    var barStyle = new LayoutStyle();
    barStyle.width = LayoutAxis.grow();
    barStyle.height = LayoutAxis.fixed(38.0);
    barStyle.direction = LayoutDirection.LeftToRight;
    barStyle.childAlignY = LayoutAlignmentY.Center;
    barStyle.childGap = tiny ? 2.0 : 4.0;
    barStyle.padding = new Insets(tiny ? 4.0 : 8.0, 4.0, tiny ? 4.0 : 8.0, 4.0);
    barStyle.background = appearance.theme.tokens.surfaceRaised;
    var toolbar = new Row("viewport-toolbar", controls, barStyle);
    var divider = new Spacer("viewport-toolbar-bottom-divider", LayoutAxis.grow(),
      LayoutAxis.fixed(1.0));
    divider.style.background = appearance.theme.tokens.border;
    var panelStyle = fillStyle();
    var viewAngle = new Button(perspectiveViewport == null ? "Perspective" :
      perspectiveViewport.viewAngleLabel(), null, null, "view-angle-button");
    viewAngle.variant = ButtonVariant.Navigation;
    viewAngle.trailingIcon = IconName.ChevronDown;
    viewAngle.accessibilityLabel = "Choose perspective view angle";
    viewAngle.selected = viewAngleMenuVisible;
    viewAngle.onClickEvent = function(event) {
      if (viewAngleMenuVisible) {
        viewAngleMenuVisible = false;
      } else {
        var bounds = menuTriggerBounds(event);
        viewAngleMenuX = Math.max(8.0, Math.min(viewportWidth - 190.0, bounds.x));
        viewAngleMenuY = Math.max(8.0, Math.min(viewportHeight - 155.0, bounds.y + bounds.height));
        viewAngleMenuVisible = true;
      }
      invalidateView();
    };
    var canvas:View = new Stack("perspective-canvas-overlay", [
      new StackChild("scene", content, 0.0, 0.0, 0,
        LayoutAxis.grow(), LayoutAxis.grow()),
      new StackChild("view-angle", viewAngle, 10.0, 10.0, 1)
    ]);
    return new Column("perspective-with-toolbar", [
      new KeyedView("toolbar", toolbar),
      new KeyedView("divider", divider),
      new KeyedView("canvas", new SizedBox("viewport-canvas-slot", canvas,
        LayoutAxis.grow(), LayoutAxis.grow()))
    ], panelStyle);
  }

  function viewportToolbarAction(key:String, commandId:String, label:String,
      icon:IconName, compact:Bool):View {
    var action = new CommandButton(key, commandId, commands);
    action.displayLabel = compact ? "" : label;
    action.leadingIcon = icon;
    action.variant = ButtonVariant.Navigation;
    action.classes = ["viewport-tool"];
    return compact ? new Tooltip(key + "-tooltip", action, new Text(label), 0.0, 34.0) : action;
  }

  function gridSpacingLabel():String {
    var tenths = Std.int(Math.round(gridSpacing * 10.0));
    return Std.string(Std.int(tenths / 10)) + "." + Std.string(tenths % 10);
  }

  function menuTriggerBounds(event:UiEvent):Rect {
    var root = ui.root;
    var node = root == null || event.currentTarget == null ? null : root.find(event.currentTarget);
    return node == null || node.resolved == null
      ? new Rect(event.x, event.y, 0.0, 0.0) : node.resolved.clippedViewportBounds();
  }

  function perspectivePanel(availableWidth:Float = 0.0):View {
    if (perspectiveViewport != null) {
      perspectiveViewport.setPlacementOptions(gridSnapEnabled, gridSpacing, gridVisible);
      var frame = framePresentation;
      perspectiveViewport.setSimulationState(simulation.isActive(),frame == null ? [] : frame.environment,
        frame == null ? 0 : frame.revision,frame == null ? [] : frame.robots);
    }
    return perspectiveViewport == null
      ? new Text("Perspective rendering requires the desktop GPU host.")
      : viewportWithControls(perspectiveViewport, availableWidth);
  }

  function inspectorPanel():View return InspectorPanel.build(this);

  function consolePanel():View {
    var style = fillStyle();
    style.padding = new Insets(12.0, 12.0, 12.0, 12.0);
    style.background = appearance.theme.tokens.surface;
    var rows:Array<KeyedView> = [];
    var stale = session.staleEdits();
    if (stale.length > 0) {
      rows.push(new KeyedView("stale-heading", new Text("Stale project edits: " + stale.length)));
      for (index in 0...stale.length)
        rows.push(new KeyedView("stale:" + index, new Text(stale[index])));
      rows.push(new KeyedView("stale-discard", sceneAction("stale-discard-command",
        "editor.discard-stale-edits", "Discard stale edits", IconName.Trash)));
    }
    var logStyle = fillStyle();
    logStyle.padding = new Insets(4.0, 2.0, 4.0, 2.0);
    logStyle.background = appearance.theme.tokens.surface;
    var logView = TextArea.withDocument("console-log", consoleDocument, null, logStyle, "Console output", null,
      appearance.theme.tokens.text);
    logView.readOnly = true;
    logView.followTail = true;
    var copyAll = new CommandButton("console-copy-all", "console.copy-all", commands);
    copyAll.displayLabel = "Copy all";
    copyAll.leadingIcon = IconName.Copy;
    var headerStyle = new LayoutStyle();
    headerStyle.width = LayoutAxis.grow();
    headerStyle.direction = LayoutDirection.LeftToRight;
    headerStyle.childAlignY = LayoutAlignmentY.Center;
    headerStyle.childGap = 8.0;
    rows.unshift(new KeyedView("header", new Row("console-header", [
      new KeyedView("heading", sectionHeading("CONSOLE")),
      new KeyedView("space", new Spacer("console-header-space", LayoutAxis.grow(), LayoutAxis.fixed(1.0))),
      new KeyedView("copy-all", copyAll)
    ], headerStyle)));
    rows.push(new KeyedView("log", logView));
    return new Column("console-panel", rows, style);
  }

  function sectionHeading(label:String):Text {
    return new Text(label, null, appearance.theme.tokens.textSecondary, TextStyleOverride.text(11.0, 0.8));
  }

  function runSceneEdit(label:String, action:Void->Bool):Void {
    try {
      action();
    } catch (error:ParametricError) {
      log(label + ": " + error.message);
    }
    commands.refresh();
  }

  function installCommands():Void {
    DockWorkspaceCommands.install(workspace, commands, "workspace");
    SceneObjectCommands.install(this);
    EditorDocumentCommands.install(this);
    commands.register(new Command("workspace.save", "Save workspace", function() {
      saveWorkspace();
      log("Workspace saved");
    }));
    commands.register(new Command("editor.toggle-dark-theme", "Toggle dark theme", function() {
      var dark = !appearance.dark;
      preferences.store.set(AppSettings.COLOR_SCHEME, PropertyValue.Enum(dark ? "dark" : "light"));
      // Also apply directly: a --dark launch can differ from the saved scheme, so the set may not change it.
      applyTheme(dark ? Theme.dark() : Theme.light());
    }, null, null, function() return appearance.dark));
    var openPalette = new Command("editor.command-palette", "Open command palette", function() {
      paletteVisible = true;
      contextMenuVisible = false;
      invalidateView();
    }, new Shortcut(UiKey.K, UiModifier.Control), function() return !documents.blocked());
    openPalette.addShortcut(new Shortcut(UiKey.P, UiModifier.Control));
    commands.register(openPalette);
    commands.register(new Command("editor.settings", "Editor Settings...", openSettings,
      new Shortcut(UiKey.Comma, UiModifier.Control), function() return !documents.blocked()));
    SceneViewCommands.install(this);
    AssemblyCommands.install(this);
    SimulationCommands.install(this);
    commands.register(new Command("start.show", "Show Start page", showStartPage));
    commands.register(new Command("console.copy-all", "Copy console output", function() {
      ui.clipboard.writeText(consoleDocument.text);
      log("Console output copied");
    }));
    for (candidate in EditorMode.all()) {
      var target = candidate;
      commands.register(new Command("editor.mode." + target.id, target.label + " mode",
        function() switchMode(target),
        new Shortcut(target == EditorMode.Design ? 49 : 50, UiModifier.Control),
        function() return !documents.blocked(), function() return mode == target));
    }
  }

  function documentChanged():Void {
    if (preferences != null && session.path != null && session.path != lastRecordedPath) {
      lastRecordedPath = session.path;
      preferences.addRecent(session.path);
    }
    attachSceneRecorder();
    if (bimEditor.model != session.bim) bimEditor = makeBimEditor();
    cancelActiveDrag();
    if (sceneGeneration != session.generation) {
      if (projectUiExtension != null) projectUiExtension.dispose();
      projectUiExtension = null;
      projectUiReference = null;
      projectUiError = null;
      var ownership=session.scriptOwnership;
      if(ownership!=null){simulation.setBackend(ownership.backend());simulation.setTimestep(ownership.timestep());}
      log("Document configuration replaced");
      sceneGeneration = session.generation;
      treeModel = new EditorSceneTree(scene, session.projectAssemblyDefinition, session.generatedLabels());
      treeModel.setFilter(hierarchySearch);
      if (perspectiveViewport != null) perspectiveViewport.dispose();
      perspectiveViewport = hostContext == null ? null :
        new EditorPerspectiveViewport("scene-perspective", scene, hostContext);
      if (perspectiveViewport != null) {
        perspectiveViewport.setSampleCount(antialiasing);
        perspectiveViewport.setLightingPreset(lightingPreset);
      }
      sceneInspector = null;
    }
    scene.onSelectionChanged = updateCommandContext;
    updateCommandContext();
    paletteVisible = false;
    contextMenuVisible = false;
    commands.refresh();
  }

  function makeBimEditor():BimModelEditor return new BimModelEditor("bim-model-editor", session.bim,
    session.document, function(label, change, undo) session.applyBimEdit(label, change, undo));

  function refreshScriptMaterialization(message:String):Void {
    try {var result=session.refreshScriptOverrides();simulation.setBackend(result.backend);
      simulation.setTimestep(result.timestep);documentChanged();log(message+"; Apply/Rebuild restarts simulation");}
    catch(error:Dynamic)log("Script override rejected: "+Std.string(error));
    commands.refresh();
  }

  function canEditObjects():Bool return session.scriptOwnership==null && !documents.blocked() && !simulation.isActive() &&
    (perspectiveViewport == null || !perspectiveViewport.dragging());

  function commitActiveDrag():Void {
    var changed = false;
    if (perspectiveViewport != null && perspectiveViewport.dragging()) {
      var pointer = perspectiveViewport.commitDrag();
      if (pointer != null) ui.pointerCancel(pointer.id, pointer.x, pointer.y);
      changed = true;
    }
    if (!changed) return;
    commands.refresh();
  }

  function cancelActiveDrag():Void {
    if (scene.hasActiveSketchEdit()) {
      scene.cancelSelectedSketchEdit();
    }
    if (perspectiveViewport != null && perspectiveViewport.dragging()) {
      var pointer = perspectiveViewport.cancelDrag();
      if (pointer != null) ui.pointerCancel(pointer.id, pointer.x, pointer.y);
    }
    commands.refresh();
  }

  function makeDocumentDialog():Null<View> {
    if (!documents.needsConfirmation() && !documents.needsTrustConfirmation() &&
        documents.error == null) return null;
    var style = new LayoutStyle();
    style.width = LayoutAxis.grow();
    style.childGap = 12.0;
    style.padding = new Insets(16, 16, 16, 16);
    var buttons:Array<KeyedView> = [];
    var title = "Unsaved changes";
    var message = "Save changes to " + session.label() + " before continuing?";
    var dismiss = function() documents.resolve("cancel");
    if (documents.needsTrustConfirmation()) {
      title = "Run project code?";
      message = "Opening this file runs registered code: " + documents.trustReference;
      dismiss = function() documents.resolveTrust(false);
      var run = new Button("Run code and open", null, function() documents.resolveTrust(true),
        "document-trust-run");
      run.variant = ButtonVariant.Primary;
      buttons.push(new KeyedView("run", run));
      buttons.push(new KeyedView("cancel", new Button("Cancel", null, dismiss, "document-trust-cancel")));
    } else if (documents.error != null) {
      title = "Scene document error";
      message = documents.error;
      dismiss = documents.dismissError;
      buttons.push(new KeyedView("close", new Button("Close", null, documents.dismissError, "document-error-close")));
    } else {
      var save = new Button("Save", null, function() documents.resolve("save"), "document-confirm-save");
      save.variant = ButtonVariant.Primary;
      buttons.push(new KeyedView("save", save));
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
    if (projectUiExtension != null) try {
      projectUiExtension.select(scene.selectedId);
      var requested = projectUiExtension.requestedSelection();
      if (requested != null && scene.selectedId != requested) {
        scene.select(requested);
        return;
      }
      commands.refresh();
    } catch (error:Dynamic) log("Project UI extension: " + Std.string(error));
    var context = scene.context();
    if (semanticRecordPath != null) context.onPropertyEdit = function(edit) {
      recordSemantic("property.edit", edit);
    };
    ui.setCommandContext(context);
  }

  function projectUiPanel():Null<View> {
    var reference = session.projectReference;
    if (reference == null) return null;
    if (projectUiReference != reference) {
      if (projectUiExtension != null) projectUiExtension.dispose();
      projectUiExtension = null;
      projectUiError = null;
      projectUiReference = reference;
      try {
        projectUiExtension = ProjectUiExtension.open(reference, projectInitialAction);
        projectInitialAction = null;
        syncProjectUi();
      } catch (error:Dynamic) {
        projectUiError = Std.string(error);
        log("Project UI extension: " + projectUiError);
      }
    }
    if (projectUiError != null) return new Text("Project UI extension: " + projectUiError);
    return projectUiExtension == null ? null : projectUiExtension.panel(projectUiAction);
  }

  function projectUiAction(id:String):Void {
    if (projectUiExtension == null) return;
    try {
      projectUiExtension.action(id);
      var selected = projectUiExtension.requestedSelection();
      if (selected != null && scene.selectedId != selected) scene.select(selected);
      syncProjectUi();
      commands.refresh();
    } catch (error:Dynamic) {
      log("Project UI extension: " + Std.string(error));
      commands.refresh();
    }
  }

  function syncProjectUi():Void {
    if (projectUiExtension == null) return;
    for (colour in projectUiExtension.colours()) {
      var id:String = Reflect.field(colour, "id");
      if (id != null && scene.object(id) != null)
        scene.setColour(id, Reflect.field(colour, "r"), Reflect.field(colour, "g"),
          Reflect.field(colour, "b"));
    }
  }

  function attachSceneRecorder():Void {
    if (semanticRecordPath != null)
      scene.onSemanticAction = function(action, data) recordSemantic(action, data);
  }

  function recordSemantic(action:String, data:Dynamic):Void {
    if (semanticRecordPath == null) return;
    lastSemanticAction = {action:action, data:data};
    try File.appendContent(semanticRecordPath,
      Json.stringify({kind:"action", at:Sys.time(), action:action, data:data}) + "\n")
    catch (error:Dynamic) Sys.println("Materia recording failed: " + Std.string(error));
  }

  /**
   * Chooses the viewport's samples per pixel. `remember` keeps the choice for the next launch;
   * a launch option passes false so it does not overwrite the saved preference.
   */
  public function setAntialiasing(samples:Int, remember:Bool = true):Void {
    if (samples < 1) throw "Anti-aliasing needs at least one sample per pixel";
    antialiasing = samples;
    if (remember) preferences.setAntialiasing(samples);
    if (perspectiveViewport != null) perspectiveViewport.setSampleCount(samples);
    invalidateView();
  }

  /** Switches dock layout and toolbar emphasis; the document, selection, and history are untouched. */
  public function switchMode(next:EditorMode):Void {
    if (next == mode) return;
    modeSnapshots.set(mode.id, layoutSnapshot());
    mode = next;
    // The mode's default layout backs "Reset workspace" while it is active.
    workspace.setDefaultLayout(next.layout());
    var saved = modeSnapshots.get(next.id);
    if (saved != null) workspace.restorePersisted(saved);
    log(next.label + " mode");
    commands.refresh();
    invalidateView();
    queueWorkspaceSave();
  }

  /** Play enters Simulate and remembers where to return; Stop and Design restore it. */
  public function enterSimulationMode():Void {
    if (mode == EditorMode.Simulate) return;
    modeBeforePlay = mode;
    switchMode(EditorMode.Simulate);
  }

  public function leaveSimulationMode():Void {
    var previous = modeBeforePlay;
    modeBeforePlay = null;
    if (previous == null || mode != EditorMode.Simulate) return;
    switchMode(previous);
    // The restored layout keeps whichever tab was in front when the mode was left, which can be the Start
    // page if simulation began there. After a simulation the model is what should be showing.
    if (perspectiveViewport != null) workspace.activate("perspective");
  }

  // Only the Design layout is written, so a session ending in another mode reopens in Design.
  function persistedWorkspace():DockWorkspaceSnapshot
    return mode == EditorMode.Design || !modeSnapshots.exists(EditorMode.Design.id)
      ? layoutSnapshot() : modeSnapshots.get(EditorMode.Design.id);

  /**
   * Panels that belong to the session rather than to a layout. They are opened by an explicit action
   * (launch, or the Show Start page command) and are left out of every saved layout, so switching modes
   * or restarting can never bring them back on their own.
   */
  static final TRANSIENT_PANELS:Array<String> = ["start"];

  /** The current layout without the transient panels, for the per-mode memory and the workspace file. */
  function layoutSnapshot():DockWorkspaceSnapshot {
    var snapshot = workspace.snapshot();
    var root = snapshot.root;
    var active = snapshot.activePanelId;
    for (id in TRANSIENT_PANELS) {
      root = DockNodeTools.remove(root, id);
      // The tab that was in front goes with the panel; the model is what should be showing instead.
      if (active == id) active = DockNodeTools.contains(root, "perspective") ? "perspective" : null;
    }
    return new DockWorkspaceSnapshot(root, active);
  }

  function saveWorkspace():Void {
    try {
      var error = workspaceSaves.saveNow(persistedWorkspace());
      if (error != null) log("Workspace save failed: " + error);
    }
    catch (error:Dynamic) log("Workspace save failed: " + Std.string(error));
  }

  function queueWorkspaceSave():Void {
    try workspaceSaves.schedule(persistedWorkspace());
    catch (error:Dynamic) log("Workspace save failed: " + Std.string(error));
  }

  function log(message:String):Void {
    if (logLines == null) return;
    logLines.push(message);
    var before = consoleDocument.codepointCount;
    consoleDocument.replace(before, before, "> " + message + "\n");
    logLengths.push(consoleDocument.codepointCount - before);
    while (logLines.length > MAX_LOG_LINES) {
      logLines.shift();
      consoleDocument.replace(0, logLengths.shift(), "");
    }
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
    var temporary = path + ".tmp";
    File.saveContent(temporary, value);
    FileSystem.rename(temporary, path);
  }
}
