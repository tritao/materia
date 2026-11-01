package app.editor;

import app.ApplicationSimulation;
import app.ProgramErrorMessage;
import app.ProjectDocumentSession;
import Diffable;
import Equatable;
import haxeon.ui.Color;
import haxeon.ui.Insets;
import haxeon.ui.LayoutAxis;
import haxeon.ui.LayoutStyle;
import haxeon.ui.core.TextStyleOverride;
import haxeon.ui.core.View;
import haxeon.ui.core.BuildContext;
import haxeon.ui.core.RenderNode;
import haxeon.ui.core.WidgetId;
import haxeon.ui.core.Key;
import haxeon.ui.LayoutVisualKind;
import haxeon.ui.theme.ThemeTokens;
import haxeon.ui.widgets.KeyedView;
import haxeon.ui.widgets.collections.ListView;
import haxeon.ui.widgets.collections.ListViewModel;
import haxeon.ui.widgets.controls.Button;
import haxeon.ui.widgets.controls.Slider;
import haxeon.ui.widgets.layout.Column;
import haxeon.ui.widgets.layout.Row;
import haxeon.ui.widgets.scroll.ScrollController;
import haxeon.ui.widgets.text.Text;

/**
 * The CNC dock panel: the running program's G-code with the executing line marked and followed,
 * feed hold and resume, restart from a chosen line, and a speed override.
 */
class CncPanel {
  public static inline final ROW_HEIGHT = 20.0;
  final scroll = new ScrollController();
  final model = new GcodeListModel();
  /** The line the operator picked to restart at; 0 for none. */
  public var selectedLine(default, null):Int = 0;
  var followedLine = 0;
  var source:Null<String>;
  var generation = -1;
  var errorDetails = false;
  var lastFailure:Null<String>;

  final presentation = new CncPresentation();
  final nextPresentation = new CncPresentation();
  final report = new HobbingReportData();
  final nextReport = new HobbingReportData();
  var player:Null<app.ToolpathPlayer>;
  var hobbing:Null<app.HobbingProgramPlayer>;
  var bindingRevision = 0;
  var patchContext:Null<BuildContext>;
  var patchId:Null<WidgetId>;

  public function new() {
  }

  function requestPatch():Void {
    var context = patchContext, id = patchId;
    if (context == null || id == null || !context.requestPatch(id)) bindingRevision++;
  }

  /** Sample execution and advance following before checking retained dependencies. */
  public function update(simulation:ApplicationSimulation, ?project:ProjectDocumentSession):Void {
    var nextPlayer = simulation.toolpathPlayer(), nextHobbing = simulation.hobbingPlayer();
    var cnc = project == null ? null : project.cncJob, print = project == null ? null : project.printJob;
    var text = nextPlayer != null ? nextPlayer.sourceText() : cnc != null ? cnc.source : print != null ? print.source : null;
    var nextGeneration = project == null ? -1 : project.generation;
    if (source != text || generation != nextGeneration) {
      source = text;
      generation = nextGeneration;
      selectedLine = 0;
      followedLine = 0;
      scroll.jumpTo(0, 0);
    }
    var failure = nextPlayer == null ? null : nextPlayer.failure;
    if (failure != lastFailure) {
      lastFailure = failure;
      errorDetails = false;
    }
    if (nextPlayer != null && nextPlayer.currentLine > 0 && nextPlayer.currentLine != followedLine) {
      followedLine = nextPlayer.currentLine;
      scroll.jumpTo(scroll.offsetX, Math.max(0.0, (followedLine - 4) * ROW_HEIGHT));
    }
    nextPresentation.source = text;
    nextPresentation.generation = nextGeneration;
    nextPresentation.print = print != null;
    nextPresentation.running = simulation.isRunning();
    nextPresentation.line = nextPlayer == null ? 0 : nextPlayer.currentLine;
    nextPresentation.failure = failure;
    nextPresentation.held = nextPlayer != null && nextPlayer.held();
    nextPresentation.finished = nextPlayer != null && nextPlayer.finished();
    nextPresentation.started = nextPlayer != null && nextPlayer.hasStarted();
    nextPresentation.restart = nextPlayer != null && nextPlayer.canRestart();
    nextPresentation.speed = nextPlayer == null ? 1 : nextPlayer.speedOverride;
    nextPresentation.title = nextPlayer == null ? null : nextPlayer.title();
    nextPresentation.detail = nextPlayer == null ? null : nextPlayer.statusDetail();
    var changed = player != nextPlayer || hobbing != nextHobbing;
    var mask = haxeon.Diff.mask(presentation, nextPresentation);
    if (mask != 0) {
      haxeon.Diff.apply(presentation, presentation, nextPresentation);
      changed = true;
    }
    if (nextHobbing != null) {
      reportData(nextHobbing, nextReport);
      var reportMask = haxeon.Diff.mask(report.telemetry, nextReport.telemetry);
      if (report.measurement != nextReport.measurement) {
        report.measurement = nextReport.measurement;
        changed = true;
      }
      if (reportMask != 0) {
        haxeon.Diff.apply(report.telemetry, report.telemetry, nextReport.telemetry);
        changed = true;
      }
    }
    player = nextPlayer;
    hobbing = nextHobbing;
    if (changed) requestPatch();
  }

  /** Describe the last explicit update; `printer` supplies the connected device's current summary, if any. */
  public function build(
    simulation:ApplicationSimulation,
    tokens:ThemeTokens,
    ?printer:String
  ):View {
    return new CncPanelView(this, simulation, tokens, printer);
  }

  function describe(simulation:ApplicationSimulation, tokens:ThemeTokens, printer:Null<String>):View {
    var style = fill();
    style.padding = new Insets(12.0, 12.0, 12.0, 12.0);
    style.childGap = 8.0;
    style.background = tokens.surface;
    if (hobbing != null) return new HobbingGearReport(report);
    var player = this.player, text = source;
    var heading = new KeyedView(
      "heading",
      new Text(
        player == null ? (presentation.print ? "PRINT PROGRAM" : "CNC PROGRAM") : presentation.title,
        null,
        tokens.textSecondary,
        TextStyleOverride.text(11.0, 0.8)
      )
    );
    var link = new KeyedView("printer", new Text(printer == null ? "" : "Printer: " + printer));
    var linked = printer != null;
    if (player == null) {
      var items = [heading];
      if (linked) items.push(link);
      if (text == null) items.push(new KeyedView("empty", new Text("This project has no machining or print program.")));
      else {
        model.show(text, 0, tokens.selectionField);
        items.push(new KeyedView("status", new Text("Ready · Play to start")));
        items.push(new KeyedView("gcode", programList()));
      }
      return new Column("cnc-panel", items, style);
    }
    model.show(text ?? "", presentation.line, tokens.selectionField);
    var held = presentation.held, failure = presentation.failure;
    var problem = failure == null ? null : ProgramErrorMessage.fromFailure(failure);
    var status = problem != null ? problem.title + (presentation.line > 0 ? ' · line ${presentation.line}'
      : "") : (presentation.finished ? "Completed" : !presentation.started ? "Ready" : !presentation.running
      ? "Paused" : held ? "Held" : "Running") + (presentation.line > 0 ? ' · line ${presentation.line}'
      : "") + presentation.detail + ' · speed ${Math.round(presentation.speed * 100)}%';
    var hold = new Button("Hold", null, () -> simulation.control(() -> player.hold()), "cnc-hold");
    hold.enabled = !held && failure == null;
    var resume = new Button("Resume", null, () -> simulation.control(() -> player.resume()), "cnc-resume");
    resume.enabled = held && failure == null;
    var restart = new Button(selectedLine > 0 ? 'Restart at line $selectedLine' : "Restart at line",
      null, () -> {var line = selectedLine;
    if (line > 0) simulation.control(() -> player.restartFromLine(line));
    }, "cnc-restart");
    restart.enabled = selectedLine > 0 && presentation.restart;
    var buttonsStyle = new LayoutStyle();
    buttonsStyle.width = LayoutAxis.grow();
    buttonsStyle.childGap = 6.0;
    var speed = new Slider(
      "cnc-speed",
      "Speed override",
      presentation.speed,
      0.05,
      2.0,
      0.05,
      value -> simulation.control(() -> player.setSpeedOverride(value))
    );
    var list = programList();
    var buttons = new KeyedView(
      "buttons",
      new Row(
        "cnc-buttons",
        [new KeyedView("hold", hold), new KeyedView("resume", resume), new KeyedView("restart", restart)],
        buttonsStyle
      )
    );
    var state = new KeyedView("status",
      new Text(status)), speedRow = new KeyedView("speed", speed), gcode = new KeyedView(
        "gcode",
        list
      );
    var items:Array<KeyedView> = [heading, state];
    if (problem != null) {
      items.push(new KeyedView("recovery", new Text(problem.recovery)));
      var reset = new Button("Reset program", null, () -> {simulation.reset();
      }, "cnc-reset");
      var details = new Button(
        errorDetails ? "Hide technical details" : "Technical details",
        null,
        () -> {errorDetails = !errorDetails; requestPatch();},
        "cnc-error-details"
      );
      items.push(new KeyedView(
        "recovery-actions",
        new Row("cnc-recovery", [new KeyedView("reset", reset), new KeyedView("details", details)], buttonsStyle)
      ));
      if (errorDetails) items.push(new KeyedView("error-details", new Text(failure)));
    }
    if (linked) items.push(link);
    items.push(buttons);
    items.push(speedRow);
    items.push(gcode);
    return new Column("cnc-panel", items, style);
  }

  function programList():View {
    var style = fill();
    style.clipVertical = true;
    var list = new ListView("cnc-gcode", model, style, scroll, 300.0, selectedLine - 1, index -> {
      selectedLine = index + 1;
      requestPatch();
    });
    list.selectionProvider = () -> selectedLine - 1;
    return list;
  }

  static function reportData(player:app.HobbingProgramPlayer, data:HobbingReportData):Void {
    var stock = player.stock;
    data.telemetry.complete = player.completed;
    data.telemetry.failure = player.failure;
    data.telemetry.status = player.failure != null
      ? "Stopped" : player.handling != null ? player.handling.phase : player.completed ? "Cycle complete" : "Cutting";
    data.telemetry.seconds = player.elapsed;
    data.telemetry.syncArcsec = player.syncError * 648000 / Math.PI;
    data.telemetry.contacts = stock.collisions;
    data.telemetry.windows = stock.stream.passes;
    data.telemetry.poses = stock.stream.poses;
    data.telemetry.buffered = stock.stream.peakFrames;
    data.telemetry.stockKiB = Math.round(stock.maxBytes / 1024);
    data.telemetry.cutting = stock.forces.maxTangential;
    data.telemetry.radial = stock.forces.maxRadial;
    data.telemetry.power = stock.forces.maxPower;
    data.telemetry.hobTorque = stock.forces.maxHobTorque;
    data.telemetry.tableTorque = stock.forces.maxTableTorqueBound;
    data.telemetry.materialBasis = stock.forces.material.basis;
    data.measurement = stock.measurement;
    data.telemetry.inspectionFailure = stock.measurementFailure;
  }

  static function fill():LayoutStyle {
    var style = new LayoutStyle();
    style.width = LayoutAxis.grow();
    style.height = LayoutAxis.grow();
    return style;
  }
}

/** A program's G-code lines, numbered, with the executing line tinted. */
private class GcodeListModel implements ListViewModel {
  var lines:Array<String> = [];
  var source:Null<String>;
  var running = 0;
  var tint:Null<Color> = null;
  var version = 0;

  public function new() {
  }

  public function show(source:String, running:Int, tint:Color):Void {
    if (source != this.source || running != this.running || tint != this.tint) version++;
    if (source != this.source) this.lines = source.split("\n");
    this.source = source;
    this.running = running;
    this.tint = tint;
  }

  public function count():Int return lines.length;
  public function keyAt(index:Int):String return 'line-$index';
  public function estimatedExtent():Float return CncPanel.ROW_HEIGHT;
  public function extentIsUniform():Bool return true;
  public function extentAt(index:Int):Float return CncPanel.ROW_HEIGHT;
  public function extentRevisionAt(index:Int):Int return 0;
  public function totalExtent():Null<Float> return lines.length * CncPanel.ROW_HEIGHT;
  public function revision():Int return version;

  public function buildItem(index:Int):View {
    var style = new LayoutStyle();
    style.width = LayoutAxis.grow();
    style.height = LayoutAxis.fixed(CncPanel.ROW_HEIGHT);
    style.padding = new Insets(6.0, 2.0, 6.0, 2.0);
    if (index + 1 == running && tint != null) style.background = tint;
    return new Text(StringTools.lpad(Std.string(index + 1), " ", 5) + "  " + lines[index], style);
  }
}


private class CncPresentation implements Equatable, Diffable {
  public var source:Null<String>;
  public var generation = -1;
  public var print = false;
  public var running = false;
  public var line = 0;
  public var failure:Null<String>;
  public var held = false;
  public var finished = false;
  public var started = false;
  public var restart = false;
  public var speed = 1.0;
  public var title:Null<String>;
  public var detail:Null<String>;
  public function new() {}
}

@:access(app.editor.CncPanel) private class CncPanelView implements View {
  final panel:CncPanel;
  final simulation:ApplicationSimulation;
  final tokens:ThemeTokens;
  final printer:Null<String>;
  public function new(panel:CncPanel, simulation:ApplicationSimulation, tokens:ThemeTokens, printer:Null<String>) {
    this.panel = panel;
    this.simulation = simulation;
    this.tokens = tokens;
    this.printer = printer;
  }
  public function build(context:BuildContext):RenderNode {
    context.observeInt(() -> panel.bindingRevision);
    return context.withScope(new Key("cnc-panel"), () -> {
      var id = context.id("content");
      panel.patchContext = context;
      panel.patchId = id;
      return context.selfUpdating(id, () -> {
        var root = new RenderNode(id, LayoutVisualKind.Box, CncPanel.fill());
        root.add(panel.describe(simulation, tokens, printer).build(context));
        return root;
      });
    });
  }
}
