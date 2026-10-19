package app.editor;

import app.ApplicationSimulation;
import app.ProgramErrorMessage;
import app.ProjectDocumentSession;
import haxeon.ui.Color;
import haxeon.ui.Insets;
import haxeon.ui.LayoutAxis;
import haxeon.ui.LayoutStyle;
import haxeon.ui.core.TextStyleOverride;
import haxeon.ui.core.View;
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

  public function new() {}

  /** `printer` is the line about a printer the editor is connected to (its state, progress and temperatures), if any. */
  public function build(simulation:ApplicationSimulation, tokens:ThemeTokens, ?printer:String, ?project:ProjectDocumentSession):View {
    var style = fill();
    style.padding = new Insets(12.0, 12.0, 12.0, 12.0);
    style.childGap = 8.0;
    style.background = tokens.surface;
    var player = simulation.toolpathPlayer();
    var cnc=project==null?null:project.cncJob,print=project==null?null:project.printJob;
    var text=player!=null?player.sourceLines().join("\n"):cnc!=null?cnc.source:print!=null?print.source:null;
    var nextGeneration=project==null?-1:project.generation;
    if(source!=text||generation!=nextGeneration) {
      source=text;generation=nextGeneration;selectedLine=0;followedLine=0;scroll.jumpTo(0,0);
    }
    var heading = new KeyedView("heading", new Text(player == null ? (print!=null?"PRINT PROGRAM":"CNC PROGRAM") : player.title(), null, tokens.textSecondary,
      TextStyleOverride.text(11.0, 0.8)));
    var link = new KeyedView("printer", new Text(printer == null ? "" : "Printer: " + printer));
    var linked = printer != null;
    if (player == null) {
      var items=[heading];if(linked)items.push(link);
      if(text==null)items.push(new KeyedView("empty",new Text("This project has no machining or print program.")));
      else {
        model.show(text,0,tokens.selectionField);
        items.push(new KeyedView("status",new Text("Ready · Play to start")));
        items.push(new KeyedView("gcode",programList()));
      }
      return new Column("cnc-panel",items,style);
    }
    model.show(text??"", player.currentLine, tokens.selectionField);
    // Follow the running line, keeping a few lines of context above it.
    if (player.currentLine > 0 && player.currentLine != followedLine) {
      followedLine = player.currentLine;
      scroll.jumpTo(scroll.offsetX, Math.max(0.0, (player.currentLine - 4) * ROW_HEIGHT));
    }
    var held = player.held(), failure = player.failure;
    if (failure != lastFailure) { lastFailure = failure; errorDetails = false; }
    var problem = failure == null ? null : ProgramErrorMessage.fromFailure(failure);
    var status = problem != null ? problem.title + (player.currentLine > 0 ? ' · line ${player.currentLine}' : "") :
      (player.finished() ? "Completed" : !player.hasStarted() ? "Ready" : !simulation.isRunning() ? "Paused" : held ? "Held" : "Running") + (player.currentLine > 0 ? ' · line ${player.currentLine}' : "") +
      player.statusDetail() + ' · speed ${Math.round(player.speedOverride * 100)}%';
    var hold = new Button("Hold", null, () -> simulation.control(() -> player.hold()), "cnc-hold");
    hold.enabled = !held && failure == null;
    var resume = new Button("Resume", null, () -> simulation.control(() -> player.resume()), "cnc-resume");
    resume.enabled = held && failure == null;
    var restart = new Button(selectedLine > 0 ? 'Restart at line $selectedLine' : "Restart at line", null,
      () -> {
        var line = selectedLine;
        if (line > 0) simulation.control(() -> player.restartFromLine(line));
      }, "cnc-restart");
    restart.enabled = selectedLine > 0 && player.canRestart();
    var buttonsStyle = new LayoutStyle();
    buttonsStyle.width = LayoutAxis.grow();
    buttonsStyle.childGap = 6.0;
    var speed = new Slider("cnc-speed", "Speed override", player.speedOverride, 0.05, 2.0, 0.05,
      value -> simulation.control(() -> player.setSpeedOverride(value)));
    var list = programList();
    var buttons = new KeyedView("buttons", new Row("cnc-buttons", [new KeyedView("hold", hold),
      new KeyedView("resume", resume), new KeyedView("restart", restart)], buttonsStyle));
    var state = new KeyedView("status", new Text(status)), speedRow = new KeyedView("speed", speed), gcode = new KeyedView("gcode", list);
    var items:Array<KeyedView> = [heading, state];
    if (problem != null) {
      items.push(new KeyedView("recovery", new Text(problem.recovery)));
      var reset = new Button("Reset program", null, () -> { simulation.reset(); }, "cnc-reset");
      var details = new Button(errorDetails ? "Hide technical details" : "Technical details", null,
        () -> errorDetails = !errorDetails, "cnc-error-details");
      items.push(new KeyedView("recovery-actions", new Row("cnc-recovery", [new KeyedView("reset", reset), new KeyedView("details", details)], buttonsStyle)));
      if (errorDetails) items.push(new KeyedView("error-details", new Text(failure)));
    }
    if (linked) items.push(link);
    items.push(buttons); items.push(speedRow); items.push(gcode);
    return new Column("cnc-panel", items, style);
  }

  function programList():View {
    var style=fill();style.clipVertical=true;
    return new ListView("cnc-gcode",model,style,scroll,300.0,selectedLine-1,index -> selectedLine=index+1);
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

  public function new() {}

  public function show(source:String, running:Int, tint:Color):Void {
    if (source != this.source || running != this.running || tint != this.tint) version++;
    if(source!=this.source)this.lines=source.split("\n");
    this.source=source;
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
