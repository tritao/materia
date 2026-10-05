package app.editor;

import app.ApplicationSimulation;
import app.CncProgramPlayer;
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

  public function new() {}

  public function build(simulation:ApplicationSimulation, tokens:ThemeTokens):View {
    var style = fill();
    style.padding = new Insets(12.0, 12.0, 12.0, 12.0);
    style.childGap = 8.0;
    style.background = tokens.surface;
    var player = simulation.cncPlayer();
    var heading = new KeyedView("heading", new Text("CNC PROGRAM", null, tokens.textSecondary, TextStyleOverride.text(11.0, 0.8)));
    if (player == null)
      return new Column("cnc-panel", [heading, new KeyedView("empty",
        new Text("Play a project with a machining job to run its program here."))], style);
    model.show(player.sourceLines(), player.currentLine, tokens.selectionField);
    // Follow the running line, keeping a few lines of context above it.
    if (player.currentLine > 0 && player.currentLine != followedLine) {
      followedLine = player.currentLine;
      scroll.jumpTo(scroll.offsetX, Math.max(0.0, (player.currentLine - 4) * ROW_HEIGHT));
    }
    var held = player.held(), failure = simulation.cncFailure();
    var status = failure != null ? 'Stopped: $failure' :
      (held ? "Held" : "Running") + (player.currentLine > 0 ? ' · line ${player.currentLine}' : "") +
      ' · tool ${player.loadedTool} · speed ${Math.round(player.speedOverride * 100)}%';
    var hold = new Button("Hold", null, () -> player.hold(), "cnc-hold");
    hold.enabled = !held && failure == null;
    var resume = new Button("Resume", null, () -> player.resume(), "cnc-resume");
    resume.enabled = held;
    var restart = new Button(selectedLine > 0 ? 'Restart at line $selectedLine' : "Restart at line", null,
      () -> {
        if (selectedLine > 0) player.restartFromLine(selectedLine);
      }, "cnc-restart");
    restart.enabled = selectedLine > 0;
    var buttonsStyle = new LayoutStyle();
    buttonsStyle.width = LayoutAxis.grow();
    buttonsStyle.childGap = 6.0;
    var speed = new Slider("cnc-speed", "Speed override", player.speedOverride, 0.05, 2.0, 0.05,
      value -> player.setSpeedOverride(value));
    var listStyle = fill();
    listStyle.clipVertical = true;
    var list = new ListView("cnc-gcode", model, listStyle, scroll, 300.0, selectedLine - 1,
      index -> selectedLine = index + 1);
    return new Column("cnc-panel", [
      heading,
      new KeyedView("status", new Text(status)),
      new KeyedView("buttons", new Row("cnc-buttons", [new KeyedView("hold", hold),
        new KeyedView("resume", resume), new KeyedView("restart", restart)], buttonsStyle)),
      new KeyedView("speed", speed),
      new KeyedView("gcode", list)
    ], style);
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
  var running = 0;
  var tint:Null<Color> = null;
  var version = 0;

  public function new() {}

  public function show(lines:Array<String>, running:Int, tint:Color):Void {
    if (lines.length != this.lines.length || running != this.running) version++;
    this.lines = lines;
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
