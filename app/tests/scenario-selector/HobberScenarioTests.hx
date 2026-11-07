import app.editor.HobberScenarioSelector;
import haxeon.platform.Runtime;
import haxeon.ui.FontCollection;
import haxeon.ui.LayoutFrame;
import haxeon.ui.core.RenderNode;
import haxeon.ui.core.UiContext;
import haxeon.ui.core.UiEventKind;
import haxeon.ui.core.UiKey;
import haxeon.ui.semantics.AccessibilityRole;
import haxeon.ui.semantics.AccessibilityState;
import haxeon.ui.theme.Theme;

class HobberScenarioTests {
  static function check(value:Bool, message:String):Void if (!value) throw message;
  static function find(root:RenderNode, label:String):RenderNode {
    var found:Null<RenderNode> = null;
    root.walk(node -> { if (node.semantics != null && node.semantics.role != AccessibilityRole.Text && node.semantics.label == label) found = node; });
    if (found == null) throw "Missing control: " + label;
    return found;
  }
  public static function main():Int {
    var runtime = Runtime.start();
    var fonts = FontCollection.create(); fonts.addSystemFallbacks();
    for (theme in [Theme.light(), Theme.dark()]) for (width in [200, 260, 439, 440, 640, 1000]) {
      var ui = new UiContext(null, fonts, theme);
      ui.buildContext.environment.reducedMotion = true;
      var changes:Array<String> = [];
      var view = HobberScenarioSelector.build(theme.tokens, "cutting", true, id -> changes.push(id), width, null);
      var frame = new LayoutFrame(width, 80);
      var root = ui.submit(view, frame);
      check(root.globalBounds().height <= 34, "scenario bar became taller than one compact row");
      root.walk(node -> {
        if (node.layout.text != null) check(node.layout.text.indexOf("Select a scenario") < 0, "idle instructions occupy the toolbar");
        if (node.focusable && node.resolved != null) {
          var bounds = node.globalBounds();
          check(bounds.x >= 0 && bounds.x + bounds.width <= width + 0.5, "scenario control overflows its pane");
        }
      });
      if (width >= 440) {
        var current = find(root, "Scenario: Cutting");
        var semantics = current.semantics;
        if (semantics == null) throw "missing scenario semantics";
        check((semantics.states & AccessibilityState.Selected) != 0, "current scenario has no selected state");
        var idle = find(root, "Scenario: Idle / doors");
        var fill = idle.layout.style.background;
        check(fill != null && fill.alpha == 0, "inactive scenario has a filled background");
        var currentFill = current.layout.style.background;
        check(currentFill != null && currentFill.alpha > 0, "selected scenario has no highlight");
        var bounds = current.globalBounds();
        ui.pointerDown(bounds.x + 4, bounds.y + 4, 0);
        ui.pointerUp(bounds.x + 4, bounds.y + 4, 0);
        check(changes.length == 0, "clicking the current scenario resets playback");
        ui.focusWidget(idle.id);
        ui.key(UiEventKind.KeyDown, UiKey.Enter); ui.key(UiEventKind.KeyUp, UiKey.Enter);
        check(changes.join(",") == "idle", "keyboard scenario selection failed");
      } else {
        var select = find(root, "Scenario");
        var semantics = select.semantics;
        if (semantics == null) throw "missing dropdown semantics";
        check(semantics.role == AccessibilityRole.ComboBox, "narrow pane has no scenario dropdown");
        ui.focusWidget(select.id);
        ui.key(UiEventKind.KeyDown, UiKey.Down); ui.key(UiEventKind.KeyUp, UiKey.Down);
        root = ui.submit(view, frame);
        var popup = find(root, "Idle / doors");
        ui.focusWidget(popup.id);
        ui.key(UiEventKind.KeyDown, UiKey.Enter); ui.key(UiEventKind.KeyUp, UiKey.Enter);
        check(changes.join(",") == "idle", "dropdown scenario selection failed");
      }
      ui.clearFocus(); ui.pointerLeave();
      var disabled = HobberScenarioSelector.build(theme.tokens, "cutting", false, id -> changes.push(id), width, "Cutting");
      root = ui.submit(disabled, frame);
      var target = find(root, width >= 440 ? "Scenario: Idle / doors" : "Scenario");
      check(!target.enabled, "scenario changes remain enabled during playback");
      ui.dispose();
    }
    fonts.dispose(); runtime.dispose();
    Sys.println("PASS: scenario selection fits wide/narrow panes in both themes, supports keyboard input and preserves playback locks");
    return 0;
  }
}
