package app.editor;

import haxeon.ui.Insets;
import haxeon.ui.LayoutAlignmentY;
import haxeon.ui.LayoutAxis;
import haxeon.ui.LayoutStyle;
import haxeon.ui.TextWrap;
import haxeon.ui.core.TextStyleOverride;
import haxeon.ui.core.View;
import haxeon.ui.theme.ThemeTokens;
import haxeon.ui.widgets.KeyedView;
import haxeon.ui.widgets.controls.Button;
import haxeon.ui.widgets.controls.ButtonVariant;
import haxeon.ui.widgets.controls.Select;
import haxeon.ui.widgets.controls.SelectOption;
import haxeon.ui.widgets.layout.Row;
import haxeon.ui.widgets.overlays.Tooltip;
import haxeon.ui.widgets.overlays.TooltipPlacement;
import haxeon.ui.widgets.text.MiddleEllipsisText;
import haxeon.ui.widgets.text.Text;

/** Compact scenario selection, independent of the simulation and document lifecycle. */
class HobberScenarioSelector {
  static final choices:Array<{id:String, label:String}> = [
    {id: "idle", label: "Idle / doors"},
    {id: "cutting", label: "Cutting"},
    {id: "loading", label: "Load / cut / unload"}
  ];
  static inline final hint:String = "Select a scenario, then press Play. Changing scenarios resets playback.";

  public static function build(tokens:ThemeTokens, selected:String, enabled:Bool, select:String->Void,
      width:Float, status:Null<String>):View {
    var explanation = hint + (status == null ? "" : "\n" + status);
    var style = new LayoutStyle();
    style.width = LayoutAxis.grow();
    style.height = LayoutAxis.fixed(32);
    style.childAlignY = LayoutAlignmentY.Center;
    style.childGap = 8;
    style.padding = new Insets(8, 2, 8, 2);
    var cells:Array<KeyedView> = [new KeyedView("label", help("scenario-help",
      new Text("Scenario", null, tokens.textSecondary, TextStyleOverride.text(12)), explanation))];
    var change:String -> Void = id -> { if (id != selected) select(id); };
    if (width < 440) {
      var selectStyle = new LayoutStyle();
      selectStyle.width = LayoutAxis.grow(0, 280);
      selectStyle.height = LayoutAxis.fixed(28);
      selectStyle.padding = new Insets(8, 0, 8, 0);
      var control = new Select<String>("hobber-scenario", [for (choice in choices)
        new SelectOption<String>(choice.id, choice.label, choice.id)], selected, change, selectStyle);
      control.accessibilityLabel = "Scenario";
      control.enabled = enabled;
      var tooltip = help("scenario-select-help", control, explanation);
      tooltip.fillAnchor = true;
      cells.push(new KeyedView("selector", tooltip));
    } else {
      var segments:Array<KeyedView> = [];
      for (choice in choices) {
        var id = choice.id;
        var buttonStyle = new LayoutStyle();
        buttonStyle.height = LayoutAxis.fixed(24);
        buttonStyle.padding = new Insets(8, 0, 8, 0);
        if (selected == id) buttonStyle.background = tokens.selection;
        buttonStyle.radiusTopLeft = buttonStyle.radiusTopRight = 4;
        buttonStyle.radiusBottomLeft = buttonStyle.radiusBottomRight = 4;
        var button = new Button(choice.label, buttonStyle, () -> change(id), "hobber-scenario-" + id);
        button.variant = ButtonVariant.Navigation;
        button.classes = ["viewport-tool"];
        button.selected = selected == id;
        button.enabled = enabled;
        button.accessibilityLabel = "Scenario: " + choice.label;
        button.labelView = new Text(choice.label, null, enabled && selected == id ? tokens.textPrimary : tokens.textSecondary,
          TextStyleOverride.combine(TextStyleOverride.text(13), TextStyleOverride.paragraph(TextWrap.None)));
        segments.push(new KeyedView(id, help("scenario-help-" + id, button, explanation)));
      }
      var groupStyle = new LayoutStyle();
      groupStyle.childGap = 2;
      groupStyle.padding = new Insets(1, 1, 1, 1);
      cells.push(new KeyedView("selector", new Row("hobber-scenario-segments", segments, groupStyle)));
    }
    if (status != null && width >= 640) cells.push(new KeyedView("state",
      new MiddleEllipsisText("hobber-scenario-state", status, false,
        TextStyleOverride.combine(TextStyleOverride.text(12), TextStyleOverride.foreground(tokens.textSecondary)))));
    return new Row("hobber-scene-controls", cells, style);
  }

  static function help(key:String, anchor:View, explanation:String):Tooltip {
    var tooltip = new Tooltip(key, anchor, new Text(explanation));
    tooltip.placement = TooltipPlacement.Right;
    return tooltip;
  }
}
