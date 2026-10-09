package app.editor;

import app.MissionController;
import haxeon.ui.Insets;
import haxeon.ui.LayoutAxis;
import haxeon.ui.LayoutStyle;
import haxeon.ui.TextWrap;
import haxeon.ui.core.TextStyleOverride;
import haxeon.ui.core.View;
import haxeon.ui.theme.ThemeTokens;
import haxeon.ui.widgets.KeyedView;
import haxeon.ui.widgets.controls.Button;
import haxeon.ui.widgets.controls.Select;
import haxeon.ui.widgets.controls.SelectOption;
import haxeon.ui.widgets.layout.Column;
import haxeon.ui.widgets.text.Text;

/** The same selector in a dock panel or a small viewport card. */
class MissionJobView {
  public static function build(controller:MissionController, tokens:ThemeTokens, key:String,
      ?showSteps:Void->Void):View {
    var state = controller.snapshot();
    var rows:Array<KeyedView> = [];
    if (state.jobs.length > 0) {
      var selectorStyle = new LayoutStyle(); selectorStyle.width = LayoutAxis.grow();
      var options = [for (job in state.jobs) new SelectOption<String>(job.id, job.label, job.id)];
      var selected = state.selectedJob == null ? state.jobs[0].id : state.selectedJob;
      var selector = new Select<String>(key + "-job", options, selected, controller.selectJob, selectorStyle);
      selector.accessibilityLabel = "Mission job";
      selector.enabled = state.switchAllowed;
      rows.push(new KeyedView("selector", selector));
      if (state.summary != "") rows.push(new KeyedView("summary", new Text(state.summary, null, tokens.textSecondary, TextStyleOverride.paragraph(TextWrap.WordCharacter))));
      if (state.simulationActive) rows.push(new KeyedView("locked",
        new Text("Stop simulation to change jobs.", null, tokens.textSecondary, TextStyleOverride.paragraph(TextWrap.WordCharacter))));
    }
    rows.push(new KeyedView("status", new Text(state.status, null,
      state.progress != null && state.progress.phase == "failed" ? tokens.danger : tokens.text, TextStyleOverride.paragraph(TextWrap.WordCharacter))));
    if (state.error != null) rows.push(new KeyedView("error", new Text(state.error, null, tokens.danger, TextStyleOverride.paragraph(TextWrap.WordCharacter))));
    if (state.loading) rows.push(new KeyedView("cancel", new Button("Cancel", null, controller.cancel, key + "-cancel")));
    if (showSteps != null) rows.push(new KeyedView("steps", new Button("Show steps", null, showSteps, key + "-steps")));
    var style = new LayoutStyle(); style.width = LayoutAxis.grow(); style.childGap = 6.0;
    if (showSteps != null) {
      style.padding = new Insets(10, 10, 10, 10);
      style.background = tokens.surfaceRaised;
      style.radiusTopLeft = style.radiusTopRight = style.radiusBottomLeft = style.radiusBottomRight = 6.0;
    }
    return new Column(key, rows, style);
  }
}
