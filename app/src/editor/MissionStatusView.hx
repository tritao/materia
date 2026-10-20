package app.editor;

import app.MissionController;
import haxeon.ui.LayoutAxis;
import haxeon.ui.LayoutStyle;
import haxeon.ui.TextWrap;
import haxeon.ui.core.TextStyleOverride;
import haxeon.ui.core.View;
import haxeon.ui.theme.ThemeTokens;
import haxeon.ui.widgets.KeyedView;
import haxeon.ui.widgets.controls.Button;
import haxeon.ui.widgets.layout.Column;
import haxeon.ui.widgets.text.Text;

/** Runtime mission status in a dock panel or viewport card. */
class MissionStatusView {
  public static function build(controller:MissionController, tokens:ThemeTokens, key:String,
      ?showSteps:Void->Void):View {
    var state = controller.snapshot();
    var rows:Array<KeyedView> = [];
    rows.push(new KeyedView("status", new Text(state.status, null,
      (state.progress != null && state.progress.phase == "failed") || StringTools.startsWith(state.machiningStatus,"Failed") ? tokens.danger : tokens.text, TextStyleOverride.paragraph(TextWrap.WordCharacter))));
    if(state.machiningPreview)rows.push(new KeyedView("preview",new Text("Kinematic preview · motor loads are not qualified.",null,tokens.textSecondary,
      TextStyleOverride.paragraph(TextWrap.WordCharacter))));
    if(state.mission!=null&&state.machining!=null)rows.push(new KeyedView("machining",new Text(state.machining.label+" · "+state.machiningStatus)));
    if (showSteps != null) rows.push(new KeyedView("steps", new Button(state.mission==null?"Show mission":"Show steps", null, showSteps, key + "-steps")));
    var style = new LayoutStyle(); style.width = LayoutAxis.grow(); style.childGap = 6.0;
    return new Column(key, rows, style);
  }
}
