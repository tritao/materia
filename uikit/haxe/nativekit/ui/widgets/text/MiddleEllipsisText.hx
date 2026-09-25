package nativekit.ui.widgets.text;

import LayoutAxis;
import LayoutStyle;
import ParagraphStyle;
import TextLayout;
import TextWrap;
import nativekit.ui.core.BuildContext;
import nativekit.ui.core.Key;
import nativekit.ui.core.RenderNode;
import nativekit.ui.core.TextStyleOverride;
import nativekit.ui.core.View;
import nativekit.ui.theme.TextRole;

/** Keeps both ends of a single-line label visible within its resolved width. */
class MiddleEllipsisText implements View {
  public final key:String;
  public final value:String;
  public var truncated(default, null):Bool = false;

  public function new(key:String, value:String) {
    this.key = key;
    this.value = value;
  }

  public function build(context:BuildContext):RenderNode {
    return context.withScope(new Key(key), function() {
      var displayed = context.state(context.id("displayed"), value);
      var style = new LayoutStyle();
      style.width = LayoutAxis.grow();
      style.clipHorizontal = true;
      var text = new Text(displayed.value, style, null,
        TextStyleOverride.paragraph(TextWrap.None));
      var node = text.build(context);
      if (node.semantics != null) node.semantics.label = value;
      var resolved = context.resolveTextRole(TextRole.Body);
      node.onResolved(function(geometry) {
        if (context.fonts == null) return;
        var paragraph = new ParagraphStyle(TextWrap.None, resolved.paragraphStyle.alignment,
          resolved.paragraphStyle.lineHeight, resolved.paragraphStyle.direction);
        var layout = TextLayout.createStyled(context.fonts, value, 100000.0,
          resolved.textStyle, paragraph);
        var available = Math.max(0.0, geometry.clippedViewportBounds().width);
        var next = value;
        if (layout.measure().width > available + 1.0) {
          var low = 0, high = value.length;
          while (low < high) {
            var count = (low + high + 1) >> 1;
            var prefix = (count + 1) >> 1;
            var candidate = value.substr(0, prefix) + "…" + value.substr(value.length - (count - prefix));
            layout.setText(candidate);
            if (layout.measure().width <= available) low = count;
            else high = count - 1;
          }
          var prefix = (low + 1) >> 1;
          next = value.substr(0, prefix) + "…" + value.substr(value.length - (low - prefix));
        }
        layout.dispose();
        truncated = next != value;
        if (displayed.value != next) displayed.update(next);
      });
      return node;
    });
  }
}
