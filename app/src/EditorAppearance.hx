package app;

import Color;
import nativekit.ui.theme.Theme;

/** Editor-specific surfaces and control colors shared by both theme variants. */
class EditorAppearance {
  public final theme:Theme;
  public final dark:Bool;
  public final canvas:Color;
  public final toolbar:Color;

  public function new(?source:Theme) {
    theme = source == null ? Theme.light() : source;
    dark = theme.tokens.panelBackground.red < 0.5;
    canvas = dark ? rgb(0.075, 0.09, 0.11) : rgb(0.945, 0.955, 0.97);
    toolbar = theme.tokens.surfaceRaised;

    theme.body.textStyle.fontSize = 14.0;
    theme.label.textStyle.fontSize = 13.0;
    theme.caption.textStyle.fontSize = 12.0;
    theme.button.textStyle.fontSize = 13.0;
    theme.refreshStyles();
  }

  static inline function rgb(red:Float, green:Float, blue:Float):Color
    return Color.rgba(red, green, blue, 1.0);
}
