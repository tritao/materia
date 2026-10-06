package app;

import haxeon.ui.Color;
import haxeon.ui.theme.Theme;

/** Editor-specific surfaces and control colors shared by both theme variants. */
class EditorAppearance {
  public final theme:Theme;
  public final dark:Bool;
  public final canvas:Color;
  public final toolbar:Color;
  /** Toolbar surface while a simulation is active, so the non-editing state is visible at a glance. */
  public final toolbarSimulating:Color;

  public function new(?source:Theme) {
    theme = source == null ? Theme.light() : source;
    dark = theme.tokens.panelBackground.red < 0.5;
    canvas = dark ? rgb(0.075, 0.09, 0.11) : rgb(0.945, 0.955, 0.97);
    toolbar = theme.tokens.surfaceRaised;
    var accent = theme.tokens.accent;
    toolbarSimulating = Color.rgba(
      toolbar.red + (accent.red - toolbar.red) * 0.22,
      toolbar.green + (accent.green - toolbar.green) * 0.22,
      toolbar.blue + (accent.blue - toolbar.blue) * 0.22, 1.0);

    theme.body.textStyle.fontSize = 14.0;
    theme.label.textStyle.fontSize = 13.0;
    theme.caption.textStyle.fontSize = 12.0;
    theme.button.textStyle.fontSize = 13.0;
    theme.refreshStyles();
  }

  static inline function rgb(red:Float, green:Float, blue:Float):Color
    return Color.rgba(red, green, blue, 1.0);
}
