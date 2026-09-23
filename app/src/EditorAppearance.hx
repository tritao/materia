package app;

import Color;
import nativekit.ui.theme.Theme;

/** Editor-specific surfaces and control colors shared by both theme variants. */
class EditorAppearance {
  public final theme:Theme;
  public final canvas:Color;
  public final surface:Color;
  public final toolbar:Color;
  public final muted:Color;
  public final heading:Color;
  public final divider:Color;

  public function new(?source:Theme) {
    theme = source == null ? Theme.light() : source;
    var dark = theme.tokens.panelBackground.red < 0.5;
    canvas = dark ? rgb(0.075, 0.09, 0.11) : rgb(0.945, 0.955, 0.97);
    surface = dark ? rgb(0.105, 0.125, 0.15) : rgb(0.985, 0.99, 1.0);
    toolbar = dark ? rgb(0.13, 0.15, 0.18) : rgb(0.965, 0.975, 0.985);
    muted = dark ? rgb(0.62, 0.68, 0.76) : rgb(0.38, 0.44, 0.53);
    heading = dark ? rgb(0.72, 0.79, 0.87) : rgb(0.30, 0.39, 0.51);
    divider = dark ? rgb(0.28, 0.32, 0.38) : rgb(0.82, 0.86, 0.90);

    var tokens = theme.tokens;
    tokens.buttonBackground = dark ? rgb(0.20, 0.24, 0.29) : rgb(0.90, 0.93, 0.96);
    tokens.buttonHover = dark ? rgb(0.25, 0.30, 0.36) : rgb(0.84, 0.89, 0.94);
    tokens.buttonPressed = dark ? rgb(0.17, 0.22, 0.28) : rgb(0.78, 0.85, 0.92);
    tokens.buttonFocused = dark ? rgb(0.22, 0.36, 0.51) : rgb(0.78, 0.86, 0.96);
    tokens.buttonSelected = dark ? rgb(0.20, 0.35, 0.50) : rgb(0.78, 0.87, 0.97);
    tokens.buttonDisabled = dark ? rgb(0.15, 0.18, 0.22) : rgb(0.94, 0.95, 0.96);
    tokens.navigationBackground = toolbar;
    tokens.navigationHover = tokens.buttonHover;
    tokens.navigationPressed = tokens.buttonPressed;
    tokens.navigationFocused = tokens.buttonFocused;
    tokens.navigationSelected = tokens.buttonSelected;
    tokens.navigationDisabled = tokens.buttonDisabled;
    tokens.panelBackground = surface;
    theme.body.textStyle.fontSize = 14.0;
    theme.label.textStyle.fontSize = 13.0;
    theme.caption.textStyle.fontSize = 12.0;
    theme.button.textStyle.fontSize = 13.0;
    theme.refreshStyles();
  }

  static inline function rgb(red:Float, green:Float, blue:Float):Color
    return Color.rgba(red, green, blue, 1.0);
}
