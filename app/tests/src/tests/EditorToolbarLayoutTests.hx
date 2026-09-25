package tests;

import app.EditorToolbarLayout;
import app.EditorToolbarLayout.EditorToolbarDensity;

class EditorToolbarLayoutTests {
  public static function main():Int {
    var density = Full;
    for (width in [965.0, 955.0, 965.0, 955.0])
      density = EditorToolbarLayout.forWidth(density, width);
    if (density != Full) return 1;

    density = EditorToolbarLayout.forWidth(density, 920.0);
    if (density != Compact) return 2;
    for (width in [955.0, 965.0, 955.0, 965.0])
      density = EditorToolbarLayout.forWidth(density, width);
    if (density != Compact) return 3;

    density = EditorToolbarLayout.forWidth(density, 1000.0);
    if (density != Full) return 4;
    density = EditorToolbarLayout.forWidth(density, 600.0);
    if (density != Minimal) return 5;
    density = EditorToolbarLayout.forWidth(density, 690.0);
    if (density != Compact) return 6;
    return 0;
  }
}
