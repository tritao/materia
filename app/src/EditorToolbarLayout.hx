package app;

enum EditorToolbarDensity {
  Full;
  Compact;
  Minimal;
}

/** Responsive toolbar policy, with room around each content-fit breakpoint. */
class EditorToolbarLayout {
  // Preferred widths of the full and compact toolbar compositions.
  static inline var FULL_FIT_WIDTH:Float = 1040.0;
  static inline var COMPACT_FIT_WIDTH:Float = 800.0;
  static inline var RESIZE_MARGIN:Float = 24.0;

  public static function forWidth(previous:EditorToolbarDensity, width:Float):EditorToolbarDensity {
    return switch (previous) {
      case Full:
        if (width < COMPACT_FIT_WIDTH - RESIZE_MARGIN) Minimal
        else if (width < FULL_FIT_WIDTH - RESIZE_MARGIN) Compact
        else Full;
      case Compact:
        if (width < COMPACT_FIT_WIDTH - RESIZE_MARGIN) Minimal
        else if (width > FULL_FIT_WIDTH + RESIZE_MARGIN) Full
        else Compact;
      case Minimal:
        if (width > FULL_FIT_WIDTH + RESIZE_MARGIN) Full
        else if (width > COMPACT_FIT_WIDTH + RESIZE_MARGIN) Compact
        else Minimal;
    };
  }
}
