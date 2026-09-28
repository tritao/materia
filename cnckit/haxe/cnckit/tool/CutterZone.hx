package stockkit.tool;

/** What happens when this part of a tool meets stock. */
enum CutterZone {
  /** Flutes: overlap with stock removes material. */
  Cutting;
  /** Overlap with stock is a shank collision. */
  Shank;
  /** Overlap with stock is a holder collision. */
  Holder;
}
