package machinekit.pendant;

/**
 * The first layout of the one-SKU pendant: wheel on the right, display and E-stop across the top left,
 * the two rotary switches and F1, F2 below them, then F3 and the fixed Start, Hold and Stop/Reset keys in the
 * bottom row. One slot at the bottom right is free. A starting point for the face-plate mock-up, tuned
 * with the checker, and every assumed dimension in it is to be replaced by the measured part.
 */
class ReferencePendant {
  public static function face():PendantFace {
    var controls = [
      PendantParts.vallderWheel(65.0, 0.0),
      PendantParts.eStopCompact(-85.0, 30.0),
      PendantParts.display242(-29.0, 33.0),
      PendantParts.button("wheel-enable", 17.0, 33.0),
      PendantParts.rotary("axis", -92.5, -8.0),
      PendantParts.rotary("step", -62.5, -8.0),
      PendantParts.button("f1", -35.0, -8.0),
      PendantParts.button("f2", -10.0, -8.0),
      PendantParts.button("f3", -95.0, -38.0),
      PendantParts.button("start", -70.0, -38.0),
      PendantParts.button("hold", -45.0, -38.0),
      PendantParts.button("stop", -20.0, -38.0)
    ];
    return new PendantFace(220.0, 110.0, 3.0, 3.0, 30.0, controls);
  }
}
