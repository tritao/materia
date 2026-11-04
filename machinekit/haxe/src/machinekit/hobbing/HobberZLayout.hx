package machinekit.hobbing;
/** Central vertical drive datums, mm in the work frame at X=Z=0.
 * Fixed bearing/motor above the nut; the loaded screw segment works in tension.
 */
class HobberZLayout {
  /** Height of the work seat above the work frame's zero (z), mm: the hob axis, the mandrel's seat and the origin of the Z, Y and swivel parts. */
  public static inline final SEAT_Z = 363.0;
  public static inline final FOUNDATION_ORIGIN_Z = 290.0;
  /** Screw axis, 25 mm (the BK12 and BF12 shaft height) in front of the 40 mm ground bar pads on the column's drive seat at 241. */
  public static inline final AXIS_X = 176.0;
  /**
   * Screw axis across the machine (y): the eRob housing of the A axis (Ø110, 88 mm deep) stands behind the Z saddle at y = 0, so the screw
   * and its nut block, 60 wide, are offset 82 mm to -y (the BF12 support and its pad keep 2 mm from the housing's Ø110 at the lowest Z; at -80 they came within 0.8 and 1.6 mm, and at -85 the 80 mm seat insert lacks casting cover to the lifting insert). The drive's
   * 80 mm pads stand on the spine's left side, which is widened for them (`FoundationLayout`: the left head recess is 65 mm deep).
   */
  public static inline final AXIS_Y = -82.0;
  public static inline final NUT_OFFSET_Z = 55.0;
  public static inline final SHAFT_TOP = 730.0;
  public static inline final SHAFT_LENGTH = 420.0;
  public static inline final NUT_STATION = SHAFT_TOP - (SEAT_Z + NUT_OFFSET_Z);
  /** BK12 centre, from the top of the standard 51 mm screw end. */
  public static inline final FIXED_BEARING_Z=SHAFT_TOP-machinekit.motion.HzMotionDrawingScrew.BK12_SUPPORT_CENTRE;
  /** BF12 centre, from the screw tip on the standard 11 mm floating end. */
  public static inline final FLOATING_BEARING_Z = SHAFT_TOP - SHAFT_LENGTH + machinekit.motion.HzMotionDrawingScrew.BF12_SUPPORT_CENTRE;
  /** Clamp coupling length: its screw half stays clear of the BK12 housing on the 51 mm screw end. */
  public static inline final COUPLING_LENGTH=40.0;
  public static inline final SEAT_FACE_X = 241.0;
  public static inline final SEAT_COLUMN_Y = 25.0;
  public static inline final SPINE_FRONT_COLUMN_Y = 45.0;
  public static inline final SCALE_Y = 70.0;
}
