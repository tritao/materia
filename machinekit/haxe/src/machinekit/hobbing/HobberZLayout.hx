package machinekit.hobbing;
/** Central vertical drive datums, mm in the work frame at X=Z=0.
 * Fixed bearing/motor above the nut; the loaded screw segment works in tension.
 */
class HobberZLayout {
  public static inline final FOUNDATION_ORIGIN_Z = 290.0;
  /** Screw axis, 25 mm (the BK12 and BF12 shaft height) in front of the 40 mm ground bar pads on the column's drive seat at 241. */
  public static inline final AXIS_X = 176.0;
  public static inline final NUT_OFFSET_Z = 55.0;
  public static inline final SHAFT_TOP = 730.0;
  public static inline final SHAFT_LENGTH = 420.0;
  public static inline final NUT_STATION = SHAFT_TOP - (363 + NUT_OFFSET_Z);
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
