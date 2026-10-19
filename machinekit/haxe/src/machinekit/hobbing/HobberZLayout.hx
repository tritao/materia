package machinekit.hobbing;
/** Central vertical drive datums, mm in the work frame at X=Z=0.
 * Fixed bearing/motor above the nut; the loaded screw segment works in tension.
 */
class HobberZLayout {
  public static inline final AXIS_X=180.0;
  public static inline final NUT_OFFSET_Z=55.0;
  public static inline final SHAFT_TOP=730.0;
  public static inline final SHAFT_LENGTH=420.0;
  public static inline final NUT_STATION=SHAFT_TOP-(363+NUT_OFFSET_Z);
  public static inline final SEAT_FACE_X=241.0;
  public static inline final SEAT_COLUMN_Y=25.0;
  public static inline final SPINE_FRONT_COLUMN_Y=45.0;
  public static inline final SCALE_Y=70.0;
}
