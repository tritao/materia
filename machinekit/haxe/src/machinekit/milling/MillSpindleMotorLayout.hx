package machinekit.milling;

/** Shared datums of the centered spindle-drive study, in the cartridge gauge
 * frame. The 440 mm belt and equal 40T HTD5M pulleys give 120 mm centres. */
class MillSpindleMotorLayout {
	public static inline var CENTRES:Float = 120;
	public static inline var ADJUSTMENT:Float = 5;
	public static inline var PLATE_FRONT:Float = 34;
	public static inline var PLATE_REAR:Float = 180;
	public static inline var FOOT_BOLT_X:Float = 68;
	public static inline var FOOT_BOLT_FRONT:Float = 110;
	public static inline var FOOT_BOLT_REAR:Float = 130;
	public static inline var GUARD_BOLT_X:Float = 56;
	public static inline var GUARD_BOLT_Y:Float = 112;
	public static inline var TENSION_X:Float = 40;
	public static inline var TENSION_SEAT_Y:Float = 25;
}
