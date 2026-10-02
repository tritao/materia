package machinekit.assembly;

/**
 * Allowances a drive makes when its record gives none. They are assumptions for hobby-class
 * hardware, not datasheet values: nothing here is a measured property of a particular nut, belt or
 * bearing, and a machine that knows better states it.
 */
class DriveDefaults {
	/**
	 * Lost motion of a lead screw's nut on reversal, in mm. Assumption: an anti-backlash (preloaded
	 * plastic or split) nut on a Tr10 x 2 screw, which makers rate from zero to about 0.1 mm; a plain
	 * bronze nut is 0.1 to 0.3 mm.
	 */
	public static inline var LEAD_SCREW_BACKLASH:Float = 0.05;
	/**
	 * Drag torque of a lead screw while it turns, N m: nut preload drag plus support bearings.
	 * Assumption: 0.02 N m, about 1.6% of a NEMA 23's holding torque, for a lightly preloaded nut.
	 */
	public static inline var LEAD_SCREW_DRAG:Float = 0.02;
	/** Drag torque of a belt pulley and its idler bearings, N m. Assumption: 0.005 N m. */
	public static inline var BELT_DRAG:Float = 0.005;
	/** Share of a screw's first bending speed to stay under: the usual 80%. */
	public static inline var CRITICAL_SPEED_MARGIN:Float = 0.8;
}
