package machinekit.milling;

/** Compact BT30 release study, in the cartridge gauge frame, millimetres.
 * Interfaces and the force allowance are assumptions pending supplier selection. */
class MillDrawbarLayout {
	public static inline var BORE:Float = 80;
	public static inline var WIDTH:Float = 100;
	public static inline var HEIGHT:Float = 46;
	public static inline var BRIDGE_BOTTOM:Float = 302;
	public static inline var BRIDGE_THICKNESS:Float = 8;
	public static inline var CYLINDER_BOTTOM:Float = BRIDGE_BOTTOM+BRIDGE_THICKNESS;
	public static inline var CAP:Float = 8;
	public static inline var MOUNT_X:Float = 40;
	public static inline var MOUNT_Y:Float = 25;
	public static inline var PISTON_BOTTOM:Float = CYLINDER_BOTTOM+24;
	public static inline var PISTON_THICKNESS:Float = 8;
	public static inline var ROD_TIP:Float = 287;
	public static inline var STUDY_STROKE:Float = 10;
	public static inline var RELEASE_STROKE:Float = 6.5;
	public static inline var PRESSURE_BAR:Float = 6;
	public static inline var FORCE_ALLOWANCE:Float = 0.8;
	public static inline var REFERENCE_RELEASE_FORCE:Float = 2000;
	public static function theoreticalPushForce(pressureBar:Float):Float {
		if (!Math.isFinite(pressureBar) || pressureBar<=0) throw "Cylinder pressure must be positive";
		return pressureBar*0.1*Math.PI*BORE*BORE/4;
	}
}
