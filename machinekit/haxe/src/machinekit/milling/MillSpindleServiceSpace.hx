package machinekit.milling;

import cadkit.modeling.Part;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.PosedParts;
import machinekit.assembly.Diagnostics;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Project allocation above the spindle, not a supplier actuator or installed mass.
 * Dimensions include intended actuator/mount space; force, stroke and reaction-path
 * qualification await a selected spindle and release mechanism.
 */
class MillSpindleServiceSpace {
	public static inline var DIAMETER:Float = 120;
	public static inline var HEIGHT:Float = 200;
	public static inline var STANDOFF:Float = 80;

	public static function bottom(layout:MillZAxisLayout):Float
		return layout.headBottom + layout.headHeight + STANDOFF;

	/** Payload shares the head's Z motion, so compare at their common zero pose. */
	public static function checkPayload(assembly:MachineAssembly, layout:MillZAxisLayout):Diagnostics {
		var out = new Diagnostics(), cache = new PosedParts(), space = geometry(layout);
		var poses:Map<String,AssemblyFrame> = assembly.solvedPoses();
		try for (member in assembly.components()) if (StringTools.startsWith(member.id, "spindle/")) {
			var part = cache.posed(member.component, poses.get(member.id));
			var common = space.intersect(part), volume = common.volume(); common.close(); part.close();
			if (volume > 1e-5) out.error("mill-z.release-space-collision", member.id, "Spindle payload enters reserved release space");
		} catch (e:Dynamic) { space.close(); cache.close(); throw e; }
		space.close(); cache.close(); return out;
	}

	public static function geometry(layout:MillZAxisLayout, swept:Bool = false):Part
		return Part.cylinderSpan(DIAMETER / 2, bottom(layout) + (swept ? layout.lower : 0),
			bottom(layout) + HEIGHT, 0, layout.spindleY);
}
