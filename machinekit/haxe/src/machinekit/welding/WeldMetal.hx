package machinekit.welding;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/**
 * The weld metal of a weldment. Before any welding there is none, so in the model it is a speck, a 1 mm cube at the
 * workpiece's origin that is mated to the workpiece and weighs what that much steel does: the body that carries the
 * bead. A simulated welder lays its bead on this part at run time, as runtime geometry, so the weld metal is what the
 * torch actually deposited, not a shape drawn into the model; a real weld would add its mass to this part.
 *
 * CAD frame: the cube stands on z=0, centred over its origin. Connector `base` (z=0, to mate to the workpiece).
 */
class WeldMetal extends MachineComponent {
	public static inline var SIZE:Float = 1;

	public function new() {
		super("WELD-METAL-ER70S-6", "Weld metal, ER70S-6 steel wire deposited along the seams", "steel", true);
		addConnector("base", Mount, Solids.axial(0, 0, 0));
		declareMass(7.85e-6, new Vector(0, 0, SIZE / 2));
	}

	/**
	 * The occurrence that carries the bead of `weldment` in `assembly`: the weld metal part mated to the weldment's reference
	 * member. It is found, not named, so the mission follows whatever the CAD calls it and where it is; a weldment with no
	 * weld metal, or with two, is an error.
	 */
	public static function carrierOf(assembly:MachineAssembly, weldment:Weldment):String {
		var carriers:Array<String> = [];
		for (entry in assembly.components()) if (Std.isOfType(entry.component, WeldMetal) && assembly.matedMembers(entry.id).indexOf(weldment.reference) >= 0)
			carriers.push(entry.id);
		if (carriers.length != 1)
			throw 'The weldment of "${weldment.reference}" needs exactly one weld metal part mated to its reference member, found ${carriers.length}';
		return carriers[0];
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.box(SIZE, SIZE, SIZE);
}
