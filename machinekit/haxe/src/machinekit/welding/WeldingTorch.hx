package machinekit.welding;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import materia.assembly.AssemblyFrames;

/** Robot MIG torch: a breakaway mount, a straight handle, a swan neck bent 22 or 45 degrees, a gas
 * nozzle and a contact tip, 360 mm from mount to wire tip. The component is the mount and the handle; the swan
 * neck with its elbow is `WeldingTorchNeck`, mated to the `neck` connector at the bend, and the nozzle with the contact tip
 * is `WeldingTorchNozzle` on the neck's end, so that each is a convex body of its own for collision (a hull around the whole
 * bent torch would fill in the crook of the neck, and around the neck and nozzle together the step between their radii).
 *
 * CAD frame: the mount face at z=0 and the handle along +Z; the neck bends toward +X, so the
 * nozzle points along (sin bend, 0, cos bend). Connectors: `robot` (the mount face, mate it to the
 * tool plate's `tool`, +Y along +Z like every tool mount) and `tcp` (the wire tip at the nominal
 * stickout past the contact tip, with +Z along the wire out of the torch). Ports: the weld
 * `power` cable, `gas`, the `wire` liner and the `control` signal that starts the arc, all inlets
 * supplied through the feeder.
 *
 * The breakaway mount is the torch's own base: a spring-loaded collar that lets go in a crash.
 * Geometry is solid, square-shouldered stock, so the torch is its own collision shape.
 */
class WeldingTorch extends MachineComponent {
	/** The breakaway collar, the straight handle, the neck, the nozzle and the tip's reach beyond the
	 * nozzle, along their axes. The bend sits at the end of the handle. */
	public static inline var MOUNT_LENGTH:Float = 45;
	public static inline var BEND_Z:Float = 165;
	public static inline var NECK_LENGTH:Float = 120;
	public static inline var NOZZLE_LENGTH:Float = 55;
	public static inline var TIP_PROTRUSION:Float = 3;
	/** Wire past the contact tip when the arc is lit. */
	public static inline var STICKOUT:Float = 15;
	static inline var MOUNT_RADIUS:Float = 30;
	static inline var HANDLE_RADIUS:Float = 14;
	static inline var NECK_RADIUS:Float = 8;
	static inline var NOZZLE_RADIUS:Float = 10;
	static inline var TIP_RADIUS:Float = 4;

	/** The torch's mass, kg, and the neck's part of it. */
	public static inline var MASS:Float = 1.6;
	public static inline var NECK_MASS:Float = 0.45;
	public static inline var NOZZLE_MASS:Float = 0.25;

	/** Neck bend from the handle's axis, in degrees: 22 or 45. */
	public final bendDegrees:Float;

	public function new(bendDegrees:Float = 45) {
		if (bendDegrees != 22 && bendDegrees != 45) throw "Welding torch neck bends 22 or 45 degrees";
		super('WELD-TORCH-MIG-${Dimension.format(bendDegrees)}DEG',
			'Robot MIG torch, ${Dimension.format(bendDegrees)} degree swan neck, breakaway mount', "brass", true);
		this.bendDegrees = bendDegrees;
		addConnector("robot", Mount, Solids.axial(0, 0, 0));
		var bend = bendRadians(), reach = tipEnd() + STICKOUT;
		// The neck's mount: at the bend, with the joint axis (+Y) along the nozzle, as a part's axial connector has it.
		addConnector("neck", Mount, AssemblyFrames.alongY(0, 0, BEND_Z, Math.sin(bend), 0, Math.cos(bend)));
		addConnector("tcp", Mount, AssemblyFrames.compose(
			AssemblyFrames.translation(reach * Math.sin(bend), 0, BEND_Z + reach * Math.cos(bend)),
			AssemblyFrames.turnY(bend)));
		addPort({name: "power", kind: ElectricalPower, role: Consumer, iface: WeldingInterfaces.weldCable(), required: true});
		addPort({name: "gas", kind: Gas, role: Consumer, iface: WeldingInterfaces.gas(), required: true});
		addPort({name: "wire", kind: Wire, role: Consumer, iface: WeldingInterfaces.wireLiner(), required: true});
		addPort({name: "control", kind: Signal, role: Consumer, iface: WeldingInterfaces.control(), required: true});
		addCapability(ArcTorch("tcp", "control", STICKOUT));
		declareMass(MASS - NECK_MASS - NOZZLE_MASS, new Vector(0, 0, 0.5 * BEND_Z));
	}

	/** Distance from the bend to the contact tip's end, along the nozzle axis. */
	static function tipEnd():Float return NECK_LENGTH + NOZZLE_LENGTH + TIP_PROTRUSION;

	public function bendRadians():Float return bendDegrees * Math.PI / 180;

	/** Direction the wire leaves the torch in, in the torch's frame. */
	public function wireDirection():Vector return new Vector(Math.sin(bendRadians()), 0, Math.cos(bendRadians()));

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		return Solids.union([
			Solids.named(Part.cylinderSpan(MOUNT_RADIUS, 0, MOUNT_LENGTH), "breakaway"),
			Solids.named(Part.cylinderSpan(HANDLE_RADIUS, MOUNT_LENGTH, BEND_Z), "handle")
		]);
	}

	/** The radii of the neck's parts, for `WeldingTorchNeck`. */
	public static inline var NECK_R:Float = NECK_RADIUS;
	public static inline var NOZZLE_R:Float = NOZZLE_RADIUS;
	public static inline var TIP_R:Float = TIP_RADIUS;
}
