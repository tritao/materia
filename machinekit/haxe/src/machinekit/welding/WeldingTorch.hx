package machinekit.welding;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import materia.assembly.AssemblyFrames;

/** Robot MIG torch: a breakaway mount, a straight handle, a swan neck bent 22 or 45 degrees, a gas
 * nozzle and a contact tip, 360 mm from mount to wire tip.
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
	static inline var NECK_RADIUS:Float = 9;
	static inline var NOZZLE_RADIUS:Float = 13;
	static inline var TIP_RADIUS:Float = 4;

	/** Neck bend from the handle's axis, in degrees: 22 or 45. */
	public final bendDegrees:Float;

	public function new(bendDegrees:Float = 45) {
		if (bendDegrees != 22 && bendDegrees != 45) throw "Welding torch neck bends 22 or 45 degrees";
		super('WELD-TORCH-MIG-${Dimension.format(bendDegrees)}DEG',
			'Robot MIG torch, ${Dimension.format(bendDegrees)} degree swan neck, breakaway mount', "brass", true);
		this.bendDegrees = bendDegrees;
		addConnector("robot", Mount, Solids.axial(0, 0, 0));
		var bend = bendRadians(), reach = tipEnd() + STICKOUT;
		addConnector("tcp", Mount, AssemblyFrames.compose(
			AssemblyFrames.translation(reach * Math.sin(bend), 0, BEND_Z + reach * Math.cos(bend)),
			AssemblyFrames.turnY(bend)));
		addPort({name: "power", kind: ElectricalPower, role: Consumer, iface: WeldingInterfaces.weldCable(), required: true});
		addPort({name: "gas", kind: Gas, role: Consumer, iface: WeldingInterfaces.gas(), required: true});
		addPort({name: "wire", kind: Wire, role: Consumer, iface: WeldingInterfaces.wireLiner(), required: true});
		addPort({name: "control", kind: Signal, role: Consumer, iface: WeldingInterfaces.control(), required: true});
		addCapability(ArcTorch("tcp", "control"));
		declareMass(1.6, new Vector(0.05 * Math.sin(bend) * reach, 0, 0.5 * BEND_Z));
	}

	/** Distance from the bend to the contact tip's end, along the nozzle axis. */
	static function tipEnd():Float return NECK_LENGTH + NOZZLE_LENGTH + TIP_PROTRUSION;

	public function bendRadians():Float return bendDegrees * Math.PI / 180;

	/** Direction the wire leaves the torch in, in the torch's frame. */
	public function wireDirection():Vector return new Vector(Math.sin(bendRadians()), 0, Math.cos(bendRadians()));

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var bend = new Vector(0, 0, BEND_Z), axis = wireDirection();
		function along(distance:Float):Vector return bend.add(axis.scale(distance));
		var parts = [
			Solids.named(Part.cylinderSpan(MOUNT_RADIUS, 0, MOUNT_LENGTH), "breakaway"),
			Solids.named(Part.cylinderSpan(HANDLE_RADIUS, MOUNT_LENGTH, BEND_Z), "handle"),
			Solids.named(Part.sphere(NECK_RADIUS).translated(bend), "elbow"),
			Solids.named(Part.cylinderAlong(NECK_RADIUS, bend, axis, NECK_LENGTH), "neck"),
			Solids.named(Part.cylinderAlong(NOZZLE_RADIUS, along(NECK_LENGTH), axis, NOZZLE_LENGTH), "nozzle"),
			Solids.named(Part.cylinderAlong(TIP_RADIUS, along(NECK_LENGTH + NOZZLE_LENGTH), axis, TIP_PROTRUSION), "tip")
		];
		return Solids.union(parts);
	}
}
