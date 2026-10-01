import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.motion.NemaStepper;

/**
 * A mounting plate for a NEMA stepper, as the router makes it from a block: a recess that locates
 * the motor's pilot and, on the motor's bolt pattern, a counterbore for each mounting screw's head
 * over a clearance hole through the plate. Standing on its base (z=0), centred on its origin, with
 * the features in the top face.
 */
class NemaMountPlate extends MachineComponent {
	public final motor:NemaStepper;
	public final width:Float;
	public final depth:Float;
	public final height:Float;
	/** Depth of the pilot recess below the top face. */
	public final recessDepth:Float;
	/** Diameter of the pilot recess: the motor's pilot plus a small clearance. */
	public final recessDiameter:Float;
	public final counterboreDiameter:Float;
	public final counterboreDepth:Float;
	/** Diameter of the screws' clearance holes through the plate. */
	public final holeDiameter:Float;

	public function new(motor:NemaStepper, width:Float, depth:Float, height:Float, recessDepth:Float,
			pilotClearance:Float = 0.2) {
		var screw = motor.mountScrew(10);
		super('MOTOR-PLATE-${motor.designation}-${Dimension.format(width)}x${Dimension.format(depth)}x${Dimension.format(height)}',
			'Motor plate for ${motor.designation}', "aluminium 6061", true);
		this.motor = motor;
		this.width = width;
		this.depth = depth;
		this.height = height;
		this.recessDepth = recessDepth;
		this.recessDiameter = motor.spec.pilotDiameter + pilotClearance;
		this.counterboreDiameter = screw.spec.counterboreDiameter;
		this.counterboreDepth = screw.spec.counterboreDepth;
		this.holeDiameter = screw.spec.clearanceMedium;
		if (!(recessDepth > 0) || !(recessDepth < height) || !(counterboreDepth < height))
			throw "Motor plate features must stay inside the plate";
		for (point in motor.boltPattern())
			if (Math.abs(point.x) + counterboreDiameter / 2 > width / 2 || Math.abs(point.y) + counterboreDiameter / 2 > depth / 2)
				throw "Motor plate is too small for the motor's bolt pattern";
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Part.box(width, depth, height);
		if (detail == Envelope) return body;
		var tools = [Part.cylinderSpan(recessDiameter / 2, height - recessDepth, height + 1)];
		for (point in motor.boltPattern()) {
			tools.push(Part.cylinderSpan(counterboreDiameter / 2, height - counterboreDepth, height + 1, point.x, point.y));
			tools.push(Part.cylinderSpan(holeDiameter / 2, -1, height, point.x, point.y));
		}
		return Solids.cut(body, tools);
	}

	/** Volume the router removes from the block, in cubic millimetres. */
	public function removedVolume():Float {
		var holes = motor.boltPattern().length;
		return Math.PI * (recessDiameter * recessDiameter / 4 * recessDepth +
			holes * counterboreDiameter * counterboreDiameter / 4 * counterboreDepth +
			holes * holeDiameter * holeDiameter / 4 * (height - counterboreDepth));
	}
}
