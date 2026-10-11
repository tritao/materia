import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.motion.NemaStepper;

/**
 * A printable L bracket for a NEMA stepper, in the orientation it prints in: the motor plate lies on the bed (the
 * pilot and the bolt pattern are vertical holes through it) and a wall rises from its rear edge, drilled for two
 * screws. Centred on its origin in x and y, standing on z = 0. The demo print of the printer example.
 */
class NemaMountBracket extends MachineComponent {
	public final motor:NemaStepper;
	public final plate:Float;
	public final wallHeight:Float;
	public final wallScrewDiameter:Float;

	public function new(motor:NemaStepper, plate:Float = 4.0, wallHeight:Float = 30.0, wallScrewDiameter:Float = 4.4) {
		super('MOTOR-BRACKET-${motor.designation}', 'Printable bracket for ${motor.designation}', "plastic", true);
		this.motor = motor;
		this.plate = plate;
		this.wallHeight = wallHeight;
		this.wallScrewDiameter = wallScrewDiameter;
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var face = motor.spec.face;
		var base = Solids.named(Part.box(face, face, plate), "plate");
		var wall = Solids.named(Part.box(face, plate, wallHeight).translated(new Vector(0, face / 2 - plate / 2, 0)), "wall");
		var body = Solids.union([base, wall]);
		if (detail == Envelope) return body;
		var tools = [Part.cylinderSpan(motor.spec.pilotDiameter / 2 + 0.1, -1, plate + 1)];
		for (point in motor.boltPattern()) tools.push(Part.cylinderSpan(1.7, -1, plate + 1, point.x, point.y));
		for (side in [-1, 1])
			tools.push(Part.cylinderAlongY(wallScrewDiameter / 2, face / 2 - plate - 1, face / 2 + 1, side * face / 4, wallHeight / 2));
		return Solids.cut(body, tools);
	}
}
