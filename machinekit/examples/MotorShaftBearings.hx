import cadkit.modeling.AssemblyModel;
import cadkit.modeling.Part;
import machinekit.component.Bom;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.motion.NemaStepper;
import machinekit.standard.DeepGrooveBearing;
import machinekit.standard.SocketHeadCapScrew;

/** Plate with a NEMA pilot and bolt cutout. Its back face (z=0) mates to the motor face. */
class MotorPlate extends MachineComponent {
	public final motor:NemaStepper;
	public final thickness:Float;
	public final size:Float;

	public function new(motor:NemaStepper, thickness:Float, margin:Float = 10) {
		super('PLATE-${motor.designation}-${thickness}', 'Motor plate for ${motor.designation}', "aluminium 6061");
		this.motor = motor;
		this.thickness = thickness;
		size = motor.spec.face + 2 * margin;
		addConnector("motor", Mount, Solids.axial(0, 0, 0));
		var i = 1;
		for (point in motor.boltPattern()) addConnector('bolt${i++}', Face, Solids.axial(point.x, point.y, thickness));
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var plate = Part.box(size, size, thickness);
		return detail == Envelope ? plate : Solids.cut(plate, [motor.mountingCutout(thickness)]);
	}
}

/** Plain round shaft along +Z with named bearing seats; a stepped shaft generator comes later. */
class PlainShaft extends MachineComponent {
	public final diameter:Float;
	public final length:Float;

	public function new(diameter:Float, length:Float, seats:Array<{name:String, z:Float}>) {
		super('SHAFT-D${diameter}-L${length}', 'Shaft ${diameter} x ${length}', "steel C45");
		this.diameter = diameter;
		this.length = length;
		addConnector("input", Axis, Solids.axial(0, 0, 0));
		addConnector("output", Shaft, Solids.axial(0, 0, length));
		for (seat in seats) {
			if (seat.z < 0 || seat.z > length) throw 'Seat "${seat.name}" lies outside the shaft';
			addConnector(seat.name, Face, Solids.axial(0, 0, seat.z));
		}
	}

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.cylinder(diameter / 2, 0, length);
}

/** NEMA 17 motor on a plate, four M3 screws, and an output shaft carried by two 608 bearings. */
class MotorShaftBearings {
	public static inline var PLATE_THICKNESS:Float = 6;
	public static inline var SHAFT_LENGTH:Float = 60;

	public final motor = NemaStepper.frame(17);
	public final plate:MotorPlate;
	public final screw:SocketHeadCapScrew;
	public final bearing = DeepGrooveBearing.metric("608");
	public final shaft:PlainShaft;

	public function new() {
		plate = new MotorPlate(motor, PLATE_THICKNESS);
		// Engage at least 1.3 d without bottoming out in the tapped hole.
		var engagement = motor.spec.mountHoleDepth - 0.5;
		screw = motor.mountScrew(Math.ffloor(PLATE_THICKNESS + engagement));
		if (screw.length - PLATE_THICKNESS < 1.3 * screw.diameter) throw "Mount screw engagement is too short";
		shaft = new PlainShaft(bearing.bore, SHAFT_LENGTH, [
			{name: "bearingA", z: 10},
			{name: "bearingB", z: SHAFT_LENGTH - 10 - bearing.width},
		]);
	}

	public function assembly():AssemblyModel {
		var model = new AssemblyModel();
		motor.addTo(model, "motor");
		plate.addTo(model, "plate");
		model.mate("plate-mount", "fixed", "motor", "mountFace", "plate", "motor");
		for (i in 1...5) {
			var id = 'screw$i';
			screw.addTo(model, id);
			model.mate('$id-seat', "fixed", "plate", 'bolt$i', id, "head");
		}
		shaft.addTo(model, "shaft");
		model.mate("coupling", "continuous", "motor", "shaftTip", "shaft", "input");
		for (seat in ["bearingA", "bearingB"]) {
			bearing.addTo(model, seat);
			model.mate('$seat-seat', "fixed", "shaft", seat, seat, "front");
		}
		return model;
	}

	public function bom():Bom {
		var result = new Bom();
		result.addComponent(motor);
		result.addComponent(plate);
		result.addComponent(screw, 4);
		result.addComponent(shaft);
		result.addComponent(bearing, 2);
		return result;
	}

	/** Instance id to component, for geometry generation by a preview or exporter. */
	public function components():Array<{id:String, component:MachineComponent}> {
		var result:Array<{id:String, component:MachineComponent}> = [
			{id: "motor", component: motor}, {id: "plate", component: plate}, {id: "shaft", component: shaft},
			{id: "bearingA", component: bearing}, {id: "bearingB", component: bearing}];
		for (i in 1...5) result.push({id: 'screw$i', component: screw});
		return result;
	}
}
