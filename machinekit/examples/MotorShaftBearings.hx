import cadkit.modeling.AssemblyModel;
import cadkit.modeling.Part;
import machinekit.component.Bom;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.motion.NemaStepper;
import machinekit.motion.SteppedShaft;
import machinekit.standard.DeepGrooveBearing;
import machinekit.standard.ParallelKey;
import machinekit.standard.RetainingRing;
import machinekit.standard.SocketHeadCapScrew;

/** Plate with a NEMA pilot and bolt cutout. Its back face (z=0) mates to the motor face. */
class MotorPlate extends MachineComponent {
	public final motor:NemaStepper;
	public final thickness:Float;
	public final size:Float;

	public function new(motor:NemaStepper, thickness:Float, margin:Float = 10) {
		super('PLATE-${motor.designation}-${Dimension.format(thickness)}', 'Motor plate for ${motor.designation}', "aluminium 6061");
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

/** NEMA 17 motor on a plate, four M3 screws, and an output shaft carried by two 608 bearings.
 * The shaft steps down past the outboard bearing to carry a retaining ring and an output key,
 * exercising `SteppedShaft`'s keyway and groove machining.
 */
class MotorShaftBearings {
	public static inline var PLATE_THICKNESS:Float = 6;
	public static inline var SHAFT_LENGTH:Float = 60;
	public static inline var COLLAR_LENGTH:Float = 51.5;

	public final motor = NemaStepper.frame(17);
	public final plate:MotorPlate;
	public final screw:SocketHeadCapScrew;
	public final bearing = DeepGrooveBearing.metric("608");
	public final key = ParallelKey.forShaft(6, 6);
	public final ring = RetainingRing.forShaft(8);
	public final shaft:SteppedShaft;

	public function new() {
		plate = new MotorPlate(motor, PLATE_THICKNESS);
		// Engage at least 1.3 d without bottoming out in the tapped hole.
		var engagement = motor.spec.mountHoleDepth - 0.5;
		screw = motor.mountScrew(Math.ffloor(PLATE_THICKNESS + engagement));
		if (screw.length - PLATE_THICKNESS < 1.3 * screw.diameter) throw "Mount screw engagement is too short";
		shaft = new SteppedShaft(
			[{diameter: bearing.bore, length: COLLAR_LENGTH}, {diameter: 6, length: SHAFT_LENGTH - COLLAR_LENGTH}],
			[{name: "bearingA", z: 10}, {name: "bearingB", z: SHAFT_LENGTH - 10 - bearing.width}],
			[{name: "outputKey", z0: 52, key: key}],
			[{name: "bearingBRing", z0: 50, width: ring.thickness, diameter: ring.spec.grooveDiameter}]
		);
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
		key.addTo(model, "key");
		model.mate("key-seat", "fixed", "shaft", "outputKey", "key", "seat");
		ring.addTo(model, "ring");
		model.mate("ring-seat", "fixed", "shaft", "bearingBRing", "ring", "seat");
		return model;
	}

	public function bom():Bom {
		var result = new Bom();
		result.addComponent(motor);
		result.addComponent(plate);
		result.addComponent(screw, 4);
		result.addComponent(shaft);
		result.addComponent(bearing, 2);
		result.addComponent(key);
		result.addComponent(ring);
		return result;
	}

	/** Instance id to component, for geometry generation by a preview or exporter. */
	public function components():Array<{id:String, component:MachineComponent}> {
		var result:Array<{id:String, component:MachineComponent}> = [
			{id: "motor", component: motor}, {id: "plate", component: plate}, {id: "shaft", component: shaft},
			{id: "bearingA", component: bearing}, {id: "bearingB", component: bearing}, {id: "key", component: key},
			{id: "ring", component: ring}];
		for (i in 1...5) result.push({id: 'screw$i', component: screw});
		return result;
	}
}
