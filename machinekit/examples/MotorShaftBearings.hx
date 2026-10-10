import cadkit.modeling.AssemblyModel;
import cadkit.modeling.Part;
import cadkit.parametric.InstanceElement;
import machinekit.document.MachineKitDocumentAssembly;
import machinekit.component.Bom;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.motion.NemaStepper;
import machinekit.assembly.NemaMountPlate;
import machinekit.motion.SteppedShaft;
import machinekit.standard.DeepGrooveBearing;
import machinekit.standard.ParallelKey;
import machinekit.standard.RetainingRing;
import machinekit.standard.SocketHeadCapScrew;

/** NEMA 17 motor on a plate, four M3 screws, and an output shaft carried by two 608 bearings.
 * The shaft steps down past the outboard bearing to carry a retaining ring and an output key,
 * exercising `SteppedShaft`'s keyway and groove machining.
 */
class MotorShaftBearings {
	public static inline var PLATE_THICKNESS:Float = 6;
	public static inline var SHAFT_LENGTH:Float = 60;
	public static inline var COLLAR_LENGTH:Float = 51.5;

	public final motor = NemaStepper.frame(17);
	public final plate:NemaMountPlate;
	public final screw:SocketHeadCapScrew;
	public final bearing = DeepGrooveBearing.metric("608");
	public final key = ParallelKey.forShaft(6, 6);
	public final ring = RetainingRing.forShaft(8);
	public final shaft:SteppedShaft;

	public function new() {
		plate = new NemaMountPlate(motor, PLATE_THICKNESS);
		screw = plate.mountScrew();
		shaft = new SteppedShaft(
			[{diameter: bearing.bore, length: COLLAR_LENGTH}, {diameter: 6, length: SHAFT_LENGTH - COLLAR_LENGTH}],
			[{name: "bearingA", z: 10}, {name: "bearingB", z: SHAFT_LENGTH - 10 - bearing.width}],
			[{name: "outputKey", z0: 52, key: key}],
			[{name: "bearingBRing", z0: 50, width: ring.thickness, diameter: ring.spec.grooveDiameter}]
		);
	}

	public function assembly(?instances:Map<String, InstanceElement>):AssemblyModel {
		var model = new AssemblyModel();
		if (instances == null) instances = [];
		add(model, "motor", motor, instances);
		add(model, "plate", plate, instances);
		model.mate("plate-mount", "fixed", "motor", "mountFace", "plate", "motor");
		for (i in 1...5) {
			var id = 'screw$i';
			add(model, id, screw, instances);
			model.mate('$id-seat', "fixed", "plate", 'bolt$i', id, "head");
		}
		add(model, "shaft", shaft, instances);
		model.mate("coupling", "continuous", "motor", "shaftTip", "shaft", "input");
		for (seat in ["bearingA", "bearingB"]) {
			add(model, seat, bearing, instances);
			model.mate('$seat-seat', "fixed", "shaft", seat, seat, "front");
		}
		add(model, "key", key, instances);
		model.mate("key-seat", "fixed", "shaft", "outputKey", "key", "seat");
		add(model, "ring", ring, instances);
		model.mate("ring-seat", "fixed", "shaft", "bearingBRing", "ring", "seat");
		return model;
	}

	static function add(model:AssemblyModel, id:String, component:MachineComponent,
			instances:Map<String, InstanceElement>):Void {
		var instance = instances.get(id);
		if (instance == null) component.addTo(model, id);
		else MachineKitDocumentAssembly.add(model, id, instance);
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
