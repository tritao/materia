package machinekit.assembly;

import cadkit.modeling.AssemblyModel;
import machinekit.component.Bom;
import machinekit.component.MachineComponent;
import machinekit.motion.FlangeBearingHousing;
import machinekit.standard.DeepGrooveBearing;
import machinekit.standard.SocketHeadCapScrew;
import materia.project.AssemblyRecord.AssemblyFrame;

/** Flange bearing housing with its bearing pressed in and four mounting screws, composed from
 * standalone `MachineComponent`s rather than being one itself: `addTo` places the housing at
 * `pose` and mates the bearing and screws onto it, so the whole block moves together.
 *
 * The bearing is centred in the housing (its `axis` on the housing's mid-depth `bore`). The
 * screws pass through the housing from its outer face (z=depth), heads seated there, into the
 * frame the mounting face (z=0) bolts to: their length is the next ISO 4762 standard length of at
 * least the housing depth plus 1.5 screw diameters of thread engagement. `addTo` adds a
 * `bolt<i>Head` connector (the bolt centre on the outer face, +Y along +Z) to the housing
 * instance for each screw seat. The bearing's own `front`/`axis`/`back` connectors (named
 * `'<id>-bearing'`) stay reachable for mating a shaft through it.
 */
class FlangeBearingAssembly {
	public static inline var ENGAGEMENT_DIAMETERS:Float = 1.5;

	public final housing:FlangeBearingHousing;
	public final bearing:DeepGrooveBearing;
	public final screw:SocketHeadCapScrew;

	public function new(bearing:DeepGrooveBearing) {
		this.bearing = bearing;
		housing = new FlangeBearingHousing(bearing);
		var diameter = housing.mountScrewPart(10).diameter;
		screw = housing.mountScrewPart(standardScrewLength(housing.depth + ENGAGEMENT_DIAMETERS * diameter));
	}

	/** Shortest ISO 4762 preferred length at least `minimum` millimetres. */
	public static function standardScrewLength(minimum:Float):Float {
		var lengths:Array<Float> = [6.0, 8.0, 10.0, 12.0, 16.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0,
			65.0, 70.0, 80.0, 90.0, 100.0, 110.0, 120.0, 130.0, 140.0, 150.0, 160.0, 180.0, 200.0];
		for (length in lengths)
			if (length >= minimum - 1e-9) return length;
		throw 'No standard socket head cap screw is at least ${Math.ceil(minimum)} mm long';
	}

	public function addTo(model:AssemblyModel, id:String, ?pose:AssemblyFrame):Void {
		var housingId = '$id-housing';
		housing.addTo(model, housingId, pose);
		bearing.addTo(model, '$id-bearing');
		model.mate('$id-bearing-seat', "fixed", housingId, "bore", '$id-bearing', "axis");
		for (i in 1...5) {
			var bolt = housing.connector('bolt$i').frame;
			model.connector(housingId, 'bolt${i}Head', {x: bolt.x, y: bolt.y, z: bolt.z + housing.depth,
				qx: bolt.qx, qy: bolt.qy, qz: bolt.qz, qw: bolt.qw});
			var screwId = '$id-screw$i';
			screw.addTo(model, screwId);
			model.mate('$screwId-seat', "fixed", housingId, 'bolt${i}Head', screwId, "head");
		}
	}

	public function bom():Bom {
		var result = new Bom();
		result.addComponent(housing);
		result.addComponent(bearing);
		result.addComponent(screw, 4);
		return result;
	}

	/** Instance id to component, for geometry generation by a preview or exporter. */
	public function components():Array<{id:String, component:MachineComponent}> {
		var result:Array<{id:String, component:MachineComponent}> = [
			{id: "housing", component: housing}, {id: "bearing", component: bearing}];
		for (i in 1...5) result.push({id: 'screw$i', component: screw});
		return result;
	}
}
