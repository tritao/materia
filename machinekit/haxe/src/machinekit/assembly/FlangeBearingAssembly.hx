package machinekit.assembly;

import machinekit.component.Bom;
import machinekit.motion.FlangeBearingHousing;
import machinekit.standard.DeepGrooveBearing;
import machinekit.standard.SocketHeadCapScrew;
import materia.assembly.AssemblyRecord.AssemblyFrame;

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
class FlangeBearingAssembly extends MachineAssembly {
	public static inline var ENGAGEMENT_DIAMETERS:Float = 1.5;

	public final housing:FlangeBearingHousing;
	public final bearing:DeepGrooveBearing;
	public final screw:SocketHeadCapScrew;

	public function new(bearing:DeepGrooveBearing) {
		super();
		this.bearing = bearing;
		housing = new FlangeBearingHousing(bearing);
		var diameter = housing.mountScrewPart(10).diameter;
		screw = housing.mountScrewPart(standardScrewLength(housing.depth + ENGAGEMENT_DIAMETERS * diameter));
		addComponent("housing", housing);
		addComponent("bearing", bearing);
		addMate("bearing-seat", "fixed", "housing", "bore", "bearing", "axis");
		exposeConnector("housingBore", "housing", "bore");
		exposeConnector("bearingAxis", "bearing", "axis");
		exposeConnector("bearingFront", "bearing", "front");
		exposeConnector("bearingBack", "bearing", "back");
		for (i in 1...5) {
			var bolt = housing.connector('bolt$i').frame;
			addMemberConnector("housing", 'bolt${i}Head', {x: bolt.x, y: bolt.y, z: bolt.z + housing.depth,
				qx: bolt.qx, qy: bolt.qy, qz: bolt.qz, qw: bolt.qw});
			addComponent('screw$i', screw);
			addMate('screw$i-seat', "fixed", "housing", 'bolt${i}Head', 'screw$i', "head");
			exposeConnector('screw${i}Tip', 'screw$i', "tip");
		}
	}

	/** Shortest ISO 4762 preferred length at least `minimum` millimetres. */
	public static function standardScrewLength(minimum:Float):Float {
		var lengths:Array<Float> = [6.0, 8.0, 10.0, 12.0, 16.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0,
			65.0, 70.0, 80.0, 90.0, 100.0, 110.0, 120.0, 130.0, 140.0, 150.0, 160.0, 180.0, 200.0];
		for (length in lengths)
			if (length >= minimum - 1e-9) return length;
		throw 'No standard socket head cap screw is at least ${Math.ceil(minimum)} mm long';
	}

	public function bom():Bom return billOfMaterials();
}
