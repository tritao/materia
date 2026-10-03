package machinekit.welding;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Shielding gas cylinder with its regulator: a 50 litre steel bottle 1.2 m tall.
 *
 * CAD frame: standing on its base at z=0 on the origin, the regulator on top. Connector: `base`
 * (+Y along +Z). Port: the regulated `gas` outlet.
 */
class GasCylinder extends MachineComponent {
	public static inline var DIAMETER:Float = 230;
	public static inline var BOTTLE_HEIGHT:Float = 1100;
	public static inline var HEIGHT:Float = 1200;
	static inline var REGULATOR_DIAMETER:Float = 120;

	/** What the bottle holds, such as "Ar/CO2 82/18". */
	public final mixture:String;

	public function new(mixture:String = "Ar/CO2 82/18") {
		if (mixture == null || mixture.length == 0) throw "Gas cylinder needs a gas mixture";
		super('GAS-CYLINDER-50L-${mixture.split(" ").join("-").split("/").join("-")}',
			'Shielding gas cylinder, 50 L, $mixture, with regulator', "painted steel", true);
		this.mixture = mixture;
		addConnector("base", Mount, Solids.axial(0, 0, 0));
		addPort({name: "gas", kind: Gas, role: Supply, iface: WeldingInterfaces.gas(), required: false});
		declareMass(75, new Vector(0, 0, 0.4 * HEIGHT));
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.union([
			Solids.named(Part.cylinderSpan(DIAMETER / 2, 0, BOTTLE_HEIGHT), "bottle"),
			Solids.named(Part.cylinderSpan(REGULATOR_DIAMETER / 2, BOTTLE_HEIGHT, HEIGHT), "regulator")]);
}
