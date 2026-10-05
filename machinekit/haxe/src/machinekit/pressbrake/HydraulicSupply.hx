package machinekit.pressbrake;

import cadkit.modeling.Part;
import machinekit.fluid.PressureSource;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Assumed pressure-regulated hydraulic service, including its tank return.
 * Pressure is gauge Pa. The envelope declares no pump flow or power rating.
 */
class HydraulicSupply extends MachineComponent implements PressureSource {
	public final pressurePa:Float;
	static var recipe:Null<ComponentType>;
	public function new(pressurePa:Float) {
		if (!Math.isFinite(pressurePa) || pressurePa < 0) throw "Hydraulic supply pressure must be finite and nonnegative";
		super('HYDRAULIC-SUPPLY-$pressurePa', "Assumed regulated hydraulic service", "steel", true);
		this.pressurePa = pressurePa;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addPort({name: "P", kind: Hydraulic, role: Supply, iface: Thread("G1/2"), required: false});
		addPort({name: "T", kind: Hydraulic, role: Consumer, iface: Thread("G1/2"), required: false});
	}
	public function outputPressure(port:String):Float {
		if (port != "P") throw 'Unknown hydraulic pressure outlet "$port"';
		return pressurePa;
	}
	override public function pressureSource():Null<PressureSource> return this;
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.named(Part.box(100, 100, 200), "hydraulic-service");
	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.pressbrake.hydraulic-supply", [
			new machinekit.component.ComponentParameter("pressurePa", Scalar, Number(0), "Pa", 0)
		], v -> new HydraulicSupply(v.number("pressurePa")));
		return recipe;
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function values():ComponentValues return new ComponentValues().setNumber("pressurePa", pressurePa)
		.setToken("material", materialSpec());
}
