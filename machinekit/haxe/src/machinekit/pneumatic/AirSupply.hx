package machinekit.pneumatic;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Regulated air service. Pressure is gauge pressure in Pa; the FRL envelope is assumed. */
class AirSupply extends MachineComponent implements PressureSource {
	public final pressurePa:Float;
	static var recipe:Null<ComponentType>;
	public function new(pressurePa:Float = 600000) {
		if (!(pressurePa > 0) || !Math.isFinite(pressurePa)) throw "Air supply needs finite positive pressure";
		super('AIR-FRL-$pressurePa', "Reference regulated air supply", "aluminium 6061", true);
		this.pressurePa = pressurePa;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addPort({name: "air", kind: Pneumatic, role: Supply, iface: PushIn(6), required: false});
	}
	public function outputPressure(port:String):Float {
		if (port != "air") throw 'Unknown regulated-air outlet "$port"';
		return pressurePa;
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.named(Part.box(60, 40, 100), "frl");
	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.pneumatic.air-supply", [
			new machinekit.component.ComponentParameter("pressurePa", Scalar, Number(600000), "Pa", 0)
		], v -> new AirSupply(v.number("pressurePa")));
		return recipe;
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function values():ComponentValues return new ComponentValues().setNumber("pressurePa", pressurePa)
		.setToken("material", materialSpec());
}
