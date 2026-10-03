package machinekit.motion;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentParameter;
import machinekit.component.ComponentParameterType;
import machinekit.component.ComponentValue;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** A DC supply with stated output ratings and an assumed rectangular envelope.
 * Current is a DC output rating; aggregate electrical loading is not modelled here.
 */
class PowerSupply extends MachineComponent implements ElectricalSource {
	static var recipe:Null<ComponentType>;
	public final voltage:Float;
	public final current:Float;
	public final outlets:Int;
	public final width:Float;
	public final height:Float;
	public final depth:Float;

	public function new(voltage:Float, current:Float, outlets:Int = 1, width:Float = 120, height:Float = 60, depth:Float = 30) {
		if (outlets < 1 || outlets > 64) throw "A power supply needs from 1 to 64 output ports";
		for (value in [voltage, current, width, height, depth])
			if (!(value > 0) || !Math.isFinite(value)) throw "A power supply needs finite positive ratings and dimensions";
		super('DC-SUPPLY-${Dimension.format(voltage)}V-${Dimension.format(current)}A-' +
			'${Dimension.format(width)}x${Dimension.format(height)}x${Dimension.format(depth)}-$outlets-OUT',
			'DC supply, ${Dimension.format(voltage)} V, ${Dimension.format(current)} A', "aluminium 6061");
		this.voltage = voltage;
		this.current = current;
		this.outlets = outlets;
		this.width = width;
		this.height = height;
		this.depth = depth;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		for (index in 1...outlets + 1)
			addPort({name: 'power$index', kind: ElectricalPower, role: Supply, iface: Unspecified, required: false});
	}

	public function outputVoltage(name:String):Float {
		var output = port(name);
		return voltage;
	}

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.motion.power-supply", [
			new ComponentParameter("voltage", ComponentParameterType.Scalar, ComponentValue.Number(24), "V", 0.001),
			new ComponentParameter("current", ComponentParameterType.Scalar, ComponentValue.Number(10), "A DC", 0.001),
			ComponentRecipeSupport.count("outlets", 1),
			ComponentRecipeSupport.length("width", 120), ComponentRecipeSupport.length("height", 60),
			ComponentRecipeSupport.length("depth", 30)
		], v -> new PowerSupply(v.number("voltage"), v.number("current"), v.integer("outlets"), v.number("width"),
			v.number("height"), v.number("depth")));
		return recipe;
	}

	override public function componentType():Null<ComponentType> return Std.isExactType(this, PowerSupply) ? recipeType() : null;
	override public function values():ComponentValues return new ComponentValues()
		.setNumber("voltage", voltage).setNumber("current", current).setInteger("outlets", outlets).setNumber("width", width)
		.setNumber("height", height).setNumber("depth", depth).setToken("material", materialSpec());
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.named(Part.box(width, height, depth), "body");
}
