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
 * Current is a DC output rating shared by all outlets. A regenerative rating
 * represents an integrated shunt and braking resistor, with an assumed envelope.
 */
class PowerSupply extends MachineComponent implements ElectricalSource {
	static var recipe:Null<ComponentType>;
	public final voltage:machinekit.power.DcVoltage;
	public final current:machinekit.power.DcCurrent;
	public final regenerativePower:Float;
	public final outlets:Int;
	public final width:Float;
	public final height:Float;
	public final depth:Float;

	public function new(voltage:machinekit.power.DcVoltage, current:machinekit.power.DcCurrent, outlets:Int = 1, width:Float = 120, height:Float = 60, depth:Float = 30, regenerativePower:Float = 0) {
		if (outlets < 1 || outlets > 64) throw "A power supply needs from 1 to 64 output ports";
		for (value in [voltage.volts(), current.amps(), width, height, depth])
			if (!(value > 0) || !Math.isFinite(value)) throw "A power supply needs finite positive ratings and dimensions";
		if (!(regenerativePower >= 0) || !Math.isFinite(regenerativePower)) throw "Regenerative capacity must be finite and nonnegative";
		super('DC-SUPPLY-${Dimension.format(voltage.volts())}V-${Dimension.format(current.amps())}A-' +
			'${Dimension.format(width)}x${Dimension.format(height)}x${Dimension.format(depth)}-$outlets-OUT-${Dimension.format(regenerativePower)}W-SHUNT',
			'DC supply, ${Dimension.format(voltage.volts())} V, ${Dimension.format(current.amps())} A', "aluminium 6061");
		this.voltage = voltage;
		this.current = current;
		this.regenerativePower = regenerativePower;
		this.outlets = outlets;
		this.width = width;
		this.height = height;
		this.depth = depth;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		for (index in 1...outlets + 1)
			addPort({name: 'power$index', kind: ElectricalPower, role: Supply, iface: Unspecified, required: false});
	}

	override public function electricalSource():Null<ElectricalSource> return this;

	public function outputRating(name:String):ElectricalSupplyRating {
		var output = port(name);
		return {voltage: voltage, current: current, regenerativePower: regenerativePower};
	}

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.motion.power-supply", [
			new ComponentParameter("voltage", ComponentParameterType.Scalar, ComponentValue.Number(24), "V", 0.001),
			new ComponentParameter("current", ComponentParameterType.Scalar, ComponentValue.Number(10), "A DC", 0.001),
			ComponentRecipeSupport.count("outlets", 1),
			ComponentRecipeSupport.length("width", 120), ComponentRecipeSupport.length("height", 60),
			ComponentRecipeSupport.length("depth", 30),
			ComponentRecipeSupport.scalar("regenerativePower", 0, 0)
		], v -> new PowerSupply(new machinekit.power.DcVoltage(v.number("voltage")), new machinekit.power.DcCurrent(v.number("current")), v.integer("outlets"), v.number("width"),
			v.number("height"), v.number("depth"), v.number("regenerativePower")));
		return recipe;
	}

	override public function componentType():Null<ComponentType> return Std.isExactType(this, PowerSupply) ? recipeType() : null;
	override public function values():ComponentValues return new ComponentValues()
		.setNumber("voltage", voltage.volts()).setNumber("current", current.amps()).setInteger("outlets", outlets).setNumber("width", width)
		.setNumber("height", height).setNumber("depth", depth).setNumber("regenerativePower", regenerativePower).setToken("material", materialSpec());
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.named(Part.box(width, height, depth), "body");
}
