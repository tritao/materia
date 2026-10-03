package machinekit.pneumatic;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Double-acting cylinder body. Lengths are mm, pressure is Pa and forces are N.
 * Catalogue bore/rod pairs are reference sizes; envelopes and flow-limited speeds are assumed.
 * The rod belongs to the guided moving member, so the cylinder adds no second mechanical joint.
 */
class PneumaticCylinder extends MachineComponent {
	public final family:String;
	public final bore:Float;
	public final rod:Float;
	public final stroke:Float;
	public final ratedSpeed:Float;
	public final capLength:Float;
	public final bodyLength:Float;
	public final bodyWidth:Float;
	public final stemLength:Float;
	static var recipe:Null<ComponentType>;

	public function new(family:String, bore:Float, stroke:Float, ratedSpeed:Float) {
		var bores = family == "ISO6432" ? [8, 10, 12, 16, 20, 25] :
			family == "ISO15552" ? [32, 40, 50, 63, 80, 100, 125] : null;
		var rods = family == "ISO6432" ? [4, 4, 6, 6, 8, 10] : [12, 16, 20, 20, 25, 25, 32];
		if (bores == null) throw 'Unknown cylinder family "$family"';
		var index = -1;
		for (i in 0...bores.length) if (bores[i] == bore) index = i;
		if (index < 0 || !(stroke > 0) || !(ratedSpeed > 0) || !Math.isFinite(stroke + ratedSpeed))
			throw "Cylinder needs a catalogue bore, finite positive stroke and speed";
		super('$family-${bore}x$stroke', "Reference double-acting pneumatic cylinder", "aluminium 6061", true);
		this.family = family; this.bore = bore; this.rod = rods[index];
		this.stroke = stroke; this.ratedSpeed = ratedSpeed;
		capLength = Math.max(8, bore * 0.25);
		bodyLength = stroke + 2 * capLength;
		bodyWidth = bore + 12;
		stemLength = rod * 1.5;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addConnector("extension", Axis, Solids.axial(0, 0, bodyLength + stemLength));
		addConnector("portA", Face, materia.assembly.AssemblyFrames.alongY(bodyWidth / 2, 0, capLength, 1, 0, 0));
		addConnector("portB", Face, materia.assembly.AssemblyFrames.alongY(bodyWidth / 2, 0, bodyLength - capLength, 1, 0, 0));
		addPort({name: "A", kind: Pneumatic, role: Consumer, iface: PushIn(6), required: true, connector: "portA"});
		addPort({name: "B", kind: Pneumatic, role: Consumer, iface: PushIn(6), required: true, connector: "portB"});
	}

	public function pistonArea():Float return Math.PI * bore * bore / 4e6;
	public function annularArea():Float return Math.PI * (bore * bore - rod * rod) / 4e6;
	public function extendForce(pressurePa:Float):Float return checkedPressure(pressurePa) * pistonArea();
	public function retractForce(pressurePa:Float):Float return checkedPressure(pressurePa) * annularArea();
	static function checkedPressure(value:Float):Float {
		if (!(value >= 0) || !Math.isFinite(value)) throw "Cylinder pressure must be finite and nonnegative";
		return value;
	}
	public function movingRod():PneumaticCylinderRod return new PneumaticCylinderRod(rod, stroke + stemLength);
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.named(Solids.building([], parts -> {
			parts.push(Solids.named(family == "ISO15552" ? Part.box(bodyWidth, bodyWidth, bodyLength) :
				Part.cylinderSpan(bodyWidth / 2, 0, bodyLength), "barrel"));
			parts.push(Solids.named(Part.cylinderSpan(bore / 2, capLength, bodyLength - capLength), "pistonBore"));
			parts.push(Solids.named(Part.cylinderSpan(rod / 2 + 0.1, bodyLength - capLength, bodyLength + 0.1), "rodBearing"));
			return Solids.cut(parts[0], [parts[1], parts[2]]);
		}), "body");
	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.pneumatic.cylinder", [
			ComponentRecipeSupport.choice("family", ["ISO6432", "ISO15552"], "ISO6432"),
			ComponentRecipeSupport.length("bore", 25), ComponentRecipeSupport.length("stroke", 100),
			new machinekit.component.ComponentParameter("ratedSpeed", Scalar, Number(300), "mm/s", 0)
		], v -> new PneumaticCylinder(v.token("family"), v.number("bore"), v.number("stroke"), v.number("ratedSpeed")));
		return recipe;
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function values():ComponentValues return new ComponentValues().setToken("family", family)
		.setNumber("bore", bore).setNumber("stroke", stroke).setNumber("ratedSpeed", ratedSpeed)
		.setToken("material", materialSpec());
}
