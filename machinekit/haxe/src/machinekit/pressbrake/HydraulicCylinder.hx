package machinekit.pressbrake;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Assumed double-acting hydraulic cylinder body. Its guided ram owns the moving rod.
 * Dimensions are mm; pressure is Pa and force is N. This part declares no actuator yet.
 */
class HydraulicCylinder extends MachineComponent {
	public final bore:Float;
	public final rod:Float;
	public final stroke:Float;
	public final wall:Float;
	public final capLength:Float;
	public final stemLength:Float;
	public var bodyLength(get, never):Float;
	public var outerDiameter(get, never):Float;

	public function new(bore:Float, rod:Float, stroke:Float, wall:Float = 20, capLength:Float = 40,
			stemLength:Float = 50) {
		if (!Math.isFinite(bore) || !Math.isFinite(rod) || !Math.isFinite(stroke)
			|| !Math.isFinite(wall) || !Math.isFinite(capLength) || !Math.isFinite(stemLength)
			|| !(bore > rod && rod > 0 && stroke > 0 && wall > 0 && capLength > 0 && stemLength > 0))
			throw "Hydraulic cylinder needs positive finite dimensions and a rod smaller than its bore";
		super('HYDRAULIC-${Dimension.format(bore)}-${Dimension.format(rod)}x${Dimension.format(stroke)}',
			"Assumed double-acting press-brake cylinder body", "steel", true);
		this.bore = bore; this.rod = rod; this.stroke = stroke;
		this.wall = wall; this.capLength = capLength; this.stemLength = stemLength;
		if (!Math.isFinite(bodyLength + outerDiameter + stemLength)) throw "Hydraulic cylinder dimensions overflow";
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addConnector("extension", Axis, Solids.axial(0, 0, bodyLength + stemLength));
	}

	function get_bodyLength():Float return stroke + 2 * capLength;
	function get_outerDiameter():Float return bore + 2 * wall;
	public function pistonArea():Float return Math.PI * bore * bore / 4e6;
	public function annularArea():Float return Math.PI * (bore * bore - rod * rod) / 4e6;
	public function extendForce(pressurePa:Float):Float return checkedPressure(pressurePa) * pistonArea();
	public function retractForce(pressurePa:Float):Float return checkedPressure(pressurePa) * annularArea();
	static function checkedPressure(value:Float):Float {
		if (!Math.isFinite(value) || value < 0) throw "Hydraulic pressure must be finite and nonnegative";
		return value;
	}
	public function movingRod():HydraulicCylinderRod return new HydraulicCylinderRod(rod, stroke + capLength + stemLength);
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.building([], parts -> {
			parts.push(Solids.named(Part.cylinderSpan(outerDiameter / 2, 0, bodyLength), "barrel"));
			parts.push(Solids.named(Part.cylinderSpan(bore / 2, capLength, bodyLength - capLength), "piston-bore"));
			parts.push(Solids.named(Part.cylinderSpan(rod / 2, bodyLength - capLength, bodyLength + 1), "rod-bearing"));
			return Solids.cut(parts[0], [parts[1], parts[2]]);
		});
}
