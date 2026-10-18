package machinekit.milling;
import cadkit.modeling.Part;
import machinekit.component.*;

/** X return-drive motor plate, across X, shaft along Y, bottom Z=0.
 * Fits a machined saddle recess; generic 30 mm square motor pattern is assumed.
 */
class MillBeltMotorPlate extends MachineComponent {
	static var recipe:Null<ComponentType>;
	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.milling.belt-motor-plate", [], _ -> new MillBeltMotorPlate());
		return recipe;
	}
	public function new() {
		super("MILL-X-RETURN-MOTOR-PLATE", "Recessed X motor plate, vendor pattern pending", "steel", false, new ComponentValues());
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var holes = [Part.cylinderAlongY(8.5, -7, 7, 0, 40)];
		for (x in [-15.0, 15.0]) for (z in [25.0, 55.0]) holes.push(Part.cylinderAlongY(2.75, -7, 7, x, z));
		for (x in [-32.0, 32.0]) for (z in [10.0, 70.0]) holes.push(Part.cylinderAlongY(3.3, -7, 7, x, z));
		return Solids.cut(Part.box(84, 12, 80), holes);
	}
}
