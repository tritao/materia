package machinekit.milling;

import cadkit.modeling.Part;
import machinekit.component.*;

/** Concept M16 leveling screw with pad and adjusting nut. Casting bottom is Z=0;
 * the nut bears on a steel seat at Z=40. No supplier load rating is assigned.
 */
class MillLevelingFoot extends MachineComponent {
	static var recipe:Null<ComponentType>;
	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.milling.leveling-foot", [], _ -> new MillLevelingFoot());
		return recipe;
	}
	public function new() {
		super("MILL-LEVELING-FOOT-M16-CONCEPT", "Leveling foot concept, supplier rating pending", "steel", false, new ComponentValues());
		addConnector("mount", Mount, Solids.axial(0, 0, 40));
		addConnector("floor", Mount, Solids.axial(0, 0, -50));
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part return Solids.union([
		Part.cylinderSpan(35, -50, -30), Part.cylinderSpan(8, -30, 65), Part.cylinderSpan(14, 40, 50)]);
}
