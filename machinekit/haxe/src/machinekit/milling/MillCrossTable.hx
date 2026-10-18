package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** T-slot table, with a recessed seat for the central X nut bracket. */
class MillCrossTable extends MachineComponent {
	static var recipe:Null<ComponentType>;

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.milling.cross-table", [], _ -> new MillCrossTable());
		return recipe;
	}

	public function new() {
		super("MILL-CROSS-TABLE", "400 x 200 machined T-slot table", "cast iron", false, new ComponentValues());
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addConnector("top", Face, Solids.axial(0, 0, 40));
	}

	override public function componentType():Null<ComponentType> return recipeType();

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var tools = [Part.box(42, 86, 13).translated(new Vector(20, 0, -1))];
		for (y in [-80.0, -40.0, 0.0, 40.0, 80.0]) {
			tools.push(Part.box(402, 10, 9).translated(new Vector(0, y, 32)));
			tools.push(Part.box(402, 18, 6).translated(new Vector(0, y, 26)));
		}
		for (y in [-25.0, 25.0]) {
			tools.push(Part.cylinderSpan(3.3, 11, 41, 20, y));
			tools.push(Part.cylinderSpan(5.5, 33, 41, 20, y));
		}
		return Solids.cut(Part.box(400, 200, 40), tools);
	}
}
