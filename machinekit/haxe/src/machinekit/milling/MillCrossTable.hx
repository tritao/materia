package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** T-slot table with integral longitudinal ribs and a recessed X nut bracket seat.
 * Origin is the underside of the deck; ribs extend down to the guide blocks. */
class MillCrossTable extends MachineComponent {
	static var recipe:Null<ComponentType>;

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.milling.cross-table", [P.length("depth", 200), P.length("ribSpacing", 124)], v -> new MillCrossTable(v.number("depth"), v.number("ribSpacing")));
		return recipe;
	}

	public function new(depth:Float = 200, ribSpacing:Float = 124) {
		if (!Math.isFinite(depth) || !Math.isFinite(ribSpacing) || depth < 200 || ribSpacing < 124 || ribSpacing + 34 > depth - 12) throw "Table must retain its slots, drive corridor and rib edge margins";
		super("MILL-CROSS-TABLE", "Ribbed 400 mm T-slot table", "cast iron", false, new ComponentValues().setNumber("depth", depth).setNumber("ribSpacing", ribSpacing));
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
		var pieces = [Part.box(400, parameterNumber("depth"), 40)];
		for (side in [-1, 1]) pieces.push(Part.box(400, 34, MillXYStageLayout.RISER_HEIGHT)
			.translated(new Vector(0, side * parameterNumber("ribSpacing") / 2, -MillXYStageLayout.RISER_HEIGHT)));
		return Solids.cut(Solids.union(pieces), tools);
	}
}
