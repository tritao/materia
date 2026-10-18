package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Y drive service hatch. Origin on its front plane; +Y points out of the machine.
 * Four holes land on dedicated steel anchors rather than the mineral casting.
 */
class MillDriveHatch extends MachineComponent {
	static var recipe:Null<ComponentType>;
	public static function parameters():Array<ComponentParameter> return [P.length("lowerZ", 74), P.length("upperZ", 170)];
	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.milling.drive-service-hatch", parameters(), v -> new MillDriveHatch(v));
		return recipe;
	}
	public function new(?values:ComponentValues) {
		var v = P.resolve(parameters(), values);
		if (v.number("lowerZ") < 10 || v.number("upperZ") - v.number("lowerZ") < 40) throw "Drive hatch has no flange or useful opening";
		super("MILL-Y-DRIVE-HATCH", "Removable Y drive service hatch", "painted steel", false, v);
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var holes:Array<Part> = [];
		for (x in [-75.0, 75.0]) for (z in [parameterNumber("lowerZ"), parameterNumber("upperZ")]) holes.push(Part.cylinderAlongY(2.25, -1, 3, x, z));
		return Solids.cut(Part.box(180, 2, parameterNumber("upperZ") - parameterNumber("lowerZ") + 20).translated(new Vector(0, 1, parameterNumber("lowerZ") - 10)), holes);
	}
}
