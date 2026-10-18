package machinekit.milling;

import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Machined steel crossplate supporting a bearing unit on its rear (+Y) face.
 * Origin at plate centre X/Y and bottom Z. Mounting holes are nominal clearances.
 */
class MillZBearingPlate extends MachineComponent {
	static var recipe:Null<ComponentType>;
	public static function parameters():Array<ComponentParameter> return [
			P.length("width", 220), P.length("height", 80), P.length("thickness", 20),
			P.length("crossShaftZ", 0), P.length("bearingZ", 40), P.length("anchorZ", 10), P.length("anchorSpacing", 180)];
	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.milling.z-bearing-plate", parameters(), v -> new MillZBearingPlate(v));
		return recipe;
	}
	public function new(?values:ComponentValues) {
		var v = P.resolve(parameters(), values);
		if (v.number("width") <= v.number("anchorSpacing") + 12 || v.number("height") < 20 ||
			v.number("thickness") <= 0 || v.number("bearingZ") < 6 || v.number("bearingZ") > v.number("height") - 6 ||
			v.number("anchorZ") < 6 || v.number("anchorZ") > v.number("height") - 6)
			throw "Bearing plate must retain material around its mounting holes";
		var cross = v.number("crossShaftZ");
		if (cross > 0 && (cross < 18 || cross + 18 > v.number("height") || Math.abs(cross-v.number("bearingZ")) < 31))
			throw "Cross-axis shaft passage leaves insufficient bearing-plate material";
		super("MILL-Z-BEARING-PLATE", "Machined Z screw bearing crossplate", v.get("material") == null ? "steel" : v.token("material"), false, v);
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var t = parameterNumber("thickness"), tools:Array<Part> = [];
		for (x in [-23.0, 23.0]) tools.push(Part.cylinderAlongY(2.75, -t / 2 - 1, t / 2 + 1, x, parameterNumber("bearingZ")));
		for (side in [-1, 1]) tools.push(Part.cylinderAlongY(3.3, -t / 2 - 1, t / 2 + 1,
			side * parameterNumber("anchorSpacing") / 2, parameterNumber("anchorZ")));
		if (parameterNumber("crossShaftZ") > 0) tools.push(Part.cylinderAlongY(12, -t/2-1, t/2+1, 0, parameterNumber("crossShaftZ")));
		return Solids.cut(Part.box(parameterNumber("width"), t, parameterNumber("height")), tools);
	}
}
