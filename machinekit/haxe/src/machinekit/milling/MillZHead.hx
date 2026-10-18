package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Ribbed fabricated steel spindle head. Origin on rear mounting face at bottom;
 * projects toward -Y. Plate/rib thicknesses are explicit design assumptions.
 */
class MillZHead extends MachineComponent {
	static var recipe:Null<ComponentType>;
	public static function parameters():Array<ComponentParameter> return [
			P.length("width", 180), P.length("reach", 204), P.length("height", 130),
			P.length("spindleReach", 159), P.length("plate", 14), P.length("wall", 12)];
	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.milling.z-head", parameters(), v -> new MillZHead(v));
		return recipe;
	}
	public function new(?values:ComponentValues) {
		var v = P.resolve(parameters(), values), r = SpindleCartridge.DIAMETER / 2;
		if (v.number("width") <= 2 * (r + v.number("wall")) || v.number("height") <= 2 * v.number("plate") ||
			v.number("spindleReach") <= r + v.number("wall") ||
			v.number("reach") - v.number("spindleReach") <= r || v.number("wall") <= 0 || v.number("plate") <= 0)
			throw "Z head must surround its spindle bore and retain connected ribs";
		super("MILL-Z-RIBBED-HEAD", "Ribbed fabricated spindle head", v.get("material") == null ? "steel" : v.token("material"), false, v);
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addConnector("spindle", Axis, Solids.axial(0, -v.number("spindleReach"), 0));
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var w = parameterNumber("width"), d = parameterNumber("reach"), h = parameterNumber("height");
		var plate = parameterNumber("plate"), wall = parameterNumber("wall"), spindle = parameterNumber("spindleReach");
		var collar = 2 * (SpindleCartridge.DIAMETER / 2 + 10), collarRear = -spindle + collar / 2;
		var pieces = [Part.box(w, collar, h).translated(new Vector(0, -spindle, 0))];
		for (z in [0.0, h - plate]) pieces.push(Part.box(w, d, plate).translated(new Vector(0, -d / 2, z)));
		for (side in [-1, 1]) pieces.push(Part.box(wall, -collarRear, h)
			.translated(new Vector(side * (w / 2 - wall / 2), collarRear / 2, 0)));
		// Two cross webs connect the side walls, top and bottom into one body.
		for (fraction in [1.0 / 3, 2.0 / 3]) pieces.push(Part.box(w, wall, h)
			.translated(new Vector(0, collarRear * fraction, 0)));
		return Solids.cut(Solids.union(pieces), [Part.cylinderSpan(SpindleCartridge.DIAMETER / 2, -1, h + 1, 0, -spindle)]);
	}
}
