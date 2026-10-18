package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Ribbed fabricated steel spindle head. Origin on rear mounting face at bottom;
 * projects toward -Y. Plate/rib thicknesses are explicit design assumptions.
 */
class MillZHead extends MachineComponent {
	public static inline var COUNTERBORE_DEPTH:Float = 6.5;
	static var recipe:Null<ComponentType>;
	public static function parameters():Array<ComponentParameter> return [
			P.length("width", 180), P.length("reach", 204), P.length("height", 130),
			P.length("noseWidth", 120), P.length("rearWidth", 180), P.length("rearPlate", 12), P.length("boltX", 65), P.length("boltZ", 42),
			P.length("spindleReach", 159), P.length("plate", 14), P.length("wall", 12)];
	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.milling.z-head", parameters(), v -> new MillZHead(v));
		return recipe;
	}
	public function new(?values:ComponentValues) {
		var v = P.resolve(parameters(), values), r = SpindleCartridge.DIAMETER / 2;
		if (v.number("width") <= 2 * (r + v.number("wall")) || v.number("height") <= 3 * v.number("plate") + 8 ||
			v.number("spindleReach") <= r + v.number("wall") ||
			v.number("reach") - v.number("spindleReach") <= r || v.number("wall") <= 0 || v.number("plate") <= 0)
			throw "Z head must surround its spindle bore and retain connected ribs";
		if (v.number("noseWidth") <= 2 * (r + v.number("wall")) || v.number("noseWidth") > v.number("width") || v.number("rearWidth") < v.number("width") || v.number("rearPlate") < 10 ||
			v.number("boltX") + 7 > v.number("rearWidth") / 2 || 2 * (v.number("boltZ") + 7) > v.number("height"))
			throw "Head mounting flange cannot retain its bolt pattern";
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
		var rearPlate = parameterNumber("rearPlate");
		var pieces = [Part.box(w, collar, h).translated(new Vector(0, -spindle, 0))];
		for (z in [0.0, h - plate]) pieces.push(Part.box(w, d, plate).translated(new Vector(0, -d / 2, z)));
		for (side in [-1, 1]) pieces.push(Part.prism([
			new Vector(side * w/2, 0), new Vector(side * parameterNumber("noseWidth")/2, collarRear),
			new Vector(side * (parameterNumber("noseWidth")/2-wall), collarRear), new Vector(side * (w/2-wall), 0)], 0, h));
		// Two cross webs connect the side walls, top and bottom into one body.
		for (fraction in [1.0 / 3, 2.0 / 3]) pieces.push(Part.box(w, wall, h)
			.translated(new Vector(0, collarRear * fraction, 0)));
		var tools = [Part.cylinderSpan(SpindleCartridge.DIAMETER / 2, -1, h + 1, 0, -spindle)];
		// Two rows of side windows leave top/bottom chords, cross webs and a middle rib.
		for (row in 0...2) for (cell in 0...3) {
			var rear = collarRear * cell / 3 - wall / 2 - 2;
			var front = collarRear * (cell + 1) / 3 + wall / 2 + 2;
			if (cell == 0) rear = -rearPlate - 2;
			if (rear > front) tools.push(Part.box(w + 2, rear - front, (h - 3 * plate) / 2)
				.translated(new Vector(0, (rear + front) / 2, plate + row * (h - plate) / 2)));
		}
		// Front-access head bolts pass only through the rear flange. Counterbores seat M6 heads.
		for (x in [-parameterNumber("boltX"), parameterNumber("boltX")])
			for (z in [h / 2 - parameterNumber("boltZ"), h / 2 + parameterNumber("boltZ")]) {
				tools.push(Part.cylinderAlongY(3.3, -rearPlate - 1, 1, x, z));
				tools.push(Part.cylinderAlongY(5.5, -rearPlate - 1, -rearPlate + COUNTERBORE_DEPTH, x, z));
			}
		var raw = Solids.union(pieces), nose = parameterNumber("noseWidth");
		var outline = Part.prism([new Vector(-w/2, 0), new Vector(w/2, 0), new Vector(nose/2, collarRear),
			new Vector(nose/2, -d), new Vector(-nose/2, -d), new Vector(-nose/2, collarRear)], 0, h);
		var tapered = raw.intersect(outline); raw.close(); outline.close();
		var flange = Part.box(parameterNumber("rearWidth"), rearPlate, h).translated(new Vector(0, -rearPlate/2, 0));
		return Solids.cut(Solids.union([tapered, flange]), tools);
	}
}
