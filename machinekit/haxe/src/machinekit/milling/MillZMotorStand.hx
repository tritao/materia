package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Steel U stand carries the stationary motor above the upper bearing plate.
 * Open sides allow access to the coupling and spring-applied shaft brake.
 * The caller supplies the selected motor hole pitch and pilot recess.
 */
class MillZMotorStand extends MachineComponent {
	static var recipe:Null<ComponentType>;
	public static function parameters():Array<ComponentParameter> return [
			P.flag("anchored",false), P.length("registerDiameter",0), P.length("registerDepth",2),
			P.length("width", 100), P.length("depth", 100), P.length("height", 32.5),
			P.length("plate", 8), P.length("leg", 12), P.length("shaftBore", 22), P.length("boltPitch", 40)];
	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.milling.z-motor-stand", parameters(), v -> new MillZMotorStand(v));
		return recipe;
	}
	public function new(?values:ComponentValues) {
		var v = P.resolve(parameters(), values);
		if (v.number("height") <= v.number("plate") || v.number("plate") <= 0 || v.number("leg") <= 0 ||
			v.number("width") - 2 * v.number("leg") <= v.number("shaftBore") ||
			v.number("boltPitch") + 12 >= Math.min(v.number("width"), v.number("depth")) ||
			v.number("boltPitch") <= v.number("shaftBore") + 8)
			throw "Motor stand cannot fit its shaft and mounting holes";
		super("MILL-Z-MOTOR-STAND", "Open steel Z motor and brake stand", v.get("material") == null ? "steel" : v.token("material"), false, v);
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addConnector("motor", Mount, Solids.axial(0, 0, v.number("height")));
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var w = parameterNumber("width"), d = parameterNumber("depth"), h = parameterNumber("height"), t = parameterNumber("plate");
		var leg = parameterNumber("leg"), pieces = [Part.box(w, d, t).translated(new Vector(0, 0, h - t))];
		for (side in [-1, 1]) pieces.push(Part.box(leg, d, h - t).translated(new Vector(side * (w - leg) / 2, 0, 0)));
		if (values().boolean("anchored")) pieces.push(Part.box(160,20,8).translated(new Vector(0,-35,0)));
		var tools = [Part.cylinderSpan(parameterNumber("shaftBore") / 2, h - t - 1, h + 1)];
		for (x in [-parameterNumber("boltPitch") / 2, parameterNumber("boltPitch") / 2])
			for (y in [-parameterNumber("boltPitch") / 2, parameterNumber("boltPitch") / 2]) tools.push(Part.cylinderSpan(3.3, h - t - 1, h + 1, x, y));
		if (values().boolean("anchored")) for (side in [-1,1]) tools.push(Part.cylinderAlongY(1.65,-d/2-1,-d/2+10,side*(w-leg)/2,16));
		if (values().boolean("anchored")) for (x in [-70.0,70.0]) tools.push(Part.cylinderSpan(3.3,-1,9,x,-35));
		if (parameterNumber("registerDiameter") > 0) tools.push(Part.cylinderSpan(parameterNumber("registerDiameter")/2,h-parameterNumber("registerDepth"),h+1));
		return Solids.cut(Solids.union(pieces), tools);
	}
}
