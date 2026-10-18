package machinekit.milling;

import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Cross slide, bottom Z datum; through-Y tunnel clears the Y nut over its complete stroke. */
class MillCrossSaddle extends MachineComponent {
	static var recipe:Null<ComponentType>;

	public static function parameters():Array<ComponentParameter> return [
		P.length("width", 450),
		P.length("depth", 174),
		P.length("height", 80),
		P.length("bore", 54)
	];

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.milling.cross-saddle", parameters(), v -> new MillCrossSaddle(v));
		return recipe;
	}

	public function new(?values:ComponentValues) {
		var v = P.resolve(parameters(), values);
		if (v.number("bore") + 20 > v.number("height") || v.number("bore") >= v.number("width")) throw "Saddle tunnel leaves insufficient material";
		super("MILL-CROSS-SADDLE", "Machined cross saddle with through nut tunnel", "cast iron", false, v);
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addConnector("top", Face, Solids.axial(0, 0, v.number("height")));
	}

	override public function componentType():Null<ComponentType> return recipeType();

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var w = parameterNumber("width"), d = parameterNumber("depth"), h = parameterNumber("height");
		var tools = [Part.cylinderAlongY(parameterNumber("bore") / 2, -d / 2 - 1, d / 2 + 1, 0, h / 2)];
		for (x in [-35.0, 35.0]) for (z in [-20.0, 20.0]) tools.push(Part.cylinderAlongY(2.5, d / 2 - 16, d / 2 + 1, x, h / 2 + z));
		return Solids.cut(Part.box(w, d, h), tools);
	}
}
