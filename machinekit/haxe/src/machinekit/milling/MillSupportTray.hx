package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Tray with nominal seal clearance around the two bed support strips. */
class MillSupportTray extends MachineComponent {
	static var recipe:Null<ComponentType>;
	public static function parameters():Array<ComponentParameter> return [P.length("width", 1100),
		P.length("depth", 800), P.length("height", 2), P.quantity("millX", 0, "mm", null),
		P.quantity("millY", 0, "mm", null)];
	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.milling.support-tray", parameters(),
			v -> new MillSupportTray(v));
		return recipe;
	}
	public function new(?overrides:ComponentValues) {
		var v = P.resolve(parameters(), overrides);
		var width = v.number("width"), depth = v.number("depth"), height = v.number("height");
		var millX = v.number("millX"), millY = v.number("millY");
		if (width <= 0 || depth <= 0 || height <= 0 || Math.abs(millX) + 261 >= width / 2 || Math.abs(millY) + 276 >= depth / 2)
			throw "Tray must surround the mill support slots";
		super("MILL-SUPPORT-TRAY", "Mill chip tray with support penetrations", v.get("material") == null ? "steel" : v.token("material"), false, v);
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var h = parameterNumber("height"), tools:Array<Part> = [];
		for (side in [-1, 1]) tools.push(Solids.named(Part.box(27, 552, h + 2)
			.translated(new Vector(parameterNumber("millX") + side * 247.5, parameterNumber("millY"), -1)), 'seat-slot-$side'));
		return Solids.cut(Solids.named(Part.box(parameterNumber("width"), parameterNumber("depth"), h), "tray"), tools);
	}
}
