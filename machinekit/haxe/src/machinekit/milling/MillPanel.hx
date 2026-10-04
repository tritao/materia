package machinekit.milling;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Rectangular fabricated panel or block, centred in X/Y with its bottom at Z zero. */
class MillPanel extends MachineComponent {
	public final width:Float;
	public final depth:Float;
	public final height:Float;
	static var recipe:Null<ComponentType>;

	public function new(width:Float, depth:Float, height:Float, material:String = "steel") {
		if (!(width > 0) || !(depth > 0) || !(height > 0) ||
			!Math.isFinite(width + depth + height)) throw "Mill panel needs finite positive dimensions";
		super('MILL-PANEL-${width}x${depth}x${height}-$material', "Fabricated mill panel", material);
		this.width = width; this.depth = depth; this.height = height;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addConnector("top", Face, Solids.axial(0, 0, height));
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.named(Part.box(width, depth, height), "panel");

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.milling.panel", [
			ComponentRecipeSupport.length("width", 100), ComponentRecipeSupport.length("depth", 2),
			ComponentRecipeSupport.length("height", 100)
		], v -> new MillPanel(v.number("width"), v.number("depth"), v.number("height")));
		return recipe;
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function values():ComponentValues return new ComponentValues().setNumber("width", width)
		.setNumber("depth", depth).setNumber("height", height).setToken("material", materialSpec());
}
