package machinekit.structural;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** A cut rectangular tube with end mounting frames; dimensions are in millimetres. */
class RectTubeMember extends MachineComponent {
	public final profile:RectTube;
	public final length:Float;
	static var recipe:Null<ComponentType>;

	public function new(width:Float, depth:Float, wall:Float, length:Float) {
		if (!Math.isFinite(width + depth + wall + length) || !(length > 0))
			throw "Tube member needs finite positive dimensions";
		var section = new RectTube(width, depth, wall);
		super('${section.designation}-L$length', "Cut rectangular structural tube", "steel");
		profile = section;
		this.length = length;
		addConnector("base", Mount, Solids.axial(0, 0, 0));
		addConnector("end", Mount, Solids.axial(0, 0, length));
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part return profile.geometry(length);

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.structural.rect-tube-member", [
			ComponentRecipeSupport.length("width", 30), ComponentRecipeSupport.length("depth", 30),
			ComponentRecipeSupport.length("wall", 2), ComponentRecipeSupport.length("length", 750)
		], v -> new RectTubeMember(v.number("width"), v.number("depth"), v.number("wall"), v.number("length")));
		return recipe;
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function values():ComponentValues return new ComponentValues().setNumber("width", profile.width)
		.setNumber("depth", profile.height).setNumber("wall", profile.wall).setNumber("length", length)
		.setToken("material", materialSpec());
}
