package machinekit.milling;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** ER20 cartridge, origin at its gauge line and body along +Z. Bearing and nose envelopes are
 * assumed reference dimensions; the gauge line is the controlled point for G43.
 */
class SpindleCartridge extends MachineComponent {
	public static inline var DIAMETER:Float = 70;
	public static inline var LENGTH:Float = 180;
	public static inline var MAX_RPM:Float = 10000;

	public function new() {
		super("SPINDLE-ER20-D70", "ER20 spindle cartridge (assumed envelope)", "steel");
		addConnector("gaugeLine", Axis, Solids.axial(0, 0, 0));
		addConnector("nose", Mount, Solids.axial(0, 0, 0));
		addConnector("mount", Mount, Solids.axial(0, 0, 30));
		addConnector("pulley", Axis, Solids.axial(0, 0, LENGTH));
	}

	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part return Solids.union([
		Solids.named(Part.cylinderSpan(Er20Holder.DIAMETER / 2, Er20Holder.LENGTH, 30), "nose"),
		Solids.named(Part.cylinderSpan(DIAMETER / 2, 30, LENGTH), "body"),
		Solids.named(Part.cylinderSpan(10, LENGTH, LENGTH + 30), "drive-shaft")]);

	static var spindleRecipe:Null<ComponentType>;
	public static function recipeType():ComponentType {
		if (spindleRecipe == null) spindleRecipe = new ComponentType("machinekit.milling.spindle-cartridge", [],
			v -> new SpindleCartridge());
		return spindleRecipe;
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function values():ComponentValues return new ComponentValues().setToken("material", materialSpec());
}

/** ER20 collet nut and holder around the protruding tool shank, with a 6 mm tool bore.
 * Origin at the gauge line on the collet nut bottom; the holder extends along +Z.
 */
class Er20Holder extends MachineComponent {
	public static inline var LENGTH:Float = 25;
	public static inline var DIAMETER:Float = 34;
	public static inline var BORE_DIAMETER:Float = 6;
	public function new() {
		super("HOLDER-ER20-D6", "ER20 holder, 6 mm collet (assumed envelope)", "steel");
		addConnector("gaugeLine", Mount, Solids.axial(0, 0, 0));
		addConnector("tool", Axis, Solids.axial(0, 0, 0));
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part return Solids.cut(
		Solids.named(Part.cylinderSpan(DIAMETER / 2, 0, LENGTH), "collet-nut"),
		[Solids.named(Part.cylinderSpan(BORE_DIAMETER / 2, -0.1, LENGTH + 0.1), "tool-bore")]);
	static var holderRecipe:Null<ComponentType>;
	public static function recipeType():ComponentType {
		if (holderRecipe == null) holderRecipe = new ComponentType("machinekit.milling.er20-holder", [], v -> new Er20Holder());
		return holderRecipe;
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function values():ComponentValues return new ComponentValues().setToken("material", materialSpec());
}
