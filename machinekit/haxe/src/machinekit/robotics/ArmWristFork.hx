package machinekit.robotics;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import materia.assembly.AssemblyFrames;

/** Clevis carried by j5, with the hand axis through the wrist centre.
 * The pitch module spans local X from -span/2 to +span/2. Its rotor mates to
 * start at +span/2; the next module sits on +Z, rather than inheriting the rotor
 * face's lateral offset. Dimensions are assumed engineering geometry in mm.
 */
class ArmWristFork extends MachineComponent {
	public final length:Float;
	public final span:Float;
	public final wall:Float;
	public final depth:Float;
	static var recipe:Null<ComponentType>;

	public function new(length:Float, span:Float, wall:Float = 8, depth:Float = 24) {
		for (value in [length, span, wall, depth])
			if (!Math.isFinite(value) || value <= 0) throw "Wrist fork needs finite positive dimensions";
		if (length <= 2 * wall || span <= 2 * wall || depth <= wall)
			throw "Wrist fork needs space around its joint module";
		super('ARM-WRIST-FORK-${Dimension.format(length)}x${Dimension.format(span)}-W${Dimension.format(wall)}-D${Dimension.format(depth)}',
			"Spherical wrist clevis, assumed dimensions", "aluminium 6061", true);
		this.length = length; this.span = span; this.wall = wall; this.depth = depth;
		addConnector("start", Mount, AssemblyFrames.alongY(span / 2, 0, 0, 1, 0, 0));
		addConnector("end", Mount, Solids.axial(0, 0, length));
	}

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.robotics.arm-wrist-fork", [
			ComponentRecipeSupport.length("length", 60), ComponentRecipeSupport.length("span", 70),
			ComponentRecipeSupport.length("wall", 8), ComponentRecipeSupport.length("depth", 24)
		], v -> new ArmWristFork(v.number("length"), v.number("span"), v.number("wall"), v.number("depth")));
		return recipe;
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function values():ComponentValues
		return new ComponentValues().setNumber("length", length).setNumber("span", span)
			.setNumber("wall", wall).setNumber("depth", depth).setToken("material", materialSpec());
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var pieces = [Solids.named(Part.box(span + 2 * wall, depth, wall)
			.translated(new Vector(0, 0, length - wall)), "crossbar")];
		for (side in [-1, 1]) pieces.push(Solids.named(Part.box(wall, depth, length)
			.translated(new Vector(side * (span + wall) / 2, 0, 0)), side < 0 ? "left-cheek" : "right-cheek"));
		return Solids.union(pieces);
	}
}
