package machinekit.motion;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import materia.assembly.AssemblyFrames;

/** Generic BK12 fixed and BF12 floating bearing units. Millimetre envelopes and bearing seats
 * are assumed reference dimensions; the support condition is the mechanical beam boundary.
 * Origin is at the shaft centre; the shaft runs along +Z and the foot is at y=-25.
 */
class ScrewSupportUnit extends MachineComponent {
	public final fixed:Bool;
	public final support:ScrewSupport;
	public final width:Float;
	public final height:Float;
	public final length:Float;

	public static function bk12():ScrewSupportUnit return new ScrewSupportUnit(true);
	public static function bf12():ScrewSupportUnit return new ScrewSupportUnit(false);

	public function new(fixed:Bool) {
		super(fixed ? "BK12" : "BF12", fixed ? "Fixed ball-screw support, 12 mm seat (assumed)" :
			"Floating ball-screw support, 12 mm seat (assumed)", "steel");
		this.fixed = fixed;
		support = fixed ? Fixed : Simple;
		width = 60;
		height = 43;
		length = fixed ? 25 : 20;
		addConnector("axis", Axis, Solids.axial(0, 0, 0));
		addConnector("mount", Mount, AssemblyFrames.alongY(0, -25, 0, 0, -1, 0));
		for (i in 0...2) addConnector('mount${i + 1}', Mount,
			AssemblyFrames.alongY(i == 0 ? -23 : 23, -25, 0, 0, -1, 0));
	}

	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Solids.named(Part.box(width, height, length).translated(new Vector(0, height / 2 - 25, -length / 2)), "body");
		var tools = [Solids.named(Part.cylinderSpan(6, -length / 2 - 0.1, length / 2 + 0.1), "shaft-seat")];
		if (detail != Envelope) for (x in [-23.0, 23.0]) tools.push(
			Solids.named(Part.cylinderAlongY(2.75, -25.1, 18.1, x, 0), x < 0 ? "mount-left" : "mount-right"));
		return Solids.cut(body, tools);
	}

	static var supportRecipe:Null<ComponentType>;
	public static function recipeType():ComponentType {
		if (supportRecipe == null) supportRecipe = new ComponentType("machinekit.motion.screw-support-unit",
			[ComponentRecipeSupport.flag("fixed", true)], v -> new ScrewSupportUnit(v.boolean("fixed")));
		return supportRecipe;
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function values():ComponentValues return new ComponentValues()
		.setBoolean("fixed", fixed).setToken("material", materialSpec());
}
