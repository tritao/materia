package machinekit.motion;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.component.Dimension;
import materia.assembly.AssemblyFrames;

/** Generic BK12 fixed and BF12 floating bearing units. Millimetre envelopes and bearing seats
 * are assumed reference dimensions; the support condition is the mechanical beam boundary.
 * Origin is at the shaft centre; the shaft runs along +Z and the foot is at y=-25.
 */
class ScrewSupportUnit extends MachineComponent {
	static inline var BODY_HEIGHT:Float = 43;
	static inline var SHAFT_HEIGHT:Float = 25;
	public final fixed:Bool;
	public final boreDiameter:Float;
	public final support:ScrewSupport;
	public final width:Float;
	public final height:Float;
	public final length:Float;

	public static function bk12():ScrewSupportUnit return new ScrewSupportUnit(true);
	public static function bf12():ScrewSupportUnit return new ScrewSupportUnit(false);

	public function new(fixed:Bool, boreDiameter:Float = 12) {
		if (!Math.isFinite(boreDiameter) || boreDiameter <= 0 || boreDiameter >= 2 * (BODY_HEIGHT - SHAFT_HEIGHT))
			throw "Screw support bore must fit inside its 43 mm body above the 25 mm mounting foot";
		var size = Dimension.format(boreDiameter);
		super(boreDiameter == 12 ? (fixed ? "BK12" : "BF12") : 'SCREW-SUPPORT-${fixed ? "FIXED" : "FLOATING"}-D$size',
			'${fixed ? "Fixed" : "Floating"} screw support, $size mm seat (assumed)', "steel");
		this.boreDiameter = boreDiameter;
		this.fixed = fixed;
		support = fixed ? Fixed : Simple;
		width = 60;
		height = BODY_HEIGHT;
		length = fixed ? 25 : 20;
		addConnector("axis", Axis, Solids.axial(0, 0, 0));
		addConnector("mount", Mount, AssemblyFrames.alongY(0, -SHAFT_HEIGHT, 0, 0, -1, 0));
		for (i in 0...2) addConnector('mount${i + 1}', Mount,
			AssemblyFrames.alongY(i == 0 ? -23 : 23, -SHAFT_HEIGHT, 0, 0, -1, 0));
	}

	/** Assumed catalogue-typical axial rigidity of the preloaded fixed pair, N/mm; floating supports carry no axial load. */
	public function axialStiffness():Null<Float> return fixed ? LeadScrew.SUPPORT_BEARING_STIFFNESS : null;

	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Solids.named(Part.box(width, height, length).translated(new Vector(0, height / 2 - SHAFT_HEIGHT, -length / 2)), "body");
		var tools = [Solids.named(Part.cylinderSpan(boreDiameter / 2, -length / 2 - 0.1, length / 2 + 0.1), "shaft-seat")];
		if (detail != Envelope) for (x in [-23.0, 23.0]) tools.push(
			Solids.named(Part.cylinderAlongY(2.75, -25.1, 18.1, x, 0), x < 0 ? "mount-left" : "mount-right"));
		return Solids.cut(body, tools);
	}

	static var supportRecipe:Null<ComponentType>;
	public static function recipeType():ComponentType {
		if (supportRecipe == null) supportRecipe = new ComponentType("machinekit.motion.screw-support-unit",
			[ComponentRecipeSupport.flag("fixed", true), ComponentRecipeSupport.length("boreDiameter", 12)],
			v -> new ScrewSupportUnit(v.boolean("fixed"), v.number("boreDiameter")));
		return supportRecipe;
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function values():ComponentValues return new ComponentValues()
		.setBoolean("fixed", fixed).setNumber("boreDiameter", boreDiameter).setToken("material", materialSpec());
}
