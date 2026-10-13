package machinekit.pressbrake;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import materia.project.SceneContactSurface;
import materia.project.SceneContactFilter;

/**
 * Assumed gravity-return contact finger. Its stock datum is z=0, its hinge is
 * 50 mm behind the front face and 20 mm below the datum. The top projects 1 mm
 * above the datum so thin stock can engage it after the finger drops.
 * Contact compliance is an explicit simulation assumption, not a material rating.
 */
class GaugeFinger extends MachineComponent {
	static var recipe:Null<ComponentType>;
	public static inline final WIDTH_MM = 20.0;
	public static inline final DEPTH_MM = 60.0;
	public static inline final HEIGHT_MM = 21.0;
	public static inline final DATUM_HEIGHT_MM = 20.0;
	public static inline final PIVOT_DEPTH_MM = 50.0;
	/** Assumed nominal hinge travel and observed-position fault margin, radians. */
	public static final DROP_ANGLE = Math.PI / 360;
	public static inline final OVERTRAVEL = 0.001;
	public static inline final SWITCH_WINDOW = 0.001;
	public static inline final SWITCH_HYSTERESIS = 0.0001;

	public function new() {
		super("PRESS-GAUGE-FINGER", "Assumed gravity-return backgauge contact finger", "steel", true);
		addConnector("mount", Mount, Solids.axial(0, PIVOT_DEPTH_MM, -DATUM_HEIGHT_MM));
		addConnector("contact", Face, Solids.axial(0, 0, 0));
		addPort({name: "contact", kind: Signal, role: Supply, iface: Plug("generic", 3), required: false});
	}

	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.named(Part.box(WIDTH_MM, DEPTH_MM, HEIGHT_MM)
			.translated(new Vector(0, DEPTH_MM / 2, -DATUM_HEIGHT_MM)), "finger");

	override public function simulationContact():Null<SceneContactSurface>
		return {friction: [0, 0, 0], frictionDimensions: 0, contactTimeConstant: stopResponse().timeConstantSec,
			contactDampingRatio: 1, filter: SceneContactFilter.PairsAndEnvironment};

	/** Assumed elastic contact absorbs the complete gravity-drop energy within the declared compression. */
	public static function stopResponse():materia.assembly.AssemblyJointStop {
		var depth = DEPTH_MM * 0.001, height = HEIGHT_MM * 0.001;
		var y = depth / 2 - PIVOT_DEPTH_MM * 0.001, z = height / 2;
		var inertiaPerMass = (depth * depth + height * height) / 12 + y * y + z * z;
		var energyPerMass = 9.80665 * (z - (y * Math.sin(DROP_ANGLE) + z * Math.cos(DROP_ANGLE)));
		var stiffnessPerMass = 2 * energyPerMass / (OVERTRAVEL * OVERTRAVEL);
		return {timeConstantSec: Math.sqrt(inertiaPerMass / stiffnessPerMass), dampingRatio: 1};
	}

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.pressbrake.gauge-finger", [], _ -> new GaugeFinger());
		return recipe;
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function values():ComponentValues return new ComponentValues().setToken("material", materialSpec());
}
