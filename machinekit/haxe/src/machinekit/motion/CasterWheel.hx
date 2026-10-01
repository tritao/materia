package machinekit.motion;

import cadkit.modeling.Align;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Plate-mounted swivel caster: a square top plate, a swivel boss, a fork and a wheel on its axle.
 * It is one rigid part; the swivel is not a joint. CAD frame: the top of the plate is at z=0 with
 * the body hanging toward -Z, the swivel axis is the Z axis, the wheel trails it along -X by `trail`
 * and touches the floor at z = -height. Connectors: `mount` (top of the plate) and `swivel` (axis),
 * both with +Y along +Z, and `floor` (the contact point, +Y up).
 */
class CasterWheel extends MachineComponent {
	public final wheelDiameter:Float;
	public final wheelWidth:Float;
	/** From the top of the plate to the floor. */
	public final height:Float;
	/** Horizontal distance from the swivel axis back to the wheel axle. */
	public final trail:Float;
	public final plateSize:Float;

	static inline var PLATE_THICKNESS:Float = 4;
	static inline var SWIVEL_HEIGHT:Float = 10;
	static inline var LEG_THICKNESS:Float = 3;
	static inline var LEG_GAP:Float = 2;

	public function new(wheelDiameter:Float, wheelWidth:Float, height:Float, trail:Float, plateSize:Float) {
		for (value in [wheelDiameter, wheelWidth, height, trail, plateSize])
			if (!Math.isFinite(value)) throw "Caster dimensions must be finite";
		var forkWidth = wheelWidth + 2 * (LEG_GAP + LEG_THICKNESS);
		if (!(wheelDiameter > 0) || !(wheelWidth > 0) || !(trail >= 0) || !(plateSize >= forkWidth) ||
				!(height - wheelDiameter >= PLATE_THICKNESS + SWIVEL_HEIGHT + 6))
			throw "Caster needs room above its wheel for the plate, swivel and fork crown";
		var text = 'D${Dimension.format(wheelDiameter)}x${Dimension.format(wheelWidth)}-H${Dimension.format(height)}';
		super('CASTER-$text-T${Dimension.format(trail)}-P${Dimension.format(plateSize)}',
			'Swivel caster, ${Dimension.format(wheelDiameter)} mm wheel, ${Dimension.format(height)} mm high', "painted steel", true);
		this.wheelDiameter = wheelDiameter;
		this.wheelWidth = wheelWidth;
		this.height = height;
		this.trail = trail;
		this.plateSize = plateSize;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addConnector("swivel", Axis, Solids.axial(0, 0, -PLATE_THICKNESS));
		addConnector("floor", Mount, Solids.axial(-trail, 0, -height));
	}

	/** Height of the wheel axle above the floor. */
	public var axleHeight(get, never):Float;

	function get_axleHeight():Float return wheelDiameter / 2;

	public static function recipeType():ComponentType
		return machinekit.component.MachineKitAdditionalRecipes.byId("machinekit.motion.caster-wheel");

	override public function componentType():Null<ComponentType>
		return Std.isExactType(this, CasterWheel) ? recipeType() : null;

	override public function values():ComponentValues
		return new ComponentValues().setNumber("wheelDiameter", wheelDiameter).setNumber("wheelWidth", wheelWidth)
			.setNumber("height", height).setNumber("trail", trail).setNumber("plateSize", plateSize)
			.setToken("material", materialSpec());

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var axleZ = wheelDiameter / 2 - height;
		var crownTop = -PLATE_THICKNESS - SWIVEL_HEIGHT, crownBottom = crownTop - 4;
		var halfFork = wheelWidth / 2 + LEG_GAP + LEG_THICKNESS;
		var legLength = trail + 16;
		var pieces = [
			Solids.named(Part.box(plateSize, plateSize, PLATE_THICKNESS, Align.Center, Align.Center, Align.Max), "plate"),
			Solids.named(Part.cylinderSpan(plateSize * 0.3, crownTop, -PLATE_THICKNESS), "swivel"),
			Solids.named(Part.box(legLength, 2 * halfFork, 4, Align.Max, Align.Center, Align.Min)
				.translated(new Vector(8, 0, crownBottom)), "crown"),
			Solids.named(Part.cylinderAlongY(wheelDiameter / 2, -wheelWidth / 2, wheelWidth / 2, -trail, axleZ), "wheel")
		];
		for (side in [-1, 1]) {
			var y = side * (halfFork - LEG_THICKNESS / 2);
			var leg = Part.box(16, LEG_THICKNESS, crownBottom - axleZ + 8, Align.Center, Align.Center, Align.Min)
				.translated(new Vector(-trail, y, axleZ - 8));
			pieces.push(Solids.named(leg, side < 0 ? "legRight" : "legLeft"));
		}
		// The axle is what holds the wheel to the fork, so the envelope keeps it too.
		pieces.push(Solids.named(Part.cylinderAlongY(4, -halfFork - 3, halfFork + 3, -trail, axleZ), "axle"));
		return Solids.union(pieces);
	}
}
