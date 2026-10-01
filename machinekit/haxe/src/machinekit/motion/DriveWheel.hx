package machinekit.motion;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Drive wheel of a wheeled base: a tread disc on a hub boss, bored to sit straight on a motor or
 * gearbox shaft. CAD frame: the axle runs along +Z, the hub's inner end is at z=0 and the tread
 * spans z = hubLength..hubLength+width, with a shallow dish in its outer face. Connectors: `bore`
 * (shaft, at the hub end) and `centre` (axis, mid-tread), both with +Y along +Z. The tread's
 * floor contact is `radius` from the axle.
 */
class DriveWheel extends MachineComponent {
	public final diameter:Float;
	public final width:Float;
	public final boreDiameter:Float;
	public final hubDiameter:Float;
	public final hubLength:Float;
	public var radius(get, never):Float;

	/** Depth of the dish in the outer face, and the tread ring it leaves round it. */
	static inline var DISH_DEPTH:Float = 3;
	static inline var TREAD_RING:Float = 12;

	public function new(diameter:Float, width:Float, boreDiameter:Float, hubDiameter:Float, hubLength:Float) {
		for (value in [diameter, width, boreDiameter, hubDiameter, hubLength])
			if (!Math.isFinite(value)) throw "Drive wheel dimensions must be finite";
		if (!(boreDiameter > 0) ||
				!(hubDiameter > boreDiameter + 4) || !(diameter / 2 - TREAD_RING > hubDiameter / 2 + 2) ||
				!(width > DISH_DEPTH + 2) || !(hubLength >= 0))
			throw "Drive wheel needs a bore inside its hub, the hub inside its tread, and a positive width";
		var text = 'D${Dimension.format(diameter)}-W${Dimension.format(width)}-B${Dimension.format(boreDiameter)}';
		super('DRIVE-WHEEL-$text-H${Dimension.format(hubDiameter)}x${Dimension.format(hubLength)}',
			'Drive wheel, ${Dimension.format(diameter)} x ${Dimension.format(width)} mm, ${Dimension.format(boreDiameter)} mm bore',
			"polyurethane PU", true);
		this.diameter = diameter;
		this.width = width;
		this.boreDiameter = boreDiameter;
		this.hubDiameter = hubDiameter;
		this.hubLength = hubLength;
		addConnector("bore", Shaft, Solids.axial(0, 0, 0));
		addConnector("centre", Axis, Solids.axial(0, 0, hubLength + width / 2));
	}

	function get_radius():Float return diameter / 2;

	public static function recipeType():ComponentType
		return machinekit.component.MachineKitAdditionalRecipes.byId("machinekit.motion.drive-wheel");

	override public function componentType():Null<ComponentType>
		return Std.isExactType(this, DriveWheel) ? recipeType() : null;

	override public function values():ComponentValues
		return new ComponentValues().setNumber("diameter", diameter).setNumber("width", width)
			.setNumber("boreDiameter", boreDiameter).setNumber("hubDiameter", hubDiameter)
			.setNumber("hubLength", hubLength).setToken("material", materialSpec());

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var outer = hubLength + width;
		var pieces = [Solids.named(Part.cylinderSpan(diameter / 2, hubLength, outer), "tread")];
		if (hubLength > 0) pieces.push(Solids.named(Part.cylinderSpan(hubDiameter / 2, 0, hubLength + 1), "hub"));
		var body = Solids.union(pieces);
		if (detail == Envelope) return body;
		// The dish's two walls are named apart, so the faces it leaves stay distinguishable.
		var dish = Solids.cut(Solids.named(Part.cylinderSpan(diameter / 2 - TREAD_RING, outer - DISH_DEPTH, outer + 0.1), "dish"),
			[Solids.named(Part.cylinderSpan(hubDiameter / 2, outer - DISH_DEPTH - 0.1, outer + 0.2), "hubFace")]);
		return Solids.cut(body, [dish,
			Solids.named(Part.cylinderSpan(boreDiameter / 2, -0.1, outer + 0.1), "bore")]);
	}
}
