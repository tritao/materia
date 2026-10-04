import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.component.Dimension;

/** Single-setup bearing-block exercise: a 608 seat, two blind fastening recesses and a rounded
 * contour pocket. The stock and target share a bottom-centred frame, in millimetres.
 */
class BearingBlock extends MachineComponent {
	public final width:Float;
	public final depth:Float;
	public final height:Float;
	public static inline var SEAT_DIAMETER:Float = 22;
	public static inline var SEAT_DEPTH:Float = 7;
	public static inline var COUNTERBORE_DIAMETER:Float = 10;
	public static inline var COUNTERBORE_DEPTH:Float = 5;
	public static inline var POCKET_DEPTH:Float = 3;
	public static inline var POCKET_RADIUS:Float = 3.5;
	public final seatY:Float;
	public final counterboreY:Float;
	public final counterboreX:Float;
	public final pocketWidth:Float;
	public final pocketDepth:Float;
	public final pocketY:Float;

	public function new(width:Float, depth:Float, height:Float) {
		if (width < 60 || depth < 40 || height <= SEAT_DEPTH) throw "Bearing-block stock is too small for its recesses";
		super('BEARING-BLOCK-${Dimension.format(width)}x${Dimension.format(depth)}x${Dimension.format(height)}',
			"608 bearing block, single-tool pocket exercise", "aluminium 6061", true);
		this.width = width;
		this.depth = depth;
		this.height = height;
		seatY = 0;
		counterboreY = -depth * 0.3;
		counterboreX = width / 3;
		pocketWidth = width / 5;
		pocketDepth = depth / 5;
		pocketY = -depth * 0.3875;
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var block = Solids.named(Part.box(width, depth, height), "blank");
		if (detail == Envelope) return block;
		var tools = [Solids.named(Part.cylinderSpan(SEAT_DIAMETER / 2, height - SEAT_DEPTH, height + 1, 0, seatY), "bearing-seat")];
		for (side in [-1, 1]) tools.push(Solids.named(Part.cylinderSpan(COUNTERBORE_DIAMETER / 2,
			height - COUNTERBORE_DEPTH, height + 1, side * counterboreX, counterboreY), side < 0 ? "counterbore-left" : "counterbore-right"));
		var bottom = height - POCKET_DEPTH, cutHeight = POCKET_DEPTH + 1;
		var pieces = [Part.box(pocketWidth - 2 * POCKET_RADIUS, pocketDepth, cutHeight)
			.translated(new Vector(0, pocketY, bottom)),
			Part.box(pocketWidth, pocketDepth - 2 * POCKET_RADIUS, cutHeight).translated(new Vector(0, pocketY, bottom))];
		for (sx in [-1, 1]) for (sy in [-1, 1]) pieces.push(Part.cylinderSpan(POCKET_RADIUS, bottom, height + 1,
			sx * (pocketWidth / 2 - POCKET_RADIUS), pocketY + sy * (pocketDepth / 2 - POCKET_RADIUS)));
		tools.push(Solids.named(Solids.union(pieces), "contour-pocket"));
		return Solids.cut(block, tools);
	}
	/** Closed-form removal, mm³, independent of the CAD booleans and stock simulation. */
	public function removedVolume():Float return Math.PI * (SEAT_DIAMETER * SEAT_DIAMETER / 4 * SEAT_DEPTH +
		2 * COUNTERBORE_DIAMETER * COUNTERBORE_DIAMETER / 4 * COUNTERBORE_DEPTH) +
		(pocketWidth * pocketDepth - (4 - Math.PI) * POCKET_RADIUS * POCKET_RADIUS) * POCKET_DEPTH;
}
