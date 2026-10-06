import machinekit.assembly.MachineAssembly;
import machinekit.component.MachineComponent;
import machinekit.milling.MillPanel;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

typedef TendingTablePiece = {var id:String; var frame:AssemblyFrame; var width:Float; var depth:Float; var height:Float;}

/** Welded table carrying both tray banks, represented as independent convex members. */
class TendingCellTable extends MachineAssembly {
	public static inline final TOP_THICKNESS:Float = 10;
	public static inline final LEG_SIZE:Float = 50;
	public final bankWidth:Float;
	public final depth:Float;
	public final topHeight:Float;
	public final bankOffsetX:Float;
	final pieces:Array<TendingTablePiece> = [];

	public function new(bankWidth:Float, depth:Float, topHeight:Float, bankOffsetX:Float) {
		super();
		if (!positive(bankWidth) || !positive(depth) || !positive(topHeight) || topHeight <= TOP_THICKNESS ||
				!Math.isFinite(bankOffsetX) || bankOffsetX <= bankWidth * 0.5)
			throw 'Tending table needs a positive ${TOP_THICKNESS} mm top above the floor';
		this.bankWidth = bankWidth;
		this.depth = depth;
		this.topHeight = topHeight;
		this.bankOffsetX = bankOffsetX;
		var legHeight = topHeight - TOP_THICKNESS;
		for (bank in [-1, 1]) {
			var centreX = bank * bankOffsetX;
			addBox('top-$bank', bankWidth, depth, TOP_THICKNESS, centreX, 0, topHeight - TOP_THICKNESS);
			for (sideX in [-1, 1]) for (sideY in [-1, 1]) {
				var y = sideY * (depth * 0.5 - LEG_SIZE * 0.5);
				// The robot works from negative Y. Move that support row inward by
				// two leg widths so it stays clear of the base and approach sweep,
				// while all four legs remain beneath the load-bearing top.
				if (sideY < 0) y += 2 * LEG_SIZE;
				addBox('leg-$bank-$sideX-$sideY', LEG_SIZE, LEG_SIZE, legHeight,
					centreX + sideX * (bankWidth * 0.5 - LEG_SIZE * 0.5),
					y, 0);
			}
		}
	}

	/** Convex box hulls in table coordinates, matching the table members. */
	public function collisionPieces():Array<{id:String, frame:AssemblyFrame, vertices:Array<Float>}> return [for (piece in pieces) {
		var vertices:Array<Float> = [];
		for (x in [-piece.width * 0.5, piece.width * 0.5])
			for (y in [-piece.depth * 0.5, piece.depth * 0.5])
				for (z in [0.0, piece.height]) {
					vertices.push(x); vertices.push(y); vertices.push(z);
				}
		{id: piece.id, frame: piece.frame, vertices: vertices};
	}];

	function addBox(id:String, boxWidth:Float, boxDepth:Float, boxHeight:Float,
			x:Float, y:Float, z:Float):Void {
		var frame = AssemblyFrames.translation(x, y, z);
		addComponent(id, new MillPanel(boxWidth, boxDepth, boxHeight, "steel"), frame);
		pieces.push({id: id, frame: frame, width: boxWidth, depth: boxDepth, height: boxHeight});
	}

	static function positive(value:Float):Bool return Math.isFinite(value) && value > 0;
}
