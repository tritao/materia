import machinekit.assembly.MachineAssembly;
import machinekit.structural.TSlotTable;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Two reusable extrusion tables supporting the tray banks. */
class TendingCellTable extends MachineAssembly {
	public static inline final TOP_THICKNESS:Float = 10;
	public static inline final LEG_SIZE:Float = 40;
	public final bankWidth:Float;
	public final depth:Float;
	public final topHeight:Float;
	public final bankOffsetX:Float;
	public final tables:Array<TSlotTable> = [];

	public function new(bankWidth:Float, depth:Float, topHeight:Float, bankOffsetX:Float) {
		super();
		if (!Math.isFinite(bankOffsetX) || bankOffsetX <= bankWidth / 2)
			throw "Table banks must be separate";
		this.bankWidth = bankWidth; this.depth = depth;
		this.topHeight = topHeight; this.bankOffsetX = bankOffsetX;
		for (bank in [-1, 1]) {
			var table = new TSlotTable(bankWidth, depth, topHeight, "HFS5-4040", TOP_THICKNESS, 100);
			tables.push(table);
			include('bank-$bank', table, AssemblyFrames.translation(bank * bankOffsetX, 0, 0));
		}
	}

	public function collisionPieces():Array<{id:String, frame:AssemblyFrame, vertices:Array<Float>}> {
		var result = [];
		for (i in 0...tables.length) for (piece in tables[i].collisionPieces()) {
			var bank = i == 0 ? -1 : 1;
			result.push({id: 'bank-$bank/' + piece.id,
				frame: AssemblyFrames.compose(AssemblyFrames.translation(bank * bankOffsetX, 0, 0), piece.frame),
				vertices: piece.vertices});
		}
		return result;
	}
}
