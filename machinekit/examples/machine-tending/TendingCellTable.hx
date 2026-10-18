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
			// Expand outwards from the central robot corridor and forward from the rear
			// top edge. Only the low base extends; the tray and upright locations stay fixed.
			var baseWidth=Math.max(bankWidth,Math.ceil(topHeight*0.7/10)*10);
			var baseDepth=Math.max(depth,Math.ceil(topHeight*0.65/10)*10);
			var table = new TSlotTable(bankWidth,depth,topHeight,"HFS5-4040",TOP_THICKNESS,100,30,220,null,true,
				{width:baseWidth,depth:baseDepth,offsetX:bank*(baseWidth-bankWidth)/2,offsetY:(depth-baseDepth)/2});
			tables.push(table);
			include(bank < 0 ? "left-table" : "right-table", table, AssemblyFrames.translation(bank * bankOffsetX, 0, 0));
		}
	}

	public function collisionPieces():Array<{id:String, frame:AssemblyFrame, vertices:Array<Float>}> {
		var result = [];
		for (i in 0...tables.length) for (piece in tables[i].collisionPieces()) {
			var bank = i == 0 ? -1 : 1;
			result.push({id: (bank < 0 ? "left-table/" : "right-table/") + piece.id,
				frame: AssemblyFrames.compose(AssemblyFrames.translation(bank * bankOffsetX, 0, 0), piece.frame),
				vertices: piece.vertices});
		}
		return result;
	}
}
