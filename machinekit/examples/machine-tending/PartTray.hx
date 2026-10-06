import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.milling.MillPanel;
import cadkit.modeling.Curve;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

typedef PartTraySlot = {var name:String; var frame:AssemblyFrame;}
typedef PartTrayPlaceTarget = {var occurrence:String; var connector:String;}
typedef PartTrayCollisionPiece = {
	var id:String;
	var frame:AssemblyFrame;
	var width:Float;
	var depth:Float;
	var height:Float;
}

/** A fixed grid tray built from separate seat rails and chamfered corner pads. */
class PartTray extends MachineAssembly {
	public static inline final EDGE_MARGIN:Float = 20;
	public static inline final PAD_HEIGHT:Float = 6;
	public static inline final RAIL_HEIGHT:Float = 8;
	public static inline final SEAT_Z:Float = PAD_HEIGHT + RAIL_HEIGHT;
	public static inline final PAD_EDGE_WIDTH:Float = 12;
	public static inline final PAD_EDGE_DEPTH:Float = 1.5;
	public static inline final PAD_CHAMFER:Float = 1;

	public final rows:Int;
	public final columns:Int;
	public final partWidth:Float;
	public final partDepth:Float;
	public final pitchX:Float;
	public final pitchY:Float;
	public final width:Float;
	public final depth:Float;
	final pieces:Array<PartTrayCollisionPiece> = [];

	public function new(rows:Int, columns:Int, partWidth:Float, partDepth:Float,
			pitchX:Float, pitchY:Float) {
		super();
		if (rows < 1 || columns < 1 || !positive(partWidth) || !positive(partDepth) ||
				!positive(pitchX) || !positive(pitchY) || pitchX < partWidth + PAD_EDGE_WIDTH ||
				pitchY < partDepth + PAD_EDGE_DEPTH)
			throw 'Part tray ${rows}x${columns} needs positive stock sizes and room for its chamfered seats';
		this.rows = rows;
		this.columns = columns;
		this.partWidth = partWidth;
		this.partDepth = partDepth;
		this.pitchX = pitchX;
		this.pitchY = pitchY;
		width = (columns - 1) * pitchX + partWidth + 2 * EDGE_MARGIN;
		depth = (rows - 1) * pitchY + partDepth + 2 * EDGE_MARGIN;

		// The cross rails carry the four small seat pads in each pocket. The outer
		// rails tie the rows together into a rigid frame. Every piece remains a
		// separate convex collision body so the fingers can enter the seat.
		for (row in 0...rows) {
			var slotY = (row - (rows - 1) * 0.5) * pitchY;
			for (side in [-1, 1]) {
				var railY = slotY + side * (partDepth * 0.5 - PAD_EDGE_DEPTH * 0.5);
				addBox('rail-r$row-$side', width, PAD_EDGE_DEPTH, RAIL_HEIGHT,
					0, railY, 0);
			}
		}
		for (side in [-1, 1])
			addBox('side-rail-$side', PAD_EDGE_WIDTH, depth, RAIL_HEIGHT,
				side * (width * 0.5 - PAD_EDGE_WIDTH * 0.5), 0, 0);
		for (row in 0...rows) for (column in 0...columns) {
			var slotX = (column - (columns - 1) * 0.5) * pitchX;
			var slotY = (row - (rows - 1) * 0.5) * pitchY;
			var padMember:Null<String> = null;
			for (sideX in [-1, 1]) for (sideY in [-1, 1]) {
				var x = slotX + sideX * (partWidth * 0.5 - PAD_EDGE_WIDTH * 0.5);
				var y = slotY + sideY * (partDepth * 0.5 - PAD_EDGE_DEPTH * 0.5);
				var id = 'seat-r$row-c$column-x$sideX-y$sideY';
				addPad(id, x, y);
				if (sideX == -1 && sideY == -1) padMember = id;
			}
			var member = padMember;
			if (member == null) throw 'Part tray slot $row-$column has no datum pad';
			var name = slot(row, column);
			addMemberConnector(member, "seat", {x: partWidth * 0.5 - PAD_EDGE_WIDTH * 0.5,
				y: partDepth * 0.5 - PAD_EDGE_DEPTH * 0.5,
				z: PAD_HEIGHT, qx: 0, qy: 0, qz: 0, qw: 1});
			exposeConnector(name, member, "seat");
		}
	}

	public function slot(row:Int, column:Int):String {
		if (row < 0 || row >= rows || column < 0 || column >= columns)
			throw 'Part tray slot $row-$column is outside ${rows}x${columns}';
		return 'slot-$row-$column';
	}

	/** The fixed datum pad that carries this pocket's seat connector. */
	public function placeTarget(row:Int, column:Int):PartTrayPlaceTarget {
		slot(row, column);
		return {occurrence: 'seat-r$row-c$column-x-1-y-1', connector: "seat"};
	}

	/** Blank bottom datum at the top of the four chamfered support pads. */
	public function seat(row:Int, column:Int):AssemblyFrame {
		slot(row, column);
		return {x: (column - (columns - 1) * 0.5) * pitchX,
			y: (row - (rows - 1) * 0.5) * pitchY,
			z: SEAT_Z, qx: 0, qy: 0, qz: 0, qw: 1};
	}

	public function seatFrames():Array<PartTraySlot> {
		var result:Array<PartTraySlot> = [];
		for (row in 0...rows) for (column in 0...columns)
			result.push({name: slot(row, column), frame: seat(row, column)});
		return result;
	}

	/** Exact convex box hulls for the independent rail and seat members, in tray coordinates. */
	public function collisionPieces():Array<{id:String, frame:AssemblyFrame, vertices:Array<Float>}> return [for (piece in pieces) {
		var vertices:Array<Float> = [];
		for (x in [-piece.width * 0.5, piece.width * 0.5])
			for (y in [-piece.depth * 0.5, piece.depth * 0.5])
				for (z in [0.0, piece.height]) {
					vertices.push(x); vertices.push(y); vertices.push(z);
				}
		{id: piece.id, frame: piece.frame, vertices: vertices};
	}];

	public function collisionFrames():Array<PartTrayCollisionPiece> return pieces.copy();

	function addBox(id:String, boxWidth:Float, boxDepth:Float, boxHeight:Float,
			x:Float, y:Float, z:Float):Void {
		var frame = AssemblyFrames.translation(x, y, z);
		addComponent(id, new MillPanel(boxWidth, boxDepth, boxHeight, "steel"), frame);
		pieces.push({id: id, frame: frame, width: boxWidth, depth: boxDepth, height: boxHeight});
	}

	function addPad(id:String, x:Float, y:Float):Void {
		var frame = AssemblyFrames.translation(x, y, RAIL_HEIGHT);
		addComponent(id, new PartTraySeatPad(PAD_EDGE_WIDTH, PAD_EDGE_DEPTH, PAD_HEIGHT, PAD_CHAMFER), frame);
		pieces.push({id: id, frame: frame, width: PAD_EDGE_WIDTH, depth: PAD_EDGE_DEPTH, height: PAD_HEIGHT});
	}

	static function positive(value:Float):Bool return Math.isFinite(value) && value > 0;
}

/** A shallow chamfered seat land with its own convex envelope for contact and clearance. */
class PartTraySeatPad extends MachineComponent {
	final width:Float;
	final depth:Float;
	final height:Float;
	final chamfer:Float;

	public function new(width:Float, depth:Float, height:Float, chamfer:Float) {
		super('Tray seat pad ${width}x${depth}x${height}', "Chamfered blank seat land", "steel", true);
		if (!(width > 2 * chamfer) || !(depth > 0) || !(height > chamfer))
			throw "Tray seat chamfer must fit inside its pad";
		this.width = width;
		this.depth = depth;
		this.height = height;
		this.chamfer = chamfer;
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var low = wire(0, width, depth);
		var high = wire(height, width - 2 * chamfer, depth);
		try {
			var result = Part.loft([low, high], true);
			low.close(); high.close();
			return result;
		} catch (error:Dynamic) {
			low.close(); high.close(); throw error;
		}
	}

	function wire(z:Float, x:Float, y:Float):Curve {
		var hx = x * 0.5, hy = y * 0.5;
		return Curve.polyline([new Vector(-hx, -hy, z), new Vector(hx, -hy, z),
			new Vector(hx, hy, z), new Vector(-hx, hy, z)], true);
	}
}

class PartTrayChecks {
	public static function run():Void {
		var tray = new PartTray(2, 3, 60, 60, 80, 80);
		if (tray.width != 260 || tray.depth != 180 || tray.connectorNames().length != 6 ||
			tray.collisionPieces().length != 30)
			throw 'Part tray frame, seats or separate collision pieces changed: ${tray.width}x${tray.depth}, ' +
				'${tray.connectorNames().length} seats, ${tray.collisionPieces().length} bodies';
		var first = tray.seat(0, 0), last = tray.seat(1, 2);
		if (first.x != -80 || first.y != -40 || first.z != 14 ||
				last.x != 80 || last.y != 40 || last.z != 14)
			throw 'Part tray seat datums do not follow the configured grid: $first / $last';
		for (piece in tray.collisionPieces()) if (piece.vertices.length != 24)
			throw 'Part tray piece "${piece.id}" is not an independent convex box';
		var rejected = false;
		try new PartTray(2, 2, 60, 60, 71, 80) catch (_:Dynamic) rejected = true;
		if (!rejected) throw "Part tray accepted overlapping seat frames";
	}
}
