package cadkit.sketch;

/**
	Where `SketchEquations` writes Jacobian rows: reusable row storage, the
	first row of the constraint being written, and that constraint's scale.
	Owned by one solve; callers consume the rows before writing again.
*/
class SketchRowWriter {
	public var rows(default, null):Array<SketchSparseRow> = [];
	var storage:Array<SketchSparseRow> = [];
	var base:Int = 0;
	var scale:Float = 1;

	public function new() {}

	/** Starts `count` empty rows, reusing storage. */
	public function reset(count:Int):Void {
		while (storage.length < count) storage.push({index: [], value: []});
		for (r in 0...count) { storage[r].index.resize(0); storage[r].value.resize(0); }
		rows = count == storage.length ? storage : storage.slice(0, count);
	}

	/** The next constraint's rows start at `row` and its entries are multiplied by `scale`. */
	public inline function begin(row:Int, scale:Float):Void {
		base = row;
		this.scale = scale;
	}

	public inline function entry(r:Int, index:Int, value:Float):Void {
		var target = rows[base + r];
		target.index.push(index);
		target.value.push(value * scale);
	}
}
