package cadkit.threads;

/** The end from which a thread proceeds along the face's native axis. */
enum ThreadStart {
	AxisMinimum;
	AxisMaximum;
}

/** Immutable manufacturing interval on a strong face name, in native axial parameters. */
class ThreadedFace {
	public final faceName:String;
	public final spec:ThreadSpec;
	public final start:Float;
	public final direction:Int;
	public final length:Null<Float>;

	public function new(faceName:String, spec:ThreadSpec, start:Float, direction:Int, length:Null<Float>) {
		if (!Math.isFinite(start) || (direction != 1 && direction != -1)
			|| (length != null && (!Math.isFinite(length) || length <= 0)))
			throw 'Invalid thread interval on $faceName';
		this.faceName = faceName;
		this.spec = spec;
		this.start = start;
		this.direction = direction;
		this.length = length;
	}

	public function renamed(name:String):ThreadedFace
		return new ThreadedFace(name, spec, start, direction, length);
}
