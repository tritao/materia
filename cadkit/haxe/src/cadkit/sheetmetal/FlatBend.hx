package cadkit.sheetmetal;

/** Target bend data and the material-covered fold-line intervals in the exported frame. */
class FlatBend {
	public final id:Int;
	public final angle:Float;
	public final innerRadius:Float;
	public final allowance:Float;
	public final nominalWidth:Float;
	public final ripped:Bool;
	final segments:Array<Segment>;
	public function new(id:Int, angle:Float, innerRadius:Float, allowance:Float, nominalWidth:Float, ripped:Bool, segments:Array<Segment>) {
		this.id = id; this.angle = angle; this.innerRadius = innerRadius; this.allowance = allowance;
		this.nominalWidth = nominalWidth; this.ripped = ripped; this.segments = segments.copy();
	}
	public function lines():Array<Segment> { return segments.copy(); }
	public function length():Float { var result = 0.0; for (segment in segments) result += segment.length(); return result; }
	public function up():Bool { return angle > 0; }
}
