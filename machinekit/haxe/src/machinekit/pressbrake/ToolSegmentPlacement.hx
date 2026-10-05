package machinekit.pressbrake;

/** A segment's derived bed interval, including the gaps before it. */
class ToolSegmentPlacement {
	public final segment:ToolSegment;
	public final start:Float;
	public final end:Float;

	public function new(segment:ToolSegment, start:Float) {
		if (segment == null || !Math.isFinite(start)) throw "Tool placement needs a segment and a finite bed position";
		this.segment = segment;
		this.start = start;
		end = start + segment.length;
		if (!Math.isFinite(end)) throw "Tool placement exceeds the finite bed range";
	}
}
