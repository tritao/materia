package machinekit.pressbrake;

/** One homogeneous punch or die row. Material length excludes gaps; span includes them. */
class ToolRow {
	final pieces:Array<ToolSegment>;
	final spaces:Array<Float>;
	public var length(get, never):Float;
	public var span(get, never):Float;
	public final profile:ToolProfile;

	public function new(segments:Array<ToolSegment>, gaps:Array<Float>) {
		if (segments == null || segments.length == 0 || gaps == null || gaps.length != segments.length - 1)
			throw "Tool row needs segments and one gap between each adjacent pair";
		pieces = segments.copy();
		spaces = gaps.copy();
		if (pieces[0] == null) throw "Tool row has a null segment";
		profile = pieces[0].profile;
		for (segment in pieces) if (segment == null || !ToolProfiles.sameSpecification(segment.profile, profile))
			throw "Tool row segments need the same contact profile";
		for (gap in spaces) if (!Math.isFinite(gap) || gap < 0) throw "Tool row gap must be finite and nonnegative";
		if (!Math.isFinite(span)) throw "Tool row exceeds the finite bed range";
	}

	public function segments():Array<ToolSegment> return pieces.copy();
	public function gaps():Array<Float> return spaces.copy();

	function get_length():Float {
		var result = 0.0;
		for (segment in pieces) result += segment.length;
		return result;
	}

	function get_span():Float {
		var result = length;
		for (gap in spaces) result += gap;
		return result;
	}

	public function placements(offset:Float):Array<ToolSegmentPlacement> {
		if (!Math.isFinite(offset)) throw "Tool row needs a finite bed offset";
		var result:Array<ToolSegmentPlacement> = [];
		var position = offset;
		for (i in 0...pieces.length) {
			var placed = new ToolSegmentPlacement(pieces[i], position);
			result.push(placed);
			position = placed.end;
			if (i < spaces.length) position += spaces[i];
		}
		return result;
	}
}
