package machinekit.pressbrake;

import cadkit.sheetmetal.process.ProcessTooling;

/** Paired punch and die rows at one bed offset. Each row may have its own segmentation and gaps. */
class ToolStation {
	public final id:String;
	public final punches:ToolRow;
	public final dies:ToolRow;
	public final offset:Float;
	public var length(get, never):Float;
	public var span(get, never):Float;
	public var end(get, never):Float;

	public function new(id:String, punches:ToolRow, dies:ToolRow, offset:Float) {
		if (id == null || id.length == 0 || punches == null || dies == null) throw "Tool station needs an identity and both rows";
		if (!ToolProfiles.isPunch(punches.profile) || ToolProfiles.isPunch(dies.profile))
			throw 'Tool station "$id" needs a punch row above a die row';
		if (!Math.isFinite(offset) || offset < 0) throw 'Tool station "$id" needs a nonnegative bed offset';
		this.id = id;
		this.punches = punches;
		this.dies = dies;
		this.offset = offset;
		if (!Math.isFinite(end)) throw 'Tool station "$id" exceeds the finite bed range';
	}

	/** Punch material length; die material length remains available from the die row. */
	function get_length():Float return punches.length;
	function get_span():Float return Math.max(punches.span, dies.span);
	function get_end():Float return offset + span;

	/** Derive contact dimensions from the installed parts whenever a process is rebuilt. */
	public function processTooling():ProcessTooling {
		var punch = switch punches.profile {
			case Punch(p): p;
			case _: throw 'Tool station "$id" has no punch profile';
		};
		var groove = switch dies.profile {
			case SingleV(p): p.groove;
			case FourV(p, face): p.groove(face);
			case _: throw 'Tool station "$id" has no die profile';
		};
		return new ProcessTooling(punch.id, ToolProfiles.identity(dies.profile), punch.tipRadius, punch.includedAngle,
			groove.vWidth, groove.includedAngle, groove.shoulderRadius);
	}
}
