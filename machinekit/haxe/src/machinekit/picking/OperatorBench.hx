package machinekit.picking;

import cadkit.modeling.Align;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.structural.FrameAssembly;
import machinekit.structural.FrameAssembly.CutListLine;
import machinekit.structural.TSlotExtrusion;

/** Adjacent operator bench with an extrusion frame, worktop, and simple screen mount. */
class OperatorBench extends MachineComponent {
	public final width:Float;
	public final depth:Float;
	public final height:Float;
	public final worktopThickness:Float;
	final frame:TSlotExtrusion;
	final frameAssembly:FrameAssembly;
	final frameMembers:Array<String>;

	public function new(width:Float = 900, depth:Float = 600, height:Float = 850,
		worktopThickness:Float = 18) {
		if (!positive(width) || !positive(depth) || !positive(height) || !positive(worktopThickness))
			throw "Operator bench dimensions must be finite and positive";
		if (worktopThickness >= height) throw "Bench worktop is thicker than its height";
		this.width = width;
		this.depth = depth;
		this.height = height;
		this.worktopThickness = worktopThickness;
		frame = TSlotExtrusion.forProfile("HFS5-2020");
		frameAssembly = new FrameAssembly();
		frameMembers = [];
		buildFrame();
		super('OPERATOR-BENCH-${fmt(width)}x${fmt(depth)}',
			'Operator bench with extrusion frame, sheet worktop, and screen mount', "aluminium 6061");
	}

	public function frameCutList():Array<CutListLine> return frameAssembly.cutList();
	public function memberCuts():Array<{name:String, length:Float}>
		return [for (name in frameMembers) {name: name, length: frameAssembly.cutLength(name)}];

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var parts:Array<Part> = [for (name in frameMembers) frameAssembly.geometry(name)];
		parts.push(Part.box(width, depth, worktopThickness, Align.Center, Align.Center, Align.Min)
			.translated(new Vector(0, 0, height - worktopThickness)));
		if (detail == Envelope) return Solids.union(parts);
		var postHeight = 260.0;
		var post = Part.box(frame.size, frame.size, postHeight,
			Align.Center, Align.Center, Align.Min).translated(new Vector(0, depth / 2 - frame.size, height));
		var screenRail = Part.box(360, frame.size, frame.size, Align.Center, Align.Center, Align.Min)
			.translated(new Vector(0, depth / 2 - frame.size, height + postHeight - frame.size));
		var screen = Part.box(320, 35, 190, Align.Center, Align.Center, Align.Min)
			.translated(new Vector(0, depth / 2 - frame.size - 20, height + 50));
		parts.push(post); parts.push(screenRail); parts.push(screen);
		return Solids.union(parts);
	}

	function buildFrame():Void {
		var x = width / 2 - frame.size / 2, y = depth / 2 - frame.size / 2;
		for (corner in [
			{name: "front-left", x: -x, y: -y}, {name: "front-right", x: x, y: -y},
			{name: "back-left", x: -x, y: y}, {name: "back-right", x: x, y: y}]) {
			frameAssembly.point(corner.name + "-floor", corner.x, corner.y, 0);
			frameAssembly.point(corner.name + "-top", corner.x, corner.y, height - worktopThickness);
			addMember(corner.name, corner.name + "-floor", corner.name + "-top");
		}
		frameAssembly.point("front-left-rail", -x, -y, height - worktopThickness);
		frameAssembly.point("front-right-rail", x, -y, height - worktopThickness);
		frameAssembly.point("back-left-rail", -x, y, height - worktopThickness);
		frameAssembly.point("back-right-rail", x, y, height - worktopThickness);
		addMember("front-rail", "front-left-rail", "front-right-rail");
		addMember("back-rail", "back-left-rail", "back-right-rail");
		addMember("left-rail", "front-left-rail", "back-left-rail");
		addMember("right-rail", "front-right-rail", "back-right-rail");
	}

	function addMember(name:String, start:String, end:String):Void {
		frameAssembly.member(name, start, end, frame);
		frameMembers.push(name);
	}

	static function positive(value:Float):Bool return Math.isFinite(value) && value > 0;
	static function fmt(value:Float):String return Std.string(Math.round(value * 10) / 10);
}
