package machinekit.structural;

import cadkit.modeling.Part;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.AxisBuilder;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.milling.MillPanel;
import machinekit.structural.FrameAssembly.CutListLine;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

typedef TableBox = {var id:String; var frame:AssemblyFrame; var width:Float; var depth:Float; var height:Float;}

/** Floor-standing, square-profile extrusion table. Dimensions are mm, top height includes feet
 * and worktop. Front inset moves the entire front support row toward the back. Hardware is
 * simplified: angle brackets and levelling feet have nominal geometry, without bolt holes. */
class TSlotTable extends MachineAssembly {
	public final width:Float;
	public final depth:Float;
	public final height:Float;
	public final profile:TSlotExtrusion;
	public final topThickness:Float;
	public final frontInset:Float;
	public final footHeight:Float;
	public final lowerRailHeight:Float;
	final frame:FrameAssembly = new FrameAssembly();
	final names:Array<String> = [];
	final boxes:Array<TableBox> = [];

	public function new(width:Float, depth:Float, height:Float, profileName:String = "HFS5-4040",
			topThickness:Float = 10, frontInset:Float = 0, footHeight:Float = 30, lowerRailHeight:Float = 220) {
		super();
		profile = TSlotExtrusion.forProfile(profileName);
		var s = profile.size;
		for (v in [width, depth, height, topThickness, footHeight, lowerRailHeight])
			if (!Math.isFinite(v) || v <= 0) throw "Table dimensions must be finite and positive";
		if (profile.height != s) throw "Table requires a square extrusion profile";
		if (!Math.isFinite(frontInset) || frontInset < 0 || width <= 4 * s || depth - frontInset <= 4 * s ||
				lowerRailHeight - 1.5 * s <= footHeight || lowerRailHeight + 2.5 * s >= height - topThickness)
			throw "Table needs room for legs, rails, brackets and feet";
		this.width = width; this.depth = depth; this.height = height;
		this.topThickness = topThickness; this.frontInset = frontInset;
		this.footHeight = footHeight; this.lowerRailHeight = lowerRailHeight;
		var x = width / 2 - s / 2;
		var front = -depth / 2 + s / 2 + frontInset, back = depth / 2 - s / 2;
		var top = height - topThickness;
		var upper = top - s / 2;
		for (ix in [-1, 1]) for (iy in [-1, 1]) {
			var px = ix * x, py = iy < 0 ? front : back;
			var id = 'leg-$ix-$iy';
			member(id, px, py, footHeight, px, py, top);
			var foot = new TableFoot(s, footHeight);
			addComponent('foot-$ix-$iy', foot, AssemblyFrames.translation(px, py, 0));
			box('foot-$ix-$iy', s, s, footHeight, px, py, 0);
		}
		for (level in [{id: "upper", z: upper}, {id: "lower", z: lowerRailHeight}]) {
			for (iy in [-1, 1]) {
				var py = iy < 0 ? front : back;
				member('${level.id}-cross-$iy', -x + s / 2, py, level.z, x - s / 2, py, level.z);
			}
			for (ix in [-1, 1])
				member('${level.id}-side-$ix', ix * x, front + s / 2, level.z, ix * x, back - s / 2, level.z);
			// Two angle brackets per corner, below the rail and against the leg's inner face.
			for (ix in [-1, 1]) for (iy in [-1, 1]) for (axis in 0...2) {
				var px = ix * x, py = iy < 0 ? front : back;
				var z = level.z - s / 2 - s;
				var bx = axis == 0 ? px - ix * s : px;
				var by = axis == 1 ? py - iy * s : py;
				var w = axis == 0 ? s : 6.0, d = axis == 1 ? s : 6.0;
				var minX = bx - w / 2, minY = by - d / 2;
				var blocks = [{min: [minX, minY, z + s - 6], max: [minX + w, minY + d, z + s]},
					{min: [axis == 0 && ix < 0 ? minX : axis == 0 ? minX + w - 6 : minX,
						axis == 1 && iy < 0 ? minY : axis == 1 ? minY + d - 6 : minY, z],
					 max: [axis == 0 && ix < 0 ? minX + 6 : minX + w,
						axis == 1 && iy < 0 ? minY + 6 : minY + d, z + s]}];
				var id = '${level.id}-bracket-$ix-$iy-$axis';
				addComponent(id, new BlockBracket("Table angle", blocks));
				for (i in 0...blocks.length) {
					var b = blocks[i];
					box('$id-$i', b.max[0] - b.min[0], b.max[1] - b.min[1], b.max[2] - b.min[2],
						(b.min[0] + b.max[0]) / 2, (b.min[1] + b.max[1]) / 2, b.min[2]);
				}
			}
		}
		addComponent("top", new MillPanel(width, depth, topThickness, "steel"), AssemblyFrames.translation(0, 0, top));
		box("top", width, depth, topThickness, 0, 0, top);
	}

	public function cutList():Array<CutListLine> return frame.cutList();
	public function memberCuts():Array<{name:String, length:Float}>
		return [for (name in names) {name: name, length: frame.cutLength(name)}];

	/** Conservative individual envelopes; slot cavities and foot-pad corners are filled. */
	public function collisionPieces():Array<{id:String, frame:AssemblyFrame, vertices:Array<Float>}> return [for (b in boxes) {
		var vertices:Array<Float> = [];
		for (x in [-b.width / 2, b.width / 2]) for (y in [-b.depth / 2, b.depth / 2]) for (z in [0.0, b.height]) {
			vertices.push(x); vertices.push(y); vertices.push(z);
		}
		{id: b.id, frame: AssemblyFrames.translation(b.frame.x, b.frame.y, b.frame.z), vertices: vertices};
	}];

	function member(id:String, x0:Float, y0:Float, z0:Float, x1:Float, y1:Float, z1:Float):Void {
		frame.point(id + "-start", x0, y0, z0); frame.point(id + "-end", x1, y1, z1);
		frame.member(id, id + "-start", id + "-end", profile); names.push(id);
		var length = frame.length(id);
		var along = [(x1-x0)/length, (y1-y0)/length, (z1-z0)/length];
		addComponent(id, new TableMember(profile, length), AxisBuilder.orient(x0,y0,z0,
			z0 == z1 ? [0.0,0.0,1.0] : [0.0,1.0,0.0], along));
		var s = profile.size;
		box(id, Math.abs(x1-x0) + (x0 == x1 ? s : 0), Math.abs(y1-y0) + (y0 == y1 ? s : 0),
			Math.abs(z1-z0) + (z0 == z1 ? s : 0), (x0+x1)/2, (y0+y1)/2, Math.min(z0,z1) - (z0 == z1 ? s/2 : 0));
	}
	function box(id:String, w:Float, d:Float, h:Float, x:Float, y:Float, z:Float):Void
		boxes.push({id:id, frame:AssemblyFrames.translation(x,y,z), width:w, depth:d, height:h});
}

private class TableMember extends MachineComponent {
	final profile:TSlotExtrusion;
	final length:Float;
	public function new(profile:TSlotExtrusion, length:Float) {
		super('TABLE-${profile.designation}-$length', "Table extrusion", "aluminium 6061", true);
		this.profile = profile; this.length = length;
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return detail == Envelope ? Part.box(profile.size, profile.height, length) : profile.geometry(length);
}

private class TableFoot extends MachineComponent {
	final size:Float;
	final height:Float;
	public function new(size:Float, height:Float) {
		super('TABLE-FOOT-$size-$height', "Nominal levelling foot with threaded stem", "steel", true);
		this.size = size; this.height = height;
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.union([Part.cylinderSpan(size / 2, 0, Math.min(8, height / 3)),
			Part.cylinderSpan(size / 8, Math.min(8, height / 3) / 2, height)]);
}
