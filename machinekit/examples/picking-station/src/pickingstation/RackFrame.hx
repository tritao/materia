package pickingstation;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.structural.FrameAssembly;
import machinekit.structural.FrameAssembly.CutListLine;
import machinekit.structural.TSlotExtrusion;

/** Extrusion frame for the storage rack. Member stock is exposed through cutList(). */
class RackFrame extends MachineComponent {
	public final width:Float;
	public final depth:Float;
	public final height:Float;
	public final footHeight:Float;
	public final shelfCount:Int;
	public final shelfSpacing:Float;
	public final profile:TSlotExtrusion;
	final assembly:FrameAssembly;
	final memberNames:Array<String>;

	public function new(width:Float, depth:Float, height:Float, footHeight:Float = 25,
		shelfCount:Int = 2, shelfSpacing:Float = 300) {
		if (!positive(width) || !positive(depth) || !positive(height) || !positive(footHeight))
			throw "Rack frame dimensions must be finite and positive";
		this.width = width;
		this.depth = depth;
		this.height = height;
		this.footHeight = footHeight;
		this.shelfCount = shelfCount;
		this.shelfSpacing = shelfSpacing;
		profile = TSlotExtrusion.forProfile("HFS5-2020");
		assembly = new FrameAssembly();
		memberNames = [];
		buildFrame();
		super('RACK-FRAME-${fmt(width)}x${fmt(depth)}x${fmt(height)}',
			'2020 T-slot extrusion storage rack frame', "aluminium 6061");
	}

	public function cutList():Array<CutListLine> return assembly.cutList();
	public function memberCuts():Array<{name:String, length:Float}>
		return [for (name in memberNames) {name: name, length: assembly.cutLength(name)}];
	public function memberGeometry(name:String):Part return assembly.geometry(name);

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var ownedParts:Array<Part> = [];
		return Solids.building(ownedParts, tracked -> {
			var parts:Array<Part> = [];
			for (name in memberNames) {
				var part = assembly.geometry(name);
				tracked.push(part);
				parts.push(part);
			}
			var result = Solids.union(parts);
			tracked.push(result);
			return result;
		});
	}

	function buildFrame():Void {
		var x = width / 2 - profile.size / 2, y = depth / 2 - profile.size / 2;
		var innerX = width / 2 - profile.size, innerY = depth / 2 - profile.size;
		for (corner in [
			{name: "front-left", x: -x, y: -y}, {name: "front-right", x: x, y: -y},
			{name: "back-left", x: -x, y: y}, {name: "back-right", x: x, y: y}]) {
			assembly.point(corner.name + "-bottom", corner.x, corner.y, footHeight);
			assembly.point(corner.name + "-top", corner.x, corner.y, height);
			addMember(corner.name, corner.name + "-bottom", corner.name + "-top");
		}
		var levels:Array<Float> = [footHeight, height];
		for (index in 1...shelfCount + 1) levels.push(footHeight + shelfSpacing * index);
		for (levelIndex in 0...levels.length) {
			var z = levels[levelIndex];
			assembly.point('rail-left-$levelIndex', -innerX, -y, z);
			assembly.point('rail-right-$levelIndex', innerX, -y, z);
			assembly.point('rail-back-left-$levelIndex', -innerX, y, z);
			assembly.point('rail-back-right-$levelIndex', innerX, y, z);
			assembly.point('side-left-front-$levelIndex', -x, -innerY, z);
			assembly.point('side-left-back-$levelIndex', -x, innerY, z);
			assembly.point('side-right-front-$levelIndex', x, -innerY, z);
			assembly.point('side-right-back-$levelIndex', x, innerY, z);
			addMember('front-rail-$levelIndex', 'rail-left-$levelIndex', 'rail-right-$levelIndex');
			addMember('back-rail-$levelIndex', 'rail-back-left-$levelIndex', 'rail-back-right-$levelIndex');
			addMember('left-rail-$levelIndex', 'side-left-front-$levelIndex', 'side-left-back-$levelIndex');
			addMember('right-rail-$levelIndex', 'side-right-front-$levelIndex', 'side-right-back-$levelIndex');
		}
	}

	function addMember(name:String, start:String, end:String):Void {
		assembly.member(name, start, end, profile);
		memberNames.push(name);
	}

	static function positive(value:Float):Bool return Math.isFinite(value) && value > 0;
	static function fmt(value:Float):String return Std.string(Math.round(value * 10) / 10);
}
