import machinekit.assembly.MachineAssembly;
import machinekit.milling.MillPanel;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

typedef MillBounds = {var low:Array<Float>; var high:Array<Float>;}

/** The tending mill: the bare machine on its stand, with a preset vise and convex enclosure panels. */
class EnclosedBenchMill extends MachineAssembly {
	public final mill:BenchMill;
	public final standHeight:Float = 750;
	public final trayThickness:Float = 2;
	public final panelThickness:Float = 2;
	public final openingWidth:Float = 450;
	public final openingHeight:Float = 400;
	public final doorStroke:Float = 460;
	public final envelope:MillBounds;
	public final opening:AssemblyFrame;
	public final loadPosition:Array<Float>;
	public final floorOffset:Float;
	public final panelIds:Array<String> = [];
	public final doorIds:Array<String> = [];
	final poses = new Map<String, AssemblyFrame>();

	public function new() {
		super();
		mill = new BenchMill(true);
		floorOffset = standHeight + trayThickness;
		envelope = sweptBounds(mill);
		var left = envelope.low[0] - 30;
		var right = Math.max(envelope.high[0] + 30, openingWidth / 2 + doorStroke + 20);
		var front = envelope.low[1] - 30;
		var back = envelope.high[1] + 30;
		var bottom = floorOffset;
		var top = floorOffset + envelope.high[2] + 30;
		var coreModel = new AssemblyModel("mm"); mill.addTo(coreModel, "");
		var retracted = new AssemblyState(coreModel.definition("mill-door-head"));
		retracted.setJoint("z", mill.specs[2].upper);
		var headBottom = retracted.worldPose("head").z;
		opening = AssemblyFrames.translation(0, front, floorOffset + headBottom - 10 - openingHeight);
		var width = right - left, depth = back - front;
		var centreX = (left + right) / 2, centreY = (front + back) / 2;
		panel("chipTray", width, depth, trayThickness, centreX, centreY, standHeight);
		include("mill", mill, AssemblyFrames.translation(0, 0, floorOffset));
		addMemberConnector("chipTray", "millSeat", AssemblyFrames.compose(AssemblyFrames.inverse(poses.get("chipTray")),
			AssemblyFrames.translation(0, 0, floorOffset)));
		addMemberConnector("mill/base", "standSeat", AssemblyFrames.identity());
		addMate("mill-on-stand", "fixed", "chipTray", "millSeat", "mill/base", "standSeat");
		// The stand deck and four legs are convex fabricated blocks.
		panel("standDeck", mill.base.width, mill.base.depth, 5, 0, 0, standHeight - 5, false);
		for (x in [-1, 1]) for (y in [-1, 1]) panel('standLeg${x + 1}${y + 1}', 30, 30, standHeight - 5,
			x * (mill.base.width / 2 - 30), y * (mill.base.depth / 2 - 30), 0, false);
		panel("enclosureLeft", panelThickness, depth, top - bottom, left - panelThickness / 2, centreY, bottom);
		panel("enclosureRight", panelThickness, depth, top - bottom, right + panelThickness / 2, centreY, bottom);
		panel("enclosureBack", width, panelThickness, top - bottom, centreX, back + panelThickness / 2, bottom);
		panel("enclosureRoof", width, depth, panelThickness, centreX, centreY, top);
		panel("frontLeft", -openingWidth / 2 - left, panelThickness, top - bottom,
			(left - openingWidth / 2) / 2, front, bottom);
		panel("frontRight", right - openingWidth / 2, panelThickness, top - bottom,
			(right + openingWidth / 2) / 2, front, bottom);
		panel("frontBottom", openingWidth, panelThickness, opening.z - bottom, 0, front, bottom);
		panel("frontTop", openingWidth, panelThickness, top - opening.z - openingHeight,
			0, front, opening.z + openingHeight);
		var rail = new MillPanel(doorStroke + openingWidth + 20, 20, 12, "aluminium 6061");
		var railPose = AssemblyFrames.translation(doorStroke / 2, front - 16, opening.z + openingHeight + 24);
		fixed("doorRail", rail, railPose, "chipTray");
		var sliderPose = AssemblyFrames.translation(0, railPose.y, railPose.z + rail.height);
		var slider = new MillPanel(40, 20, 16, "aluminium 6061");
		addComponent("doorSlider", slider, sliderPose);
		poses.set("doorSlider", sliderPose);
		addMemberConnector("doorRail", "slide", AssemblyFrames.compose(AssemblyFrames.inverse(railPose), sliderPose));
		addMemberConnector("doorSlider", "slide", AssemblyFrames.identity());
		addMateOnAxis("door", "prismatic", "doorRail", "slide", "doorSlider", "slide", {x: 1, y: 0, z: 0}, 0,
			{lower: 0, upper: doorStroke, velocity: null, effort: null, overtravel: 0});
		var frameWidth = 25.0, leafY = front - 10;
		doorPanel("doorLeft", frameWidth, 4, openingHeight, -(openingWidth - frameWidth) / 2, leafY, opening.z);
		doorPanel("doorRight", frameWidth, 4, openingHeight, (openingWidth - frameWidth) / 2, leafY, opening.z);
		doorPanel("doorBottom", openingWidth - 2 * frameWidth, 4, frameWidth, 0, leafY, opening.z);
		doorPanel("doorTop", openingWidth - 2 * frameWidth, 4, frameWidth, 0, leafY, opening.z + openingHeight - frameWidth);
		doorPanel("doorWindow", openingWidth - 2 * frameWidth, 4, openingHeight - 2 * frameWidth,
			0, leafY, opening.z + frameWidth, "polycarbonate");
		// A closed cabinet is six panels too; it keeps its real material mass.
		var cabinetX = right + 120, cabinetY = back - 100, cabinetZ = standHeight;
		panel("cabinetLeft", 2, 200, 300, cabinetX - 100, cabinetY, cabinetZ, false);
		panel("cabinetRight", 2, 200, 300, cabinetX + 100, cabinetY, cabinetZ, false);
		panel("cabinetFront", 200, 2, 300, cabinetX, cabinetY - 100, cabinetZ, false);
		panel("cabinetBack", 200, 2, 300, cabinetX, cabinetY + 100, cabinetZ, false);
		panel("cabinetBottom", 200, 200, 2, cabinetX, cabinetY, cabinetZ, false);
		panel("cabinetTop", 200, 200, 2, cabinetX, cabinetY, cabinetZ + 300, false);
		addMemberConnector("chipTray", "doorOpening", AssemblyFrames.compose(AssemblyFrames.inverse(poses.get("chipTray")), opening));
		exposeConnector("doorOpening", "chipTray", "doorOpening");
		loadPosition = nearestLoadPosition();
	}

	function panel(id:String, w:Float, d:Float, h:Float, x:Float, y:Float, z:Float, enclosure:Bool = true):Void {
		var pose = AssemblyFrames.translation(x, y, z), part = new MillPanel(w, d, h);
		if (id == "chipTray") { addComponent(id, part, pose); poses.set(id, pose); }
		else fixed(id, part, pose, "chipTray");
		if (enclosure) panelIds.push(id);
	}
	function doorPanel(id:String, w:Float, d:Float, h:Float, x:Float, y:Float, z:Float, material:String = "steel"):Void {
		fixed(id, new MillPanel(w, d, h, material), AssemblyFrames.translation(x, y, z), "doorSlider");
		doorIds.push(id);
	}
	function fixed(id:String, part:MillPanel, pose:AssemblyFrame, parent:String):Void {
		addComponent(id, part, pose); poses.set(id, pose);
		addMemberConnector(parent, '$id-seat', AssemblyFrames.compose(AssemblyFrames.inverse(poses.get(parent)), pose));
		addMemberConnector(id, "seat", AssemblyFrames.identity());
		addMate('$id-mount', "fixed", parent, '$id-seat', id, "seat");
	}
	public function state():AssemblyState {
		var model = new AssemblyModel("mm"); addTo(model, "");
		return new AssemblyState(model.definition("enclosed-bench-mill"));
	}

	/** Project the doorway onto the two table travels; retract Z to its geometric upper limit. */
	function nearestLoadPosition():Array<Float> {
		var state = state();
		for (spec in mill.specs) state.setJoint("mill/" + spec.id, 0);
		var zero = state.worldConnector("mill/vise/body", "datum");
		var result:Array<Float> = [];
		for (spec in mill.specs) {
			if (spec.id == "z") { result.push(spec.upper); continue; }
			state.setJoint("mill/" + spec.id, 1);
			var shifted = state.worldConnector("mill/vise/body", "datum");
			state.setJoint("mill/" + spec.id, 0);
			var dx = shifted.x - zero.x, dy = shifted.y - zero.y, dz = shifted.z - zero.z;
			var value = ((opening.x - zero.x) * dx + (opening.y - zero.y) * dy + (opening.z - zero.z) * dz) /
				(dx * dx + dy * dy + dz * dz);
			result.push(Math.max(spec.lower, Math.min(spec.upper, value)));
		}
		return result;
	}

	/** Local geometry is read once; FK supplies its bounds at every travel corner. */
	static function sweptBounds(mill:BenchMill):MillBounds {
		var model = new AssemblyModel("mm"); mill.addTo(model, "");
		var state = new AssemblyState(model.definition("mill-envelope"));
		var parts = [for (entry in mill.components()) {
			var part = entry.component.geometry(), bounds = part.shape.bounds();
			var lo = bounds.get_min(), hi = bounds.get_max();
			var record = {id: entry.id, low: [lo.get_x(), lo.get_y(), lo.get_z()], high: [hi.get_x(), hi.get_y(), hi.get_z()]};
			part.close(); record;
		}];
		var result:MillBounds = {low: [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY],
			high: [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY]};
		for (x in [mill.specs[0].lower, mill.specs[0].upper]) for (y in [mill.specs[1].lower, mill.specs[1].upper])
			for (z in [mill.specs[2].lower, mill.specs[2].upper]) {
				state.setJoint("x", x); state.setJoint("y", y); state.setJoint("z", z);
				for (part in parts) {
					var pose = state.worldPose(part.id);
					for (x in [part.low[0], part.high[0]]) for (y in [part.low[1], part.high[1]]) for (z in [part.low[2], part.high[2]]) {
						var p = AssemblyFrames.transformPoint(pose, x, y, z), values = [p.x, p.y, p.z];
						for (axis in 0...3) { result.low[axis] = Math.min(result.low[axis], values[axis]);
							result.high[axis] = Math.max(result.high[axis], values[axis]); }
					}
				}
			}
		return result;
	}
}
