package machinekit.structural;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.structural.FrameAssembly.CutListLine;
import machinekit.welding.Weldment;
import machinekit.welding.Weldment.WeldmentSeams;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** A place the frame's design gives a part that mounts on it: a connector frame in the frame's coordinates, its +Z out of the frame. */
typedef FrameSeat = {
	var name:String;
	var frame:AssemblyFrame;
}

/** Two members welded together, with the fillet leg in millimetres. */
typedef FrameWeld = {
	var a:String;
	var b:String;
	var legSize:Float;
}

/**
 * A welded structural frame: members of a `FrameAssembly` cut and welded into one rigid part, in the frame's own
 * coordinates, carrying the seats its design gives the parts that mount on it. A weldment is one body, so it is one
 * member of a machine, not a tree of tubes pretending to hold each other up.
 *
 * Its geometry is the members' union and its stock `cutList()`. Its welds are declared, and `seams()` finds each one in
 * the members' geometry (`Weldment`), so a weld declared between members that do not meet is reported, not drawn.
 */
class WeldedFrame extends MachineComponent {
	public final frame:FrameAssembly;
	public final members:Array<String>;
	public final seats:Array<FrameSeat>;
	public final welds:Array<FrameWeld>;

	public function new(name:String, frame:FrameAssembly, members:Array<String>, seats:Array<FrameSeat>, welds:Array<FrameWeld>,
			material:String = "steel") {
		if (members.length == 0) throw 'Welded frame "$name" needs members';
		for (weld in welds) for (id in [weld.a, weld.b]) if (members.indexOf(id) < 0) throw 'Weld of "$name" names "$id", which is not a member';
		super('WELDED-FRAME-${name.toUpperCase().split(" ").join("-")}', 'Welded frame: $name', material, true);
		this.frame = frame;
		this.members = members.copy();
		this.seats = seats.copy();
		this.welds = welds.copy();
		for (seat in seats) addConnector(seat.name, Mount, seat.frame);
	}

	public function cutList():Array<CutListLine> return frame.cutList();

	/** The welds as a `Weldment` of the frame's members, its first member the reference. */
	public function weldment():Weldment {
		var result = new Weldment(members[0], members);
		for (weld in welds) result.join(weld.a, weld.b, weld.legSize);
		return result;
	}

	/** The seams of every declared weld, found in the members' geometry, in the frame's coordinates. */
	public function seams():WeldmentSeams {
		var parts = new Map<String, Part>();
		try {
			for (name in members) parts.set(name, frame.geometry(name));
			var found = weldment().find(parts);
			for (part in parts) part.close();
			return found;
		} catch (error:Dynamic) {
			for (part in parts) part.close();
			throw error;
		}
	}

	/** The box of member `name`, in the frame's coordinates: the part of the frame something is bolted to. */
	public function memberBounds(name:String):{min:Array<Float>, max:Array<Float>} {
		if (members.indexOf(name) < 0) throw 'Welded frame has no member "$name"';
		var part = frame.geometry(name);
		var bounds = part.shape.bounds(), low = bounds.get_min(), high = bounds.get_max();
		part.close();
		return {min: [low.get_x(), low.get_y(), low.get_z()], max: [high.get_x(), high.get_y(), high.get_z()]};
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var owned:Array<Part> = [];
		return Solids.building(owned, tracked -> {
			var parts:Array<Part> = [];
			for (name in members) {
				var part = frame.geometry(name);
				tracked.push(part);
				parts.push(part);
			}
			var result = Solids.union(parts);
			tracked.push(result);
			return result;
		});
	}
}
