package machinekit.structural;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Where something mounts on a plate: a connector frame and the through-holes it bolts through, in plate coordinates. */
typedef PlateSeat = {
	var name:String;
	var frame:AssemblyFrame;
	var holes:Array<{x:Float, y:Float, diameter:Float}>;
}

/**
 * A flat plate lying on its base (z = 0), centred on its origin. It owns the layout of what mounts on it: each seat is a
 * connector and the holes for its fasteners, so moving a seat moves both, and a part mounted on a seat sits where the seat
 * says (its own connector mated to the seat), never the other way round. Slots (centre x, centre y, length along X,
 * width along Y) are cut clean through.
 */
class SeatedPlate extends MachineComponent {
	public final length:Float;
	public final width:Float;
	public final thickness:Float;
	public final slots:Array<{x:Float, y:Float, length:Float, width:Float}>;
	public final seats:Array<PlateSeat>;

	public function new(length:Float, width:Float, thickness:Float, name:String, seats:Array<PlateSeat>,
			?slots:Array<{x:Float, y:Float, length:Float, width:Float}>, material:String = "aluminium 6061") {
		if (!(length > 0) || !(width > 0) || !(thickness > 0)) throw "A seated plate needs positive dimensions";
		var slotList = slots == null ? [] : slots;
		this.slots = slotList;
		this.seats = seats;
		super('${name.toUpperCase()}-${Dimension.format(length)}x${Dimension.format(width)}x${Dimension.format(thickness)}' + layoutTag(seats, slotList),
			name, material, true);
		this.length = length;
		this.width = width;
		this.thickness = thickness;
		for (seat in seats) addConnector(seat.name, Mount, seat.frame);
	}

	/**
	 * A short tag for the plate's layout: plates of one size whose seats, holes or slots differ are different parts (their
	 * holes are geometry, their seats connectors), so they must not share a designation and with it a definition.
	 */
	static function layoutTag(seats:Array<PlateSeat>, slots:Array<{x:Float, y:Float, length:Float, width:Float}>):String {
		if (seats.length == 0 && slots.length == 0) return "";
		var text = new StringBuf();
		function number(value:Float):Void text.add(Std.string(Math.round(value * 1000)) + ",");
		for (seat in seats) {
			text.add(seat.name + ":");
			for (value in [seat.frame.x, seat.frame.y, seat.frame.z, seat.frame.qx, seat.frame.qy, seat.frame.qz, seat.frame.qw]) number(value);
			for (hole in seat.holes) for (value in [hole.x, hole.y, hole.diameter]) number(value);
			text.add(";");
		}
		for (slot in slots) for (value in [slot.x, slot.y, slot.length, slot.width]) number(value);
		// FNV-1a, 32 bits: stable across runs and targets.
		var hash = 0x811c9dc5;
		var source = text.toString();
		for (index in 0...source.length) {
			hash = hash ^ (StringTools.fastCodeAt(source, index) & 0xff);
			hash = hash * 0x01000193;
		}
		return "-S" + StringTools.hex(hash, 8);
	}

	/** Seat frame on the top face at (x, y), facing up. */
	public static function onTop(x:Float, y:Float, thickness:Float):AssemblyFrame return Solids.axial(x, y, thickness);

	/** Seat frame on the underside at (x, y), facing up like the parts hung from it. */
	public static function underneath(x:Float, y:Float):AssemblyFrame return Solids.axial(x, y, 0);

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var plate = Solids.named(Part.box(length, width, thickness), "plate");
		if (detail == Envelope) return plate;
		var tools = [for (slot in slots) Part.box(slot.length, slot.width, thickness + 2).translated(new Vector(slot.x, slot.y, -1))];
		for (seat in seats) for (hole in seat.holes) tools.push(Part.cylinderSpan(hole.diameter / 2, -1, thickness + 1, hole.x, hole.y));
		return Solids.cut(plate, tools);
	}
}
