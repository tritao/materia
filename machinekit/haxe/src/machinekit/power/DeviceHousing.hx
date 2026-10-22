package machinekit.power;

import cadkit.modeling.Align;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentView;
import machinekit.component.Solids;

/** What a panel module carries: heat-sink fins on its front, screw terminals along its top end, vent slots. */
typedef PanelModuleOptions = {
	@:optional var fins:Int;
	@:optional var terminals:Int;
	@:optional var vents:Bool;
}

/** What a standing enclosure carries: a door or a keypad on its front, rear fins, side louvres, feet. */
typedef EnclosureOptions = {
	@:optional var door:Bool;
	@:optional var keypad:Bool;
	@:optional var fins:Int;
	@:optional var louvres:Bool;
	@:optional var feet:Bool;
}

/**
 * Housings of purchased electrical equipment, each inside its envelope box, so preview detail never moves
 * clearances or masses (they come from `EffectiveDensities` or the envelope). Two frames:
 *
 * - **Panel module** (drives, supplies, I/O modules): x across (`width`, centred), y up the panel (`height`,
 *   centred), z out of the mounting plate from 0 to `depth`.
 * - **Standing enclosure** (VFDs, controllers): x across (`width`, centred), y through it (`depth`, centred, front
 *   at -y), z up from 0 to `height`.
 */
class DeviceHousing {
	/** A panel module seen from its front: +Y up, its face (+Z) towards the viewer. */
	public static function panelView():ComponentView
		return new ComponentView([0, 1, 0], [0, 0, 1]);

	/**
	 * A panel module: a base plate on the mounting face, a body with chamfered front edges, and on its front either
	 * a heat-sink of `fins` ribs running up the panel or a cover with `vents` slots; `terminals` screw terminals sit
	 * in a stepped-down strip along its top end.
	 */
	public static function panelModule(width:Float, height:Float, depth:Float, ?options:PanelModuleOptions):Part {
		if (!(width > 0) || !(height > 0) || !(depth > 0)) throw "Panel module needs a positive envelope";
		var fins = options == null || options.fins == null ? 0 : options.fins;
		var terminals = options == null || options.terminals == null ? 0 : options.terminals;
		var vents = options != null && options.vents == true;
		var plate = Math.min(depth * 0.12, 3.0), inset = Math.min(width, height) * 0.03, c = Math.min(width, depth) * 0.06;
		var strip = terminals > 0 ? Math.min(height * 0.18, 14.0) : 0.0, step = depth * 0.35;
		var top = height / 2 - strip, w = width - 2 * inset, front = fins > 0 ? depth * 0.7 : depth;
		return Solids.building([], owned -> {
			function keep(part:Part, tag:String):Part {
				var named = Solids.named(part, tag);
				owned.push(named);
				return named;
			}
			var pieces = [
				keep(Part.box(width, height, plate, Center, Center, Min), "base-plate"),
				keep(Part.prism([new Vector(-w / 2, plate), new Vector(w / 2, plate), new Vector(w / 2, front - c), new Vector(w / 2 - c, front),
					new Vector(-w / 2 + c, front), new Vector(-w / 2, front - c)], -top, height / 2 - inset).placed(yzSwap()), "body")
			];
			if (terminals > 0)
				pieces.push(keep(Part.box(w, strip - inset, depth - step, Center, Min, Min).translated(new Vector(0, top, 0)), "terminal-strip"));
			if (fins > 0) {
				var pitch = w / fins, rib = Math.max(0.8, pitch * 0.35);
				for (index in 0...fins)
					pieces.push(keep(Part.box(rib, top - (-height / 2 + inset), depth - front + 0.01, Center, Min, Min)
						.translated(new Vector(-w / 2 + pitch * (index + 0.5), -height / 2 + inset, front - 0.01)), "fin"));
			}
			var body = Solids.union(pieces);
			var tools:Array<Part> = [];
			if (terminals > 0) {
				var pitch = w / terminals, screw = Math.min(pitch * 0.32, 2.5), face = depth - step;
				for (index in 0...terminals)
					tools.push(keep(Part.cylinderSpan(screw, face - 2, face + 0.1, -w / 2 + pitch * (index + 0.5), top + (strip - inset) / 2), "terminal"));
			}
			if (vents && fins == 0) {
				var rows = Std.int(Math.max(1, Math.min(6, (top + height / 2) / 12))), slot = Math.min(w * 0.6, 60.0);
				for (row in 0...rows) {
					var y = -height / 2 + inset + (top + height / 2 - inset) * (row + 1) / (rows + 1);
					tools.push(keep(Part.box(slot, 1.6, 0.6, Center, Center, Min).translated(new Vector(0, y, front - 0.5)), "vent"));
				}
			}
			return Solids.cut(body, tools);
		});
	}

	/**
	 * A standing enclosure: a body on optional feet, a front door with its seam and handle or a recessed keypad and
	 * display, side louvres, and `fins` heat-sink ribs across its back.
	 */
	public static function enclosure(width:Float, depth:Float, height:Float, ?options:EnclosureOptions):Part {
		if (!(width > 0) || !(depth > 0) || !(height > 0)) throw "Enclosure needs a positive envelope";
		var door = options != null && options.door == true, keypad = options != null && options.keypad == true;
		var louvres = options != null && options.louvres == true, feet = options != null && options.feet == true;
		var fins = options == null || options.fins == null ? 0 : options.fins;
		var foot = feet ? Math.min(height * 0.04, 25.0) : 0.0, finDepth = fins > 0 ? Math.min(depth * 0.18, 30.0) : 0.0;
		var front = -depth / 2, back = depth / 2 - finDepth, c = Math.min(width, depth) * 0.02;
		// With a door, the body's face sits 1.5 mm behind the envelope front so the handle stays inside it.
		var face = door ? front + 1.5 : front;
		return Solids.building([], owned -> {
			function keep(part:Part, tag:String):Part {
				var named = Solids.named(part, tag);
				owned.push(named);
				return named;
			}
			var pieces = [
				keep(Part.prism([new Vector(-width / 2 + c, face), new Vector(width / 2 - c, face), new Vector(width / 2, face + c),
					new Vector(width / 2, back), new Vector(-width / 2, back), new Vector(-width / 2, face + c)], foot, height), "body")
			];
			if (feet)
				for (x in [-1.0, 1.0]) for (y in [-1.0, 1.0])
					pieces.push(keep(Part.cylinderSpan(Math.min(width, depth) * 0.06, 0, foot + 0.01, x * width * 0.38, y * (back - front) * 0.38 + (front + back) / 2), "foot"));
			if (fins > 0) {
				var pitch = width * 0.9 / fins, rib = Math.max(1.0, pitch * 0.3);
				for (index in 0...fins)
					pieces.push(keep(Part.box(rib, finDepth + 0.01, (height - foot) * 0.8, Center, Min, Min)
						.translated(new Vector(-width * 0.45 + pitch * (index + 0.5), back - 0.01, foot + (height - foot) * 0.1)), "fin"));
			}
			if (door) {
				var handle = Math.min(width * 0.04, 20.0);
				pieces.push(keep(Part.box(handle, face - front + 0.01, (height - foot) * 0.18, Center, Min, Min)
					.translated(new Vector(width * 0.36, front, foot + (height - foot) * 0.45)), "handle"));
			}
			var body = Solids.union(pieces);
			var tools:Array<Part> = [];
			var inner = foot + (height - foot) * 0.04;
			if (door) {
				// The door seam: a 1 mm groove round the door, 0.8 mm deep.
				var dw = width * 0.92, dh = (height - foot) * 0.92, z0 = foot + (height - foot) * 0.04, depthIn = 0.8;
				for (strip in [
					{x: 0.0, z: z0, w: dw, h: 1.0}, {x: 0.0, z: z0 + dh - 1, w: dw, h: 1.0},
					{x: -dw / 2 + 0.5, z: z0, w: 1.0, h: dh}, {x: dw / 2 - 0.5, z: z0, w: 1.0, h: dh}
				])
					tools.push(keep(Part.box(strip.w, depthIn + 0.1, strip.h, Center, Min, Min).translated(new Vector(strip.x, face - 0.1, strip.z)), "door-seam"));
			}
			if (keypad) {
				var kw = Math.min(width * 0.45, 70.0), kh = Math.min((height - foot) * 0.22, 90.0), kz = foot + (height - foot) * 0.62;
				tools.push(keep(Part.box(kw, 2.0, kh, Center, Min, Min).translated(new Vector(0, front - 0.1, kz)), "keypad"));
				tools.push(keep(Part.box(kw * 0.8, 1.0, kh * 0.3, Center, Min, Min).translated(new Vector(0, front + 1.8, kz + kh * 0.62)), "display"));
			}
			if (louvres) {
				var rows = Std.int(Math.max(2, Math.min(8, (height - foot) * 0.3 / 10))), slot = Math.min((back - front) * 0.5, 80.0);
				for (side in [-1.0, 1.0]) for (row in 0...rows)
					tools.push(keep(Part.box(1.2, slot, 2.5, Center, Center, Min)
						.translated(new Vector(side * (width / 2 - 0.5), (front + back) / 2, inner + row * 7.0)), "louvre"));
			}
			return Solids.cut(body, tools);
		});
	}

	/**
	 * An aluminium-housed braking resistor: a finned extrusion along y on a base with slotted mounting tabs at both
	 * ends, and a cable gland boss. Frame x across (`width`, centred), y along (`length`, centred), z up from 0.
	 */
	public static function finnedResistor(width:Float, length:Float, height:Float):Part {
		if (!(width > 0) || !(length > 0) || !(height > 0)) throw "Resistor needs a positive envelope";
		var base = height * 0.15, core = width * 0.55, fins = 6, tab = Math.min(length * 0.1, 20.0), body = length - 2 * tab;
		return Solids.building([], owned -> {
			function keep(part:Part, tag:String):Part {
				var named = Solids.named(part, tag);
				owned.push(named);
				return named;
			}
			var pieces = [
				keep(Part.box(width, length, base, Center, Center, Min), "base"),
				keep(Part.box(core, body, height * 0.75, Center, Center, Min), "core")
			];
			var pitch = width / fins, rib = Math.max(0.8, pitch * 0.3);
			for (index in 0...fins)
				pieces.push(keep(Part.box(rib, body, height - 0.01, Center, Center, Min).translated(new Vector(-width / 2 + pitch * (index + 0.5), 0, 0)), "fin"));
			var housing = Solids.union(pieces);
			var tools = [
				for (end in [-1.0, 1.0])
					keep(Part.box(Math.min(width * 0.2, 6.0), tab * 0.6, base + 0.2, Center, Center, Min)
						.translated(new Vector(0, end * (length / 2 - tab / 2), -0.1)), "mounting-slot")
			];
			return Solids.cut(housing, tools);
		});
	}

	/** Places a prism drawn in (x, depth) and extruded along z so that depth runs along +z and the extrusion along -y. */
	static function yzSwap():cadkit.modeling.Location
		return new cadkit.modeling.Location(new cadkit.modeling.Plane(new Vector(0, 0, 0), Vector.X(), new Vector(0, -1, 0)));
}
