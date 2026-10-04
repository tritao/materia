package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import materia.assembly.AssemblyFrames;

/** The five cast bodies of the bench mill; origins are centred on their bottom faces. */
enum MillCastingKind {
	Base;
	Column;
	Saddle;
	Table;
	Head;
}

/** Machined cast-iron bodies with rail seats, screw clearances and mounting holes.
 * Dimensions describe this example's designed bodies, rather than a vendor catalogue.
 */
class MillCasting extends MachineComponent {
	public final kind:MillCastingKind;
	public final width:Float;
	public final depth:Float;
	public final height:Float;
	public final railSpacing:Float;
	public final screwHeight:Float;
	public static inline var SLOT_WIDTH:Float = 10;
	public static inline var SLOT_SPACING:Float = 40;

	public function new(kind:MillCastingKind) {
		super('MILL-${Std.string(kind).toUpperCase()}', 'Bench mill ${Std.string(kind).toLowerCase()}', "cast iron");
		this.kind = kind;
		var dims = switch kind {
			case Base: [520.0, 550, 80, 140];
			case Column: [180.0, 120, 600, 110];
			case Saddle: [400.0, 130, 45, 90];
			case Table: [400.0, 130, 40, 90];
			case Head: [180.0, 220, 120, 110];
		};
		width = dims[0]; depth = dims[1]; height = dims[2]; railSpacing = dims[3];
		screwHeight = height / 2;
		addConnector("base", Mount, Solids.axial(0, 0, 0));
		addConnector("top", Face, Solids.axial(0, 0, height));
		addConnector("nut", Mount, AssemblyFrames.alongY(0, 0, screwHeight, 0, 1, 0));
		addConnector("motor", Mount, AssemblyFrames.alongY(0, -depth / 2, screwHeight, 0, -1, 0));
		for (side in [-1, 1]) {
			var name = side < 0 ? "Left" : "Right";
			if (kind == Column) addConnector('rail$name', Mount,
				AssemblyFrames.alongY(side * railSpacing / 2, -depth / 2, 0, 0, -1, 0));
			else if (kind == Saddle) addConnector('rail$name', Mount, Solids.axial(0, side * railSpacing / 2, height));
			else addConnector('rail$name', Mount, Solids.axial(side * railSpacing / 2, 0, height));
		}
		if (kind == Base) addConnector("column", Mount, Solids.axial(0, depth / 2 - new MillColumn().depth / 2, height));
		if (kind == Head) addConnector("spindle", Axis, Solids.axial(0, -depth / 2 + SpindleCartridge.DIAMETER / 2 + 10, 0));
	}

	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body:Part;
		var pad = 4.0;
		if (kind == Base || kind == Saddle) {
			var pieces = [Solids.named(Part.box(width, depth, height - pad), "casting")];
			for (side in [-1, 1]) {
				var railPad = kind == Saddle ? Part.box(width, 30, pad)
					.translated(new Vector(0, side * railSpacing / 2, height - pad)) : Part.box(30, depth, pad)
					.translated(new Vector(side * railSpacing / 2, 0, height - pad));
				pieces.push(Solids.named(railPad, side < 0 ? "rail-pad-left" : "rail-pad-right"));
			}
			if (kind == Base) {
				var column = new MillColumn();
				pieces.push(Solids.named(Part.box(column.width, column.depth, pad)
					.translated(new Vector(0, depth / 2 - column.depth / 2, height - pad)), "column-pad"));
				pieces.push(Solids.named(Part.box(machinekit.motion.ScrewSupportUnit.bk12().width, depth, pad)
					.translated(new Vector(0, 0, height - pad)), "screw-support-pad"));
			}
			body = Solids.union(pieces);
		} else if (kind == Column) {
			var pieces = [Solids.named(Part.box(width, depth - pad, height)
				.translated(new Vector(0, pad / 2, 0)), "casting")];
			for (side in [-1, 1]) pieces.push(Solids.named(Part.box(30, pad, height)
				.translated(new Vector(side * railSpacing / 2, -depth / 2 + pad / 2, 0)),
				side < 0 ? "rail-pad-left" : "rail-pad-right"));
			body = Solids.union(pieces);
		} else body = Solids.named(Part.box(width, depth, height), "casting");
		if (detail == Envelope) return body;
		var tools:Array<Part> = [];
		if (kind == Base) tools.push(Solids.named(Part.box(width - 50, depth - 50, height - 20)
			.translated(new Vector(0, 0, -1)), "underside-core"));
		if (kind == Column) tools.push(Solids.named(Part.box(width - 40, depth - 40, height - 50)
			.translated(new Vector(0, 0, 25)), "column-core"));
		// Clearance for the screw along the body; the rail pads remain above it.
		if (kind == Saddle) tools.push(Solids.named(Part.cylinderAlongY(new machinekit.motion.BallNut().flangeDiameter / 2 + 1, -depth / 2 - 1,
			depth / 2 + 1, 0, screwHeight), "screw-bore"));
		// The flanged X nut crosses the rail gap; recess both adjacent faces.
		if (kind == Saddle || kind == Table) {
			var nut = new machinekit.motion.BallNut();
			var block = machinekit.motion.LinearRailBlock.metric("HGR15");
			var recess = (nut.flangeDiameter + 2) / 2 - block.spec.blockHeight / 2;
			tools.push(Solids.named(Part.box(width + 2, nut.flangeDiameter + 2, recess + 1)
				.translated(new Vector(0, 0, kind == Saddle ? height - recess : -1)), "x-nut-clearance"));
		}
		if (kind == Head) tools.push(Solids.named(Part.cylinderSpan(SpindleCartridge.DIAMETER / 2, -1, height + 1,
			0, -depth / 2 + SpindleCartridge.DIAMETER / 2 + 10), "spindle-bore"));
		if (kind == Table) for (i in 0...3) {
			var y = (i - 1) * SLOT_SPACING;
			tools.push(Solids.named(Part.box(width + 2, SLOT_WIDTH, 9)
				.translated(new Vector(0, y, height - 8)), 'slot${i + 1}-mouth'));
			tools.push(Solids.named(Part.box(width + 2, 18, 6)
				.translated(new Vector(0, y, height - 14)), 'slot${i + 1}-foot'));
		}
		for (sx in [-1, 1]) for (sy in [-1, 1]) tools.push(Solids.named(
			Part.cylinderSpan(4.5, -1, height + 1, sx * (width / 2 - 20), sy * (depth / 2 - 20)),
			'mount-${sx < 0 ? "left" : "right"}-${sy < 0 ? "front" : "back"}'));
		return Solids.cut(body, tools);
	}

	static function parse(value:String):MillCastingKind return switch value {
		case "Base": Base;
		case "Column": Column;
		case "Saddle": Saddle;
		case "Table": Table;
		case "Head": Head;
		default: throw 'Unknown mill casting "$value"';
	};
	static var castingRecipe:Null<ComponentType>;
	public static function recipeType():ComponentType {
		if (castingRecipe == null) castingRecipe = new ComponentType("machinekit.milling.casting",
			[ComponentRecipeSupport.choice("kind", ["Base", "Column", "Saddle", "Table", "Head"], "Base")],
			v -> new MillCasting(parse(v.token("kind"))));
		return castingRecipe;
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function values():ComponentValues return new ComponentValues()
		.setToken("kind", Std.string(kind)).setToken("material", materialSpec());
}

class MillBase extends MillCasting { public function new() super(Base); }
class MillColumn extends MillCasting { public function new() super(Column); }
class MillSaddle extends MillCasting { public function new() super(Saddle); }
class MillTable extends MillCasting { public function new() super(Table); }
class MillHead extends MillCasting { public function new() super(Head); }
