package machinekit.structural;

import cadkit.modeling.Align;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.CollisionHullFacet;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** One block of a `BlockBracket`: its corners in the bracket's own frame, millimetres. */
typedef BracketBlock = {min:Array<Float>, max:Array<Float>};

/**
 * A small bracket made of rectangular blocks square to its own frame, welded or machined into one part: what holds a
 * switch or a sensor off the member it is bolted to. Its blocks are given in its own frame.
 */
class BlockBracket extends MachineComponent {
	public final blocks:Array<BracketBlock>;

	public function new(name:String, blocks:Array<BracketBlock>, material:String = "aluminium 6061") {
		if (blocks.length == 0) throw 'Bracket "$name" needs a block';
		for (block in blocks) for (i in 0...3) if (!(block.max[i] - block.min[i] > 0)) throw 'Bracket "$name" has an empty block';
		var key = [for (block in blocks) [for (i in 0...3) Dimension.format(block.min[i]) + ":" + Dimension.format(block.max[i])].join(",")].join(";");
		super('BRACKET-${name.toUpperCase().split(" ").join("-")}-${key}', name, material, true);
		this.blocks = [for (block in blocks) {min: block.min.copy(), max: block.max.copy()}];
		// Preserve the space inside an L-shaped support in simulation and planning.
		addFacet(CollisionHullFacet.fromBoxes([for (block in this.blocks)
			[block.min[0], block.min[1], block.min[2], block.max[0], block.max[1], block.max[2]]]));
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var parts = [for (block in blocks) Part.box(block.max[0] - block.min[0], block.max[1] - block.min[1], block.max[2] - block.min[2], Min, Min, Min)
			.translated(new Vector(block.min[0], block.min[1], block.min[2]))];
		return parts.length == 1 ? parts[0] : Solids.union(parts);
	}
}
