package cadkit.sheetmetal;

import cadkit.sheetmetal.FlatDomain.FlatOwner;

/** One mapped material patch retains its graph owner for design findings. */
class FlatDomain {
	public final owner:FlatOwner;
	public final outline:WallOutline;
	public function new(owner:FlatOwner, outline:WallOutline) { this.owner = owner; this.outline = outline; }
	public function identity():String { return switch (owner) { case Wall(id): "wall:" + id; case Bend(id): "bend:" + id; }; }
}

/** Transient graph ownership, without native-name parsing or contour indices. */
enum FlatOwner { Wall(id:Int); Bend(id:Int); }
