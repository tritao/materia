package machinekit.pressbrake.planning;

import cadkit.sheetmetal.SheetDiagnosis;

/** Material pieces belonging to one physical body, with bounded boundary deformation in mm.
 * The authored-material correspondence must preserve interior ownership: points farther than
 * error from every piece face remain material. A Hausdorff bound without that correspondence
 * is insufficient. Wall chord, bend mapping and tool-profile builders supply this contract.
 */
class PressCollisionBody {
	public final owner:PressCollisionOwner;
	public final error:Float;
	final solids:Array<ConvexPrism>;
	public function new(owner:PressCollisionOwner, pieces:Array<ConvexPrism>, error:Float) {
		if (owner == null || pieces == null || !Math.isFinite(error) || error < 0)
			throw new SheetDiagnosis("press.collision-body", "Collision ownership, pieces and finite nonnegative error are required");
		for (piece in pieces) if (piece == null)
			throw new SheetDiagnosis("press.collision-body", "Collision body has a null material piece");
		this.owner = owner; this.error = error; solids = pieces.copy();
	}
	public function pieces():Array<ConvexPrism> return solids.copy();
}
