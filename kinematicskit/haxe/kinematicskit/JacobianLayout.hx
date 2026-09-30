package kinematicskit;

/**
 * The columns of a solve: one per active DOF, then one block per moving root
 * (`RootMotion`: 3 columns for `Planar` — vx, vy, ωz — or 6 for `Floating` —
 * v then ω), all in world coordinates, the twist taken about the root's
 * origin. Tasks write Jacobian rows `width` wide; DOFs outside the layout get
 * no column, so an assembly with hundreds of joints solving a three-DOF
 * linkage works with three columns.
 */
class JacobianLayout {
  /** The DOF of each of the first `dofs.length` columns. */
  public final dofs:Array<Int>;
  /** The column of each model DOF, or -1 when it is not solved for. */
  public final columnOfDof:Array<Int>;
  /** Moving roots: body, mode and first column, one entry per block. */
  public final rootBodies:Array<Int>;
  public final rootModes:Array<RootMotion>;
  public final rootColumns:Array<Int>;
  /** The block of each root body, or -1 when it does not move. */
  public final blockOfRoot:Array<Int>;
  public final width:Int;

  public function new(model:KinematicModel, dofs:Array<Int>, ?rootBodies:Array<Int>, ?rootModes:Array<RootMotion>) {
    this.dofs = dofs.copy();
    columnOfDof = [for (_ in 0...model.dofCount()) -1];
    for (column in 0...dofs.length) columnOfDof[dofs[column]] = column;
    this.rootBodies = rootBodies == null ? [] : rootBodies.copy();
    this.rootModes = rootModes == null ? [] : rootModes.copy();
    blockOfRoot = [for (_ in 0...model.bodyCount()) -1];
    rootColumns = [];
    var next = dofs.length;
    for (block in 0...this.rootBodies.length) {
      blockOfRoot[this.rootBodies[block]] = block;
      rootColumns.push(next);
      next += columnsFor(this.rootModes[block]);
    }
    width = next;
  }

  public static function all(model:KinematicModel):JacobianLayout
    return new JacobianLayout(model, [for (dof in 0...model.dofCount()) dof]);

  public static function columnsFor(mode:RootMotion):Int
    return mode == RootMotion.Planar ? 3 : mode == RootMotion.Floating ? 6 : 0;
}
