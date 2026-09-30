package kinematicskit;

/**
 * Which model DOFs a solve moves and the Jacobian column each one owns.
 * Tasks write Jacobian rows `width` wide; DOFs outside the layout get no
 * column at all, so an assembly with hundreds of joints solving a
 * three-DOF linkage works with three columns.
 */
class JacobianLayout {
  /** The DOF of each column. */
  public final dofs:Array<Int>;
  /** The column of each model DOF, or -1 when it is not solved for. */
  public final columnOfDof:Array<Int>;
  public final width:Int;

  public function new(model:KinematicModel, dofs:Array<Int>) {
    this.dofs = dofs.copy();
    width = dofs.length;
    columnOfDof = [for (_ in 0...model.dofCount()) -1];
    for (column in 0...width) columnOfDof[dofs[column]] = column;
  }

  public static function all(model:KinematicModel):JacobianLayout
    return new JacobianLayout(model, [for (dof in 0...model.dofCount()) dof]);
}
