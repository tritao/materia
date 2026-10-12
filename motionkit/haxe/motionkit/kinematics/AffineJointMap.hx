package motionkit.kinematics;

/**
  What a solver declares when its joint positions are an exact affine function of the tool position (a Cartesian,
  CoreXY or gantry machine): `joint[j] = offsets[j] + columns[0][j] * x + columns[1][j] * y + columns[2][j] * z`, with
  no tolerance of its own. The executed position then differs from the path's only by the lowering tolerance, so a
  planner may check the path where it is sampled and may bound a piece of it in closed form from its direction.
  Joints the position does not move (an extruder driven by path distance, the motor behind it) have zero columns.
**/
class AffineJointMap {
  /** The joints' positions with the tool at the origin. */
  public final offsets:Array<Float>;
  /** Per tool coordinate (x, y, z), each joint's position per metre. */
  public final columns:Array<Array<Float>>;

  public function new(offsets:Array<Float>, columns:Array<Array<Float>>) {
    if (offsets == null || columns == null || columns.length != 3)
      throw "An affine joint map needs offsets and three columns";
    for (column in columns) if (column.length != offsets.length)
      throw "An affine joint map's columns must cover every joint";
    this.offsets = offsets.copy();
    this.columns = [for (column in columns) column.copy()];
  }

  public function jointCount():Int return offsets.length;

  /** The rate of joint `joint` for a tool velocity. */
  public function rate(joint:Int, vx:Float, vy:Float, vz:Float):Float
    return columns[0][joint] * vx + columns[1][joint] * vy + columns[2][joint] * vz;
}
