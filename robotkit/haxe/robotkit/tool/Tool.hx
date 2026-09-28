package robotkit.tool;

import robotkit.spatial.Transform3;

/**
 * A tool mounted at a manipulator's flange. `flangeTTcp` places the tool
 * center point relative to the flange frame, per ARCHITECTURE.md's
 * `a_T_b` convention: `flange_T_tcp` maps coordinates in the tool's own
 * tip frame into the flange frame.
 */
class Tool {
  public final id:ToolId;
  public final name:String;
  public final flangeTTcp:Transform3;
  public final collision:ToolCollisionShape;
  public final mass:Float;
  /** Null when a legacy mass-only tool has no trustworthy centre of mass. */
  public final massProperties:Null<MassProperties>;

  public function new(id:ToolId, name:String, flangeTTcp:Transform3,
      ?collision:ToolCollisionShape, ?mass:Float = 0.0,
      ?massProperties:MassProperties) {
    if (id == null || id.length == 0) throw "Tool requires a non-empty id";
    if (flangeTTcp == null) throw "Tool requires a flange_T_tcp transform";
    if (!Math.isFinite(mass) || mass < 0.0) throw "Tool mass must be finite and non-negative";
    if (massProperties != null && mass != 0.0 && Math.abs(mass - massProperties.massKg) > 1e-9)
      throw "Tool mass and mass properties disagree";
    this.id = id;
    this.name = name;
    this.flangeTTcp = flangeTTcp;
    this.collision = collision == null ? ToolCollisionShape.NoCollision : collision;
    switch this.collision {
      case NoCollision:
      case _: ToolCollisionShapes.bounds(this.collision);
    }
    this.mass = massProperties == null ? mass : massProperties.massKg;
    this.massProperties = massProperties == null && mass == 0.0 ? MassProperties.zero() : massProperties;
  }
}
