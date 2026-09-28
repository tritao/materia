package robotkit.tool;

import robotkit.spatial.Transform3;

/** One grasped part at its pick-specific pose relative to the tool flange. */
class WorkpieceLoad {
  public final id:String;
  public final massProperties:MassProperties;
  public final flangeTWorkpiece:Transform3;

  public function new(id:String, massProperties:MassProperties, flangeTWorkpiece:Transform3) {
    if (id == null || id.length == 0 || massProperties == null ||
        massProperties.massKg <= 0 || flangeTWorkpiece == null)
      throw "Workpiece load requires an id, positive mass and flange pose";
    this.id = id;
    this.massProperties = massProperties;
    this.flangeTWorkpiece = flangeTWorkpiece;
  }

  public function atFlange():MassProperties
    return massProperties.transformed(flangeTWorkpiece);
}
