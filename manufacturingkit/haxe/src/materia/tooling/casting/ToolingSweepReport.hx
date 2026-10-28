package materia.tooling.casting;

import materia.tooling.panel.PanelAssemblyPlan.PanelBlockingRelation;

/** Geometry observations supplied by the CAD sweep checker to neutral manufacturing planners. */
typedef ToolingSweepReport = {
  var blocking:Array<PanelBlockingRelation>;
  var removalOrder:Array<String>;
  var assemblyOrder:Array<String>;
  var travelMm:Float;
  var contactVolumeToleranceMm3:Float;
}
