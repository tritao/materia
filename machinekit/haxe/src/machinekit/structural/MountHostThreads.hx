package machinekit.structural;

import materia.casting.InsertHole;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import machinekit.standard.FastenerThreads;

/** Explicit blind-root policy for thick machined hosts. These are engineering
 * assumptions for S235 / 6061, not the C45 insert policy or supplier ratings.
 * Installation target is 9 kN per M8 bolt; qualify tightening on the real finish. */
class MountHostThreads {
 public static function minimumEngagement(material:String):Float return switch material {
  case "steel":16;case "aluminium 6061":20;case _:throw "No mount-host thread policy for "+material;
 };
 public static function blindM8(material:String,thickness:Float,frame:AssemblyFrame,id:String):InsertHole {
  var engagement=minimumEngagement(material);
  if(thickness<(material=="steel"?30:40))throw "Blind mount thread requires thick host stock";
  var depth=material=="steel"?24.0:31.0;
  var h=InsertHoles.records(InsertHoles.tappedThrough(InsertHoles.metricThread(FastenerThreads.coarse(8).mating()),depth))[0];
  h.id=id;h.frame=frame;h.extent=Blind;h.depth=depth;h.threadDepth=depth-2*1.25;
  h.limits={policyId:material=="steel"?"host-s235-m8-root-v1":"host-6061-m8-root-v1",
   threaded:{minEngagement:engagement,minTipGap:1.25},minBack:6,minWall:8,minClearance:4,overrides:[]};
  return h;
 }
 public static function validate(h:InsertHole,material:String,thickness:Float):Void {
  var expected=blindM8(material,thickness,h.frame,h.id);
  var rules=h.limits.threaded;
  if(rules==null)throw "Host tapping requires explicit engagement limits";
  if(h.thread==null || h.thread.designation!="M8" || h.thread.pitch!=1.25 || h.extent!=Blind
   || h.depth!=expected.depth || h.threadDepth!=expected.threadDepth || h.limits.policyId!=expected.limits.policyId
   || rules.minEngagement!=minimumEngagement(material)
   || rules.minTipGap!=1.25 || h.limits.minBack!=6 || h.limits.minWall!=8)
   throw "Host tapping differs from its material-specific root policy";
 }
 /** Conservative half-cylinder shear area at diameter (major minus pitch); assumed host yield / sqrt(3),
  * safety factor1.5. Returns allowable N; final reaction envelope still required. */
 public static function strippingCapacityN(material:String,engagement:Float):Float {
  minimumEngagement(material);
  var yield=material=="steel"?235.0:240.0;
  return 0.5*Math.PI*(FastenerThreads.coarse(8).majorDiameter-FastenerThreads.coarse(8).pitch)*engagement*yield/Math.sqrt(3)/1.5;
 }
}
