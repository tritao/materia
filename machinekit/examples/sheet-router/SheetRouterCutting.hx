import toolpathkit.tool.RouterBit;
import toolpathkit.tool.RouterCut;
import toolpathkit.tool.RouterCut.RouterCutResult;
import materia.project.MaterialLibrary;
import motionkit.robot.PlanCheck;

/** Qualified-input seam for SR4/SR8; all numeric cutting inputs remain assumptions. */
class SheetRouterCutting {
 public static function bit():RouterBit return new RouterBit(1,0.006,0.025,0.060,0.120,2,"compression","ER32-6","ISO30",0.050,0.060);
 public static function cut(material:String,depthMm:Float,widthMm:Float,?feedChipMm:Float):RouterCutResult {
  var data=MaterialLibrary.require(material).routerCutting;
  if(data==null)throw "No router cutting data: "+material;
  var row=data.chipLoads[0];
  var result=RouterCut.evaluate(material+" profile",bit(),18000,feedChipMm==null?row.nominalChipMm:feedChipMm,
   row.minChipMm,row.maxChipMm,data.specificEnergyJPerMm3,depthMm,widthMm,20,24000);
  // Assumed 6 kW constant-torque spindle below 24 krpm: 2.387 Nm.
  var torque=6000/(24000*2*Math.PI/60);
  if(result.forceEnvelopeN>300)throw result.operation+": exceeds 300 N process envelope; reduce axial depth/feed";
  PlanCheck.spindle(result.operation,result.powerW,result.torqueNm,torque*result.rpm*2*Math.PI/60,torque);
  return result;
 }
 public static function forceEnvelopeN():Float {
  var maximum=0.0;
  for(id in ["plywood-birch","mdf","acrylic"]){var c=cut(id,6,6);maximum=Math.max(maximum,c.forceEnvelopeN);}
  return maximum;
 }
}
