import toolpathkit.tool.RouterCut;
import motionkit.robot.PlanCheck;
class SheetRouterCuttingChecks {
 static function check(v:Bool,s:String){if(!v)throw s;}
 static function main(){
  var c=SheetRouterCutting.cut("plywood-birch",6,6);
  check(Math.abs(c.feedMPerMinute-5.4)<1e-9,"18krpm * 2 * 0.15 = 5.4 m/min");
  check(Math.abs(c.powerW-388.8)<1e-6,"specific energy * removal rate");
  var acrylic=SheetRouterCutting.cut("acrylic",18,6,0.01);
  check(acrylic.meltingRisk,"acrylic under-chip melting risk");
  var overload=RouterCut.evaluate("deep acrylic slot",SheetRouterCutting.bit(),18000,0.18,0.05,0.18,0.30,25,6,20,24000);
  var refused=false;try PlanCheck.spindle(overload.operation,overload.powerW,overload.torqueNm,4500,6000/(24000*2*Math.PI/60)) catch(e:Dynamic){refused=Std.string(e).indexOf("deep acrylic slot")>=0;}
  check(refused,"named spindle overload refused");
  var clamped=RouterCut.evaluate("clamped",SheetRouterCutting.bit(),30000,0.25,0.08,0.25,0.12,18,6,4,24000);
  check(clamped.rpm==24000&&clamped.feedMPerMinute==4,"rpm/feed machine clamps");
  var tooHeavy=false;try SheetRouterCutting.cut("acrylic",18,6) catch(e:Dynamic){tooHeavy=Std.string(e).indexOf("acrylic profile")>=0;}
  check(tooHeavy,"full-depth acrylic rejected by process force limit");
  var output=haxe.Json.stringify({checks:6,cut:c,forceEnvelopeN:SheetRouterCutting.forceEnvelopeN(),acrylic:acrylic});
  if(Sys.args().length>0)sys.io.File.saveContent(Sys.args()[0],output+"\n");Sys.println(output);
 }
}
