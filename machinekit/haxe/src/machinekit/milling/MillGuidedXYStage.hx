package machinekit.milling;

import machinekit.assembly.AxisBuilder;
import machinekit.component.ComponentValues;
import machinekit.motion.LinearRail;
import machinekit.motion.LinearRailBlock;
import machinekit.standard.SocketHeadCapScrew;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

typedef MillGuidedXYLayout = {
 var centreY:Float;
 var yRailSpacing:Float;
 var yRailStart:Float;
 var yRailPlane:Float;
 var yRail:LinearRail;
 var xRail:LinearRail;
 var block:LinearRailBlock;
 var xHalfTravel:Float;
 var yHalfTravel:Float;
 var xOvertravel:Float;
 var yOvertravel:Float;
 var xBlockSpan:Float;
 var yBlockSpan:Float;
 var xRailSpacing:Float;
 var saddleWidth:Float;
 var saddleDepth:Float;
 var saddleHeight:Float;
 var tableWidth:Float;
 var tableDepth:Float;
 var tableThickness:Float;
 var riserHeight:Float;
 var driveInterfaces:Bool;
}

/** Shared guided cross slide. Drive assemblies attach separately; absent drives
 * never acquire catalog ratings or implied qualification from the guide model.
 */
class MillGuidedXYStage extends AxisBuilder {
 public final saddle:MillCrossSaddle;
 public final table:MillCrossTable;
 public final guidedLayout:MillGuidedXYLayout;
 public function new(l:MillGuidedXYLayout) {
  super();guidedLayout=l;
  for(v in [l.saddleWidth,l.saddleDepth,l.saddleHeight,l.tableWidth,l.tableDepth,l.tableThickness,l.riserHeight,
    l.xHalfTravel,l.yHalfTravel,l.xRailSpacing,l.yRailSpacing,l.xBlockSpan,l.yBlockSpan])
   if(!Math.isFinite(v)||v<=0)throw "Cross slide dimensions must be finite and positive";
  var g=l.block.spec;
  var xReach=l.xBlockSpan+g.blockLength/2+g.railEndMargin+l.xOvertravel;
  var yReach=l.yBlockSpan+g.blockLength/2+g.railEndMargin+l.yOvertravel;
  if(!Math.isFinite(l.xOvertravel)||!Math.isFinite(l.yOvertravel)||l.xOvertravel<0||l.yOvertravel<0||
    l.xRail.length/2<l.xHalfTravel+xReach-1e-6 ||
    l.centreY-l.yHalfTravel-yReach<l.yRailStart-1e-6 ||
    l.centreY+l.yHalfTravel+yReach>l.yRailStart+l.yRail.length+1e-6)
   throw "Cross-slide rails cannot retain guide support through overtravel";
  for(rail in [l.xRail,l.yRail])if(rail.spec.railWidth!=g.railWidth||rail.spec.railHeight!=g.railHeight)
   throw "Cross-slide guide blocks must match both rails";
  var saddleBottom=l.yRailPlane+g.blockHeight;
  var xRailPlane=saddleBottom+l.saddleHeight;
  var tableBottom=xRailPlane+g.blockHeight+l.riserHeight;
  function horizontal(x:Float,y:Float,z:Float,alongX:Bool):AssemblyFrame return alongX?
   orient(x,y,z,[0,0,1],[1,0,0]):orient(x,y,z,[0,0,1],[0,1,0]);
  var yp:Array<AssemblyFrame> =[],xp:Array<AssemblyFrame> =[];
  for(side in [-1,1]) {
   var id=side<0?"railYLeft":"railYRight";
   var pose=horizontal(side*l.yRailSpacing/2,l.yRailStart,l.yRailPlane+g.railHeight,false);
   if(side<0)place(id,l.yRail,pose);else attach(id,l.yRail,pose,"railYLeft");
   for(span in [-l.yBlockSpan,l.yBlockSpan])yp.push(horizontal(side*l.yRailSpacing/2,l.centreY+span,l.yRailPlane+g.railHeight,false));
  }
  slide({id:"y",lower:-l.yHalfTravel,upper:l.yHalfTravel,initial:0},"railYLeft","blockY0",l.block,yp[0],{x:0,y:-1,z:0},yp.slice(1),l.yOvertravel);
  var mountingSize=g.blockMountScrew.split("x")[0];
  var screw=SocketHeadCapScrew.metric(mountingSize,16),clearance=screw.clearanceDiameter();
  saddle=new MillCrossSaddle(new ComponentValues().setNumber("width",l.saddleWidth).setNumber("depth",l.saddleDepth)
   .setNumber("height",l.saddleHeight).setNumber("bore",l.driveInterfaces?58:0).setNumber("boreZ",0)
   .setBoolean("ribbed",true).setBoolean("blockAccess",true).setBoolean("motorPocket",l.driveInterfaces)
   .setBoolean("nutBoss",l.driveInterfaces).setBoolean("nutServiceAccess",l.driveInterfaces)
   .setNumber("crossRailSpacing",l.xRailSpacing).setNumber("crossRailLength",l.xRail.length)
   .setNumber("crossRailWidth",g.railWidth).setNumber("guideSpacing",l.yRailSpacing).setNumber("blockSpan",l.yBlockSpan)
   .setNumber("blockHolePitchB",g.blockHolePitchB).setNumber("blockHolePitchC",g.blockHolePitchC)
   .setNumber("blockClearance",clearance));
  attach("saddle",saddle,F.translation(0,l.centreY,saddleBottom),"blockY0");
  for(i in 1...4)attach('blockY$i',l.block,yp[i],"saddle");
  // 12 mm retained sole, with one nominal diameter of thread engagement.
  var boltLength=MillCrossSaddle.BLOCK_SEAT+screw.diameter;
  for(i in 0...4)for(hole in 1...5) {
   var p=F.compose(yp[i],l.block.connector('mount$hole').frame);
   attach('blockYBolt$i-$hole',SocketHeadCapScrew.metric(mountingSize,boltLength),
    F.translation(p.x,p.y,saddleBottom+MillCrossSaddle.BLOCK_SEAT),"saddle");
  }
  for(side in [-1,1]) {
   attach(side<0?"railXFront":"railXRear",l.xRail,horizontal(-l.xRail.length/2,l.centreY+side*l.xRailSpacing/2,xRailPlane+g.railHeight,true),"saddle");
   for(span in [-l.xBlockSpan,l.xBlockSpan])xp.push(horizontal(span,l.centreY+side*l.xRailSpacing/2,xRailPlane+g.railHeight,true));
  }
  slide({id:"x",lower:-l.xHalfTravel,upper:l.xHalfTravel,initial:0},"railXFront","blockX0",l.block,xp[0],{x:-1,y:0,z:0},xp.slice(1),l.xOvertravel);
  table=new MillCrossTable(l.tableDepth,l.xRailSpacing,l.tableWidth,l.tableThickness,l.riserHeight,
   l.driveInterfaces?34:Math.max(34,g.blockHolePitchC+24),l.driveInterfaces);
  attach("table",table,F.translation(0,l.centreY,tableBottom),"blockX0");
  for(i in 1...4)attach('blockX$i',l.block,xp[i],"table");
  exposeConnector("table","table","top");
 }
}
