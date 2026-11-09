package machinekit.motion;

import machinekit.component.*;
import machinekit.component.BoltedFace.BoltHole;
import machinekit.structural.GroundStock;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** X is travel, Y across the guides, Z off the ground base. Supplier geometry
 * owns the drive/guide stack; stock acceptance tolerances remain assumptions. */
class ScrewLinearModuleSpec {
  public final scaleHousing:SealedScalePart;
  public final scaleHead:SealedScalePart;
  public final hostLength:Float;
  public final hostWidth:Float;
  public final profile:String;
  public final stroke:Float;
  public final overtravel:Float;
  public final railSpacing:Float;
  public final blockSpacing:Float;
  public final guide:LinearRailProfile.LinearRailProfileSpec;
  public final drive:SupportedScrewDrive;
  public final baseThickness:Float;
  public final carriageThickness:Float;
  public final wallHeight:Float;
  public final wallWidth:Float;
  public final railLength:Float;
  public final carriageLength:Float;
  public final carriageWidth:Float;
  public final screwLength:Float;
  public final centre:Float;
  public final nutStation:Float;
  public final baseMin:Float;
  public final baseMax:Float;
  public final baseWidth:Float;
  public final carrierWidth:Float;
  public final carrierThickness:Float;
  public static inline final GAP=2.0;
  public function new(profile:String="HGH15CA",stroke:Float=20,railSpacing:Float=150,blockSpacing:Float=80,overtravel:Float=5,hostLength:Float=0,hostWidth:Float=0) {
    for(n in [stroke,railSpacing,blockSpacing,overtravel])if(!Math.isFinite(n)||n<=0)throw "Invalid linear module dimensions";
    for(n in [hostLength,hostWidth])if(!Math.isFinite(n)||n<0)throw "Invalid host footprint";
    this.hostLength=hostLength;this.hostWidth=hostWidth;
    this.profile=profile;this.stroke=stroke;this.overtravel=overtravel;this.railSpacing=railSpacing;this.blockSpacing=blockSpacing;
    var row=LinearRailSystem.catalog().get(profile);if(row==null)throw "Unknown module guide: "+profile;guide=row;
    drive=SupportedScrewDrive.m60dBk12(true);drive.fixed.footPinned=true;drive.floating.footPinned=true;
    var nut=new BallNut(), mount=drive.fixed.spec, bolt=machinekit.standard.SocketHeadCapScrew.catalog().get(guide.blockMountThread);
    var scale=SealedScalePart.pair("JFT-40",stroke+2*overtravel,"3","5",false,
      new ComponentValues().setToken("airPort","end").setToken("headSide","+x").setToken("cableBend","-y"));
    scaleHousing=scale.housing;scaleHead=scale.head;
    baseThickness=GroundStock.atLeast("plate",16);carriageThickness=GroundStock.atLeast("plate",12);
    carrierThickness=GroundStock.atLeast("flat bar",16);
    carrierWidth=80; // M6 base bolts at +/-33; match-pins at +/-20, clear of flange bores.
    wallHeight=GroundStock.atLeast("flat bar",mount.shaftHeight+nut.flangeDiameter/2+4-guide.blockHeight);
    // ASSUMED 8 mm AF3 key leg plus running gap outside the guide block.
    var widths=GroundStock.barWidths().filter(w->w>=Math.max(wallHeight,(guide.railWidth+guide.blockWidth)/2+8+GAP));
    if(widths.length==0)throw "No ground wall section";wallWidth=widths[0];
    railLength=Math.ceil((blockSpacing+guide.blockLength+stroke+2*overtravel+2*guide.railEndMargin)/20)*20;
    carriageLength=Math.ceil(Math.max(hostLength,blockSpacing+guide.blockHoleSpacingAlong+3*bolt.diameter)/10)*10;
    carriageWidth=Math.ceil(Math.max(hostWidth,railSpacing+guide.blockHoleSpacingAcross+3*bolt.diameter)/10)*10;
    if(railSpacing-wallWidth<mount.width+2*GAP)throw "Module rails do not clear the fixed support";
    var half=stroke/2+overtravel, nutLength=nut.bodyLength+nut.flangeThickness;
    var fixedReach=Math.max(nutLength+carrierThickness/2+40,carriageLength/2);
    var floatingReach=carrierThickness/2;
    if(baseThickness+drive.floating.height>=carriageBottom()-GAP)floatingReach=Math.max(floatingReach,carriageLength/2);
    var low=drive.floatingEnd+drive.floating.length/2+Math.max(floatingReach,carrierThickness/2+40)+half+GAP+1;
    var high=drive.faceToStart()-mount.length-fixedReach-half-GAP;
    high=Math.min(high,-drive.fixedEnd-nutLength-carrierThickness/2-half-GAP);
    screwLength=Math.ceil(Math.max(150,low-high)/10)*10;
    centre=(low+screwLength+high)/2;
    nutStation=screwLength-centre-nutLength-carrierThickness/2;
    var holes=drive.seatHoles(axis(),screwLength), max=railStart()+railLength+10;
    for(hole in holes)max=Math.max(max,hole.position.x+9);
    baseMin=Math.floor(Math.min(scaleStart()-scaleHousing.overallLength()-GAP,Math.min(railStart(),drive.floatingEnd-drive.floating.length/2)-10)/10)*10;
    baseMax=Math.ceil(Math.max(scaleStart()+GAP,max)/10)*10;
    baseWidth=Math.ceil(Math.max(carriageWidth+7,railSpacing+2*wallWidth-guide.railWidth+32)/10)*10;
  }
  public function scaleReadingY():Float return wallY(-1)-wallWidth/2-scaleHousing.series.housingThickness;
  public function scaleHoleY():Float return scaleReadingY();
  public function scaleHeadY():Float return scaleReadingY();
  public function scaleCarrierY():Float return scaleReadingY()-6;
  public function driveBayEnd():Float {
    var end=railStart()+railLength+10;
    for(hole in drive.seatHoles(axis(),screwLength))end=Math.max(end,hole.position.x+9);
    return Math.ceil(Math.max(end,centre+stroke/2+overtravel+carriageLength/2+70)/10)*10;
  }
  public function scaleStart():Float return centre+20+scaleHousing.startOffset()+scaleHousing.measuringLength/2;
  public function scaleHeadTop():Float return baseThickness+14+scaleHousing.series.housingWidth-(scaleHousing.series.holeOffset??scaleHousing.series.housingWidth/2)+scaleHousing.gap()+scaleHead.series.headWidth;
  public function scalePose(head:Bool=false):AssemblyFrame return F.fromRotationMatrix(head?centre+20:scaleStart(),scaleReadingY(),baseThickness+14,[0.0,0,-1,0,1,0,1,0,0]);
  public function axis():SupportedScrewDrive.ScrewDriveAxis return {start:[screwLength,0.0,axisHeight()],direction:[-1.0,0,0],normal:[0.0,0,1]};
  public function axisHeight():Float return baseThickness+drive.fixed.spec.shaftHeight;
  public function carriageBottom():Float return baseThickness+wallHeight+guide.blockHeight;
  public function carrierReach():Float {
    var nut=new BallNut(), reach=0.0;
    for(hole in nut.boltPattern())reach=Math.max(reach,Math.abs(hole.x));
    return reach+machinekit.standard.SocketHeadCapScrew.catalog().get(nut.mountScrew).diameter;
  }
  public function carrierHeight():Float return carriageBottom()-axisHeight();
  public function railStart():Float return centre-railLength/2;
  public function railY(side:Int):Float return side*railSpacing/2;
  public function wallY(side:Int):Float return railY(side)+side*(wallWidth-guide.railWidth)/2;
  public function railPose(side:Int):AssemblyFrame return machinekit.assembly.AxisBuilder.orient(railStart(),railY(side),baseThickness+wallHeight+guide.railHeight,[0.0,0,1],[1.0,0,0]);
  public function nutPose():AssemblyFrame {
    var at=drive.poses(axis(),screwLength,nutStation).get("nut");if(at==null)throw "Missing module nut pose";
    return F.compose(at,F.fromRotationMatrix(0,0,0,[0.0,-1,0,1,0,0,0,0,1]));
  }
  public function wallBolts():Array<Float> {
    var rail=LinearRail.metric(profile,railLength), points=[];
    for(i in 0...rail.holePositions.length-1)points.push(railStart()+(rail.holePositions[i]+rail.holePositions[i+1])/2);
    return points;
  }
  public function wallPins():Array<Float> {
    var rail=LinearRail.metric(profile,railLength), h=rail.holePositions;
    return [railStart()+h[0]+guide.railHolePitch/4,railStart()+h[h.length-1]-guide.railHolePitch/4];
  }
  public function values():ComponentValues return new ComponentValues().setToken("profile",profile).setNumber("stroke",stroke)
    .setNumber("railSpacing",railSpacing).setNumber("blockSpacing",blockSpacing).setNumber("overtravel",overtravel).setNumber("hostLength",hostLength).setNumber("hostWidth",hostWidth);
}
