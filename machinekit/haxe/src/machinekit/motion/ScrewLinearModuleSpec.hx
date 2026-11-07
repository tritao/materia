package machinekit.motion;

import machinekit.component.*;
import machinekit.component.BoltedFace.BoltHole;
import machinekit.structural.GroundStock;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** X is travel, Y across the guides, Z off the ground base. Supplier geometry
 * owns the drive/guide stack; stock acceptance tolerances remain assumptions. */
class ScrewLinearModuleSpec {
  public final profile:String;
  public final stroke:Float;
  public final overtravel:Float;
  public final railSpacing:Float;
  public final blockSpacing:Float;
  public final guide:LinearRailProfile.LinearRailProfileSpec;
  public final drive:SupportedScrewDrive;
  public final baseThickness:Float;
  public final carriageThickness:Float;
  public final riserHeight:Float;
  public final riserWidth:Float;
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
  public function new(profile:String="HGH15CA",stroke:Float=20,railSpacing:Float=150,blockSpacing:Float=80,overtravel:Float=5) {
    for(n in [stroke,railSpacing,blockSpacing,overtravel])if(!Math.isFinite(n)||n<=0)throw "Invalid linear module dimensions";
    this.profile=profile;this.stroke=stroke;this.overtravel=overtravel;this.railSpacing=railSpacing;this.blockSpacing=blockSpacing;
    var row=LinearRailSystem.catalog().get(profile);if(row==null)throw "Unknown module guide: "+profile;guide=row;
    drive=SupportedScrewDrive.m60dBk12(true);drive.fixed.footPinned=true;drive.floating.footPinned=true;
    var nut=new BallNut(), mount=drive.fixed.spec, bolt=machinekit.standard.SocketHeadCapScrew.catalog().get(guide.blockMountThread);
    baseThickness=GroundStock.atLeast("plate",16);carriageThickness=GroundStock.atLeast("plate",12);
    carrierThickness=GroundStock.atLeast("flat bar",16);
    carrierWidth=80; // M6 base bolts at +/-31; match-pins at +/-20, clear of flange bores.
    riserHeight=GroundStock.atLeast("flat bar",mount.shaftHeight+nut.flangeDiameter/2+4-guide.blockHeight);
    var widths=GroundStock.barWidths().filter(w->w>=Math.max(guide.railWidth,riserHeight));
    if(widths.length==0)throw "No ground riser section";riserWidth=widths[0];
    railLength=Math.ceil((blockSpacing+guide.blockLength+stroke+2*overtravel+2*guide.railEndMargin)/20)*20;
    carriageLength=Math.ceil((blockSpacing+guide.blockHoleSpacingAlong+3*bolt.diameter)/10)*10;
    carriageWidth=Math.ceil((railSpacing+guide.blockHoleSpacingAcross+3*bolt.diameter)/10)*10;
    baseWidth=Math.ceil((railSpacing+riserWidth+24)/10)*10;
    if(railSpacing-riserWidth<mount.width+2*GAP)throw "Module rails do not clear the fixed support";
    var half=stroke/2+overtravel, nutLength=nut.bodyLength+nut.flangeThickness;
    var fixedReach=Math.max(nutLength+carrierThickness/2,carriageLength/2);
    var floatingReach=carrierThickness/2;
    if(baseThickness+drive.floating.height>=carriageBottom()-GAP)floatingReach=Math.max(floatingReach,carriageLength/2);
    var low=drive.floatingEnd+drive.floating.length/2+floatingReach+half+GAP+1;
    var high=drive.faceToStart()-mount.length-fixedReach-half-GAP;
    high=Math.min(high,-drive.fixedEnd-nutLength-carrierThickness/2-half-GAP);
    screwLength=Math.ceil(Math.max(150,low-high)/10)*10;
    centre=(low+screwLength+high)/2;
    nutStation=screwLength-centre-nutLength-carrierThickness/2;
    var holes=drive.seatHoles(axis(),screwLength), max=railStart()+railLength+10;
    for(hole in holes)max=Math.max(max,hole.position.x+9);
    baseMin=Math.floor(Math.min(Math.min(railStart(),drive.floatingEnd-drive.floating.length/2)-10,centre-half-carriageLength/2-60-10)/10)*10;
    baseMax=Math.ceil(Math.max(max,centre+half+carriageLength/2+60+10)/10)*10;
  }
  public function axis():SupportedScrewDrive.ScrewDriveAxis return {start:[screwLength,0.0,axisHeight()],direction:[-1.0,0,0],normal:[0.0,0,1]};
  public function axisHeight():Float return baseThickness+drive.fixed.spec.shaftHeight;
  public function carriageBottom():Float return baseThickness+riserHeight+guide.blockHeight;
  public function carrierReach():Float {
    var nut=new BallNut(), reach=0.0;
    for(hole in nut.boltPattern())reach=Math.max(reach,Math.abs(hole.x));
    return reach+machinekit.standard.SocketHeadCapScrew.catalog().get(nut.mountScrew).diameter;
  }
  public function carrierHeight():Float return carriageBottom()-axisHeight();
  public function railStart():Float return centre-railLength/2;
  public function railY(side:Int):Float return side*railSpacing/2;
  public function riserY(side:Int):Float return railY(side)-side*(riserWidth-guide.railWidth)/2;
  public function railPose(side:Int):AssemblyFrame return machinekit.assembly.AxisBuilder.orient(railStart(),railY(side),baseThickness+riserHeight+guide.railHeight,[0.0,0,1],[1.0,0,0]);
  public function nutPose():AssemblyFrame {
    var at=drive.poses(axis(),screwLength,nutStation).get("nut");if(at==null)throw "Missing module nut pose";
    return F.compose(at,F.fromRotationMatrix(0,0,0,[0.0,-1,0,1,0,0,0,0,1]));
  }
  public function riserBolts():Array<Float> {
    var rail=LinearRail.metric(profile,railLength), points=[];
    for(i in 0...rail.holePositions.length-1)points.push(railStart()+(rail.holePositions[i]+rail.holePositions[i+1])/2);
    return points;
  }
  public function riserPins():Array<Float> {
    var rail=LinearRail.metric(profile,railLength), h=rail.holePositions;
    return [railStart()+h[0]+guide.railHolePitch/4,railStart()+h[h.length-1]-guide.railHolePitch/4];
  }
  public function values():ComponentValues return new ComponentValues().setToken("profile",profile).setNumber("stroke",stroke)
    .setNumber("railSpacing",railSpacing).setNumber("blockSpacing",blockSpacing).setNumber("overtravel",overtravel);
}
