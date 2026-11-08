package machinekit.hobbing;

import machinekit.assembly.AxisBuilder;
import machinekit.component.*;
import machinekit.cover.*;
import machinekit.cover.WayCoverMount.MountStock;
import machinekit.motion.*;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Z guard interfaces, outside the precision stiffness path. Catalogue family
 * and thin nested envelope are custom RFQ requirements, ASSUMED until ordered. */
class HobberZBellowsLayout {
  public static inline final ROW="BELLOWS-PU-LOW-8x10";
  static function guide():machinekit.motion.LinearRailProfile.LinearRailProfileSpec {
    var g=LinearRailSystem.catalog().get("HGH15CA");if(g==null)throw "Missing Z guide";return g;
  }
  static final railCache=new machinekit.cache.Once<HobberRailPlacement.HobberPlacedRail>();
  static function rail():HobberRailPlacement.HobberPlacedRail return railCache.get(()->HobberRailPlacement.placements("column",FoundationLayout.placements("column"))[0]);
  public static function surfaceX():Float return rail().contact.x;
  public static function lowerLimit():Float return HobberZLayout.TRAVEL_MIN-HobberScales.overtravel();
  public static function upperLimit():Float return HobberZLayout.TRAVEL_MAX+HobberScales.overtravel();
  public static function plan():WayCoverPlan {
    var row=WayCoverCatalog.get(ROW),g=guide();
    var width=Math.ceil((g.blockWidth+2*ScrewLinearModuleSpec.GAP)/10)*10;
    var height=g.railHeight+ScrewLinearModuleSpec.GAP;
    var space=Math.ceil((row.foldCompressed*Math.ceil((upperLimit()-lowerLimit())/(row.ratio-1)/row.foldCompressed)+2*row.flangeThickness)/10)*10;
    return new WayCoverPlan(row,width,height,upperLimit()-lowerLimit(),space,
      WayCoverPlan.boxCover(0,surfaceX()-HobberSwivelLayout.PLATE_BACK-2*ScrewLinearModuleSpec.GAP,100000,true));
  }
  public static function movingZ(upper:Bool):Float return upper
    ?HobberZLayout.SEAT_Z+HobberZLayout.BLOCK_SPACING+guide().blockLength/2+ScrewLinearModuleSpec.GAP+plan().row.flangeThickness
    :HobberSaddle.xCarriageTop()+plan().space-lowerLimit();
  public static function fixedZ(upper:Bool):Float return upper?movingZ(true)+upperLimit()+plan().space:HobberSaddle.xCarriageTop();
  public static function pose(side:Int,upper:Bool,fixed:Bool):AssemblyFrame return AxisBuilder.orient(surfaceX(),side*FoundationLayout.COLUMN_RAIL_CENTRE,fixed?fixedZ(upper):movingZ(upper),[-1.0,0,0],[0.0,0,(upper?1.0:-1.0)*(fixed?-1:1)]);
  public static function movingMount():WayCoverMount {
    var p=plan(),half=p.fullPlateHalf(),offset=surfaceX()-HobberSwivelLayout.PLATE_BACK;
    var leg=24.0;
    var stock:Array<MountStock>=[{x0:-half,x1:half,y0:offset-4,y1:offset,z0:0.0,z1:8.0}];
    for(side in [-1,1])stock.push({x0:side<0?-half:p.width/2,x1:side<0?-p.width/2:half,y0:offset-4,y1:offset,z0:-leg,z1:0.0});
    return new WayCoverMount("HOBBER-Z-BELLOWS-MOVING-MOUNT",half,p.shape.hostMax,8,[],
      [{x0:-p.width/2,x1:p.width/2,y0:-1.0,y1:p.shape.hostMax+1}],0,
      {stock:stock,bolted:[BoltedFace.clamped("saddle",BoltedFace.frameOf({x:0.0,y:offset,z:-leg/2-ScrewLinearModuleSpec.GAP},{x:0.0,y:-1,z:0},{x:1.0,y:0,z:0}),
        [{x:-26.0,y:0.0},{x:26.0,y:0.0}],4.5,4,"M4",null,false,true)],pinned:[]});
  }
  public static function fixedUpperMount():WayCoverMount {
    var p=plan(),half=p.fullPlateHalf(),railHalf=guide().railWidth/2;
    var stock:Array<MountStock>=[];
    for(side in [-1,1]) {
      stock.push({x0:side<0?-17.0:railHalf,x1:side<0?-railHalf:17.0,y0:0.0,y1:8.0,z0:-28.0,z1:0.0});
      // Continue the inner feet down inside the protected opening, clear of
      // the nested skins. Their two rail-seat contact strips exceed625 mm2.
      stock.push({x0:side<0?-p.width/2+3:railHalf,x1:side<0?-railHalf:p.width/2-3,y0:0.0,y1:8.0,z0:-1.0,z1:20.0});
    }
    var screw=machinekit.standard.SocketHeadCapScrew.catalog().get("M3"),x=railHalf+1.5*screw.diameter;
    var cuts:Array<WayCoverMount.MountCut>=[{x0:-railHalf-ScrewLinearModuleSpec.GAP,x1:railHalf+ScrewLinearModuleSpec.GAP,y0:-1.0,y1:p.height}];
    // Relieve the lower outer corners over the catalogue M60 fixed mount.
    // M3-derived4.5 x3.2 corner notches leave >=d around the receiving
    // threads and >2 mm to that mount. They do not cut the insert contact.
    for(side in [-1,1])cuts.push({x0:side<0?-half-1:half-1.5*screw.diameter,
      x1:side<0?-half+1.5*screw.diameter:half+1,y0:-1.0,y1:screw.diameter+0.2});
    return new WayCoverMount("HOBBER-Z-BELLOWS-UPPER-MOUNT",half,p.shape.hostMax,8,[],
      cuts,0,
      {stock:stock,bolted:[BoltedFace.clamped("column",BoltedFace.frameOf({x:0.0,y:0.0,z:-18.0},{x:0.0,y:1,z:0},{x:1.0,y:0,z:0}),
        [for(xx in [-x,x])for(zz in [-4.0,4.0]){x:xx,y:zz}],3.4,8,"M3",null,false,true)],pinned:[]});
  }
  public static function saddleReceivers():Array<BoltedFace> {
    var p=plan(),mount=movingMount(),face=mount.fasteningFaces().filter(f->f.id=="saddle")[0],out=[];
    for(side in [-1,1])for(upper in [false,true]) {
      var at=F.compose(pose(side,upper,false),face.frame);at=F.compose(F.translation(0,0,-HobberZLayout.SEAT_Z),at);
      out.push(BoltedFace.drilledToSuit("bellows-"+side+"-"+upper,F.compose(at,F.fromRotationMatrix(0,0,0,[1.0,0,0,0,-1,0,0,0,-1])),2*mount.halfWidth,24,12,13,BoltedFace.ThreadMaterial.Steel,"M4"));
    }
    return out;
  }
  public static function deckReceivers():Array<BoltedFace> {
    var p=plan(),out=[];
    for(side in [-1,1]) {
      var frame=F.compose(F.compose(pose(side,false,true),F.translation(0,p.shape.hostMax/2,0)),F.fromRotationMatrix(0,0,0,[1.0,0,0,0,-1,0,0,0,-1]));
      out.push(BoltedFace.drilledToSuit("z-bellows-"+side,frame,2*p.fullPlateHalf(),p.shape.hostMax,8,HobberSaddle.X_CARRIAGE_THICKNESS,BoltedFace.ThreadMaterial.Steel,"M3"));
    }
    return out;
  }
}
