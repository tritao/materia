package machinekit.motion;

import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.component.BoltedFace.BoltHole;
import machinekit.component.BoltedFace.PinnedFace;
import machinekit.structural.GroundStock;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Ground stock bodies and derived spacers. All fastening bores come from the
 * joints. The carrier meets the nut's outer face: only the screw passes it. */
class ScrewLinearModulePart extends MachineComponent {
  static final recipe=new machinekit.cache.Once<ComponentType>();
  public final spec:ScrewLinearModuleSpec;
  public final role:String;
  public static final ROLES=["base","riserLeft","riserRight","carriage","nutCarrier","stopFrontLeft","stopFrontRight","stopRearLeft","stopRearRight","scaleSeat","scaleHeadCarrier"];
  public static function parameters():Array<ComponentParameter> return [P.choice("role",ROLES,"base"),
    P.text("profile","HGH15CA"),P.length("stroke",20),P.length("railSpacing",150),P.length("blockSpacing",80),P.length("overtravel",5),P.scalar("hostLength",0),P.scalar("hostWidth",0)];
  public static function recipeType():ComponentType return recipe.get(()->new ComponentType("machinekit.motion.screw-linear-module-part",parameters(),v->new ScrewLinearModulePart(v),false,false,null,false)
    .versioned(2).numbered(numberFor).procured(_->Make("ground plate/flat bar, cut and joint-drilled")));
  static function numberFor(v:ComponentValues):String {
    var keys=[v.token("profile")];for(k in ["stroke","railSpacing","blockSpacing","overtravel","hostLength","hostWidth"])keys.push(ComponentNumberKey.encode(v.number(k)));
    return PartNumbers.geometry("SCREW-MODULE",v.token("role"),keys.join("|"));
  }
  public function new(?overrides:ComponentValues) {
    var v=P.resolve(parameters(),overrides);spec=new ScrewLinearModuleSpec(v.token("profile"),v.number("stroke"),v.number("railSpacing"),v.number("blockSpacing"),v.number("overtravel"),v.number("hostLength"),v.number("hostWidth"));role=v.token("role");
    super(numberFor(v),"Precision screw module "+role,"steel",false,v);
    var stock=role=="scaleSeat"?GroundStock.bar(40,spec.scaleSeatThickness())
      :role=="scaleHeadCarrier"?GroundStock.bar(headCarrierStockWidth(spec),GroundStock.atLeast("flat bar",spec.carriageBottom()-spec.scaleHeadTop()+0.5))
      :StringTools.startsWith(role,"riser")?GroundStock.bar(spec.riserWidth,spec.riserHeight)
      :StringTools.startsWith(role,"stop")?GroundStock.bar(60,20)
      :role=="nutCarrier"?GroundStock.bar(spec.carrierWidth,spec.carrierThickness)
      :GroundStock.plate(GroundStock.STEEL,role=="carriage"?spec.carriageThickness:spec.baseThickness);
    addFacet(ManufacturingRoute.groundStock(stock,[{name:"joint interfaces; inspect stock before assembly, replicate failing faces",tolerance:0.01,roughnessRa:0.8}]));
    addFacet(new EngineeringMetadata(new ComponentValues().setToken("errorsBasis","Assumed").setToken("ratingsBasis","Assumed").setToken("source","Stock tolerances and process acceptance unqualified")));
    addConnector("mount",Mount,F.identity());
    addConnector("interface",Mount,F.translation(spec.centre,0,spec.carriageBottom()+spec.carriageThickness));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function machinable():Bool return true;
  public static function plane(x:Float,y:Float,z:Float,into:Float):AssemblyFrame return BoltedFace.frameOf({x:x,y:y,z:z},{x:0.0,y:0,z:into},{x:1.0,y:0,z:0});
  function side():Int return StringTools.endsWith(role,"Left")?-1:1;
  public function carrierFace(into:Float):AssemblyFrame return BoltedFace.frameOf({x:spec.centre+spec.carrierThickness/2,y:0.0,z:spec.axisHeight()},{x:into,y:0.0,z:0},{x:0.0,y:0,z:1});
  public static function carrierPins():Array<BoltHole> return [{x:0.0,y:-20.0},{x:0.0,y:20.0}];
  public static function carrierBolts():Array<BoltHole> return [{x:0.0,y:-33.0},{x:0.0,y:33.0}];
  public function stopCentre():{x:Float,y:Float} {
    var front=StringTools.startsWith(role,"stopFront"), half=spec.stroke/2+spec.overtravel;
    return {x:spec.centre+(front?-1:1)*(half+spec.carriageLength/2+30),y:side()*(spec.drive.fixed.spec.width/2+ScrewLinearModuleSpec.GAP+10)};
  }
  override public function fasteningFaces():Array<BoltedFace> {
    var s=spec, faces:Array<BoltedFace>=[];
    if(role=="base") {
      faces=faces.concat(ScrewLinearModuleCover.receivers(s)).concat(ScrewLinearModuleBellows.baseReceivers(s));
      faces.push(BoltedFace.drilledToSuit("scale-seat",plane(s.scaleStart()+s.scaleHousing.overallLength()/2,s.scaleSeatY(),s.baseThickness,-1),s.scaleHousing.overallLength(),40,12,s.baseThickness,BoltedFace.ThreadMaterial.Steel,"M6"));
      // Non-overlapping receiver footprints, independently indicated and pinned.
      for(support in ["fixed","floating"]) {
        var points=[for(h in s.drive.seatHoles(s.axis(),s.screwLength))if(h.support==support&&h.kind=="tapped")h.position];
        var lo=1e9,hi=-1e9;for(p in points){lo=Math.min(lo,p.x);hi=Math.max(hi,p.x);}
        faces.push(BoltedFace.drilledToSuit(support,plane((lo+hi)/2,0,s.baseThickness,-1),hi-lo+18,s.drive.fixed.spec.width+10,12,s.baseThickness,BoltedFace.ThreadMaterial.Steel,"M6"));
      }
      for(side in [-1,1])faces.push(BoltedFace.drilledToSuit("riser"+side,plane(s.centre,s.riserY(side),s.baseThickness,-1),s.railLength,s.riserWidth,12,s.baseThickness,BoltedFace.ThreadMaterial.Steel,"M6"));
      for(r in ROLES)if(StringTools.startsWith(r,"stop")) {
        var temporary=new ScrewLinearModulePart(s.values().setToken("role",r)),p=temporary.stopCentre();
        faces.push(BoltedFace.drilledToSuit(r,plane(p.x,p.y,s.baseThickness,-1),60,20,12,s.baseThickness,BoltedFace.ThreadMaterial.Steel,"M6"));
      }
      faces.push(BoltedFace.drilledToSuit("host",plane((s.baseMin+s.baseMax)/2,0,0,1),s.baseMax-s.baseMin,s.baseWidth,12,s.baseThickness,BoltedFace.ThreadMaterial.Steel,"M8"));
    } else if(StringTools.startsWith(role,"riser")) {
      var y=s.riserY(side()), screw=machinekit.standard.SocketHeadCapScrew.catalog().get("M6");
      faces.push(BoltedFace.clamped("stock-base",plane(s.centre,y,s.baseThickness,1),[for(x in s.riserBolts()){x:x-s.centre,y:0.0}],screw.clearanceMedium,s.riserHeight-screw.counterboreDepth,"M6",{diameter:screw.counterboreDiameter,depth:screw.counterboreDepth},false));
      faces.push(BoltedFace.drilledToSuit("rail",plane(s.centre,s.railY(side()),s.baseThickness+s.riserHeight,-1),s.railLength,s.riserWidth,10,s.riserHeight,BoltedFace.ThreadMaterial.Steel,s.guide.railMountScrew.size));
    } else if(role=="carriage") {
      var screw=machinekit.standard.SocketHeadCapScrew.catalog().get(s.guide.blockMountThread);
      for(side in [-1,1])for(row in [-1,1])faces.push(BoltedFace.clamped("block"+side+row,plane(s.centre+row*s.blockSpacing/2,s.railY(side),s.carriageBottom(),1),
        [for(x in [-1,1])for(y in [-1,1]){x:x*s.guide.blockHoleSpacingAlong/2,y:y*s.guide.blockHoleSpacingAcross/2}],screw.clearanceMedium,s.carriageThickness-(screw.headHeight+0.2),s.guide.blockMountThread,{diameter:screw.counterboreDiameter,depth:screw.headHeight+0.2},false));
      var carrierScrew=machinekit.standard.SocketHeadCapScrew.catalog().get("M6");
      faces.push(BoltedFace.clamped("nut-carrier",plane(s.centre,0,s.carriageBottom(),1),carrierBolts(),carrierScrew.clearanceMedium,s.carriageThickness-carrierScrew.counterboreDepth,"M6",{diameter:carrierScrew.counterboreDiameter,depth:carrierScrew.counterboreDepth},false));
      if(s.stroke>20)for(front in [true,false])faces.push(BoltedFace.drilledToSuit("bellows-"+(front?"front":"rear"),
        BoltedFace.frameOf({x:s.centre+(front?-1:1)*s.carriageLength/2,y:0.0,z:s.carriageBottom()+s.carriageThickness/2},{x:front?1.0:-1.0,y:0,z:0},{x:0.0,y:1,z:0}),s.carriageWidth,s.carriageThickness,8,20,BoltedFace.ThreadMaterial.Steel,"M4"));
      var headScrew=machinekit.standard.SocketHeadCapScrew.catalog().get("M4");
      faces.push(BoltedFace.clamped("scale-head-carrier",plane(s.centre,s.railY(-1),s.carriageBottom(),1),[{x:0.0,y:-10.0},{x:0.0,y:10.0}],headScrew.clearanceMedium,s.carriageThickness-headScrew.counterboreDepth,"M4",{diameter:headScrew.counterboreDiameter,depth:headScrew.counterboreDepth},false));
      faces.push(BoltedFace.drilledToSuit("host",plane(s.centre,0,s.carriageBottom()+s.carriageThickness,-1),s.stroke<=20?2*(ScrewLinearModuleCover.openingHalf(s)-4):s.carriageLength,s.stroke<=20?2*(ScrewLinearModuleCover.openingWidthHalf(s)-4):s.carriageWidth,8,s.carriageThickness,BoltedFace.ThreadMaterial.Steel));
    } else if(role=="nutCarrier") {
      faces.push(BoltedFace.drilledToSuit("nut",carrierFace(-1),2*s.carrierReach(),s.carrierWidth,s.carrierThickness,s.carrierThickness,BoltedFace.ThreadMaterial.Steel,"M5",true));
      faces.push(BoltedFace.drilledToSuit("carriage",plane(s.centre,0,s.carriageBottom(),-1),s.carrierThickness,s.carrierWidth,10,s.carrierHeight(),BoltedFace.ThreadMaterial.Steel,"M6"));
    } else if(role=="scaleSeat") {
      var screw=machinekit.standard.SocketHeadCapScrew.catalog().get("M6"),length=s.scaleHousing.overallLength();
      faces.push(BoltedFace.clamped("base",plane(s.scaleStart(),s.scaleSeatY(),s.baseThickness,1),[{x:35.0,y:0.0},{x:length-35,y:0.0}],screw.clearanceMedium,s.scaleSeatThickness()-screw.counterboreDepth,"M6",{diameter:screw.counterboreDiameter,depth:screw.counterboreDepth},false));
      faces.push(BoltedFace.drilledToSuit("housing",plane(s.scaleStart()+length/2,s.scaleHoleY(),s.baseThickness+s.scaleSeatThickness(),-1),length,40,1.75*8,s.scaleSeatThickness(),BoltedFace.ThreadMaterial.Steel,"M8"));
    } else if(role=="scaleHeadCarrier") {
      faces.push(BoltedFace.drilledToSuit("carriage",plane(s.centre,s.railY(-1),s.carriageBottom(),-1),12,32,10,s.carriageBottom()-headCarrierBridgeBottom(s),BoltedFace.ThreadMaterial.Steel,"M4"));
      var headFrame=F.compose(F.compose(s.scalePose(true),s.scaleHead.fasteningFaces()[0].frame),F.translation(s.scaleHead.series.headHoleOffset,0,0));
      faces.push(BoltedFace.drilledToSuit("head",F.compose(headFrame,F.fromRotationMatrix(0,0,0,[1.0,0,0,0,-1,0,0,0,-1])),80,12,10,s.carriageBottom()-s.scaleHeadTop(),BoltedFace.ThreadMaterial.Steel,"M4"));
    } else {
      var p=stopCentre(),height=s.carriageBottom()+s.carriageThickness-s.baseThickness,screw=machinekit.standard.SocketHeadCapScrew.catalog().get("M6");
      faces.push(BoltedFace.clamped("base",plane(p.x,p.y,s.baseThickness,1),[{x:-20.0,y:0.0},{x:20.0,y:0.0}],screw.clearanceMedium,height-screw.counterboreDepth,"M6",{diameter:screw.counterboreDiameter,depth:screw.counterboreDepth},false));
    }
    return faces;
  }
  override public function pinnedFaces():Array<PinnedFace> {
    var s=spec,faces:Array<PinnedFace>=[];
    if(role=="base") {
      for(support in ["fixed","floating"]) {
        var points=[for(h in s.drive.seatHoles(s.axis(),s.screwLength))if(h.support==support&&h.kind=="dowel")h.position];
        faces.push(PinnedFace.atPoints(support,plane(0,0,s.baseThickness,-1),points,SupportedScrewDrive.DOWEL_DIAMETER,12));
      }
      for(r in ROLES)if(StringTools.startsWith(r,"stop")) {
        var temporary=new ScrewLinearModulePart(s.values().setToken("role",r)),p=temporary.stopCentre();
        faces.push(new PinnedFace(r,plane(p.x,p.y,s.baseThickness,-1),[{x:0.0,y:-4.0},{x:0.0,y:4.0}],4,6));
      }
      for(side in [-1,1])faces.push(new PinnedFace("riser"+side,plane(s.centre,s.riserY(side),s.baseThickness,-1),[for(x in s.riserPins()){x:x-s.centre,y:0.0}],4,6));
    } else if(StringTools.startsWith(role,"riser"))faces.push(new PinnedFace("base",plane(s.centre,s.riserY(side()),s.baseThickness,1),[for(x in s.riserPins()){x:x-s.centre,y:0.0}],4,6));
    else if(StringTools.startsWith(role,"stop")) {var p=stopCentre();faces.push(new PinnedFace("base",plane(p.x,p.y,s.baseThickness,1),[{x:0.0,y:-4.0},{x:0.0,y:4.0}],4,6));}
    else if(role=="carriage"||role=="nutCarrier")faces.push(new PinnedFace("carrier",plane(s.centre,0,s.carriageBottom(),role=="carriage"?1:-1),carrierPins(),6,8));
    return faces;
  }
  static function box(x0:Float,x1:Float,y0:Float,y1:Float,z0:Float,z1:Float):Part {
    var raw=Part.box(x1-x0,y1-y0,z1-z0),out=raw.translated(new Vector((x0+x1)/2,(y0+y1)/2,z0));raw.close();return out;
  }
  static function headCarrierStockWidth(s:ScrewLinearModuleSpec):Float {
    for(width in GroundStock.barWidths())if(width>=s.railY(-1)-s.scaleCarrierY()+22)return width;
    throw "No ground bar width for the stepped head carrier";
  }
  public static function headCarrierBridgeBottom(s:ScrewLinearModuleSpec):Float return Math.max(s.scaleHeadTop(),s.baseThickness+s.riserHeight+s.guide.railHeight+ScrewLinearModuleSpec.GAP+0.2);
  override public function geometry(detail:ComponentDetail=Preview):Part {
    var s=spec,body:Part;
    if(role=="base") {
      body=box(s.baseMin,s.baseMax,-s.baseWidth/2,s.baseWidth/2,0,s.baseThickness);
      if(s.baseMax>s.driveBayEnd())body=Solids.cut(body,[box(s.driveBayEnd(),s.baseMax+1,-s.drive.fixed.spec.width/2-ScrewLinearModuleSpec.GAP,s.drive.fixed.spec.width/2+ScrewLinearModuleSpec.GAP,-1,s.baseThickness+1)]);
      // Shallow clearance in the ground body, derived from carrier reach and travel.
      var reach=s.carrierReach(),depth=Math.max(0.0,ScrewLinearModuleSpec.GAP+1-(s.drive.fixed.spec.shaftHeight-reach));
      body=Solids.cut(body,[box(s.centre-s.stroke/2-s.overtravel-s.carrierThickness/2-2,s.centre+s.stroke/2+s.overtravel+s.carrierThickness/2+2,
        -s.carrierWidth/2-2,s.carrierWidth/2+2,s.baseThickness-depth,s.baseThickness+1)]);
    } else if(StringTools.startsWith(role,"riser"))body=box(s.railStart(),s.railStart()+s.railLength,s.riserY(side())-s.riserWidth/2,s.riserY(side())+s.riserWidth/2,s.baseThickness,s.baseThickness+s.riserHeight);
    else if(role=="carriage")body=box(s.centre-s.carriageLength/2,s.centre+s.carriageLength/2,-s.carriageWidth/2,s.carriageWidth/2,s.carriageBottom(),s.carriageBottom()+s.carriageThickness);
    else if(role=="nutCarrier")body=Solids.cut(box(s.centre-s.carrierThickness/2,s.centre+s.carrierThickness/2,-s.carrierWidth/2,s.carrierWidth/2,s.axisHeight()-s.carrierReach(),s.carriageBottom()),
      [Part.cylinderAlong(new BallNut().screwDiameter/2+ScrewLinearModuleSpec.GAP+0.2,new Vector(s.centre-s.carrierThickness/2-1,0,s.axisHeight()),Vector.X(),s.carrierThickness+2)]);
    else if(role=="scaleSeat")body=box(s.scaleStart(),s.scaleStart()+s.scaleHousing.overallLength(),s.scaleSeatY()-20,s.scaleSeatY()+20,s.baseThickness,s.baseThickness+s.scaleSeatThickness());
    else if(role=="scaleHeadCarrier")body=Solids.union([
      box(s.centre-40,s.centre+50,s.scaleCarrierY()-6,s.scaleCarrierY()+6,s.scaleHeadTop(),s.carriageBottom()),
      box(s.centre-6,s.centre+6,s.scaleCarrierY()-6,s.railY(-1)+16,headCarrierBridgeBottom(s),s.carriageBottom())]);
    else {
      var p=stopCentre(),edge=1.5*machinekit.standard.SocketHeadCapScrew.catalog().get("M6").diameter;
      body=Solids.cut(box(p.x-30,p.x+30,p.y-10,p.y+10,s.baseThickness,s.carriageBottom()+s.carriageThickness),
        [box(p.x-20+edge,p.x+20-edge,p.y-11,p.y+11,s.baseThickness+6+ScrewLinearModuleSpec.GAP,s.carriageBottom())]);
    }
    return machined(body);
  }
}
