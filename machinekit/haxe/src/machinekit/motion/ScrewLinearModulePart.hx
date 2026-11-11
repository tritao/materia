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

/** Ground base, full-length side walls and plain carriage. All fastening bores come from the
 * joints. The carrier meets the nut's outer face: only the screw passes it. */
class ScrewLinearModulePart extends MachineComponent {
  static final recipe=new machinekit.cache.Once<ComponentType>();
  public final spec:ScrewLinearModuleSpec;
  public final role:String;
  public static final ROLES=["base","wallLeft","wallRight","carriage","nutCarrier","stopFront","stopRear","scaleHeadCarrier","fixedPad","floatingPad","scaleSpacer"];
  public static function parameters():Array<ComponentParameter> return [P.choice("role",ROLES,"base"),
    P.text("profile","HGH15CA"),P.length("stroke",20),P.length("railSpacing",150),P.length("blockSpacing",80),P.length("overtravel",5),P.scalar("hostLength",0),P.scalar("hostWidth",0)];
  public static function recipeType():ComponentType return recipe.get(()->new ComponentType("machinekit.motion.screw-linear-module-part",parameters(),v->new ScrewLinearModulePart(v),false,false,null,false)
    .versioned(5).numbered(numberFor).procured(_->Make("ground plate/flat bar, cut and joint-drilled"))
    .requiring(v->{
      // The stock (and so the route) is worked out from the module spec when the part is made.
      var route=ManufacturingRoute.of(new ScrewLinearModulePart(v));
      if(route==null)throw "Screw module part has no manufacturing route";
      route.requirements();
    }));
  static function numberFor(v:ComponentValues):String {
    var keys=[v.token("profile")];for(k in ["stroke","railSpacing","blockSpacing","overtravel","hostLength","hostWidth"])keys.push(ComponentNumberKey.encode(v.number(k)));
    return PartNumbers.geometry("SCREW-MODULE",v.token("role"),keys.join("|"));
  }
  public function new(?overrides:ComponentValues) {
    var v=P.resolve(parameters(),overrides);spec=new ScrewLinearModuleSpec(v.token("profile"),v.number("stroke"),v.number("railSpacing"),v.number("blockSpacing"),v.number("overtravel"),v.number("hostLength"),v.number("hostWidth"));role=v.token("role");
    super(numberFor(v),"Precision screw module "+role,"steel",false,v);
    var stock=isScaleSpacer()?GroundStock.bar(40,8):isPad()?GroundStock.bar(GroundStock.barWidths().filter(w->w>=padBounds(spec,role=="fixedPad"?"fixed":"floating").maxY*2)[0],3):role=="scaleHeadCarrier"?GroundStock.bar(headCarrierStockWidth(spec),GroundStock.atLeast("flat bar",spec.carriageBottom()-headCarrierBottom(spec)))
      :StringTools.startsWith(role,"wall")?GroundStock.bar(Math.max(spec.wallWidth,spec.wallHeight),Math.min(spec.wallWidth,spec.wallHeight))
      :StringTools.startsWith(role,"stop")?GroundStock.bar(60,20)
      :role=="nutCarrier"?GroundStock.bar(spec.carrierWidth,spec.carrierThickness)
      :GroundStock.plate(GroundStock.STEEL,role=="carriage"?spec.carriageThickness:spec.baseThickness);
    addFacet(ManufacturingRoute.groundStock(stock,[{name:isScaleSpacer()?"finish scale-housing packing to 7 mm from 8 mm ground stock; ASSUMED supplier capability":isPad()?"finish matched support packing to 2 mm from 3 mm ground stock; ASSUMED supplier capability":"joint interfaces; inspect stock before assembly, supplier-finish failing small faces",tolerance:0.01,roughnessRa:0.8}]));
    addFacet(EngineeringMetadata.assumed("Stock tolerances and process acceptance unqualified"));
    addConnector("mount",Mount,F.identity());
    addConnector("interface",Mount,isScaleSpacer()?F.translation(0,0,ScrewLinearModuleSpec.SCALE_PACKING):F.translation(spec.carriageCentre,0,spec.carriageBottom()+spec.carriageThickness));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function machinable():Bool return true;
  public static function plane(x:Float,y:Float,z:Float,into:Float):AssemblyFrame return BoltedFace.frameOf({x:x,y:y,z:z},{x:0.0,y:0,z:into},{x:1.0,y:0,z:0});
  function isScaleSpacer():Bool return role=="scaleSpacer";
  function isPad():Bool return role=="fixedPad"||role=="floatingPad";
  function side():Int return StringTools.endsWith(role,"Left")?-1:1;
  public function carrierFace(into:Float):AssemblyFrame return BoltedFace.frameOf({x:spec.centre+spec.carrierThickness/2,y:0.0,z:spec.axisHeight()},{x:into,y:0.0,z:0},{x:0.0,y:0,z:1});
  public static function carrierPins():Array<BoltHole> return [{x:0.0,y:-20.0},{x:0.0,y:20.0}];
  public static function carrierBolts():Array<BoltHole> return [{x:0.0,y:-33.0},{x:0.0,y:33.0}];
  public function stopCentre():{x:Float,y:Float} {
    var front=StringTools.startsWith(role,"stopFront"), half=spec.stroke/2+spec.overtravel;
    return {x:spec.centre+(front?-1:1)*(half+spec.carrierThickness/2+20),y:42.0};
  }
  override public function fasteningFaces():Array<BoltedFace> {
    var s=spec, faces:Array<BoltedFace>=[];
    if(isPad()||isScaleSpacer())return faces;
    if(role=="base") {
      // Non-overlapping receiver footprints, independently indicated and pinned.
      for(support in ["fixed","floating"]) {
        var points=[for(h in s.drive.seatHoles(s.axis(),s.screwLength))if(h.support==support&&h.kind=="tapped")h.position];
        var lo=1e9,hi=-1e9;for(p in points){lo=Math.min(lo,p.x);hi=Math.max(hi,p.x);}
        faces.push(BoltedFace.drilledToSuit(support,plane((lo+hi)/2,0,s.baseThickness,-1),hi-lo+18,s.drive.fixed.spec.width+10,12,s.baseThickness,BoltedFace.ThreadMaterial.Steel,"M6",true));
      }
      for(side in [-1,1])faces.push(BoltedFace.drilledToSuit("wall"+side,plane(s.carriageCentre,s.wallY(side),s.baseThickness,-1),s.baseMax-s.baseMin,s.wallWidth,12,s.baseThickness,BoltedFace.ThreadMaterial.Steel,"M6",true));
      for(r in ROLES)if(StringTools.startsWith(r,"stop")) {
        var temporary=new ScrewLinearModulePart(s.values().setToken("role",r)),p=temporary.stopCentre();
        faces.push(BoltedFace.drilledToSuit(r,plane(p.x,p.y,s.baseThickness,-1),40,20,12,s.baseThickness,BoltedFace.ThreadMaterial.Steel,"M6",true));
      }
      faces.push(BoltedFace.drilledToSuit("host",plane((s.baseMin+s.baseMax)/2,0,0,1),s.baseMax-s.baseMin,s.baseWidth,12,s.baseThickness,BoltedFace.ThreadMaterial.Steel,"M8"));
    } else if(StringTools.startsWith(role,"wall")) {
      var y=s.wallY(side()), screw=machinekit.standard.SocketHeadCapScrew.catalog().get("M6");
      faces.push(BoltedFace.clamped("stock-base",plane(s.carriageCentre,y,s.baseThickness,1),[for(x in s.wallBolts()){x:x-s.carriageCentre,y:0.0}],screw.clearanceMedium,s.wallHeight-7,"M6",{diameter:screw.counterboreDiameter,depth:7},false));
      faces.push(BoltedFace.drilledToSuit("rail",plane(s.carriageCentre,s.railY(side()),s.baseThickness+s.wallHeight,-1),s.baseMax-s.baseMin,s.wallWidth,10,s.wallHeight,BoltedFace.ThreadMaterial.Steel,s.guide.railMountScrew.size));
      if(side()<0)faces.push(BoltedFace.drilledToSuit("scale",BoltedFace.frameOf({x:s.scaleStart()-s.scaleHousing.overallLength()/2,y:s.wallY(-1)-s.wallWidth/2,z:s.baseThickness+s.scaleMountHeight()},{x:0.0,y:1,z:0},{x:-1.0,y:0,z:0}),s.scaleHousing.overallLength(),s.wallHeight,14,s.wallWidth,BoltedFace.ThreadMaterial.Steel,"M8"));
    } else if(role=="carriage") {
      var screw=machinekit.standard.SocketHeadCapScrew.catalog().get(s.guide.blockMountThread);
      for(side in [-1,1])for(row in [-1,1])faces.push(BoltedFace.clamped("block"+side+row,plane(s.carriageCentre+row*s.blockSpacing/2,s.railY(side),s.carriageBottom(),1),
        [for(x in [-1,1])for(y in [-1,1]){x:x*s.guide.blockHoleSpacingAlong/2,y:y*s.guide.blockHoleSpacingAcross/2}],screw.clearanceMedium,s.carriageThickness-(screw.headHeight+0.2),s.guide.blockMountThread,{diameter:screw.counterboreDiameter,depth:screw.headHeight+0.2},false));
      var carrierScrew=machinekit.standard.SocketHeadCapScrew.catalog().get("M6");
      faces.push(BoltedFace.clamped("nut-carrier",plane(s.centre,0,s.carriageBottom(),1),carrierBolts(),carrierScrew.clearanceMedium,s.carriageThickness-carrierScrew.counterboreDepth,"M6",{diameter:carrierScrew.counterboreDiameter,depth:carrierScrew.counterboreDepth},false));
      var headScrew=machinekit.standard.SocketHeadCapScrew.catalog().get("M4");
      faces.push(BoltedFace.clamped("scale-head-carrier",plane(s.scaleMountX(),s.scaleMountY(),s.carriageBottom(),1),[{x:0.0,y:-10.0},{x:0.0,y:10.0}],headScrew.clearanceMedium,s.carriageThickness-headScrew.counterboreDepth,"M4",{diameter:headScrew.counterboreDiameter,depth:headScrew.counterboreDepth},false));
      faces.push(BoltedFace.drilledToSuit("host",plane(s.carriageCentre,s.carriageMaxY()-s.carriageWidth/2,s.carriageBottom()+s.carriageThickness,-1),s.carriageLength,s.carriageWidth,8,s.carriageThickness,BoltedFace.ThreadMaterial.Steel));
    } else if(role=="nutCarrier") {
      faces.push(BoltedFace.drilledToSuit("nut",carrierFace(-1),2*s.carrierReach(),s.carrierWidth,s.carrierThickness,s.carrierThickness,BoltedFace.ThreadMaterial.Steel,"M5",true));
      faces.push(BoltedFace.drilledToSuit("carriage",plane(s.centre,0,s.carriageBottom(),-1),s.carrierThickness,s.carrierWidth,10,s.carrierHeight(),BoltedFace.ThreadMaterial.Steel,"M6"));
    } else if(role=="scaleHeadCarrier") {
      faces.push(BoltedFace.drilledToSuit("carriage",plane(s.scaleMountX(),s.scaleMountY(),s.carriageBottom(),-1),16,28,10,s.carriageBottom()-headCarrierBridgeBottom(s),BoltedFace.ThreadMaterial.Steel,"M4"));
      var headFrame=F.compose(F.compose(s.scalePose(true),s.scaleHead.fasteningFaces()[0].frame),F.translation(s.scaleHead.series.headHoleOffset,0,0));
      faces.push(BoltedFace.drilledToSuit("head",F.compose(headFrame,F.fromRotationMatrix(0,0,0,[1.0,0,0,0,-1,0,0,0,-1])),80,24,10,12,BoltedFace.ThreadMaterial.Steel,"M4"));
    } else {
      var p=stopCentre(),screw=machinekit.standard.SocketHeadCapScrew.catalog().get("M6");
      faces.push(BoltedFace.clamped("base",plane(p.x,p.y,s.baseThickness,1),[{x:-10.0,y:0.0},{x:10.0,y:0.0}],screw.clearanceMedium,13,"M6",{diameter:screw.counterboreDiameter,depth:7},false));
    }
    return faces;
  }
  override public function pinnedFaces():Array<PinnedFace> {
    var s=spec,faces:Array<PinnedFace>=[];
    if(role=="base") {
      for(support in ["fixed","floating"]) {
        var points=[for(h in s.drive.seatHoles(s.axis(),s.screwLength))if(h.support==support&&h.kind=="dowel")h.position];
        faces.push(PinnedFace.atPoints(support,plane(0,0,s.baseThickness,-1),points,SupportedScrewDrive.DOWEL_DIAMETER,s.baseThickness,s.baseThickness<=12));
      }
      for(r in ["stopFront","stopRear"]) {
        var temporary=new ScrewLinearModulePart(s.values().setToken("role",r)),p=temporary.stopCentre();
        faces.push(new PinnedFace(r,plane(p.x,p.y,s.baseThickness,-1),[{x:0.0,y:-4.0},{x:0.0,y:4.0}],4,6));
      }
      for(side in [-1,1])faces.push(new PinnedFace("wall"+side,plane(s.carriageCentre,s.wallY(side),s.baseThickness,-1),[for(x in s.wallPins()){x:x-s.carriageCentre,y:0.0}],4,6));
    } else if(StringTools.startsWith(role,"wall"))faces.push(new PinnedFace("base",plane(s.carriageCentre,s.wallY(side()),s.baseThickness,1),[for(x in s.wallPins()){x:x-s.carriageCentre,y:0.0}],4,6));
    else if(StringTools.startsWith(role,"stop")) {var p=stopCentre();faces.push(new PinnedFace("base",plane(p.x,p.y,s.baseThickness,1),[{x:0.0,y:-4.0},{x:0.0,y:4.0}],4,6));}
    else if(role=="carriage"||role=="nutCarrier")faces.push(new PinnedFace("carrier",plane(s.centre,0,s.carriageBottom(),role=="carriage"?1:-1),carrierPins(),6,8));
    return faces;
  }
  static function box(x0:Float,x1:Float,y0:Float,y1:Float,z0:Float,z1:Float):Part {
    var raw=Part.box(x1-x0,y1-y0,z1-z0),out=raw.translated(new Vector((x0+x1)/2,(y0+y1)/2,z0));raw.close();return out;
  }
  public static function padBounds(s:ScrewLinearModuleSpec,support:String):{minX:Float,maxX:Float,minY:Float,maxY:Float} {
    var lo=1e9,hi=-1e9;
    var width=support=="fixed"?s.drive.fixed.spec.width:s.drive.floating.width;
    for(h in s.drive.seatHoles(s.axis(),s.screwLength))if(h.support==support&&h.kind=="tapped") {
      lo=Math.min(lo,h.position.x);hi=Math.max(hi,h.position.x);
      var bolt=machinekit.standard.SocketHeadCapScrew.catalog().get(h.thread);
      width=Math.max(width,2*(Math.abs(h.position.y)+machinekit.assembly.JointFasteners.EDGE_DIAMETERS*bolt.diameter));
    }
    // Extend the BF pad toward the free end for a full bearing seat, away from nut travel.
    return {minX:lo-(support=="floating"?16:9),maxX:hi+9,minY:-width/2,maxY:width/2};
  }
  static function headCarrierStockWidth(s:ScrewLinearModuleSpec):Float {
    var width=Math.max(s.scaleFootMaxY(),s.scaleCarrierY()+6)-Math.min(s.scaleFootMaxY()-44,s.scaleCarrierY()-6);
    for(stock in GroundStock.barWidths())if(stock>=width)return stock;
    throw "No ground bar width for the head bracket";
  }
  public static function headCarrierBottom(s:ScrewLinearModuleSpec):Float return s.scaleHeadTop()-s.scaleHead.series.headWidth-s.scaleHousing.gap()+ScrewLinearModuleSpec.GAP+0.2;
  public static function headCarrierBridgeBottom(s:ScrewLinearModuleSpec):Float return s.baseThickness+s.wallHeight+s.guide.railHeight+ScrewLinearModuleSpec.GAP+0.2;
  override public function geometry(detail:ComponentDetail=Preview):Part {
    var s=spec,body:Part;
    if(role=="base") {
      body=box(s.baseMin,s.baseMax,-s.baseWidth/2,s.baseWidth/2,0,s.baseThickness);

    } else if(isScaleSpacer()) {
      body=Solids.cut(box(-20,20,-15,15,0,ScrewLinearModuleSpec.SCALE_PACKING),[Part.cylinderSpan(4.2,-1,ScrewLinearModuleSpec.SCALE_PACKING+1)]);
    } else if(isPad()) {
      var support=role=="fixedPad"?"fixed":"floating",b=padBounds(s,support);
      var holes:Array<Part>=[];
      for(h in s.drive.seatHoles(s.axis(),s.screwLength))if(h.support==support) {
        var diameter=h.kind=="dowel"?h.diameter:machinekit.standard.SocketHeadCapScrew.catalog().get(h.thread).clearanceMedium;
        holes.push(Part.cylinderSpan(diameter/2,s.baseThickness-1,s.baseThickness+s.supportLift+1,h.position.x,h.position.y));
      }
      body=Solids.cut(box(b.minX,b.maxX,b.minY,b.maxY,s.baseThickness,s.baseThickness+s.supportLift),holes);
    } else if(StringTools.startsWith(role,"wall"))body=box(s.baseMin,s.baseMax,s.wallY(side())-s.wallWidth/2,s.wallY(side())+s.wallWidth/2,s.baseThickness,s.baseThickness+s.wallHeight);
    else if(role=="carriage")body=box(s.carriageCentre-s.carriageLength/2,s.carriageCentre+s.carriageLength/2,s.carriageMaxY()-s.carriageWidth,s.carriageMaxY(),s.carriageBottom(),s.carriageBottom()+s.carriageThickness);
    else if(role=="nutCarrier")body=Solids.cut(box(s.centre-s.carrierThickness/2,s.centre+s.carrierThickness/2,-s.carrierWidth/2,s.carrierWidth/2,s.axisHeight()-s.carrierReach(),s.carriageBottom()),
      [Part.cylinderAlong(new BallNut().screwDiameter/2+ScrewLinearModuleSpec.GAP+0.2,new Vector(s.centre-s.carrierThickness/2-1,0,s.axisHeight()),Vector.X(),s.carrierThickness+2)]);
    else if(role=="scaleHeadCarrier")body=Solids.union([
      box(Math.min(s.scaleCentre-60,s.scaleMountX()-8),s.scaleCentre+50,s.scaleCarrierY()-6,s.scaleCarrierY()+6,headCarrierBottom(s),s.carriageBottom()),
      box(s.scaleMountX()-8,s.scaleMountX()+8,s.scaleFootMaxY()-44,s.scaleFootMaxY(),headCarrierBridgeBottom(s),s.carriageBottom())]);
    else {var p=stopCentre();body=box(p.x-20,p.x+20,p.y-10,p.y+10,s.baseThickness,s.baseThickness+20);}

    return machined(body);
  }
}
