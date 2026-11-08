package machinekit.hobbing;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.PosedParts;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.component.BoltedFace.PinnedFace;
import machinekit.motion.*;
import machinekit.structural.GroundStock;
import materia.assembly.AssemblyFrames as F;

/** Host stock and hole patterns; the mechanism, scale and cover remain library
 * module features. The head face stays on its existing work-frame datum. */
class HobberYPart extends MachineComponent {
  static final recipe=new machinekit.cache.Once<ComponentType>();
  public final role:String;
  public final spec:ScrewLinearModuleSpec;
  public static function parameters():Array<ComponentParameter> return [P.choice("role",["base","carriage","coverFront","coverRear","retainerFront","retainerRear","lipFront","lipRear"],"base")];
  public static function recipeType():ComponentType return recipe.get(()->new ComponentType("machinekit.hobbing.y-module-part",parameters(),v->new HobberYPart(v),false,false,null,false).versioned(13).numbered(v->"HOBBER-Y-MODULE-"+v.token("role").toUpperCase()).procured(v->StringTools.startsWith(v.token("role"),"lip")?Buy(null,null,null):Make("ground stock / formed guard, joint drilling")));
  public function new(?overrides:ComponentValues) {
    var v=P.resolve(parameters(),overrides);role=v.token("role");spec=HobberYLayout.moduleSpec();
    super("HOBBER-Y-MODULE-"+role.toUpperCase(),"Y module "+role,StringTools.startsWith(role,"lip")?"polyurethane PU":isGuard()?"stainless A2-70":"steel",false,v);
    if(StringTools.startsWith(role,"cover"))addFacet(ManufacturingRoute.foldedSheet(ScrewLinearModuleCover.SHEET,5,1.5));
    else if(StringTools.startsWith(role,"retainer"))addFacet(ManufacturingRoute.cutPlate());
    else if(!StringTools.startsWith(role,"lip"))addFacet(ManufacturingRoute.groundStock(GroundStock.plate(GroundStock.STEEL,role=="base"?GroundStock.atLeast("plate",spec.baseThickness-actuatorPoint().z):spec.carriageThickness),[{name:"joint interfaces; inspect stock, replicate failing faces before grinding",tolerance:0.01,roughnessRa:0.8},{name:"mill nonprecision guard and seal lands; retain full-depth joint webs",tolerance:0.2,roughnessRa:3.2}]));
    addFacet(new EngineeringMetadata(new ComponentValues().setToken("ratingsBasis","Assumed").setToken("errorsBasis","Assumed").setToken("source","Stock acceptance and end seals require qualification")));
    addConnector("mount",Mount,F.identity());addConnector("interface",Mount,F.translation(spec.centre,0,spec.carriageBottom()+spec.carriageThickness));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function machinable():Bool return !StringTools.startsWith(role,"lip");
  function isGuard():Bool return role!="base"&&role!="carriage";
  function body():MachineComponent return isGuard()?new ScrewLinearModuleCover(spec.values().setToken("role",role)):new ScrewLinearModulePart(spec.values().setToken("role",role));
  public static function actuatorPoint():{x:Float,y:Float,z:Float} return F.transformPoint(F.inverse(HobberYLayout.modulePose()),HobberSwivelLayout.OUTPUT_X,0,HobberZLayout.SEAT_Z);
  override public function fasteningFaces():Array<BoltedFace> {
    var out=body().fasteningFaces();
    if(role=="base")out=out.filter(f->f.id!="guard-front-1"&&f.id!="guard-rear-1");
    if(isGuard()) {
      var front=StringTools.endsWith(role,"Front"),cover=StringTools.startsWith(role,"cover"),z=HobberYLayout.guardRoof()-ScrewLinearModuleCover.SHEET;
      out=out.filter(f->f.id!="seal"&&f.id!="edge-1");
      if(cover) {
        var edge=out.filter(f->f.id=="edge1")[0],span=(front?spec.centre-spec.baseMin:spec.baseMax-spec.centre);
        out=[BoltedFace.clamped(edge.id,edge.frame,[{x:-span/2+12,y:0.0},{x:0.0,y:0.0},{x:span/2-12,y:0.0}],4.5,1,"M4",null,false,true)];
      }
      if(!StringTools.startsWith(role,"lip")) {
        var frame=ScrewLinearModulePart.plane(spec.centre,0,z,cover?1:-1),inverse=F.inverse(frame);
        var holes=[for(p in HobberYLayout.sealBolts(front)){var at=F.transformPoint(inverse,p.x,p.y,z);{x:at.x,y:at.y};}];
        out.push(cover?BoltedFace.through("seal",frame,holes,3.4,1,"M3",false):BoltedFace.clamped("seal",frame,holes,3.4,2,"M3",null,false,true));
      }
      return out;
    }
    if(role=="base") {
      var at=actuatorPoint(),bolt=machinekit.standard.SocketHeadCapScrew.catalog().get("M4");
      var length=machinekit.standard.SocketHeadCapScrew.standardLengthAtLeast(3*bolt.diameter);if(length==null)throw "No actuator screw";
      var grip=length-1.25*bolt.diameter;
      out.push(BoltedFace.clamped("actuator-output",ScrewLinearModulePart.plane(at.x,at.y,at.z,1),[for(i in 0...ErobActuatorData.OUTPUT_BOLT_COUNT){x:ErobActuatorPart.outputBolt(i).x,y:-ErobActuatorPart.outputBolt(i).y}],ErobActuatorData.OUTPUT_CLEARANCE_HOLE,grip,"M4",{diameter:bolt.counterboreDiameter,depth:spec.baseThickness-at.z-grip},false));
    }
    if(role=="carriage")for(face in HobberSaddle.yHeadFaces())switch face.role {
      case Clamped(size,clearance,grip,recess):
        var holes=face.holes;if(holes==null)throw "Head interface needs its pattern";
        out.push(BoltedFace.clamped(face.id,F.compose(HobberYLayout.headInModule(),face.frame),holes,clearance,grip,size,recess,false,face.washer));
      case _:throw "Unexpected head interface";
    }
    return out;
  }
  override public function pinnedFaces():Array<PinnedFace> {
    var out=body().pinnedFaces();
    if(role=="carriage")for(face in HobberSaddle.yHeadPins())out.push(new PinnedFace(face.id,F.compose(HobberYLayout.headInModule(),face.frame),face.holes,face.diameter,face.depth,face.through,false,face.fit));
    return out;
  }
  static function box(a:Float,b:Float,c:Float,d:Float,e:Float,f:Float):Part {
    var raw=Part.box(b-a,d-c,f-e),shape=raw.translated(new Vector((a+b)/2,(c+d)/2,e));raw.close();return shape;
  }
  /** All webs are profiles cut from the ONE ground blank, not separate parts. */
  function retainedProfile():Part {
    var s=spec,z0=role=="base"?0.0:s.carriageBottom(),z1=z0+(role=="base"?s.baseThickness:s.carriageThickness);
    var lo=role=="base"?s.baseMin:s.centre-s.carriageLength/2,hi=role=="base"?s.baseMax:s.centre+s.carriageLength/2;
    var half=role=="base"?s.baseWidth/2:s.carriageWidth/2;
    var rim=role=="base"?8.0+ScrewLinearModuleSpec.GAP:10+4+ScrewLinearModuleSpec.GAP;
    // The lip spans the complete nominal stroke and overtravel, not just
    // the neutral aperture. Keep solid steel beneath that swept contact.
    var endRim=role=="base"?rim:s.stroke+2*s.overtravel+10+4;
    // The perimeter carries only the sheet guards / PU seal. Keep the
    // precision core full thickness, but mill the underside of this land.
    var frameLow=role=="base"?s.baseThickness/2-4:z0;
    var frameHigh=role=="base"?s.baseThickness/2+4:z0+3;
    var frame=Solids.cut(HobberYLayout.prism(role=="base"?HobberYLayout.outline(true):HobberYLayout.window(s.stroke/2+s.overtravel+10,10),frameLow,frameHigh),
      [HobberYLayout.prism(role=="base"?HobberYLayout.outline(true,-rim):HobberYLayout.window(-(s.stroke/2+s.overtravel+4),-4),frameLow-1,frameHigh+1)]);
    var keep:Array<Part>=[frame];
    if(role=="carriage")for(side in [-1,1]) {
      var stop=side*(s.drive.fixed.spec.width/2+ScrewLinearModuleSpec.GAP+10);
      keep.push(box(lo,hi,stop-10,stop+10,z0,z1));
    }
    if(role=="base") {
      var at=actuatorPoint(),m4=machinekit.standard.SocketHeadCapScrew.catalog().get("M4");
      var low=Math.min(at.y-ErobActuatorData.OUTPUT_PITCH_CIRCLE/2-1.5*m4.diameter-ScrewLinearModuleSpec.GAP,s.scaleSeatY()-20-ScrewLinearModuleSpec.GAP);
      var high=s.riserY(1)+s.riserWidth/2+ScrewLinearModuleSpec.GAP;
      // The deck-sweep relief interrupts the long lower guard land. A
      // central tab inside the deck window retains it as part of this plate.
      keep.push(box(at.x-rim/2,at.x+rim/2,-half,low+ScrewLinearModuleSpec.GAP,frameLow,frameHigh));
      // A central output web joins the two full rail lands. Unloaded
      // material outside those paths is profiled from the ground blank.
      var web=ErobActuatorData.OUTPUT_PITCH_CIRCLE/2+1.5*m4.diameter+ScrewLinearModuleSpec.GAP;
      keep.push(box(at.x-web,at.x+web,low,high,z0-1,z1+1));
      for(side in [-1,1])keep.push(box(s.railStart()-ScrewLinearModuleSpec.GAP,s.railStart()+s.railLength+ScrewLinearModuleSpec.GAP,
        s.riserY(side)-s.riserWidth/2-ScrewLinearModuleSpec.GAP,s.riserY(side)+s.riserWidth/2+ScrewLinearModuleSpec.GAP,z0-1,z1+1));
      for(side in [-1,1]) {
        var stop=s.drive.fixed.spec.width/2+ScrewLinearModuleSpec.GAP+10;
        keep.push(box(lo,hi,side*stop-10,side*stop+10,z0-1,z1+1));
      }
      keep.push(box(s.scaleStart()-ScrewLinearModuleSpec.GAP,s.scaleStart()+s.scaleHousing.overallLength()+ScrewLinearModuleSpec.GAP,s.scaleSeatY()-20-ScrewLinearModuleSpec.GAP,s.scaleSeatY()+20+ScrewLinearModuleSpec.GAP,z0-1,z1+1));
      var poses=s.drive.poses(s.axis(),s.screwLength);
      var supports:Array<{part:MachineComponent,key:String}>=[{part:s.drive.fixed,key:"mount"},{part:s.drive.floating,key:"floating"}];
      for(support in supports) {
        var pose=poses.get(support.key);if(pose==null)throw "Missing support footprint";
        var raw=support.part.geometry(Envelope),placed=PosedParts.moved(raw,pose);raw.close();
        var b=PosedParts.boxOf(placed);placed.close();var pad=machinekit.standard.SocketHeadCapScrew.catalog().get("M6").diameter;
        keep.push(box(b.minX-pad,b.maxX+pad,b.minY-pad,b.maxY+pad,z0-1,z1+1));
      }
    } else {
      var screw=machinekit.standard.SocketHeadCapScrew.catalog().get(s.guide.blockMountThread);
      var web=Math.ceil((s.guide.blockHoleSpacingAlong+3*screw.diameter)/5)*5;
      var reach=Math.ceil((s.railSpacing/2+s.guide.blockHoleSpacingAcross/2+1.5*screw.diameter)/5)*5;
      for(row in [-1,1])keep.push(box(s.centre+row*s.blockSpacing/2-web/2,s.centre+row*s.blockSpacing/2+web/2,-reach,reach,z0-1,z1+1));
      var carrierLand=1.5*machinekit.standard.SocketHeadCapScrew.catalog().get("M4").diameter+ScrewLinearModuleSpec.GAP;
      keep.push(box(s.centre-carrierLand,s.centre+carrierLand,s.railY(-1)-10-carrierLand,0,z0-1,z1+1));
      var bridge=0.0;for(h in ScrewLinearModulePart.carrierBolts())bridge=Math.max(bridge,Math.abs(h.y));
      bridge=Math.ceil((bridge+1.5*machinekit.standard.SocketHeadCapScrew.catalog().get("M6").diameter+ScrewLinearModuleSpec.GAP)/5)*5;
      keep.push(box(s.centre-s.blockSpacing/2-web/2,s.centre+s.blockSpacing/2+web/2,-bridge,bridge,z0-1,z1+1));
      // The head contact plate reaches the lower guide centre; unused steel
      // above it is replaced by the coplanar guide webs and seal rim.
      var top=HobberYLayout.moduleLift()-s.railSpacing/2,face=HobberSaddle.headFaceX();
      for(joint in HobberSaddle.yHeadFaces()) {
        var holes=joint.holes;if(holes==null)continue;
        switch joint.role {
          case Clamped(size,_,_,_):
            var edge=1.5*machinekit.standard.SocketHeadCapScrew.catalog().get(size).diameter;
            for(hole in holes)top=Math.max(top,F.transformPoint(joint.frame,hole.x,hole.y,0).z+edge);
          case _:
        }
      }
      var raw=Part.prism([for(p in [{y:-130.0,z:-40.0},{y:-65.0,z:-75.0},{y:65.0,z:-75.0},{y:90.0,z:-55.0},{y:90.0,z:top},{y:-130.0,z:top}])new Vector(p.y,p.z)],face,face+s.carriageThickness);
      var outline=PosedParts.moved(raw,F.fromRotationMatrix(0,0,0,[0.0,0,1,1,0,0,0,1,0]));raw.close();
      outline=Solids.union([outline,box(face,face+s.carriageThickness,90,222,-52,top)]);
      keep.push(PosedParts.moved(outline,HobberYLayout.headInModule()));outline.close();
    }
    return Solids.union(keep);
  }
  function guardGeometry():Part {
    var s=spec,front=StringTools.endsWith(role,"Front"),cover=StringTools.startsWith(role,"cover"),lip=StringTools.startsWith(role,"lip"),z=HobberYLayout.guardRoof(),shape:Part;
    if(cover) {
      var high=HobberYLayout.hoodRoof(),reach=s.carriageLength/2+s.stroke/2+s.overtravel+ScrewLinearModuleSpec.GAP;
      var left=s.centre-reach,right=s.centre+reach;
      var roof=Solids.cut(HobberYLayout.prism(HobberYLayout.outline(true,1),z-1,z),[HobberYLayout.prism(HobberYLayout.window(),z-2,z+1),box(s.baseMin-2,left,-s.baseWidth,s.baseWidth,z-2,z+1),box(right,s.baseMax+2,-s.baseWidth,s.baseWidth,z-2,z+1)]);
      var endRoof=Solids.cut(HobberYLayout.prism(HobberYLayout.outline(true,1),high,high+1),[box(left,right,-s.baseWidth,s.baseWidth,high-1,high+2)]);
      var walls=Solids.cut(HobberYLayout.prism(HobberYLayout.outline(true,1),0,high),[HobberYLayout.prism(HobberYLayout.outline(true),-1,high+1),box(s.baseMin-2,s.baseMin+0.1,-s.baseWidth,s.baseWidth,-1,high+1),box(s.baseMax-0.1,s.baseMax+2,-s.baseWidth,s.baseWidth,-1,high+1),box(left,right,-s.baseWidth,s.baseWidth,z-1,high+1)]);
      var steps=Solids.union([box(left-1,left,-s.baseWidth,s.baseWidth,z-1,high+1),box(right,right+1,-s.baseWidth,s.baseWidth,z-1,high+1)]);
      var boundary=HobberYLayout.prism(HobberYLayout.outline(true,1),z-2,high+2),stepWalls=steps.intersect(boundary);boundary.close();steps.close();
      shape=Solids.union([roof,endRoof,walls,stepWalls]);
      // Stationary stop legs pierce this lower roof. Laser-cut to their
      // undrilled source profiles; fixed seam sealing remains an RFQ item.
      var stopCuts:Array<Part>=[];
      for(r in ScrewLinearModulePart.ROLES)if(StringTools.startsWith(r,"stop")) {
        var foot=new ScrewLinearModulePart(s.values().setToken("role",r)).geometry(),raised=foot.translated(new Vector(0,0,HobberYLayout.stopLift()));
        stopCuts.push(foot);stopCuts.push(raised);
      }
      var mount=s.drive.fixed.geometry(),pose=s.drive.poses(s.axis(),s.screwLength).get("mount");
      if(pose==null)throw "Missing fixed support pose";
      stopCuts.push(PosedParts.moved(mount,pose));mount.close();
      shape=Solids.cut(shape,stopCuts);
    } else {
      var low=lip?z-1:z-3,high=lip?s.carriageBottom():z-1;
      shape=Solids.cut(HobberYLayout.prism(HobberYLayout.window(lip?0:12,lip?0:12),low,high),[HobberYLayout.prism(HobberYLayout.window(-4,-4),low-1,high+1)]);
    }
    var split=box(front?s.baseMin-2:s.centre,front?s.centre:s.baseMax+2,-s.baseWidth,s.baseWidth,-1,HobberYLayout.hoodRoof()+2),out=shape.intersect(split);split.close();shape.close();return out;
  }
  override public function geometry(detail:ComponentDetail=Preview):Part {
    var shape=isGuard()?guardGeometry():body().geometry(detail);
    if(role=="base"||role=="carriage") {
      var profile=retainedProfile(),trimmed=shape.intersect(profile);profile.close();shape.close();shape=trimmed;
      var boundary=HobberYLayout.prism(HobberYLayout.outline(role=="base"),-1000,1000),clipped=shape.intersect(boundary);boundary.close();shape.close();shape=clipped;
    }
    if(role=="base") {
      var at=actuatorPoint();
      shape=Solids.cut(shape,[box(spec.baseMin-1,spec.baseMax+1,-spec.baseWidth,spec.baseWidth,-1,spec.baseThickness-GroundStock.atLeast("plate",spec.baseThickness-at.z)),Part.cylinderSpan(ErobActuatorData.BODY_DIAMETER/2,-1,at.z,at.x,at.y),Part.cylinderSpan((ErobActuatorData.BORE_DIAMETER+0.4)/2,-1,spec.baseThickness+1,at.x,at.y)]);
    }
    if(role=="carriage") {
      var bore=Part.cylinderAlong(Bt30SpindleData.FLANGE_DIAMETER/2+3,new Vector(0,-132,0),Vector.Y(),HobberSwivelLayout.SPINDLE_Y-Bt30SpindleData.FLANGE_FROM+134);
      var tool=PosedParts.moved(bore,HobberYLayout.headInModule());bore.close();shape=Solids.cut(shape,[tool]);
    } else shape=Solids.cut(shape,[HobberYLayout.sweptDeckRelief()]);
    var made=machined(shape);
    if(made.solidCount()!=1) {
      var fragments=[];
      for(i in 0...made.solidCount()) {
        var piece=new Part(made.shape.subshape(CadKit.ShapeKind.Solid,i));
        fragments.push({volume:piece.volume(),box:PosedParts.boxOf(piece)});piece.close();
      }
      made.close();throw "Disconnected made Y member: "+role+" "+haxe.Json.stringify(fragments);
    }
    return made;
  }
}
