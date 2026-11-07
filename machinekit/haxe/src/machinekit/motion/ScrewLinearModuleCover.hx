package machinekit.motion;

import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import materia.assembly.AssemblyFrames as F;

/** Short-stroke split guard. The clip-in PU seal profile is ASSUMED until quoted;
 * each steel retainer is separate and its ordinary nuts are serviced on the bench. */
class ScrewLinearModuleCover extends MachineComponent {
  static final recipe=new machinekit.cache.Once<ComponentType>();
  public final spec:ScrewLinearModuleSpec;
  public final role:String;
  public final front:Bool;
  public static final ROLES=["coverFront","coverRear","retainerFront","retainerRear","lipFront","lipRear","hoodFront","mountFixedFront","mountFixedRear","mountMovingFront","mountMovingRear"];
  public static function parameters():Array<ComponentParameter> return ScrewLinearModulePart.parameters().filter(p->p.name!="role").concat([P.choice("role",ROLES,"coverFront")]);
  public static function recipeType():ComponentType return recipe.get(()->new ComponentType("machinekit.motion.screw-module-cover",parameters(),v->new ScrewLinearModuleCover(v),false,false,null,false)
    .numbered(numberFor).procured(v->StringTools.startsWith(v.token("role"),"lip")?Buy(null,null,null):Make(StringTools.startsWith(v.token("role"),"mount")?"machined cover interface":"cut sheet / guard forming")));
  static function numberFor(v:ComponentValues):String return PartNumbers.geometry("SCREW-MODULE-COVER",v.token("role"),v.token("profile")+"|"+v.number("stroke")+"|"+v.number("railSpacing")+"|"+v.number("blockSpacing")+"|"+v.number("overtravel"));
  public function new(?overrides:ComponentValues) {
    var v=P.resolve(parameters(),overrides);spec=new ScrewLinearModuleSpec(v.token("profile"),v.number("stroke"),v.number("railSpacing"),v.number("blockSpacing"),v.number("overtravel"));role=v.token("role");front=StringTools.endsWith(role,"Front");
    super(numberFor(v),"Screw module "+role,isLip()?"polyurethane PU":isMount()?"steel":"stainless A2-70",false,v);
    if(!isLip()&&!isMount())addFacet((isCover()||isHood())?ManufacturingRoute.foldedSheet(1.5,isHood()?4:2,1.5):ManufacturingRoute.cutPlate());
    if(isMount())addFacet(StringTools.startsWith(role,"mountFixed")?ManufacturingRoute.milled(2,"general",[],2*ScrewLinearModuleBellows.plan(spec).plateHalf()):ManufacturingRoute.cutPlate());
    addFacet(new EngineeringMetadata(new ComponentValues().setToken("ratingsBasis","Assumed").setToken("errorsBasis","Assumed").setToken("source","Short stroke gravity-chip shield; clip-in PU profile/retention and end sealing require qualification")));
  }
  function isMount():Bool return StringTools.startsWith(role,"mount");
  function isHood():Bool return role=="hoodFront";
  function isCover():Bool return StringTools.startsWith(role,"cover");
  function isLip():Bool return StringTools.startsWith(role,"lip");
  override public function componentType():Null<ComponentType> return recipeType();
  override public function machinable():Bool return !isLip();
  public static function openingHalf(s:ScrewLinearModuleSpec):Float return s.carriageLength/2-s.stroke/2-s.overtravel-10;
  public static function openingWidthHalf(s:ScrewLinearModuleSpec):Float return s.carriageWidth/2-10;
  public static function roof(s:ScrewLinearModuleSpec):Float return s.carriageBottom()+s.carriageThickness+6; // M3 nut2.4 + washer0.5 + running gap2; rounded up, ASSUMED tool room.
  static function cableSlotEnd(s:ScrewLinearModuleSpec):Float {
    var stub=s.scaleHead.cableStubEnd(),r=SealedScaleData.CABLE_BEND_RADIUS,wire=s.scaleHead.series.cableDiameter/2;
    var rise=roof(s)-ScrewLinearModuleSpec.GAP-wire-(s.scaleHeadTop()-stub[1]);
    rise=Math.max(0.0,Math.min(r,rise));
    return s.centre+s.stroke/2+s.overtravel+stub[2]-Math.sqrt(r*r-(r-rise)*(r-rise))+ScrewLinearModuleSpec.GAP;
  }
  static function hoodHalf(s:ScrewLinearModuleSpec):Float return s.scaleHead.series.cableDiameter/2+ScrewLinearModuleSpec.GAP+1.5;
  static function hoodEnd(s:ScrewLinearModuleSpec):Float return Math.max(s.baseMin+30,cableSlotEnd(s)+30);
  static function hoodHoles(s:ScrewLinearModuleSpec):Array<machinekit.component.BoltedFace.BoltHole> return [for(x in [s.baseMin+6,hoodEnd(s)-6])for(side in [-1,1]){x:x-s.centre,y:s.scaleHeadY()+side*(hoodHalf(s)+7.5)}];
  public static function holes(s:ScrewLinearModuleSpec,front:Bool):Array<machinekit.component.BoltedFace.BoltHole> {
    var sign=front?-1:1,a=openingHalf(s),b=openingWidthHalf(s);
    return [{x:sign*(a+6),y:-b+20},{x:sign*(a+6),y:b-20},{x:sign*a/2,y:-b-6},{x:sign*a/2,y:b+6}];
  }
  static function sideFrame(s:ScrewLinearModuleSpec,front:Bool,side:Int,into:Float):materia.assembly.AssemblyRecord.AssemblyFrame {
    var lo=front?s.baseMin:s.centre,hi=front?s.centre:s.baseMax;
    return BoltedFace.frameOf({x:(lo+hi)/2,y:side*s.baseWidth/2,z:s.baseThickness/2},{x:0.0,y:into,z:0},{x:1.0,y:0,z:0});
  }
  public static function receivers(s:ScrewLinearModuleSpec):Array<BoltedFace> {
    var out:Array<BoltedFace>=[];
    if(s.stroke<=20)for(front in [true,false])for(side in [-1,1]) {
      var lo=front?s.baseMin:s.centre,hi=front?s.centre:s.baseMax;
      out.push(BoltedFace.drilledToSuit("guard-"+(front?"front":"rear")+side,sideFrame(s,front,side,-side),hi-lo,s.baseThickness,8,16,BoltedFace.ThreadMaterial.Steel,"M4"));
    }
    return out;
  }
  override public function fasteningFaces():Array<BoltedFace> {
    var s=spec,out:Array<BoltedFace>=[];
    if(isMount())return ScrewLinearModuleBellows.faces(s,role);
    if(isHood())out.push(BoltedFace.clamped("foot",ScrewLinearModulePart.plane(s.centre,0,roof(s)+1.5,1),hoodHoles(s),3.4,1.5,"M3",null,false,true));
    else if(isCover()) {
      var lo=front?s.baseMin:s.centre,hi=front?s.centre:s.baseMax,screw=machinekit.standard.SocketHeadCapScrew.catalog().get("M4");
      for(side in [-1,1])out.push(BoltedFace.clamped("edge"+side,sideFrame(s,front,side,side),[{x:-(hi-lo)/2+12,y:0.0},{x:(hi-lo)/2-12,y:0.0}],screw.clearanceMedium,1.5,"M4",null,false,true));
      if(front)out.push(BoltedFace.through("hood",ScrewLinearModulePart.plane(s.centre,0,roof(s)+1.5,-1),[for(h in hoodHoles(s)){x:h.x,y:-h.y}],3.4,1.5,"M3",false));
      out.push(BoltedFace.through("seal",ScrewLinearModulePart.plane(s.centre,0,roof(s)+1.5,-1),holes(s,front),3.4,1.5,"M3",false));
    } else if(!isLip())out.push(BoltedFace.clamped("seal",ScrewLinearModulePart.plane(s.centre,0,roof(s)+1.5,1),holes(s,front),3.4,2,"M3",null,false,true));
    return out;
  }
  static function box(x0:Float,x1:Float,y0:Float,y1:Float,z0:Float,z1:Float):Part {
    var raw=Part.box(x1-x0,y1-y0,z1-z0),out=raw.translated(new Vector((x0+x1)/2,(y0+y1)/2,z0));raw.close();return out;
  }
  static function bend(lo:Float,hi:Float,half:Float,z:Float,side:Int):Part {
    var centre=side*(half-1.5),height=z-1.5;
    var ring=Solids.cut(Part.cylinderAlong(3,new Vector(lo,centre,height),Vector.X(),hi-lo),[Part.cylinderAlong(1.5,new Vector(lo-1,centre,height),Vector.X(),hi-lo+2)]);
    var trim=box(lo-1,hi+1,side>0?centre:centre-3,side>0?centre+3:centre,height,height+3),out=ring.intersect(trim);ring.close();trim.close();return out;
  }
  override public function geometry(detail:ComponentDetail=Preview):Part {
    var s=spec,a=openingHalf(s),b=openingWidthHalf(s),z=roof(s),lo=front?s.baseMin:s.centre,hi=front?s.centre:s.baseMax,body:Part;
    if(isMount())return machined(ScrewLinearModuleBellows.mount(s,role).geometry(detail));
    if(isHood()) {
      var half=hoodHalf(s),end=hoodEnd(s),cy=s.scaleHeadY(),stub=s.scaleHead.cableStubEnd();
      var hood=s.scaleHeadTop()-stub[1]+SealedScaleData.CABLE_BEND_RADIUS+s.scaleHead.series.cableDiameter/2+ScrewLinearModuleSpec.GAP+1.5;
      var parts=[box(s.baseMin,end,cy-half-1.5,cy+half+1.5,hood,hood+1.5)];
      for(side in [-1,1]) {
        parts.push(box(s.baseMin,end,side<0?cy-half-1.5:cy+half,side<0?cy-half:cy+half+1.5,z+1.5,hood));
        parts.push(box(s.baseMin,end,side<0?cy-half-13.5:cy+half,side<0?cy-half:cy+half+13.5,z+1.5,z+3));
      }
      body=Solids.union(parts);
    } else if(isCover()) {
      var top=box(lo,hi,-s.baseWidth/2+1.5,s.baseWidth/2-1.5,z,z+1.5);
      top=Solids.cut(top,[box(front?s.centre-a:s.centre-1,front?s.centre+1:s.centre+a,-b,b,z-1,z+3)]);
      var parts=[top];
      if(front&&cableSlotEnd(s)>lo)parts[0]=Solids.cut(parts[0],[box(lo-1,cableSlotEnd(s),s.scaleHeadY()-hoodHalf(s),s.scaleHeadY()+hoodHalf(s),z-1,z+3)]);
      for(side in [-1,1]) {
        parts.push(box(lo,hi,side<0?-s.baseWidth/2-1.5:s.baseWidth/2,side<0?-s.baseWidth/2:s.baseWidth/2+1.5,0,z-1.5));
        parts.push(bend(lo,hi,s.baseWidth/2,z,side));
      }
      body=Solids.union(parts);
    } else {
      var x0=front?s.centre-a:s.centre,x1=front?s.centre:s.centre+a;
      var low=isLip()?s.carriageBottom()+s.carriageThickness:z+1.5,high=isLip()?z+1.5:z+3.5;
      var inner=b-4,outer=isLip()?b:b+12;
      var cross0=front?s.centre-a-(isLip()?0:12):s.centre+a-4;
      var cross1=front?s.centre-a+4:s.centre+a+(isLip()?0:12);
      body=Solids.union([box(cross0,cross1,-outer,outer,low,high),box(x0,x1,-outer,-inner,low,high),box(x0,x1,inner,outer,low,high)]);
    }
    return machined(body);
  }
}
