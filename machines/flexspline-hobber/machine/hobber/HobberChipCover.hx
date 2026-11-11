package hobber;

import machinekit.hobbing.*;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.assembly.PosedParts;
import machinekit.hobbing.*;
import machinekit.component.BoltedFace.BoltHole;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Fixed and moving chip shields, formed from one 2 mm blank per role; no welding.
 * Mounting holes are cut only by JointFasteners. Tool ports follow PanelHolePlug
 * and the foot-bolt axes. The rear flat lid is laser cut; other roles are formed. */
class HobberChipCover extends MachineComponent {
  public static inline final OUTER_HALF_WIDTH = 70.0;
  public static inline final REAR_END = 630.0;
  public final role:String;
  public static final MADE:Array<String> = ["XleftFront", "XrightFront", "XleftRear", "XrightRear", "Xfront", "Xrear", "Xskirt"];

  static final recipe = new machinekit.cache.Once<ComponentType>();
  public static function parameters():Array<ComponentParameter> return [machinekit.component.ComponentRecipeSupport.choice("role",MADE,"XleftFront")];
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType("machinekit.hobbing.chip-cover",parameters(),v->new HobberChipCover(v),false,false,null,false)
      .versioned(2).numbered(v->"HOBBER-GUARD-"+v.token("role").toUpperCase()).procured(v->Make(v.token("role")=="Xrear"?"cut plate":"cut and formed sheet"))
      .requiring(v->routeFor(v.token("role")).requirements()));
  }
  public function new(?overrides:ComponentValues) {
    var v=machinekit.component.ComponentRecipeSupport.resolve(parameters(),overrides),r=v.token("role");
    super("HOBBER-GUARD-"+r.toUpperCase(),"Chip shield: "+r,"steel",false,v);role=r;
    addFacet(routeFor(r));
    addFacet(new EngineeringMetadata(new ComponentValues().setToken("ratingsBasis","Assumed").setToken("errorsBasis","Assumed")
      .setToken("source","Sheet guard sized to clear the X drive; no load or deflection check")));
    addConnector("mount",Mount,Solids.axial(0,0,0));
  }
  static function routeFor(role:String):ManufacturingRoute {
    var bends=StringTools.startsWith(role,"Xleft")||StringTools.startsWith(role,"Xright")?3:role=="Xfront"?1:role=="Xskirt"?16:3;
    return role=="Xrear"?ManufacturingRoute.cutPlate():ManufacturingRoute.foldedSheet(2,bends,2);
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function machinable():Bool return true;
  static function plane(x:Float,y:Float,z:Float,into:Float):AssemblyFrame return BoltedFace.frameOf({x:x,y:y,z:z},{x:0,y:0,z:into},{x:1,y:0,z:0});
  public static function installations():Array<{id:String,role:String,parent:String,seat:AssemblyFrame}> {
    var rows:Array<{id:String,role:String,parent:String,seat:AssemblyFrame}>=[];
    for(front in [true,false])for(side in [-1,1]) rows.push({id:"Xcover"+(front?"Front":"Rear")+(side<0?"Left":"Right"),
      role:"X"+(side<0?"left":"right")+(front?"Front":"Rear"),parent:"bed/driveSeat",seat:plane(footX(front),side*footY(),footZ(),1)});
    rows.push({id:"XfrontCover",role:"Xfront",parent:"XcoverFrontLeft",seat:plane(242,-lidY(),222,1)});
    rows.push({id:"XrearCover",role:"Xrear",parent:"XcoverRearLeft",seat:plane(rearLidCentre(),-lidY(),232,1)});
    rows.push({id:"Xskirt",role:"Xskirt",parent:"Xbridge",seat:plane(170,-100,260,-1)});
    return rows;
  }
  // Front row stays behind the skirt at the parked carriage, including driver radius.
  public static function rearLidCentre():Float return (386 + HobberLayout.get().xRetract + 9 + 560) / 2;
  public static function rearLidHoles():Array<BoltHole> return [
    {x:386 + HobberLayout.get().xRetract + 9 - rearLidCentre(), y:0.0}, {x:560 - rearLidCentre(), y:0.0}];
  public static function plugs():Array<{id:String,parent:String,x:Float,y:Float,z:Float}> return [
    for(front in [true,false])for(side in [-1,1]) {id:"X"+(front?"front":"rear")+"Plug"+(side<0?"Left":"Right"),
      parent:front?"XfrontCover":"XrearCover",x:footX(front),y:side*footY(),z:front?224.0:234.0}
  ];
  public static function fixedMembers():Array<String> return [for(r in installations())if(r.parent!="Xbridge")r.id].concat([for(p in plugs())p.id]);
  public static function lidY():Float return (footY()+11+OUTER_HALF_WIDTH-4)/2;
  public static function footY():Float {var d=HobberDrives.x(),b=HobberDrives.barBounds(d,"fixed");return (b[4]-b[1])/2+2+5;}
  public static function footZ():Float return HobberDrives.barBounds(HobberDrives.x(),"fixed")[2];
  public static function footX(front:Bool):Float {
    var points=HobberDrives.seatHoles(HobberDrives.x(),"fixed").concat(HobberDrives.seatHoles(HobberDrives.x(),"floating"));
    var low=1e9,high=-1e9;for(p in points){low=Math.min(low,p.position[0]);high=Math.max(high,p.position[0]);}
    return front?low+7:high-7; // Existing guard tool-port station, independent of seat stock.
  }
  public static function insertReceivers():Array<BoltedFace> return [for(front in [true,false])for(side in [-1,1])
    BoltedFace.drilledToSuit("chip-"+(front?"front":"rear")+side,plane(footX(front),side*footY(),footZ(),-1),12,12,12,20,BoltedFace.ThreadMaterial.Steel,"M3")];
  public static function carriageReceivers():Array<BoltedFace> {
    var faces=[for(x in [170.0,350.0])for(side in [-1,1]) BoltedFace.drilledToSuit("skirt-"+x+"-"+side,plane(x,side*100,260,1),14,14,10,20,BoltedFace.ThreadMaterial.Steel,"M4")];
    return faces;
  }
  override public function fasteningFaces():Array<BoltedFace> {
    var zero:Array<BoltHole>=[{x:0.0,y:0.0}],side=StringTools.startsWith(role,"Xleft")?-1:1;
    if(StringTools.startsWith(role,"Xleft")||StringTools.startsWith(role,"Xright")) {
      var front=StringTools.endsWith(role,"Front");
      return [BoltedFace.clamped("pad",plane(footX(front),side*footY(),footZ(),1),zero,4.0,2,"M3",null,false,true),
        front ? BoltedFace.clamped("lid",plane(242,side*lidY(),222,-1),zero,4.0,2,"M3",null,false,true)
          : BoltedFace.through("lid",plane(rearLidCentre(),side*lidY(),232,-1),rearLidHoles(),4.0,2,"M3",false)];
    }
    if(role=="Xfront"||role=="Xrear") {
      var front=role=="Xfront";
      // Rear nuts are fitted on the bench. A flat AF5.5 spanner enters along
      // the open rear channel for service; no captive retention is assumed.
      return [for(side in [-1,1]) front
        ? BoltedFace.through("side"+side,plane(242,side*lidY(),222,1),zero,4.0,2,"M3",false)
        : BoltedFace.clamped("side"+side,plane(rearLidCentre(),side*lidY(),232,1),rearLidHoles(),4.0,2,"M3",null,false,true)];
    }
    if(role=="Xskirt") return [for(x in [170.0,350.0])for(side in [-1,1])
      BoltedFace.clamped("tab"+x+"-"+side,plane(x,side*100,260,-1),zero,5.0,2,"M4",null,false,true)];
    throw "Unknown X chip cover role: "+role;
  }
  public function toolHoles():Array<ManufacturingRoute.RouteHole> {
    if(role!="Xfront"&&role!="Xrear")return [];
    var front=role=="Xfront",z=front?224.0:234.0;
    return [for(side in [-1,1]) {id:"tool-port"+side,kind:"bore",thread:null,diameter:machinekit.standard.PanelHolePlug.HOLE,nominal:machinekit.standard.PanelHolePlug.HOLE,
      fit:null,depth:machinekit.standard.PanelHolePlug.PANEL,position:[footX(front),side*footY(),z],direction:[0.0,0,-1]}];
  }
  public static function coverage():Array<String> {
    var b=nutBounds(),front=20+b.minX-3,rear=115+b.maxX+3;
    var frontOverlap=front-(120+115),rearOverlap=(386+20)-rear;
    if(frontOverlap<10||rearOverlap<10)throw "X chip-shield end overlap is insufficient";
    return ['Chip shields: X end overlaps ${frontOverlap} / ${rearOverlap} mm over travel plus 5 mm overtravel; side flat overlap 6 mm (projected edge 10 mm). '
      +'Z rail bellows are checked separately; offset-screw ingress remains unqualified. No channel drain declared: no fall or bore is modelled.'];
  }
  static function box(x0:Float,x1:Float,y0:Float,y1:Float,z0:Float,z1:Float):Part return Part.box(x1-x0,y1-y0,z1-z0).translated(new Vector((x0+x1)/2,(y0+y1)/2,z0));
  static function mirrorY(p:Part):Part {
    var mirrored = new Part(p.shape.mirror(new Vector().native(), Vector.Y().native())); p.close(); return mirrored;
  }
  /** Quarter of a 2 mm sheet bend, inside R2, outside R4. */
  static function bendX(x0:Float,x1:Float,y:Float,z:Float,sy:Int,sz:Int):Part {
    var ring = Solids.cut(Part.cylinderAlong(4,new Vector(x0,y,z),Vector.X(),x1-x0),[Part.cylinderAlong(2,new Vector(x0-0.1,y,z),Vector.X(),x1-x0+0.2)]);
    var quadrant = box(x0-0.1,x1+0.1,y+(sy<0?-4:0),y+(sy<0?0:4),z+(sz<0?-4:0),z+(sz<0?0:4));
    var out=ring.intersect(quadrant);ring.close();quadrant.close();return out;
  }
  static function bendY(y0:Float,y1:Float,x:Float,z:Float,sx:Int,sz:Int):Part {
    var ring = Solids.cut(Part.cylinderAlongY(4,y0,y1,x,z),[Part.cylinderAlongY(2,y0-0.1,y1+0.1,x,z)]);
    var quadrant = box(x+(sx<0?-4:0),x+(sx<0?0:4),y0-0.1,y1+0.1,z+(sz<0?-4:0),z+(sz<0?0:4));
    var out=ring.intersect(quadrant);ring.close();quadrant.close();return out;
  }
  static function bendZ(z0:Float,z1:Float,x:Float,y:Float,sx:Int,sy:Int):Part {
    var ring = Solids.cut(Part.cylinderSpan(4,z0,z1,x,y),[Part.cylinderSpan(2,z0-0.1,z1+0.1,x,y)]);
    var quadrant = box(x+(sx<0?-4:0),x+(sx<0?0:4),y+(sy<0?-4:0),y+(sy<0?0:4),z0-0.1,z1+0.1);
    var out=ring.intersect(quadrant);ring.close();quadrant.close();return out;
  }
  public static function nutBounds():machinekit.assembly.PosedParts.PartBox {
    var drive=HobberDrives.x(), block=HobberDrives.nutBlock(drive,new machinekit.motion.BallNut()), cache=new PosedParts();
    var b=cache.box(block,HobberDrives.nutBlockPose(drive,new machinekit.motion.BallNut()));cache.close();return b;
  }
  static function side(front:Bool):Part {
    var x0=front?179.0:250.0,x1=front?279.0:REAR_END,roof=front?220.0:230.0,top=front?228.0:242.0;
    var p=[box(x0,x1,footY()+11,66,roof,roof+2),bendX(x0,x1,66,roof+4,1,-1),box(x0,x1,OUTER_HALF_WIDTH-2,OUTER_HALF_WIDTH,roof+4,top)];
    var xx=front?179.0:378.0;
    p.push(box(xx,xx+100,footY()-5,footY()+5,footZ(),footZ()+2));
    p.push(bendX(xx,xx+100,footY()+5,footZ()+4,1,-1));
    p.push(box(xx,xx+100,footY()+7,footY()+9,footZ()+4,roof-2));
    p.push(bendX(xx,xx+100,footY()+11,roof-2,-1,1));
    return Solids.union(p);
  }
  static function lid(front:Bool):Part {
    var b=nutBounds(), end=20+b.minX-3, start=115+b.maxX+3;
    var p=front ? Solids.union([box(174,end,-65,65,222,224),bendY(-65,65,174,220,-1,1),box(170,172,-65,65,215,220)])
      : box(start,REAR_END,-65,65,232,234);
    var tools=[for(y in [-footY(),footY()]) Part.cylinderSpan(machinekit.standard.PanelHolePlug.HOLE/2,front?221:231,front?225:235,footX(front),y)];
    return Solids.cut(p,tools);
  }
  static function skirt():Part {
    var p=[box(124,382,-76,76,246,248),bendY(-76,76,124,244,-1,1),bendY(-76,76,382,244,1,1),box(120,122,-76,76,238.5,244),box(384,386,-76,76,238.5,244)];
    for (side in [-1,1]) {
      var q:Array<Part>=[];
      for (span in [[124.0,160.0],[180.0,340.0],[360.0,382.0]]) {
        q.push(bendX(span[0],span[1],76,244,1,1));q.push(box(span[0],span[1],78,80,216,244));
      }
      for (xx in [160.0,340.0]) {
        q.push(bendX(xx,xx+20,76,250,1,-1));q.push(box(xx,xx+20,78,80,250,256));
        q.push(bendX(xx,xx+20,82,256,-1,1));q.push(box(xx,xx+20,82,110,258,260));
      }
      var piece=Solids.union(q);p.push(side<0?mirrorY(piece):piece);
    }
    var b=nutBounds();return Solids.cut(Solids.union(p),[box(b.minX-3,b.maxX+3,b.minY-3,b.maxY+3,245,249),box(381,387,65,73,237,245),box(381,387,-73,-65,237,245)]);
  }
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail=Preview):Part return machined(switch role {
    case "XleftFront":mirrorY(side(true)); case "XrightFront":side(true);
    case "XleftRear":mirrorY(side(false)); case "XrightRear":side(false);
    case "Xfront":lid(true); case "Xrear":lid(false); case "Xskirt":skirt();
    default:throw "unknown chip cover role";
  });
}
