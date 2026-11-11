package hobber;

import machinekit.hobbing.*;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Feature;
import cadkit.parametric.recording.DocumentBuilder;
import cadkit.sheetmetal.SheetEdgeFlangeFeature;
import cadkit.sheetmetal.SheetFeature;
import cadkit.sheetmetal.SheetMaterial;
import materia.project.MaterialLibrary;
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
    addFacet(EngineeringMetadata.assumed("Sheet guard sized to clear the X drive; no load or deflection check"));
    addConnector("mount",Mount,Solids.axial(0,0,0));
  }
  static function routeFor(role:String):ManufacturingRoute {
    var bends=StringTools.startsWith(role,"Xleft")||StringTools.startsWith(role,"Xright")?3:role=="Xfront"?1:role=="Xskirt"?16:3;
    return role=="Xrear"?ManufacturingRoute.cutPlate():ManufacturingRoute.foldedSheet(2,bends,2);
  }
  override public function componentType():Null<ComponentType> return recipeType();
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
  public static function rearLidCentre():Float return (386 + HobberSkeleton.reference().xRetract + 9 + 560) / 2;
  public static function rearLidHoles():Array<BoltHole> return [
    {x:386 + HobberSkeleton.reference().xRetract + 9 - rearLidCentre(), y:0.0}, {x:560 - rearLidCentre(), y:0.0}];
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
  public static function nutBounds():machinekit.assembly.PosedParts.PartBox {
    var drive=HobberDrives.x(), block=HobberDrives.nutBlock(drive,new machinekit.motion.BallNut()), cache=new PosedParts();
    var b=cache.box(block,HobberDrives.nutBlockPose(drive,new machinekit.motion.BallNut()));cache.close();return b;
  }
  /** 2 mm mild steel, rolled along the cover's length (machine X). */
  static function stock():SheetMaterial return SheetMaterial.fromDefinition(MaterialLibrary.require("mild-steel"), THICKNESS, 0);

  static inline final THICKNESS = 2.0;
  static inline final RADIUS = 2.0;

  /**
   * A sheet in the plane `z = top`, material below it (`z = top - 2`), from a polygon given for the right-hand part. Mirrored,
   * the polygon is reflected in y and reversed so it stays counter-clockwise: edge `i` of the right-hand polygon is then edge
   * `n - 2 - i` and runs the other way, so its start and end gaps swap.
   */
  static function sheet(b:DocumentBuilder, points:Array<Array<Float>>, top:Float, mirrored:Bool):{base:SheetFeature, edge:Int->String} {
    var n = points.length;
    var corners = [for (p in points) new Vector(p[0], mirrored ? -p[1] : p[1])];
    if (mirrored) corners.reverse();
    var base = b.label(b.sheetBase(b.label(b.polygonSketch(corners, topPlane(top)), "Outline"), stock()), "Blank");
    return {base: base, edge: i -> "sketch:e" + (mirrored ? (2 * n - 2 - i) % n : i)};
  }

  /** A flange on the right-hand polygon's edge `i`, its span cut back by `start` and `end` along that edge. */
  static function flange(b:DocumentBuilder, from:SheetFeature, wall:Int, edge:String, leg:Float, up:Bool, start:Float = 0, end:Float = 0,
      mirrored:Bool = false):SheetEdgeFlangeFeature
    return b.sheetFlange(from, [{wall: wall, identity: edge}], leg, up ? Math.PI / 2 : -Math.PI / 2, RADIUS, Leg, BendOutside, 0,
      mirrored ? end : start, mirrored ? start : end);

  /** Side cover: a roof with an upstand on its outer edge, and a web down its inner edge to a foot on the drive seat. */
  static function side(b:DocumentBuilder, front:Bool, mirrored:Bool):Feature {
    var x0 = front ? 179.0 : 250.0, x1 = front ? 279.0 : REAR_END, roof = front ? 220.0 : 230.0, top = front ? 228.0 : 242.0;
    var web = front ? 179.0 : 378.0, inner = footY() + 11;
    var s = sheet(b, [[x0, inner], [x1, inner], [x1, 66], [x0, 66]], roof + THICKNESS, mirrored);
    var upstand = b.label(flange(b, s.base, 1, s.edge(2), top - (roof + 4), true), "Upstand");
    var down = b.label(flange(b, upstand, 1, s.edge(0), roof - 2 - (footZ() + 4), false, web - x0, x1 - (web + 100), mirrored), "Web");
    return b.label(flange(b, down, down.wallIds()[0], "flange:tip", 10, true), "Foot");
  }

  /** The cover's top plane, where its blank is sketched (material below). */
  static function topPlane(z:Float):Plane return new Plane(new Vector(0, 0, z), Vector.X(), Vector.Z());

  /** Lid: the front one turns down over the skirt end; both carry the tool ports, cut in the blank. */
  static function lid(b:DocumentBuilder, front:Bool):Feature {
    var nut = nutBounds(), end = 20 + nut.minX - 3, start = 115 + nut.maxX + 3, top = front ? 224.0 : 234.0;
    var body:SheetFeature = if (front) {
      var s = sheet(b, [[174, -65], [end, -65], [end, 65], [174, 65]], top, false);
      b.label(flange(b, s.base, 1, s.edge(3), 5, false), "Front lip");
    } else sheet(b, [[start, -65], [REAR_END, -65], [REAR_END, 65], [start, 65]], top, false).base;
    var ports = b.label(b.circlesSketch([for (y in [-footY(), footY()]) new Vector(footX(front), y)], machinekit.standard.PanelHolePlug.HOLE / 2,
      topPlane(top)), "Port circles");
    return b.label(b.sheetCut(body, ports, 1), "Tool ports");
  }

  /**
   * Skirt: a plate with lips down at both ends, the far one in three pieces either side of two notches, and along each side a
   * skirt down in three spans with two tabs between them that step up and out to the carriage receivers. The nut clearance is
   * cut in the blank.
   */
  static function skirt(b:DocumentBuilder):Feature {
    var spans = [124.0, 160, 180, 340, 360, 382], notches = [-76.0, -73, -65, 65, 73, 76];
    var points = [for (x in spans) [x, -76.0]].concat([for (y in notches.slice(1, 5)) [382, y]])
      .concat([for (i in 0...spans.length) [spans[spans.length - 1 - i], 76.0]]);
    var s = sheet(b, points, 248, false), n = points.length;
    var walls:SheetFeature = s.base;
    // Edges 0..4 run along y = -76 and 10..14 along y = +76 (even: skirt spans, odd: tabs); 5..9 are the far end, 15 the near end.
    for (i in [5, 7, 9, n - 1]) walls = b.label(flange(b, walls, 1, s.edge(i), 5.5, false), "End lip");
    for (i in [0, 2, 4, 10, 12, 14]) walls = b.label(flange(b, walls, 1, s.edge(i), 28, false), "Skirt");
    for (i in [1, 3, 11, 13]) {
      var riser = b.label(flange(b, walls, 1, s.edge(i), 6, true), "Tab riser");
      walls = b.label(flange(b, riser, riser.wallIds()[0], "flange:tip", 28, false), "Tab");
    }
    var nut = nutBounds(), x0 = nut.minX - 3, x1 = nut.maxX + 3, y0 = nut.minY - 3, y1 = nut.maxY + 3;
    var outline = b.label(b.polygonSketch([new Vector(x0, y0), new Vector(x1, y0), new Vector(x1, y1), new Vector(x0, y1)], topPlane(248)),
      "Nut clearance outline");
    return b.label(b.sheetCut(walls, outline, 1), "Nut clearance");
  }

  override public function record(b:DocumentBuilder, detail:ComponentDetail):Null<Feature> return switch role {
    case "XleftFront": side(b, true, true);
    case "XrightFront": side(b, true, false);
    case "XleftRear": side(b, false, true);
    case "XrightRear": side(b, false, false);
    case "Xfront": lid(b, true);
    case "Xrear": lid(b, false);
    case "Xskirt": skirt(b);
    default: throw "unknown chip cover role";
  };
}
