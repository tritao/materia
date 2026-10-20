package machinekit.hobbing;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;
import machinekit.component.ComponentValues;
import machinekit.structural.CastInsert;
import machinekit.structural.CastingFeatures;
import machinekit.structural.MineralFoundation.CastInsertPlacement;

typedef InsertLocation={id:String,role:String,width:Float,depth:Float,height:Float,pose:AssemblyFrame,openings:Array<materia.casting.CastingFeature.CastingOpening>,?clampHeight:Float};

/** Design-specific insert layout, not a casting process model. Coordinates relative to casting body. */
class FoundationLayout {
  public static inline final TABLE_FACE=228.0; // lowered 20 mm to clear the wider fixture flange at A +/-15
  public static inline final TABLE_SEAT=TABLE_FACE-210.0;
  public static inline final TABLE_WELL_FLOOR=TABLE_SEAT-10.0;
  public static inline final PLINTH_HEIGHT=800.0;
  public static inline final UPPER_OFFSET=590.0;
  public static inline final BED_RAIL_CENTRE=220.0;
  public static inline final BED_RAIL_SEAT_WIDTH=50.0;
  public static inline final BED_RAIL_BEAM_WIDTH=110.0;

  /** Keep full-height longitudinal load beams out of the general lightening cores. */
  static function bedCores(cells:Array<machinekit.structural.CastingFeatures.CastingBox>):Array<machinekit.structural.CastingFeatures.CastingBox> {
    var result:Array<machinekit.structural.CastingFeatures.CastingBox> = [];
    for(cell in cells){
      var spans=[{lo:cell.pose.y-cell.depth/2,hi:cell.pose.y+cell.depth/2}];
      for(side in [-1,1]){
        var low=side*BED_RAIL_CENTRE-BED_RAIL_BEAM_WIDTH/2,high=side*BED_RAIL_CENTRE+BED_RAIL_BEAM_WIDTH/2;
        var remaining=[];
        for(span in spans){
          if(span.hi<=low||span.lo>=high)remaining.push(span);
          else {if(span.lo<low)remaining.push({lo:span.lo,hi:low});if(span.hi>high)remaining.push({lo:high,hi:span.hi});}
        }
        spans=remaining;
      }
      for(i in 0...spans.length){var span=spans[i];result.push(CastingFeatures.box(cell.id+"-bay-"+i,cell.width,span.hi-span.lo,cell.height,cell.pose.x,(span.lo+span.hi)/2,cell.pose.z));}
    }
    return result;
  }
  /** Central drive channel: cap on twin webs, cross-ribbed into both rail beams.
      Coordinates are casting-local bounds; the channel/insert recesses are cut afterwards. */
  public static function driveSupports():Array<Array<Float>> {
    var l=HobberXLayout.front(),start=140-l.bedCentre,end=l.bedRear-l.bedCentre;
    var supports=[[start,-150.0,660.0,end,150.0,720.0]];
    for(side in [-1,1])supports.push([start,side*112.5-17.5,0.0,end,side*112.5+17.5,720.0]);
    for(station in [l.floatingCentre,l.screwStart])supports.push([station-l.bedCentre-15,-275.0,0.0,station-l.bedCentre+15,275.0,720.0]);
    return supports;
  }
  /** Dry-side enclosure seats. Local +Z of each insert points out of the casting. */
  public static function enclosureSeats():Array<InsertLocation> {
    var out:Array<InsertLocation> = [];
    for(x in [-1,1])for(y in [-1,1])out.push({id:'enclosureSeat${x+1}${y+1}',
      openings:[materia.casting.CastingPatterns.face(y<0?"body.y-min":"body.y-max")],role:"enclosure frame dry-side mounting pad",width:80,depth:80,height:20,
      pose:F.fromRotationMatrix(x*(HobberXLayout.front().bedLength/2-100),y*380,720,
        [1,0,0,0,0,y,0,-y,0])});
    return out;
  }
  /** Shallow perimeter rebate; the rear service bridge has its own deeper bearing pockets. */
  public static function trayLedges(depth:Float=machinekit.enclosure.TraySeat.DEPTH):Array<machinekit.structural.CastingFeatures.CastingBox> {
    var d=dimensions("bed"),w=d[0],l=d[1],a=machinekit.enclosure.TraySeat.WIDTH;
    return [CastingFeatures.box("tray-front-ledge",a,l,depth,-w/2+a/2,0,d[2]-depth),
      CastingFeatures.box("tray-rear-ledge",a,l,depth,w/2-a/2,0,d[2]-depth),
      CastingFeatures.box("tray-left-ledge",w-2*a,a,depth,0,-l/2+a/2,d[2]-depth),
      CastingFeatures.box("tray-right-ledge",w-2*a,a,depth,0,l/2-a/2,d[2]-depth)];
  }
  public static function trayClampSeats():Array<InsertLocation> {
    var out:Array<InsertLocation> = [],d=dimensions("bed");
    function add(x:Float,y:Float,height:Float=11):Void out.push({id:"trayClampSeat"+out.length,role:"M6 tray clamp nominal threaded insert envelope",
      width:20,depth:20,height:20,openings:[materia.casting.CastingPatterns.face("body.z-max")],pose:F.translation(x,y,d[2]-20),clampHeight:height});
    for(side in [-1,1])for(x in [-1,0,1])add(x*(d[0]/2-100),side*(d[1]/2-35));
    for(side in [-1,1])for(y in [-150.0,150.0])add(side*(d[0]/2-35),y);
    // Two taller toes positively retain the raised rear service cover.
    for(y in [-75.0,75.0])add(d[0]/2-35,y,17);
    return out;
  }
  /** Three support centres in casting-local coordinates, kept fixed through X travel. */
  public static function feet():Array<{x:Float,y:Float}> return [for(i in 0...3)
    {x:(i<2?-1:1)*(HobberXLayout.front().bedLength/2-120),y:i==0?-330.0:i==1?330.0:0.0}];
  public static function dimensions(kind:String,legacy:Bool=false):Array<Float> return switch kind {
    case "bed": legacy?[650.0,550,120]:[HobberXLayout.front().bedLength,800,PLINTH_HEIGHT]; case "column": legacy?[200.0,180,600]:[400.0,200,500];
    default: throw "Unknown hobber foundation kind";
  };
  public static function bodyValues(kind:String,legacy:Bool=false):ComponentValues {
    var d = dimensions(kind,legacy);
    var cores=legacy?CastingFeatures.grid(d[0],d[1],d[2],kind=="bed"?30:25,20,20,25):kind=="bed"?
      CastingFeatures.grid(d[0],d[1],d[2],25,85,25,35,2,1):CastingFeatures.grid(d[0],d[1],d[2],20,20,20,20);
    if(!legacy){
      if(kind=="bed"){
        var centre=HobberXLayout.front().bedCentre;
        cores=machinekit.structural.CastingCells.outsideColumns(bedCores(cores),[[-140.0-centre,-140,140-centre,140]]);
        cores=machinekit.structural.CastingCells.outsideBoxes(cores,driveSupports());
        // Solid bosses behind the side pads: do not anchor in a hollow 25 mm skin.
        cores=machinekit.structural.CastingCells.outsideBoxes(cores,[for(seat in enclosureSeats())
          [seat.pose.x-60,seat.pose.y<0?-400.0:330.0,660.0,seat.pose.x+60,seat.pose.y<0?-330.0:400.0,780.0]]);
        cores=machinekit.structural.CastingCells.outsideBoxes(cores,[for(seat in trayClampSeats())
          [seat.pose.x-20,seat.pose.y-20,740.0,seat.pose.x+20,seat.pose.y+20,800.0]]);
        for(p in feet())cores=machinekit.structural.CastingFootPocket.reserve(cores,p.x,p.y);
        var feet=feet();for(i in 0...feet.length){var p=feet[i];
          var edge=i<2?(p.y<0?-401.0:401.0):d[0]/2+1;
          for(feature in machinekit.structural.CastingFootPocket.features("foot"+i,p.x,p.y,edge,i==2))cores.push(feature);
        }
        cores.push(CastingFeatures.box("work-well",410,350,220-TABLE_WELL_FLOOR,-45-centre,0,TABLE_WELL_FLOOR+UPPER_OFFSET));
        cores.push(CastingFeatures.box("chip-drain",280,180,80,-50-centre,0,TABLE_WELL_FLOOR-80+UPPER_OFFSET));
        cores.push(CastingFeatures.box("X-drive-channel",HobberXLayout.front().bedRear+1-(HobberXLayout.front().deckStart-12.5),110,90,(HobberXLayout.front().bedRear+1+HobberXLayout.front().deckStart-12.5)/2-centre,0,130+UPPER_OFFSET));
        cores.push(CastingFeatures.box("coolant-reserved-volume",120,220,300,-215-centre,0,150));
        cores.push(CastingFeatures.box("coolant-front-access",40,240,320,-285-centre,0,140));
        cores.push(CastingFeatures.box("chip-downcomer",80,100,TABLE_WELL_FLOOR+390,-200-centre,0,200));
        cores.push(CastingFeatures.box("rotary-union-access",40,40,TABLE_SEAT+UPPER_OFFSET+1,-centre,0,0));
        for(ledge in trayLedges())cores.push(ledge);
        cores.push(CastingFeatures.box("tray-rear-bridge-seat",25,150,10,d[0]/2-12.5,0,d[2]-10));
        cores.push(CastingFeatures.box("cable-route",HobberXLayout.front().bedLength-200,30,40,0,300,500));
      }else {
        cores.push(CastingFeatures.box("head-recess",240,100,502,0,60,-1));
        // Rear spine carries the central steel drive seat to the column sole.
        cores=machinekit.structural.CastingCells.outsideBoxes(cores,[[-60.0,-100,0,60,HobberZLayout.SPINE_FRONT_COLUMN_Y,500]]);
      }
    }
    return new ComponentValues().setNumber("width",d[0]).setNumber("depth",d[1]).setNumber("height",d[2])
      .setToken("features",CastingFeatures.encodeForBody(cores,d[0],d[1],d[2]));
  }
  public static function placements(kind:String):Array<CastInsertPlacement> return
    [for(row in inserts(kind)) {id:row.id,part:insert(row),pose:row.pose,openings:row.openings}];
  public static function inserts(kind:String,legacy:Bool=false):Array<InsertLocation>{
    dimensions(kind,legacy);
    var out:Array<InsertLocation> = [];
    function add(id:String,role:String,w:Float,d:Float,h:Float,x:Float,y:Float,z:Float,openings:Array<String>):Void
      out.push({id:id,role:role,width:w,depth:d,height:h,pose:F.translation(x,y,z),openings:[for(face in openings) materia.casting.CastingPatterns.face(face)]});
    if(!legacy){
      if(kind=="bed"){
        var centre=HobberXLayout.front().bedCentre;
        for(seat in enclosureSeats())out.push(seat);
        for(seat in trayClampSeats())out.push(seat);
        add("tablePlate","table seat",280,280,20,-centre,0,TABLE_SEAT-20+UPPER_OFFSET,[]);
        for(side in [-1,1])out.push({id:side<0?"railLeft":"railRight",role:"preloaded column guide seat",width:BED_RAIL_SEAT_WIDTH,depth:HobberXLayout.front().railLength,height:20,
          pose:F.fromRotationMatrix(HobberXLayout.front().railStart+HobberXLayout.front().railLength/2-centre,side*BED_RAIL_CENTRE,200+UPPER_OFFSET,[0.0,1,0,-1,0,0,0,0,1]),openings:[materia.casting.CastingPatterns.face("body.z-max")]});
        out.push({id:"driveSeat",role:"central X screw channel seat",width:100,depth:HobberXLayout.front().motorFace-HobberXLayout.front().deckStart,height:20,pose:F.fromRotationMatrix((HobberXLayout.front().motorFace+HobberXLayout.front().deckStart)/2-centre,0,120+UPPER_OFFSET,[0.0,1,0,-1,0,0,0,0,1]),openings:[]});
        for(side in [-1,1])add(side<0?"liftLeft":"liftRight","lifting point",30,30,30,-150-centre,side*310,180+UPPER_OFFSET,["body.z-max"]);
      }else{
        add("basePlate","column carriage sole",380,100,20,0,-40,-10,["body.z-min"]);
        for(side in [-1,1])out.push({id:side<0?"railLeft":"railRight",role:"Z rail seat",width:35,depth:480,height:20,
          pose:F.fromRotationMatrix(side*145,90,245,[1.0,0,0,0,0,1,0,-1,0]),openings:[materia.casting.CastingPatterns.face("body.y-max")]});
        out.push({id:"driveSeat",role:"central Z drive back seat",width:100,depth:460,height:20,
          pose:F.fromRotationMatrix(0,HobberZLayout.SEAT_COLUMN_Y,250,[-1.0,0,0,0,0,1,0,1,0]),openings:[]});
        for(side in [-1,1])add(side<0?"liftLeft":"liftRight","lifting point",25,25,20,side*160,-40,490,["body.z-max"]);
      }
      return out;
    }
    if(kind=="bed"){
      add("tablePlate","table plate",180,180,20,0,-160,110,["body.z-max"]);
      add("columnPlate","column plate",220,180,20,0,legacy?180:280,110,["body.z-max"]);
      for(side in [-1,1])add(side<0?"railLeft":"railRight","rail seat",35,400,20,side*200,0,110,["body.z-max"]);
      for(x in [-1,1])for(y in [-1,1])add('bush${x<0?"L":"R"}${y<0?"F":"B"}',"threaded bush",20,20,20,x*275,y*210,110,["body.z-max"]);
      for(side in [-1,1])add(side<0?"liftLeft":"liftRight","lifting point",30,30,30,side*275,0,100,["body.z-max"]);
    }else{
      add("basePlate","column plate",150,130,20,0,0,-10,["body.z-min"]);
      if(!legacy)out.push({id:"driveSeat",role:"Z drive support seat",width:80,depth:800,height:20,
        pose:F.fromRotationMatrix(-90,0,425,[0.0,0,-1,1,0,0,0,-1,0]),openings:[materia.casting.CastingPatterns.face("body.x-min")]});
      for(side in [-1,1])out.push({id:side<0?"railLeft":"railRight",role:"rail seat",width:35,depth:legacy?550:800,height:20,
        pose:F.fromRotationMatrix(side*55,-80,legacy?300:425,[1.0,0,0,0,0,-1,0,1,0]),openings:[materia.casting.CastingPatterns.face("body.y-min")]});
      for(x in [-1,1])for(y in [-1,1])add('bush${x<0?"L":"R"}${y<0?"F":"B"}',"threaded bush",20,20,20,x*85,y*75,-10,["body.z-min"]);
      for(side in [-1,1])add(side<0?"liftLeft":"liftRight","lifting point",25,25,20,side*70,0,legacy?590:840,["body.z-max"]);
    }
    return out;
  }
  public static function insertRole(label:String):materia.casting.InsertSpec.InsertRole return switch label {
    case "table seat", "table plate": TableMount;
    case "column carriage sole", "column plate": ColumnMount;
    case "preloaded column guide seat", "Z rail seat", "rail seat": GuideSeat;
    case "lifting point": LiftingMount;
    case "threaded bush", "M6 tray clamp nominal threaded insert envelope": ThreadedAnchor;
    default: GenericMount;
  };
  public static function insert(row:InsertLocation):CastInsert return new CastInsert(new ComponentValues().setNumber("width",row.width)
    .setNumber("depth",row.depth).setNumber("height",row.height).setToken("holes",machinekit.structural.InsertHoles.through(row.id=="tablePlate"?40:StringTools.startsWith(row.id,"trayClampSeat")?6:0,row.height)).setToken("role",insertRole(row.role)).setToken("label",row.role));
}
