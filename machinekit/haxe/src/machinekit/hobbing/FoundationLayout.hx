package machinekit.hobbing;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;
import machinekit.component.ComponentValues;
import machinekit.structural.CastInsert;
import machinekit.structural.CastingFeatures;
import machinekit.structural.MineralFoundation.CastInsertPlacement;

typedef InsertLocation={id:String,role:String,width:Float,depth:Float,height:Float,pose:AssemblyFrame};

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
        cores.push(CastingFeatures.box("work-well",410,350,220-TABLE_WELL_FLOOR,-45-centre,0,TABLE_WELL_FLOOR+UPPER_OFFSET));
        cores.push(CastingFeatures.box("chip-drain",280,180,80,-50-centre,0,TABLE_WELL_FLOOR-80+UPPER_OFFSET));
        cores.push(CastingFeatures.box("X-drive-channel",HobberXLayout.front().bedRear+1-(HobberXLayout.front().deckStart-12.5),110,90,(HobberXLayout.front().bedRear+1+HobberXLayout.front().deckStart-12.5)/2-centre,0,130+UPPER_OFFSET));
        cores.push(CastingFeatures.box("coolant-reserved-volume",120,220,300,-215-centre,0,150));
        cores.push(CastingFeatures.box("coolant-front-access",40,240,320,-285-centre,0,140));
        cores.push(CastingFeatures.box("chip-downcomer",80,100,TABLE_WELL_FLOOR+390,-200-centre,0,200));
        cores.push(CastingFeatures.box("rotary-union-access",40,40,TABLE_SEAT+UPPER_OFFSET+1,-centre,0,0));
        cores.push(CastingFeatures.box("cable-route",HobberXLayout.front().bedLength-200,30,40,0,300,500));
      }else cores.push(CastingFeatures.box("head-recess",240,100,502,0,60,-1));
    }
    return new ComponentValues().setNumber("width",d[0]).setNumber("depth",d[1]).setNumber("height",d[2])
      .setToken("voids",CastingFeatures.encode(cores));
  }
  public static function placements(kind:String):Array<CastInsertPlacement> return
    [for(row in inserts(kind)) {id:row.id,part:insert(row),pose:row.pose}];
  public static function inserts(kind:String,legacy:Bool=false):Array<InsertLocation>{
    dimensions(kind,legacy);
    var out:Array<InsertLocation> = [];
    function add(id:String,role:String,w:Float,d:Float,h:Float,x:Float,y:Float,z:Float):Void
      out.push({id:id,role:role,width:w,depth:d,height:h,pose:F.translation(x,y,z)});
    if(!legacy){
      if(kind=="bed"){
        var centre=HobberXLayout.front().bedCentre;
        add("tablePlate","table seat",280,280,20,-centre,0,TABLE_SEAT-20+UPPER_OFFSET);
        for(side in [-1,1])out.push({id:side<0?"railLeft":"railRight",role:"preloaded column guide seat",width:BED_RAIL_SEAT_WIDTH,depth:HobberXLayout.front().railLength,height:20,
          pose:F.fromRotationMatrix(HobberXLayout.front().railStart+HobberXLayout.front().railLength/2-centre,side*BED_RAIL_CENTRE,200+UPPER_OFFSET,[0.0,1,0,-1,0,0,0,0,1])});
        out.push({id:"driveSeat",role:"central X screw channel seat",width:100,depth:HobberXLayout.front().motorFace-HobberXLayout.front().deckStart,height:20,pose:F.fromRotationMatrix((HobberXLayout.front().motorFace+HobberXLayout.front().deckStart)/2-centre,0,120+UPPER_OFFSET,[0.0,1,0,-1,0,0,0,0,1])});
        for(side in [-1,1])add(side<0?"liftLeft":"liftRight","lifting point",30,30,30,-150-centre,side*310,180+UPPER_OFFSET);
        for(i in 0...3)add("foot"+i,"levelling foot anchor",80,80,30,(i<2?-1:1)*(HobberXLayout.front().bedLength/2-120),i==0?-330:i==1?330:0,0);
      }else{
        add("basePlate","column carriage sole",380,100,20,0,-40,-10);
        for(side in [-1,1])out.push({id:side<0?"railLeft":"railRight",role:"Z rail seat",width:35,depth:480,height:20,
          pose:F.fromRotationMatrix(side*145,90,245,[1.0,0,0,0,0,1,0,-1,0])});
        out.push({id:"driveSeat",role:"Z drive support seat",width:90,depth:480,height:20,
          pose:F.fromRotationMatrix(-190,40,245,[0.0,0,-1,-1,0,0,0,1,0])});
        for(side in [-1,1])add(side<0?"liftLeft":"liftRight","lifting point",25,25,20,side*160,-40,490);
      }
      return out;
    }
    if(kind=="bed"){
      add("tablePlate","table plate",180,180,20,0,-160,110);
      add("columnPlate","column plate",220,180,20,0,legacy?180:280,110);
      for(side in [-1,1])add(side<0?"railLeft":"railRight","rail seat",35,400,20,side*200,0,110);
      for(x in [-1,1])for(y in [-1,1])add('bush${x<0?"L":"R"}${y<0?"F":"B"}',"threaded bush",20,20,20,x*275,y*210,110);
      for(side in [-1,1])add(side<0?"liftLeft":"liftRight","lifting point",30,30,30,side*275,0,100);
    }else{
      add("basePlate","column plate",150,130,20,0,0,-10);
      if(!legacy)out.push({id:"driveSeat",role:"Z drive support seat",width:80,depth:800,height:20,
        pose:F.fromRotationMatrix(-90,0,425,[0.0,0,-1,1,0,0,0,-1,0])});
      for(side in [-1,1])out.push({id:side<0?"railLeft":"railRight",role:"rail seat",width:35,depth:legacy?550:800,height:20,
        pose:F.fromRotationMatrix(side*55,-80,legacy?300:425,[1.0,0,0,0,0,-1,0,1,0])});
      for(x in [-1,1])for(y in [-1,1])add('bush${x<0?"L":"R"}${y<0?"F":"B"}',"threaded bush",20,20,20,x*85,y*75,-10);
      for(side in [-1,1])add(side<0?"liftLeft":"liftRight","lifting point",25,25,20,side*70,0,legacy?590:840);
    }
    return out;
  }
  public static function insert(row:InsertLocation):CastInsert return new CastInsert(new ComponentValues().setNumber("width",row.width)
    .setNumber("depth",row.depth).setNumber("height",row.height).setNumber("boreDiameter",row.id=="tablePlate"?40:0).setToken("role",row.role));
}
