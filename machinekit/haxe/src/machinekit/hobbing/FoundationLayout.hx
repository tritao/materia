package machinekit.hobbing;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;
import machinekit.component.ComponentValues;

typedef InsertLocation={id:String,role:String,width:Float,depth:Float,height:Float,pose:AssemblyFrame};

/** Design-specific insert layout, not a casting process model. Coordinates relative to casting body. */
class FoundationLayout {
  public static function dimensions(kind:String,legacy:Bool=false):Array<Float> return switch kind {
    case "bed": [650.0,legacy?550:750,120]; case "column": [200.0,180,legacy?600:850];
    default: throw "Unknown hobber foundation kind";
  };
  public static function bodyValues(kind:String,legacy:Bool=false):ComponentValues {
    var d = dimensions(kind,legacy);
    return new ComponentValues().setNumber("width",d[0]).setNumber("depth",d[1]).setNumber("height",d[2])
      .setToken("voids",CastingFeatures.encode(CastingFeatures.grid(d[0],d[1],d[2],kind=="bed"?30:25,20,20,25)));
  }
  public static function placements(kind:String):Array<CastInsertPlacement> return
    [for(row in inserts(kind)) {id:row.id,part:insert(row),pose:row.pose}];
  public static function inserts(kind:String,legacy:Bool=false):Array<InsertLocation>{
    dimensions(kind,legacy);
    var out:Array<InsertLocation> = [];
    function add(id:String,role:String,w:Float,d:Float,h:Float,x:Float,y:Float,z:Float):Void
      out.push({id:id,role:role,width:w,depth:d,height:h,pose:F.translation(x,y,z)});
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
    .setNumber("depth",row.depth).setNumber("height",row.height).setToken("role",row.role));
}
