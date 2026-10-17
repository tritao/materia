package machinekit.hobbing;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;
import machinekit.component.ComponentValues;

typedef InsertLocation={id:String,role:String,width:Float,depth:Float,height:Float,pose:AssemblyFrame};

/** Design-specific insert layout, not a casting process model. Coordinates relative to casting body. */
class FoundationLayout {
  public static function dimensions(kind:String):Array<Float>return kind=="bed"?[650.0,550,120]:[200.0,180,600];
  public static function inserts(kind:String):Array<InsertLocation>{
    var out:Array<InsertLocation> = [];
    function add(id:String,role:String,w:Float,d:Float,h:Float,x:Float,y:Float,z:Float):Void
      out.push({id:id,role:role,width:w,depth:d,height:h,pose:F.translation(x,y,z)});
    if(kind=="bed"){
      add("tablePlate","table plate",180,180,20,0,-100,110);
      add("columnPlate","column plate",220,180,20,0,180,110);
      for(side in [-1,1])add(side<0?"railLeft":"railRight","rail seat",35,400,20,side*200,0,110);
      for(x in [-1,1])for(y in [-1,1])add('bush${x<0?"L":"R"}${y<0?"F":"B"}',"threaded bush",20,20,20,x*275,y*210,110);
      for(side in [-1,1])add(side<0?"liftLeft":"liftRight","lifting point",30,30,30,side*275,0,100);
    }else{
      add("basePlate","column plate",150,130,20,0,0,-10);
      for(side in [-1,1])out.push({id:side<0?"railLeft":"railRight",role:"rail seat",width:35,depth:550,height:20,
        pose:F.fromRotationMatrix(side*55,-80,300,[1.0,0,0,0,0,-1,0,1,0])});
      for(x in [-1,1])for(y in [-1,1])add('bush${x<0?"L":"R"}${y<0?"F":"B"}',"threaded bush",20,20,20,x*85,y*75,-10);
      for(side in [-1,1])add(side<0?"liftLeft":"liftRight","lifting point",25,25,20,side*70,0,590);
    }
    return out;
  }
  public static function insert(row:InsertLocation):CastInsert return new CastInsert(new ComponentValues().setNumber("width",row.width)
    .setNumber("depth",row.depth).setNumber("height",row.height).setToken("role",row.role));
}
