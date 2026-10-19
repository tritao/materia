package machinekit.structural;
import machinekit.structural.CastingFeatures.CastingBox;
/** Side-access levelling pocket with a removable plate bearing UP against its roof.
 * Nominal design geometry only; strength and casting-process details need verification.
 */
class CastingFootPocket {
  public static inline final FLOOR=40.0;
  public static inline final ROOF=160.0;
  public static inline final SUPPORT_TOP=230.0;
  public static inline final PLATE_WIDTH=110.0;
  public static inline final PLATE_THICKNESS=15.0;
  public static inline final STEM_CLEARANCE=28.0;
  public static function features(id:String,x:Float,y:Float,edge:Float,alongX:Bool):Array<CastingBox> {
    var centre=alongX?x:y,lo=Math.min(centre-70,edge),hi=Math.max(centre+70,edge);
    return [CastingFeatures.box(id+"-access",alongX?hi-lo:140,alongX?140:hi-lo,ROOF-PLATE_THICKNESS-FLOOR,
      alongX?(lo+hi)/2:x,alongX?y:(lo+hi)/2,FLOOR),
      CastingFeatures.box(id+"-plate-register",PLATE_WIDTH+1,PLATE_WIDTH+1,PLATE_THICKNESS,x,y,ROOF-PLATE_THICKNESS),
      CastingFeatures.box(id+"-stem",STEM_CLEARANCE,STEM_CLEARANCE,SUPPORT_TOP+2,x,y,-1)];
  }
  /** Preserve a local support boss below general lightening cores; pockets are cut later. */
  public static function reserve(cells:Array<CastingBox>,x:Float,y:Float):Array<CastingBox> {
    var out:Array<CastingBox> =[];
    for(c in cells){
      if(!CastingCells.orthogonal(c))throw "Foot support reservation requires orthogonal cores";
      var boxes=CastingCells.remove([CastingCells.bounds(c)],
        [x-90,y-90,0,x+90,y+90,SUPPORT_TOP]);
      for(i in 0...boxes.length){var b=boxes[i];out.push(CastingFeatures.box(c.id+"-foot-"+i,b[3]-b[0],b[4]-b[1],b[5]-b[2],(b[0]+b[3])/2,(b[1]+b[4])/2,b[2]));}
    }
    return out;
  }
}
