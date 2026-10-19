typedef TendingStockFrame = {
  var x:Float; var y:Float; var z:Float;
  var qx:Float; var qy:Float; var qz:Float; var qw:Float;
}

/** Shared bottom-datum convention; pure math keeps the runtime checks headless. */
class TendingStockDatum {
  public static function local(reference:Dynamic,profile:String):TendingStockFrame {
    var g:Array<Float> = cast reference.grasp,o:Array<Float> = cast reference.offset;
    if(g==null || o==null || g.length!=3 || o.length!=3)throw "Stock datum requires its reviewed reference frame";
    for(v in g.concat(o))if(!Math.isFinite(v))throw "Stock datum must be finite";
    return switch profile {
      case "reference-side-entry":{x:g[0],y:g[1],z:g[2]-10,qx:0.0,qy:0.0,qz:0.0,qw:1.0};
      case "tending-top-entry":{x:g[0],y:50+o[1],z:g[2],qx:Math.sqrt(0.5),qy:0.0,qz:0.0,qw:Math.sqrt(0.5)};
      default:throw "Unknown stock datum profile";
    }
  }
  public static function checks():Int {
    var r={grasp:[-4.8,108.2783,37.5],offset:[-199.303,48.2783,-269.589]};
    var top=local(r,"tending-top-entry"),x=0.0,y=30.0,z=20.0;
    var tx=2*(top.qy*z-top.qz*y),ty=2*(top.qz*x-top.qx*z),tz=2*(top.qx*y-top.qy*x);
    var cx=top.x+x+top.qw*tx+top.qy*tz-top.qz*ty;
    var cy=top.y+y+top.qw*ty+top.qz*tx-top.qx*tz;
    var cz=top.z+z+top.qw*tz+top.qx*ty-top.qy*tx;
    if(Math.abs(cx+4.8)>1e-9 || Math.abs(cy-78.2783)>1e-9 || Math.abs(cz-67.5)>1e-9)
      throw "Top-entry bottom datum changed its physical stock axes";
    var side=local(r,"reference-side-entry");if(side.z!=27.5 || side.y!=108.2783)throw "Side-entry datum changed";
    var rejected=false;try local(r,"unreviewed") catch(_:Dynamic)rejected=true;
    if(!rejected)throw "Stock datum accepted an unknown grasp convention";
    return 3;
  }
}
