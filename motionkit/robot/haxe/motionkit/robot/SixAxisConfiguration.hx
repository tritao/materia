package motionkit.robot;

/** Controller-style geometric configuration and physical joint turn counts.
 * Conventions are explicit per backend, not inferred from a lifted joint sign.
 * OPW: slots 0/1 are front, 2/3 back; even slots are elbow up; slots 4..7 flip.
 * UR6R: native bits are shoulder, wrist, elbow (4,2,1 respectively).
 * These are generic conventions, not vendor-specific configuration codes.
 */
class SixAxisConfiguration {
  public final shoulder:String;
  public final elbow:String;
  public final wrist:String;
  /** Null on a geometric-only pin; six values on every labelled IK result. */
  public final turns:Null<Array<Int>>;

  public function new(shoulder:String,elbow:String,wrist:String,?turns:Array<Int>) {
    if(shoulder!="front" && shoulder!="back" || elbow!="up" && elbow!="down" ||
        wrist!="no-flip" && wrist!="flip")throw "Invalid six-axis configuration label";
    if(turns!=null && turns.length!=6)throw "Configuration turns require six arm joints";
    this.shoulder=shoulder;this.elbow=elbow;this.wrist=wrist;
    this.turns=turns==null ? null : turns.copy();
  }

  public static function of(family:String,branch:Int,q:Array<Float>):SixAxisConfiguration {
    if(branch<0 || branch>=8 || q==null || q.length!=6)
      throw "Six-axis configuration requires a branch and six arm joints";
    var shoulderBit:Int,wristBit:Int;
    switch family {
      case "OPW": shoulderBit=2;wristBit=4;
      case "UR6R": shoulderBit=4;wristBit=2;
      default: throw 'No six-axis convention mapping for "$family"';
    }
    var turns:Array<Int> = [];
    for(value in q){
      if(!Math.isFinite(value))throw "Configuration joint coordinates must be finite";
      // Principal physical coordinates use [-pi,pi). pi belongs to turn 1.
      var turn=Math.floor((value+Math.PI)/(2*Math.PI));
      if(turn < -2147483647 || turn > 2147483647)throw "Configuration turn count exceeds integer range";
      turns.push(Std.int(turn));
    }
    return new SixAxisConfiguration((branch & shoulderBit)==0 ? "front" : "back",
      (branch & 1)==0 ? "up" : "down",(branch & wristBit)==0 ? "no-flip" : "flip",turns);
  }

  public function accepts(actual:Null<SixAxisConfiguration>):Bool {
    if(actual==null)return false;
    var value:SixAxisConfiguration=cast actual;
    if(shoulder!=value.shoulder || elbow!=value.elbow || wrist!=value.wrist)return false;
    if(turns==null)return true;
    if(value.turns==null)return false;
    var wanted:Array<Int> = cast turns,found:Array<Int> = cast value.turns;
    for(i in 0...6)if(wanted[i]!=found[i])return false;
    return true;
  }

  public function label():String {
    var text=shoulder+"/"+elbow+"/"+wrist;
    return turns==null ? text : text+" turns="+haxe.Json.stringify(turns);
  }
}
