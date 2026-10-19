import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import robotkit.collision.CollisionSceneBuilder.CollisionOwner;

/** The physical state during an approach; grasp/release are separate transitions. */
class TendingElectricApproach {
  public final isTransport:Bool;
  public final isWithdrawal:Bool;
  public final station:String;
  public final side:String;
  public final targetWrist:AssemblyFrame;
  public final stockOwner:CollisionOwner;
  public final jawOpeningMm:Float;
  public function new(station:String,side:String,targetWrist:AssemblyFrame,flangeLink:String,isTransport:Bool=false,isWithdrawal:Bool=false) {
    if(side!="a" && side!="b")throw "Electric approach needs a selected gripper";
    this.isTransport=isTransport;this.isWithdrawal=isWithdrawal;
    this.station=station;this.side=side;this.targetWrist=targetWrist;
    var pick=!isTransport && (station=="vise-seat" || StringTools.startsWith(station,"infeedTray-"));
    var fixedStock=isWithdrawal ? !pick : pick;
    stockOwner=fixedStock ? Fixture : Held("electric-arm",flangeLink);
    jawOpeningMm=fixedStock ? 25 : 17;
  }
  public static function transport(flangeLink:String):TendingElectricApproach
    return new TendingElectricApproach("transport","a",AssemblyFrames.identity(),flangeLink,true);
  public static function withdrawal(station:String,side:String,wrist:AssemblyFrame,flangeLink:String):TendingElectricApproach
    return new TendingElectricApproach(station,side,wrist,flangeLink,false,true);
  public function stockFixed():Bool return switch(stockOwner) {case Fixture:true;default:false;}
  public function stockPose(tool:AssemblyState):AssemblyFrame
    return AssemblyFrames.compose(targetWrist,tool.worldPose(side+"-blank-60"));
}
