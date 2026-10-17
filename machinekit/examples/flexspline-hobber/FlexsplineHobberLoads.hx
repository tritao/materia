import stockkit.hobbing.HobbingForces;
import motionkit.robot.PlanCheck.PlanCheckOptions;
import camkit.HobbingJob;

/** HB10 load adapter. Uses installed continuous ratings, with a declared reserve.
    Forces are overlapping window means, so passing is not a tooth-peak certification. */
class FlexsplineHobberLoads {
  public static function fits(machine:FlexsplineHobber,job:HobbingJob,loads:HobbingForces,reserve:Float=0.2):Bool {
    if(!Math.isFinite(reserve)||reserve<0||reserve>=1)throw "invalid hobber load reserve";
    var b=machine.spindle.motor,c=machine.table.motor;
    var bSpeed=job.rpm*2*Math.PI/60,cSpeed=job.tableRpm*2*Math.PI/60;
    return bSpeed<=b.maxSpeed&&cSpeed<=c.maxSpeed
      &&loads.maxHobTorque<=b.continuousTorque*(1-reserve)
      &&loads.maxPower<=b.continuousTorque*b.ratedSpeed*(1-reserve)
      &&loads.maxTableTorqueBound<=c.continuousTorque*(1-reserve);
  }
  /** Conservative magnitudes for planned X/C. The process-owned B spindle is
      checked by fits(), because it is absent from the position polynomial. */
  public static function planOptions(loads:HobbingForces,?base:PlanCheckOptions):PlanCheckOptions {
    var options=base==null?new PlanCheckOptions():base.copy();
    options.resistingEfforts.set("X",loads.maxRadial);
    options.resistingEfforts.set(FlexsplineHobber.C,loads.maxTableTorqueBound);
    return options;
  }
}
