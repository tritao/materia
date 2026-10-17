package stockkit.hobbing;

/** Empirical coefficients, NOT a hardness/material database. kc11 is N/mm² at
    h=1 mm, mc dimensionless. No rake, speed, wear or ploughing correction. */
class KienzleMaterial {
  public final name:String;
  public final kc11:Float;
  public final mc:Float;
  public final basis:String;
  public function new(name:String,kc11:Float,mc:Float,basis:String) {
    if(name==null||name.length==0||basis==null||basis.length==0||!Math.isFinite(kc11)||kc11<=0
      ||!Math.isFinite(mc)||mc<0||mc>=1)throw "invalid Kienzle material assumptions";
    this.name=name;this.kc11=kc11;this.mc=mc;this.basis=basis;
  }
  public static function referenceSteel():KienzleMaterial return new KienzleMaterial(
    "40CrNiMo, 38–42 HRC",2400,0.25,
    "ASSUMED kc1.1=2400 N/mm², mc=0.25; requires cutting-test calibration at stated hardness");
  /** F=kc11*b*h^(1-mc), h=A/b. Continuous zero limit avoids 0^(-mc). */
  public function force(area:Float,edgeWidth:Float):Float {
    if(!Math.isFinite(area)||area<0||!Math.isFinite(edgeWidth)||edgeWidth<=0)throw "invalid chip section";
    return area==0?0:kc11*edgeWidth*Math.pow(area/edgeWidth,1-mc);
  }
}
