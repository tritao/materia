package stockkit.hobbing;

/** One gash's window-AVERAGE loads, not an instantaneous tooth peak.
    mm³/mm²/mm inputs; N, Nm, W, J outputs. Table torque is a magnitude bound
    (resultant force at work radius), not a signed force-vector prediction. */
class HobbingForceSample {
  public final pass:GashPass;
  public final removedVolume:Float;
  public final chipArea:Float;
  public final chipThickness:Float;
  public final tangentialForce:Float;
  public final radialForce:Float;
  public final hobTorque:Float;
  public final tableTorqueBound:Float;
  public final power:Float;
  public final energy:Float;
  public function new(pass:GashPass,volume:Float,travel:Float,radius:Float,workRadius:Float,
      edgeWidth:Float,pressureAngle:Float,material:KienzleMaterial) {
    for(v in [volume,travel,radius,workRadius,edgeWidth,pressureAngle])if(!Math.isFinite(v))throw "non-finite force input";
    if(volume<0||travel<0||radius<=0||workRadius<=0||edgeWidth<=0||pressureAngle<0||pressureAngle>=Math.PI/2
      ||(volume>0&&travel<=0))throw "invalid force geometry";
    this.pass=pass;removedVolume=volume;chipArea=travel==0?0:volume/travel;chipThickness=chipArea/edgeWidth;
    tangentialForce=material.force(chipArea,edgeWidth);
    // Assumed separating-force ratio for the rack pressure angle; measured calibration is HB11 input.
    radialForce=tangentialForce*Math.tan(pressureAngle);
    hobTorque=tangentialForce*radius/1000;
    tableTorqueBound=Math.sqrt(tangentialForce*tangentialForce+radialForce*radialForce)*workRadius/1000;
    energy=tangentialForce*travel/1000;power=energy/(pass.end-pass.start);
  }
  /** Scalar rating check, with declared reserve. Does not include acceleration,
      friction or unresolved tooth peaks; these still belong to PlanCheck. */
  public function fits(powerRating:Float,hobTorqueRating:Float,tableTorqueRating:Float,reserve:Float=0.2):Bool {
    for(v in [powerRating,hobTorqueRating,tableTorqueRating])if(!Math.isFinite(v)||v<=0)throw "invalid force ratings";
    if(!Math.isFinite(reserve)||reserve<0||reserve>=1)throw "invalid force reserve";
    return power<=powerRating*(1-reserve)&&hobTorque<=hobTorqueRating*(1-reserve)&&tableTorqueBound<=tableTorqueRating*(1-reserve);
  }
}
