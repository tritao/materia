import camkit.HobbingJob;
import camkit.HobbingLimits;
import camkit.HobbingSettings;
import toolpathkit.path.Point3;

/** HB8 adapter: CAM dimensions/limits come from the assembled HB6 reference machine. */
class FlexsplineHobberCam {
  public static function job(machine:FlexsplineHobber,?settings:HobbingSettings):HobbingJob {
    var x=machine.travel.filter(t->t.id=="X")[0],y=machine.travel.filter(t->t.id=="Y")[0],z=machine.travel.filter(t->t.id=="Z")[0];
    // A's actual installed joint range, rather than the swivel recipe's generic maximum.
    var model=new cadkit.modeling.AssemblyModel("mm");machine.addTo(model,"");
    var tilt=model.definition("hobber").joints.filter(j->j.id==FlexsplineHobber.A)[0].limits;
    if(tilt.lower==null||tilt.upper==null)throw "Hobber needs installed swivel limits";
    var low=tilt.lower,high=tilt.upper;
    if(low==null||high==null)throw "Hobber needs installed swivel limits";
    var limits=new HobbingLimits(new Point3(x.lower,y.lower,z.lower),new Point3(x.upper,y.upper,z.upper),low,high,
      machine.spindle.motor.maxSpeed*60/(2*Math.PI),machine.table.motor.maxSpeed*60/(2*Math.PI));
    return new HobbingJob(machine.spec,machine.hob.design,machine.hob.n("height"),settings,limits);
  }
}
