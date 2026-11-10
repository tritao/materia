import machinekit.hobbing.HobberLayout;
import machinekit.design.DesignRequirement;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.PosedParts;
import machinekit.hobbing.HobberDrives;
import machinekit.motion.BallNut;
import materia.assembly.AssemblyFrames as F;

/** Physical service datums and long-tool access, independent of the removal
 * sweep. The nut may reverse, but the existing carriage interface must stay. */
class HobberServiceDesignChecks {
  public static function run(machine:FlexsplineHobber):Array<String> {
    var drive=HobberDrives.x(), nut=new BallNut(), block=HobberDrives.nutBlock(drive,nut), at=HobberDrives.poses(drive).get("nut");
    var direction=F.transformVector(at,0,0,1), flange=F.transformPoint(at,0,0,nut.bodyLength);
    var b=machinekit.hobbing.HobberChipCover.nutBounds(), expected=HobberLayout.get().nutBlockFront;
    if(direction.x<0.9999 || Math.abs(flange.x-b.maxX)>1e-7)throw "X nut flange is not on the motor-side block face";
    if(Math.abs(b.minX-expected)>1e-7 || Math.abs(b.maxX-expected-block.thickness)>1e-7)throw "X nut reversal displaced the carriage interface";
    var poses=machine.solvedPoses(), back=F.inverse(machinekit.hobbing.HobberFrame.workToMachine()), members=machine.components();
    var screw:machinekit.motion.HzMotionDrawingScrew=cast members.filter(m->m.id=="Xscrew")[0].component;
    var initial=machine.travel.filter(t->t.id=="X")[0].initial, reserve=1e9, barrel=1e9;
    for(x in [HobberLayout.get().xMin-5,HobberLayout.get().xRetract+5]) {
      var delta=F.transformVector(machinekit.hobbing.HobberFrame.workToMachine(),x-initial,0,0);
      var p=F.compose(F.inverse(poses.get("Xscrew")),F.compose(F.translation(delta.x,delta.y,delta.z),poses.get("Xnut")));
      var a=F.transformPoint(p,0,0,0).z,c=F.transformPoint(p,0,0,nut.bodyLength).z,d=F.transformPoint(p,0,0,nut.bodyLength+nut.flangeThickness).z;
      barrel=Math.min(barrel,Math.min(Math.min(a,c)-screw.fixedEnd,screw.totalLength-screw.floatingEnd-Math.max(a,c)));
      reserve=Math.min(reserve,Math.min(Math.min(a,d)-screw.fixedEnd,screw.totalLength-screw.floatingEnd-Math.max(a,d)));
    }
    if(reserve<10)throw "X nut loses its 10 mm race reserve at overtravel";
    var derivation=machine.derivedFasteners();if(derivation==null)throw "missing joint-derived X flange fasteners";
    var cache=new PosedParts(), count=0, reach=0.0;
    for(j in derivation.joints)if(j.clampedMember=="Xnut"&&j.receivingMember=="XnutBlock")for(v in j.volumes)if(v.access)for(h in j.worldHoles) {
      var p=F.transformPoint(back,h.x,h.y,h.z), start=p.x-v.to, length=HobberLayout.get().xNominal().bedRear+5-start;
      var tool=Part.cylinderAlong(v.diameter/2,new Vector(start,p.y,p.z),Vector.X(),length), box=PosedParts.boxOf(tool);
      for(m in members)if(["Xmotor","Xmotor-shaft","Xfixed","XfloatingRing"].indexOf(m.id)<0) {
        var pose=F.compose(back,poses.get(m.id)), other=cache.box(m.component,pose);if(PosedParts.apart(box,other,0))continue;
        var part=cache.posed(m.component,pose), common=part.intersect(tool), volume=common.volume();common.close();part.close();
        if(volume>1e-3)throw 'X flange long driver meets ${m.id}: $volume mm3';
      }
      reach=Math.max(reach,length);tool.close();count++;
    }
    var moduleMass=0.0, stroke=0.0, endClear=0.0;
    for(r in machine.serviceRemovals()) switch r {
      case ServiceRemoval("erobWithSaddle", group, _, _, _): moduleMass=HobberJointLoads.massOf(machine,group);
      case ServiceRemoval("xScrewWithdrawal", _, steps, _, _):
        stroke=steps[0].distance;
        var bounds=cache.box(screw,F.compose(back,poses.get("Xscrew")));
        endClear=bounds.minX+stroke-HobberLayout.get().xNominal().bedRear;
      default:
    }
    if(endClear<HobberServiceAccess.SERVICE_CLEARANCE-1e-7)throw "X screw stroke does not clear the rear of the bed";
    cache.close();if(count==0)throw "no X flange driver axes checked";
    return ['X service: motor-side flange at X${flange.x}; block datum ${b.minX}..${b.maxX} retained. Whole-nut race reserve $reserve mm, barrel reserve $barrel mm at 5 mm overtravel.',
      'X flange access: $count joint-derived long-driver corridors clear to bed rear +5; maximum reach ${Math.ceil(reach)} mm. Motor, BK12 and BF ring removed; chip guards retained.',
      'Service module: Z saddle, eRob and head ${Math.round(moduleMass*100)/100} kg with Y servo off; X withdrawal $stroke mm, screw-tip clearance past bed $endClear mm.'];
  }
}
