package processkit.screwdriving;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneScrewDefinition;
import materia.project.SceneScrewDefinition.SceneScrew;
import processkit.screwdriving.TighteningModel.TighteningJoint;
import processkit.simulation.SimulatedScrewdriver.ScrewdriverWorkState;
import robotkit.spatial.Transform3;
import processkit.screwdriving.ScrewPlan.ScrewFinalContact;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import nativekit.sim.SimPose;

/** Interprets a current CAD packet in live occurrence frames; no geometry is guessed. */
class ScrewScenePlan {
  public static function frame(value:AssemblyFrame):Transform3
    return new Transform3(new Vec3(value.x,value.y,value.z),new Quat(value.qx,value.qy,value.qz,value.qw));
  public static function pose(value:Transform3):SimPose {
    var p = value.translation, q = value.rotation;
    return new SimPose(p.x,p.y,p.z,q.x,q.y,q.z,q.w);
  }
  public static function plan(job:SceneScrew, presenter:Null<Transform3>, work:Transform3,
      ?finalContact:ScrewFinalContact, stopAtSeat:Bool = false, slot:Int = 0):ScrewPlan
    {
      var pickup:Null<Transform3> = null, toolFed = false;
      switch job.delivery {
        case Presented:
          var point = job.pickup;
          if (presenter == null || point == null) throw "Presented fastening lost its live pickup fixture/frame";
          pickup = presenter.compose(frame(point));
        case BlowFeed(_): toolFed = true;
      }
      return new ScrewPlan(job.fastening,slot == 0 ? job.slot : slot,new TighteningProgram(job.program,stopAtSeat),
        pickup,work.compose(frame(job.entry)),job.leadMm,SceneScrewDefinition.runDownMm(job),job.approach,finalContact,toolFed);
    }
  public static function joint(job:SceneScrew, springRate:Float):TighteningJoint {
    var response = job.simulation;
    return {leadMm:job.leadMm,runDownMm:SceneScrewDefinition.runDownMm(job),leadInMm:response.leadInMm,
      maximumAxisAngle:response.maximumAxisAngle,springRate:springRate,prevailingTorque:response.prevailingTorque,
      stiffnessNmPerRad:response.stiffnessNmPerRad,stripTorque:response.stripTorque};
  }
  /** Body origins are collision centres, while socket and hole frames are CAD datums. */
  public static function work(job:SceneScrew, live:Transform3, center:Array<Float>, damagedLeadIn:Bool = false):ScrewdriverWorkState {
    if (center == null || center.length != 3) throw 'Fastening "${job.fastening}" needs the screw collision centre';
    var head = live.compose(frame(job.entry));
    var body = head.compose(frame(job.socket).inverse()).compose(new Transform3(Vec3.fromArray(center),Quat.identity()));
    var hole = live.compose(frame(job.hole));
    return {head:pose(head),screwPose:pose(body),engagementPoint:hole.translation.toArray(),
      hole:hole.translation.toArray(),direction:hole.transformVector(new Vec3(0,0,1)).toArray(),damagedLeadIn:damagedLeadIn};
  }
}
