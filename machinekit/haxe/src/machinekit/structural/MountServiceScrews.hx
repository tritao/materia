package machinekit.structural;

import machinekit.assembly.Diagnostics;
import machinekit.design.DesignChecks;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.standard.PropertyClass;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;

/** One engagement/counterbore/coaxial selection for all mounted elements. */
class MountServiceScrews {
  public static function select(
    id:String,
    element:MountedElement,
    seat:AssemblyFrame,
    hole:machinekit.design.DesignInterfaces.HoleInterface,
    holePose:AssemblyFrame,
    preferredLengths:Array<Float>
  ):SocketHeadCapScrew {
    var thread = hole.thread;
    if (thread == null) throw "Element mounting needs a source steel thread: " + id;
    var lengths = preferredLengths.copy();
    lengths.sort(Reflect.compare);
    for (length in lengths) {
      if (!Math.isFinite(length) || length <= 0) throw "Invalid preferred service screw length";
      var screw = SocketHeadCapScrew.metric(thread.spec.designation, length, PropertyClass.Steel88);
      var geometry = element.bore;
      if (geometry == null || screw.spec.headDiameter > geometry.counterboreDiameter
        || screw.spec.headHeight > geometry.counterboreDepth || screw.diameter
        > geometry.diameter) throw "Element mounting screw does not fit the declared bore and counterbore: " + id;
      if (!screw.thread.matches(thread.spec)
        || !screw.thread.matches(element.thread)) throw "Element and receiver threads differ: " + id;
      var pose = F.compose(seat, F.inverse(screw.connector("head").frame));
      var bearing = F.compose(pose, screw.bearingFrame()), axis = F.transformVector(bearing, 0, 0, 1);
      var grip = (holePose.x
        - bearing.x) * axis.x + (holePose.y - bearing.y) * axis.y + (holePose.z - bearing.z) * axis.z;
      if (grip <= 0) throw "Element screw has no positive grip: " + id;
      var checks = new Diagnostics();
      DesignChecks.coaxial(checks, id, bearing, holePose, 1e-6);
      DesignChecks.blindFastener(
        checks,
        id,
        length,
        grip,
        thread.usableDepth,
        thread.cylindricalDepth,
        thread.point,
        thread.stock,
        thread.minEngagement,
        thread.minTipGap,
        thread.minBack,
        hole.relief == null ? null : hole.relief.start
      );
      if (length - screw.threadLength > grip + 1e-6) checks.error(
        "design.unthreaded-shank",
        id,
        "Unthreaded shank enters the insert thread"
      );
      if (!checks.hasErrors()) return screw;
    }
    throw "No preferred element screw satisfies source engagement, tip gap and back wall: " + id;
  }
}
