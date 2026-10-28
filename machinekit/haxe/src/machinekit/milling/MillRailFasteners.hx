package machinekit.milling;

import machinekit.assembly.MachineAssembly;
import machinekit.design.DesignInterfaces;
import machinekit.design.DesignRequirement;
import machinekit.motion.LinearRail;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.structural.MineralFoundation;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Install real service screws from the actual placed rail and steel interfaces.
 * Numerical engagement uses the same source policy as fixtures and anchors.
 */
class MillRailFasteners {
  public static function install(
    assembly:MachineAssembly,
    foundation:MineralFoundation,
    prefix:String,
    preferredLengths:Array<Float>
  ):Void {
    var poses = assembly.solvedPoses();
    var members:Map<String, machinekit.component.MachineComponent> = [];
    for (member in assembly.components()) members.set(member.id, member.component);
    for (location in foundation.insertLocations()) {
      var spec = location.part.insertSpec();
      var manufacturing = switch spec.origin {
        case Fabricated(plan):
          plan;
        case Catalogue(_):
          continue;
      };
      var target = prefix + "/" + location.id;
      for (requirement in manufacturing.mountingRequirements) {
        var part = members.get(requirement.componentPath);
        if (!Std.isOfType(part, LinearRail)) throw "Source mounting rail is absent: " + requirement.componentPath;
        var rail:LinearRail = cast part;
        var localId = requirement.componentPath.split("/").pop();
        if (MillRailPlacement.fingerprint(rail,
          localId) != requirement.componentFingerprint) throw "Placed rail differs from source hole provenance: "
          + target + "/" + requirement.holeId;
        var geometry = rail.mountingHole();
        if (geometry == null) throw "Rail counterbore geometry is unsupported: " + requirement.componentPath;
        var localSeat = F.compose(
          rail.connector(requirement.connectorId).frame,
          F.translation(0, -geometry.counterboreDepth, 0)
        );
        var seat = F.compose(poses.get(requirement.componentPath), localSeat);
        var targetPart = members.get(target), facet = targetPart == null ? null : DesignInterfaces.of(targetPart);
        if (facet == null) throw "Source steel interface is absent: " + target;
        var hole = facet.hole(requirement.holeId);
        var holePose = F.compose(poses.get(target), hole.frame);
        var bolt = select(target + "/" + hole.id, rail, seat, hole, holePose, preferredLengths);
        var id = idFor(location.id, hole.id), seatId = materia.tooling.ToolingIds.derive("rail-seat-", hole.id);
        assembly.addMemberConnector(requirement.componentPath, seatId, localSeat);
        assembly.addComponent(id, bolt);
        assembly.addMate("fix-" + id, "fixed", requirement.componentPath, seatId, id, "head");
        assembly.addDesignRequirement(Coaxial(
          "seat-" + id,
          {member: id, feature: "head"},
          {member: target, feature: hole.id},
          1e-6
        ));
        assembly.addDesignRequirement(Coaxial(
          "rail-" + id,
          {member: id, feature: "head"},
          {member: requirement.componentPath, feature: requirement.connectorId},
          1e-6
        ));
      }
    }
  }

  public static function idFor(insertId:String, holeId:String):String return materia.tooling.ToolingIds.derive(
    "rail-bolt-",
    insertId,
    "-" + holeId
  );

  static function select(
    id:String,
    rail:LinearRail,
    seat:AssemblyFrame,
    hole:machinekit.design.DesignInterfaces.HoleInterface,
    holePose:AssemblyFrame,
    preferredLengths:Array<Float>
  ):SocketHeadCapScrew return machinekit.structural.MountServiceScrews.select(
    id,
    machinekit.structural.MountedElement.of(rail),
    seat,
    hole,
    holePose,
    preferredLengths
  );

}
