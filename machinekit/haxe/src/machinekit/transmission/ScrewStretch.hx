package machinekit.transmission;

import machinekit.assembly.FlatAssembly;
import machinekit.motion.LeadScrew;
import machinekit.motion.LeadScrewNut;
import machinekit.motion.ScrewSupportUnit;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyDefinition.AssemblyJointType;

/** Axial load path from the nut's physical bore to the actual fixed bearing, not the bending support or full shaft length. */
class ScrewStretch {
  public static function law(flat:FlatAssembly, screwId:String, nutId:String, leader:String,
      ?context:BeltPoseContext):Array<materia.assembly.AssemblyDefinition.AssemblyComplianceSpring> {
    var screw:LeadScrew = cast flat.member(screwId), nut:LeadScrewNut = cast flat.member(nutId);
    if (context == null) context = new BeltPoseContext(flat.definition);
    context.reset();
    var state = context.state, inverse = AssemblyFrames.inverse(state.worldPose(screwId));
    var bearing = Math.NaN, bearingStiffness = 0.0;
    for (member in flat.members) if (Std.isOfType(member.component,ScrewSupportUnit)) {
      var unit:ScrewSupportUnit = cast member.component;
      if (!unit.fixed) continue;
      var world = state.worldConnector(member.id,"axis");
      var local = AssemblyFrames.transformPoint(inverse,world.x,world.y,world.z);
      if (Math.abs(local.x)>1e-5 || Math.abs(local.y)>1e-5 || local.z<0 || local.z>screw.totalLength) continue;
      var axis = AssemblyFrames.transformVector(AssemblyFrames.compose(inverse,world),0,1,0);
      if (Math.abs(axis.x)>1e-5 || Math.abs(axis.y)>1e-5) continue;
      if (Math.isFinite(bearing)) throw new TransmissionDesignError("Two axial fixed supports need an explicit parallel shaft/support model");
      var rigidity = unit.axialStiffness();
      if (rigidity == null) throw new TransmissionDesignError("Fixed support needs axial rigidity");
      bearing = local.z; bearingStiffness = rigidity;
    }
    if (!Math.isFinite(bearing)) throw new TransmissionDesignError('Ball screw "$screwId" needs its physical fixed axial support');
    var bore = state.worldConnector(nutId,"bore");
    var local = AssemblyFrames.transformPoint(inverse,bore.x,bore.y,bore.z);
    if (Math.abs(local.x)>1e-5 || Math.abs(local.y)>1e-5) throw new TransmissionDesignError("Ball screw nut and fixed bearing must be coaxial");
    var joint = [for (candidate in flat.definition.joints) if (candidate.id == leader) candidate];
    if (joint.length!=1 || joint[0].type!=AssemblyJointType.Prismatic || joint[0].limits.lower==null || joint[0].limits.upper==null)
      throw new TransmissionDesignError("Ball screw stiffness needs bounded nut travel");
    var edge = joint[0];
    var axis = AssemblyFrames.transformVector(AssemblyFrames.compose(inverse,state.worldConnector(edge.parent,edge.parentConnector)),
      edge.axis.x,edge.axis.y,edge.axis.z);
    if (Math.abs(axis.x)>1e-5 || Math.abs(axis.y)>1e-5 || Math.abs(Math.abs(axis.z)-1.0)>1e-5)
      throw new TransmissionDesignError("Ball screw carriage must travel along its shaft");
    var direction = local.z>bearing ? 1.0 : -1.0;
    var gradient = direction*axis.z/screw.axialRigidityAt(local.z);
    var shaft = screw.axialComplianceBetween(bearing,local.z);
    var lower = edge.limits.lower, upper = edge.limits.upper;
    if (lower == null || upper == null) throw new TransmissionDesignError("Ball screw nut travel must be bounded");
    var overtravel = edge.limits.overtravel == null ? 0.0 : edge.limits.overtravel;
    for (q in [lower-overtravel,upper+overtravel]) {
      var station = local.z+axis.z*(q-edge.defaultValue);
      if ((station-bearing)*direction<=0 || Math.abs(screw.axialComplianceBetween(bearing,station)-shaft-gradient*(q-edge.defaultValue))>1e-12)
        throw new TransmissionDesignError("Ball screw nut travel crosses its fixed bearing or a shaft-section boundary");
    }
    var nutStiffness = nut.axialStiffness();
    if (nutStiffness == null) throw new TransmissionDesignError("Ball screw nut needs its axial rigidity");
    return [{compliance:shaft+1.0/bearingStiffness+1.0/nutStiffness-gradient*edge.defaultValue,
      coordinates:[{joint:leader,coefficient:gradient}]}];
  }
}
