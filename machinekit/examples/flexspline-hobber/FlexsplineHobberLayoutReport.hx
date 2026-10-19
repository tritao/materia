import materia.assembly.AssemblyFrames as F;
class FlexsplineHobberLayoutReport {
  public static function sceneBounds(path:String):Dynamic {
    var data=materia.project.SceneArtifact.decode(sys.io.File.getBytes(path));
    var definition=data.assemblyDefinition;if(definition==null)throw "scene definition required";
    var state=cadkit.modeling.AssemblyState.fromModel(cadkit.modeling.CompiledAssembly.takeOwnership(definition),data.assemblyState);state.forwardKinematics();
    var loX=1e30,hiX=-1e30,loY=1e30,hiY=-1e30;
    for(occurrence in definition.occurrences){var matches=data.parts.filter(p->p.id==occurrence.definition);if(matches.length==0)continue;
      var mesh=matches[0],pose=state.worldPose(occurrence.id);
      for(i in 0...mesh.vertexCount){var p=F.transformPoint(pose,mesh.vertices.getDouble(i*24),mesh.vertices.getDouble(i*24+8),mesh.vertices.getDouble(i*24+16));
        loX=Math.min(loX,p.x);hiX=Math.max(hiX,p.x);loY=Math.min(loY,p.y);hiY=Math.max(hiY,p.y);}
    }
    return {globalXmm:hiX-loX,globalYmm:hiY-loY,scope:"mesh bounds at exported pose, including electrical cabinet"};
  }
  public static function measure(machine:FlexsplineHobber):Dynamic {
    var poses=machine.solvedPoses(),members=machine.components(),def=machine.definition(),masses:Map<String,Float> = [],total=0.0;
    for(m in members){var mass=m.component.massProperties().mass;masses.set(m.id,mass);total+=mass;}
    var tool:materia.assembly.AssemblyRecord.AssemblyFrame=F.compose(poses.get("hob"),machine.hob.connector("centre").frame),offsets:Map<String,Float> = [];
    for(axis in ["X","Y","Z"]){var rail=members.filter(m->Std.isOfType(m.component,machinekit.motion.LinearRail)&&m.id.indexOf("rail"+axis)==0)[0];
      var frame:materia.assembly.AssemblyRecord.AssemblyFrame=poses.get(rail.id);var normal=F.transformVector(frame,0,1,0);offsets.set(axis,Math.abs((tool.x-frame.x)*normal.x+(tool.y-frame.y)*normal.y+(tool.z-frame.z)*normal.z));}
    var moving:Map<String,Float> = [];
    for(axis in ["X","Y","Z"]){var root=def.joints.filter(j->j.id==axis)[0].child,ids:Map<String,Bool> = [];ids.set(root,true);
      var changed=true;while(changed){changed=false;for(j in def.joints)if(ids.exists(j.parent)&&!ids.exists(j.child)){ids.set(j.child,true);changed=true;}}
      var mass=0.0;for(id in ids.keys())if(masses.exists(id))mass+=masses.get(id);moving.set(axis,mass);}
    var drives:Array<Dynamic> = [];
    for(axis in ["X","Y","Z"]){
      var motor:machinekit.motion.ServoMotor=cast members.filter(m->m.id==axis+"motor")[0].component;
      var nut:machinekit.motion.BallNut=cast members.filter(m->m.id==axis+"nut")[0].component;
      var available=2*Math.PI*motor.rating.ratedTorque*nut.efficiency()/0.005*0.8;
      var mass:Float=cast moving.get(axis);var demand=mass*(axis=="Z"?9.80665+0.5:0.5)+200;
      drives.push({axis:axis,movingMassKg:mass,continuousThrustWith20PercentReserveN:available,designDemandN:demand,passed:available>demand,
        basis:"assumed 0.5 m/s2 acceleration, 100 N process force plus 100 N friction; Z includes gravity; 5 mm lead and declared nut efficiency"});
    }
    return {drives:drives,bedFootprintRadialTangentialMm:[1000,650],guideToHobMm:{X:offsets.get("X"),Y:offsets.get("Y"),Z:offsets.get("Z")},movingMassKg:{X:moving.get("X"),Y:moving.get("Y"),Z:moving.get("Z")},machineMassKg:total,members:members.length};
  }
}
