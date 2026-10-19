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
  /** Radial side view from actual posed solid envelopes; Y-offset parts are projected. */
  public static function writeSection(machine:FlexsplineHobber,path:String):Void {
    var poses=machine.solvedPoses(),cache=new machinekit.assembly.PosedParts();
    var work=F.inverse(FlexsplineHobber.MACHINE_FROM_WORK);
    var hob=F.compose(work,F.compose(poses.get("hob"),machine.hob.connector("centre").frame));
    var lines=['<svg xmlns="http://www.w3.org/2000/svg" width="1120" height="900" viewBox="0 0 1120 900">',
      '<rect width="1120" height="900" fill="white"/><g font-family="sans-serif" font-size="14" stroke-width="1.2">',
      '<text x="30" y="28">GE hobber — dimensioned radial section (schematic), mm</text>',
      '<text x="30" y="50">Central mounting section; Y-offset guides, spindle and hob projected. Not a manufacturing drawing.</text>'];
    function px(x:Float):Float return 220+1.2*x;
    function py(z:Float):Float return 580-1.05*z;
    var ids=["column/casting","column/driveSeat","Zplate","railZLeft","column/railLeft","blockZLeft","swivel/housing","swivel/bearing","swivel/motor","swivel/clamp","swivel/head","swivelPlate","railYLeft","railYRight","blockYLeft","blockYRight","Yplate","spindle/housing","hob","table/platen","mandrel"];
    var row=0;
    try {
      for(id in ids){var member=machine.components().filter(m->m.id==id)[0];
        var pose=F.compose(work,poses.get(id)),b=cache.box(member.component,pose);
        var x0=b.minX-hob.x,x1=b.maxX-hob.x,z0=b.minZ-hob.z,z1=b.maxZ-hob.z;
        var fill=id=="hob"?"#e4a33a":id.indexOf("swivel")>=0?"#85b9de":"#c5ccd0";
        // Show the central load path, rather than an off-axis nut carrier's enclosing box.
        if(id=="Zplate"){x0=135;x1=148;z0=-65;z1=170;}
        if(id=="Yplate"){x0=45;x1=57;z0=-75;z1=95;}
        if(id=="column/casting")x0=261; // Central recess: the rear wall, not its side cheeks.
        function rect(a:Float,c:Float,d:Float,e:Float):Void lines.push('<rect x="'+px(a)+'" y="'+py(e)+'" width="'+((c-a)*1.2)+'" height="'+((e-d)*1.05)+'" fill="'+fill+'" fill-opacity="0.55" stroke="#334455"/>');
        if(id=="swivel/housing"){
          for(side in [-1,1]){rect(130,135,side<0?-110:50,side<0?-50:110);rect(120,130,side<0?-110:90,side<0?-90:110);rect(110,120,side<0?-110:100,side<0?-100:110);}
        }else if(id=="swivel/bearing"||id=="swivel/motor"||id=="swivel/clamp"||id=="swivel/head"){
          var bore=member.component.parameterNumber("bore")/2,outside=member.component.parameterNumber("outside")/2;
          rect(x0,x1,bore,outside);rect(x0,x1,-outside,-bore);
        }else if(id=="spindle/housing"||id=="hob"){
          var r=(x1-x0)/2;
          lines.push('<ellipse cx="'+px((x0+x1)/2)+'" cy="'+py((z0+z1)/2)+'" rx="'+(r*1.2)+'" ry="'+(r*1.05)+'" fill="none" stroke="#334455" stroke-dasharray="5 3"/>');
        }else rect(x0,x1,z0,z1);
        lines.push('<text x="740" y="'+(95+row*22)+'">'+id+': radial '+Math.round(x0*100)/100+'..'+Math.round(x1*100)/100+' mm</text>');row++;
      }
    }catch(e:Dynamic){cache.close();throw e;}cache.close();
    // Projected column cheek joins the off-axis rail insert to the central rear wall.
    lines.push('<rect x="'+px(186)+'" y="'+py(427)+'" width="'+(100*1.2)+'" height="'+(500*1.05)+'" fill="none" stroke="#667788" stroke-dasharray="6 4"/><text x="740" y="565">Dashed column cheek: Y-offset projection</text>');
    lines.push('<path d="M '+px(0)+' 80 V 680" stroke="#aa4433" stroke-dasharray="6 4"/><text x="'+(px(0)-100)+'" y="670">hob centre radial = 0</text>');
    var dimensions=[{x:70.0,label:"Y guide axis plane: 70 mm"},{x:114.0,label:"A bearing front face: 114 mm"},{x:135.0,label:"Z saddle front / A mounting face: 135 mm"},{x:161.0,label:"Z guide axis plane: 161 mm"}];
    for(i in 0...dimensions.length){var d=dimensions[i],y=730+i*36;
    lines.push('<path d="M '+px(0)+' '+y+' H '+px(d.x)+' M '+px(0)+' '+(y-5)+' V '+(y+5)+' M '+px(d.x)+' '+(y-5)+' V '+(y+5)+'" fill="none" stroke="#333"/><text x="560" y="'+(y+5)+'">'+d.label+'</text>');}
    lines.push('</g></svg>');sys.io.File.saveContent(path,lines.join("\n"));
  }
  public static function measure(machine:FlexsplineHobber):Dynamic {
    var poses=machine.solvedPoses(),members=machine.components(),def=machine.definition(),masses:Map<String,Float> = [],total=0.0;
    for(m in members){var mass=m.component.massProperties().mass;masses.set(m.id,mass);total+=mass;}
    var tool:materia.assembly.AssemblyRecord.AssemblyFrame=F.compose(poses.get("hob"),machine.hob.connector("centre").frame),offsets:Map<String,Float> = [];
    for(axis in ["X","Y","Z"]){var rail=members.filter(m->Std.isOfType(m.component,machinekit.motion.LinearRail)&&m.id.indexOf("rail"+axis)==0)[0];
      var frame:materia.assembly.AssemblyRecord.AssemblyFrame=poses.get(rail.id);var normal=F.transformVector(frame,0,1,0);offsets.set(axis,Math.abs((tool.x-frame.x)*normal.x+(tool.y-frame.y)*normal.y+(tool.z-frame.z)*normal.z));}
    var collisionPieces:Array<Dynamic> = [],collisionTotal=0;
    for(m in members){var facet=machinekit.component.CollisionHullFacet.of(m.component),count=facet==null?1:facet.hulls.length;
      collisionTotal+=count;collisionPieces.push({part:m.id,hulls:count});}
    var moving:Map<String,Float> = [];
    for(axis in ["X","Y","Z"]){var root=def.joints.filter(j->j.id==axis)[0].child,ids:Map<String,Bool> = [];ids.set(root,true);
      var changed=true;while(changed){changed=false;for(j in def.joints)if(ids.exists(j.parent)&&!ids.exists(j.child)){ids.set(j.child,true);changed=true;}}
      var mass=0.0;for(id in ids.keys())if(masses.exists(id))mass+=masses.get(id);moving.set(axis,mass);}
    var drives:Array<Dynamic> = [];
    for(axis in ["X","Y","Z"]){
      var motor:machinekit.motion.ServoMotor=cast members.filter(m->m.id==axis+"motor")[0].component;
      var nut:machinekit.motion.BallNut=cast members.filter(m->m.id==axis+"nut")[0].component;
      var available=2*Math.PI*motor.rating.ratedTorque*nut.efficiency()/0.005*0.8;
      var mass:Float=cast moving.get(axis);var demand=mass*(0.5+(axis=="Z"?9.80665:axis=="Y"?9.80665*Math.sin(Math.PI/12):0))+200;
      if(available<=demand)throw "continuous thrust requirement failed for "+axis;
      drives.push({thrustMarginN:available-demand,axis:axis,movingMassKg:mass,continuousThrustWith20PercentReserveN:available,designDemandN:demand,passed:available>demand,
        basis:"assumed 0.5 m/s2 acceleration, 100 N process force plus 100 N friction; Z includes full gravity and Y includes gravity at A +/-15 degrees; 5 mm lead and declared nut efficiency"});
    }
    var cache=new machinekit.assembly.PosedParts(),loX=1e30,hiX=-1e30,loY=1e30,hiY=-1e30;
    var canonical=F.inverse(FlexsplineHobber.MACHINE_FROM_WORK);
    try {for(m in members){var b=cache.box(m.component,F.compose(canonical,poses.get(m.id)));loX=Math.min(loX,b.minX);hiX=Math.max(hiX,b.maxX);loY=Math.min(loY,b.minY);hiY=Math.max(hiY,b.maxY);}}
    catch(e:Dynamic){cache.close();throw e;}cache.close();
    return {collisionShapes:{authoredPlusDefaultTotal:collisionTotal,parts:collisionPieces},installedPoseEnvelopeMm:{radial:hiX-loX,tangential:hiY-loY,scope:"initial pose including separate electrical rack; CAD envelope bounds"},headStackMm:{yGuidePlane:70,swivelBearingFrontFace:114,swivelOutputFace:100,zSaddleFront:135,zGuidePlane:161,verticalGuideSeparation:120},drives:drives,bedFootprintRadialTangentialMm:[machine.bed.body.parameterNumber("width"),machine.bed.body.parameterNumber("depth")],guideToHobMm:{X:offsets.get("X"),Y:offsets.get("Y"),Z:offsets.get("Z")},movingMassKg:{X:moving.get("X"),Y:moving.get("Y"),Z:moving.get("Z")},machineMassKg:total,members:members.length};
  }
}
