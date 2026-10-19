import gearkit.GearSpec;
import materia.assembly.AssemblyFrames as F;
import machinekit.assembly.PosedParts;

/** Configured size limits, guide travel and unobstructed top-loading provision. */
class HobberGERangeChecks {
  public static function run():Void {
    var count=0;
    for(spec in [new GearSpec(0.5,48,20,20*Math.PI/180),new GearSpec(0.5,278,20,20*Math.PI/180)]){
      var machine=new FlexsplineHobber(spec),model=new cadkit.modeling.AssemblyModel("mm");machine.addTo(model,"");
      var state=model.initialState(),cache=new PosedParts();
      try {
        FlexsplineHobberCam.job(machine);count+=HobberDesignChecks.run(machine);
        var guide:machinekit.motion.LinearRail=cast machine.components().filter(m->m.id=="railXLeft")[0].component;
        if(guide.spec.preloadTarget==null)throw "column guide needs stated preload target";
        for(id in ["X","Y","Z"]){var screw:machinekit.motion.BallScrew=cast machine.components().filter(m->m.id==id+"screw")[0].component;if(!screw.ground)throw "ground screw requirement missing";count++;}
        state.setJoint("X",machine.travel[0].initial);state.forwardKinematics();var p=state.worldPose("hob");
        state.setJoint("X",machine.travel[0].initial+1);state.forwardKinematics();var q=state.worldPose("hob");
        if(Math.abs(q.x-p.x)>1e-8||Math.abs(q.y-p.y-1)>1e-8||Math.abs(q.z-p.z)>1e-8)throw "X must move along machine +Y";
        var axis=F.transformVector(q,0,0,1);if(Math.abs(axis.x-1)+Math.abs(axis.y)+Math.abs(axis.z)>1e-8)throw "untilted hob axis must be machine +X";count+=2;
        state.setJoint("X",250);state.setJoint("Z",190);
        for(a in [-Math.PI/12,0.0,Math.PI/12]){state.setJoint(FlexsplineHobber.A,a);state.forwardKinematics();
          for(member in machine.components()){
            if(member.id=="mandrel"||member.id=="blank")continue; // Intended loading/locating interfaces.
            var b=cache.box(member.component,state.worldPose(member.id));
            if(!(b.minX>=80||b.maxX<=-80||b.minY>=80||b.maxY<=-80||b.maxZ<=363)){
              var solid=cache.posed(member.component,state.worldPose(member.id));
              var opening=cadkit.modeling.Part.cylinderSpan(80,363,1200),volume=PosedParts.sharedVolume(solid,opening);
              solid.close();opening.close();if(volume>0.001)
                throw 'part obstructs reserved top-loading cylinder: ${member.id}, OD=${2*spec.tipRadius()}';
            }
            count++;
          }
        }
      }catch(error:Dynamic){cache.close();throw error;}cache.close();
    }
    Sys.println('GE_RANGE_PASS checks=$count; OD25/140, face20, X retract250, top opening radius80; guide margins >=5 mm');
  }
}
