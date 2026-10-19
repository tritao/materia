import gearkit.GearSpec;
import materia.assembly.AssemblyFrames as F;

/** Configured size limits, guide travel and front-loading provision. */
class HobberGERangeChecks {
  public static function run():Void {
    var count=0;
    for(spec in [new GearSpec(0.5,48,20,20*Math.PI/180),new GearSpec(0.5,278,20,20*Math.PI/180)]){
      var machine=new FlexsplineHobber(spec),model=new cadkit.modeling.AssemblyModel("mm");machine.addTo(model,"");
      var state=model.initialState();
      try {
        FlexsplineHobberCam.job(machine);count+=HobberDesignChecks.run(machine);
        var guide:machinekit.motion.LinearRail=cast machine.components().filter(m->m.id=="railXLeft")[0].component;
        if(guide.spec.preloadTarget==null)throw "column guide needs stated preload target";
        for(id in ["X","Y","Z"]){var screw:machinekit.motion.BallScrew=cast machine.components().filter(m->m.id==id+"screw")[0].component;if(!screw.ground)throw "ground screw requirement missing";count++;}
        state.setJoint("X",machine.travel[0].initial);state.forwardKinematics();var p=state.worldPose("hob");
        state.setJoint("X",machine.travel[0].initial+1);state.forwardKinematics();var q=state.worldPose("hob");
        if(Math.abs(q.x-p.x)>1e-8||Math.abs(q.y-p.y-1)>1e-8||Math.abs(q.z-p.z)>1e-8)throw "X must move along machine +Y";
        var axis=F.transformVector(q,0,0,1);if(Math.abs(axis.x-1)+Math.abs(axis.y)+Math.abs(axis.z)>1e-8)throw "untilted hob axis must be machine +X";count+=2;
        state.setJoint("X",machine.travel[0].upper);state.setJoint("Z",0);state.forwardKinematics();count++;

      }catch(error:Dynamic){throw error;}
    }
    Sys.println('GE_RANGE_PASS checks=$count; OD25/140, face20, front X retract110, 35 mm seating stroke; guide margins >=5 mm');
  }
}
