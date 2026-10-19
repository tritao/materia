import materia.assembly.AssemblyFrames as F;
import machinekit.assembly.PosedParts;
import gearkit.GearSpec;

/** Exact top-loading path, separate from the illustrative animation and pressure response. */
class HobberLoadingChecks {
  public static function run():Void {
    var count=0;
    for(reference in [false,true]) {
      var machine=reference?new FlexsplineHobber():new FlexsplineHobber(new GearSpec(1,40,4,20*Math.PI/180),4);
      var model=new cadkit.modeling.AssemblyModel("mm");machine.addTo(model,"");var state=model.initialState();
      state.setJoint("X",250);state.setJoint("Z",0);state.setJoint(FlexsplineHobber.A,0);state.forwardKinematics();
      var members=machine.components(),cup=members.filter(m->m.id=="blank")[0],sleeve=members.filter(m->m.id=="mandrelSleeve")[0],cache=new PosedParts();
      var diameter=cup.component.parameterNumber("bore"),gap=(diameter-machine.mandrel.parameterNumber("outside"))/2;
      if(Math.abs(gap-0.025)>1e-8)throw "collapsed sleeve must have 25 um radial loading clearance";
      var seat=machine.mandrel.parameterNumber("workSeat"),start=machine.mandrel.parameterNumber("sleeveStart"),end=start+machine.mandrel.parameterNumber("sleeveLength");
      if(start>seat||end<seat+machine.spec.faceWidth)throw "sleeve must support entire tooth band";
      if(machine.mandrel.parameterNumber("height")>=seat+cup.component.parameterNumber("height")-cup.component.parameterNumber("endThickness"))throw "mandrel hits closed cup end";
      try {
        for(lift in [0.0,10.0,20.0,30.0,40.0,60.0,80.0,100.0,120.0]) {
          var pose=F.compose(state.worldPose("blank"),F.translation(0,0,lift));
          for(m in members)if(m.id!="blank") {
            var volume=cache.overlap(cup.component,pose,m.component,state.worldPose(m.id));count++;
            if(volume>0.001)throw 'cup loading at $lift mm hits ${m.id}: $volume mm3';
          }
        }
        var cupShape=cup.component.geometry();if(cupShape.solidCount()!=1)throw "cup blank must be one continuous solid";cupShape.close();
      }catch(e:Dynamic){cache.close();throw e;}
      cache.close();
    }
    Sys.println('HOBBER_LOADING_PASS $count exact cup/neighbour checks; 25 um collapsed radial gap; rim support and closed-end clearance passed');
  }
}
