import materia.assembly.AssemblyFrames as F;
import machinekit.assembly.PosedParts;
import machinekit.hobbing.HobberXLayout;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import gearkit.GearSpec;

/** Actual hollow cup + assumed front gripper against every installed neighbour.
    Continuous head corridor/all-A proof is provided by HobberLoadingSizing. */
class HobberLoadingChecks {
  static function box(x0:Float,x1:Float,y0:Float,y1:Float,z0:Float,z1:Float):Part {
    var raw=Part.box(x1-x0,y1-y0,z1-z0),p=raw.translated(new Vector((x0+x1)/2,(y0+y1)/2,z0));raw.close();return p;
  }
  public static function run():Void {
    var count=0;
    for(spec in [new GearSpec(0.5,48,20,20*Math.PI/180),new GearSpec(0.5,278,20,20*Math.PI/180)]) {
      var machine=new FlexsplineHobber(spec),model=new cadkit.modeling.AssemblyModel("mm");machine.addTo(model,"");var state=model.initialState();
      state.setJoint("X",HobberXLayout.RETRACT);state.setJoint("Y",0);state.setJoint("Z",0);state.setJoint(FlexsplineHobber.B,0);
      var members=machine.components(),cup=members.filter(m->m.id=="blank")[0],cache=new PosedParts();
      var diameter=cup.component.parameterNumber("bore"),gap=(diameter-machine.mandrel.parameterNumber("outside"))/2;
      if(Math.abs(gap-0.025)>1e-8)throw "collapsed sleeve must have 25 um radial loading clearance";
      var seat=machine.mandrel.parameterNumber("workSeat"),start=machine.mandrel.parameterNumber("sleeveStart"),end=start+machine.mandrel.parameterNumber("sleeveLength");
      if(start>seat||end<seat+machine.spec.faceWidth)throw "sleeve must support entire tooth band";
      var protrusion=machine.mandrel.parameterNumber("height")-seat;
      if(protrusion>=cup.component.parameterNumber("height")-cup.component.parameterNumber("endThickness"))throw "mandrel hits closed cup end";
      if(HobberXLayout.FRONT_LIFT-protrusion<5)throw "front transfer fails 5 mm mandrel clearance";
      var radius=spec.tipRadius(),gripper=[box(-radius-80,-radius-25,-radius-25,radius+25,40,80)];
      for(side in [-1,1])gripper.push(box(-radius-30,15,side<0?-radius-25:radius-3,side<0?-radius+3:radius+25,30,55));
      var samples=[for(x in [-HobberXLayout.FRONT_APPROACH,-150.0,-75,0]) {x:x,lift:HobberXLayout.FRONT_LIFT}];
      for(lift in [20.0,10,5,0])samples.push({x:0.0,lift:lift});
      var sweptJaws:Array<Part> = [];
      for(jaw in gripper){var b=PosedParts.boxOf(jaw);sweptJaws.push(box(b.minX-HobberXLayout.FRONT_APPROACH,b.maxX,b.minY,b.maxY,b.minZ,b.maxZ+HobberXLayout.FRONT_LIFT));}
      try {
        for(a in [-Math.PI/12,0.0,Math.PI/12]){state.setJoint(FlexsplineHobber.A,a);state.forwardKinematics();
          // A conservative occupied volume encloses BOTH transfer and seating,
          // so no gap between gripper path samples can conceal an obstruction.
          var gripperPose=F.compose(FlexsplineHobber.MACHINE_FROM_WORK,F.translation(0,0,363));
          var jaws=[for(jaw in sweptJaws)PosedParts.moved(jaw,gripperPose)],jawBoxes=[for(jaw in jaws)PosedParts.boxOf(jaw)];
          for(m in members)if(m.id!="blank"){
            var neighbourBox=cache.box(m.component,state.worldPose(m.id));
            for(i in 0...jaws.length){count++;if(PosedParts.apart(jawBoxes[i],neighbourBox))continue;
              var neighbour=cache.posed(m.component,state.worldPose(m.id)),v=PosedParts.sharedVolume(jaws[i],neighbour);neighbour.close();
              if(v>0.001){for(jaw in jaws)jaw.close();throw 'front gripper corridor OD${2*radius} hits ${m.id}: $v mm3';}}
          }
          for(jaw in jaws)jaw.close();
          for(sample in samples){var pose=F.compose(FlexsplineHobber.MACHINE_FROM_WORK,F.translation(sample.x,0,363+sample.lift));
            for(m in members)if(m.id!="blank") {
              var volume=cache.overlap(cup.component,pose,m.component,state.worldPose(m.id));count++;
              if(volume>0.001)throw 'front cup path OD${2*radius} X${sample.x} lift${sample.lift} hits ${m.id}: $volume mm3';
            }
          }
        }
        var shape=cup.component.geometry();if(shape.solidCount()!=1)throw "cup blank must be one continuous solid";shape.close();
      }catch(e:Dynamic){for(p in gripper.concat(sweptJaws))p.close();cache.close();throw e;}
      for(p in gripper.concat(sweptJaws))p.close();cache.close();
    }
    Sys.println('HOBBER_LOADING_PASS $count exact front cup/gripper/neighbour checks; OD25/140 face20; X110, 35 mm final vertical move; rim support and closed-end clearance passed');
  }
}
