import materia.assembly.AssemblyFrames as F;
import machinekit.hobbing.FloatingRadialBearing;
import gearkit.GearSpec;
import stockkit.hobbing.HobbingAxes;

/** Nominal installation checks, separate from fits, tool retention and cutting accuracy. */
class HobberInstallationChecks {
  static var assertions=0;
  static function check(ok:Bool,label:String):Void {assertions++;if(!ok)throw label;}
  static function near(a:Float,b:Float,label:String):Void check(Math.abs(a-b)<1e-8,'$label: $a vs $b');
  static function scene(path:String):Void {
    var data=materia.project.SceneArtifact.decode(sys.io.File.getBytes(path));
    var definition=data.assemblyDefinition,record=data.assemblyState;
    if(definition==null||record==null)throw "installed scene needs assembly topology and state";
    var actual=cadkit.modeling.AssemblyState.fromModel(cadkit.modeling.CompiledAssembly.takeOwnership(definition),record);actual.forwardKinematics();
    var machine=new FlexsplineHobber(),expected=FlexsplineHobberPreview.inspectionState(machine);
    check(definition.occurrences.length==machine.components().length,"complete machine occurrences exported");
    check(definition.actuators!=null&&definition.actuators.length==6,"six installed drives exported");
    for(member in machine.components()){
      var a=actual.worldPose(member.id),b=expected.worldPose(member.id);
      near(a.x,b.x,"exported "+member.id+" X");near(a.y,b.y,"exported "+member.id+" Y");near(a.z,b.z,"exported "+member.id+" Z");
      for(i in 0...3){var u=F.transformVector(a,i==0?1:0,i==1?1:0,i==2?1:0),v=F.transformVector(b,i==0?1:0,i==1?1:0,i==2?1:0);
        near(u.x,v.x,"exported axis X");near(u.y,v.y,"exported axis Y");near(u.z,v.z,"exported axis Z");}
    }
    var parts=data.parts.filter(p->p.id=="hob");check(parts.length==1,"one hob geometry definition");var tool=parts[0];
    check(tool.indexCount>10000,"scene contains detailed teeth rather than the blank");
    var maxRadius=0.0,minZ=1e30,maxZ=-1e30;
    for(i in 0...tool.vertexCount){var x=tool.vertices.getDouble(i*24),y=tool.vertices.getDouble(i*24+8),z=tool.vertices.getDouble(i*24+16);
      var r=Math.sqrt(x*x+y*y);check(Math.isFinite(r)&&Math.isFinite(z),"finite installed tool mesh");
      check(r<=machine.hob.design.outsideDiameter/2+1e-6&&z>=-1e-6&&z<=machine.hob.parameterNumber("height")+1e-6,"detailed tool lies inside audited outer envelope");
      maxRadius=Math.max(maxRadius,r);minZ=Math.min(minZ,z);maxZ=Math.max(maxZ,z);
    }
    near(minZ,0,"installed tool near end");near(maxZ,machine.hob.parameterNumber("height"),"installed tool far end");
    var report={assertions:assertions,occurrences:definition.occurrences.length,hobTriangles:Std.int(tool.indexCount/3),hobMaxRadiusMm:maxRadius,
      hobMinZMm:minZ,hobMaxZMm:maxZ,swivelRadians:actual.joint(FlexsplineHobber.A),faceHeightMm:actual.joint("Z"),scope:"exported reference setup and detailed mesh enclosed by audited conservative cylinder"};
    var output=Sys.getEnv("HOBBER_INSTALLATION_EVIDENCE");if(output!=null)sys.io.File.saveContent(output+"/scene.json",haxe.Json.stringify(report));
    Sys.println("HOBBER_INSTALLED_SCENE_PASS "+haxe.Json.stringify(report));
  }
  public static function run():Void {
    var input=Sys.args().indexOf("scene");if(input>=0){scene(Sys.args()[input+1]);return;}

    var rows:Array<Dynamic> = [];
    for(reference in [false,true]){
      var machine=reference?new FlexsplineHobber():new FlexsplineHobber(new GearSpec(1,40,4,20*Math.PI/180),4);
      var state=FlexsplineHobberPreview.inspectionState(machine),arbor=machine.spindle.arbor,hob=machine.hob;
      near(hob.parameterNumber("bore"),arbor.parameterNumber("outside"),"nominal bore/arbor diameter");
      near(hob.parameterNumber("height"),arbor.parameterNumber("hobSeatLength"),"tool spans designed seat length");
      var seatStart=arbor.parameterNumber("hobCentreOffset")-arbor.parameterNumber("hobSeatLength")/2;
      var seatEnd=seatStart+arbor.parameterNumber("hobSeatLength");
      var inverseArbor=F.inverse(state.worldPose("spindle/arbor"));
      var outboard=F.compose(inverseArbor,state.worldPose("outboard"));
      var spindle=F.compose(inverseArbor,state.worldPose("spindle/housing"));
      var housing=machine.components().filter(m->m.id=="spindle/housing")[0].component;
      var housingEnd=F.transformPoint(spindle,0,0,housing.parameterNumber("height"));
      var nearGap=seatStart-housingEnd.z,farGap=outboard.z-seatEnd;
      check(nearGap>0.1&&farGap>0.1,"both hob end faces clear spindle and outboard housings");
      function mounts():Void {
        var tool=F.compose(F.inverse(state.worldPose("spindle/arbor")),state.worldPose("hob"));
        near(tool.x,0,"tool/arbor coaxial X");near(tool.y,0,"tool/arbor coaxial Y");near(tool.z,seatStart,"tool begins at seat datum");
        var axis=F.transformVector(tool,0,0,1);near(axis.x,0,"tool axis X");near(axis.y,0,"tool axis Y");near(axis.z,1,"tool axis Z");
        near(tool.z+hob.parameterNumber("height"),seatEnd,"tool ends at seat datum");
        var bearing:FloatingRadialBearing=cast machine.components().filter(m->m.id=="outboardBearing")[0].component;
        var support=F.compose(F.inverse(state.worldPose("spindle/arbor")),state.worldPose("outboardBearing"));
        near(support.x,0,"support bearing coaxial X");near(support.y,0,"support bearing coaxial Y");
        var supportAxis=F.transformVector(support,0,0,1);near(supportAxis.z,1,"support bearing axis");
        near(bearing.parameterNumber("bore"),arbor.parameterNumber("supportDiameter"),"support journal nominal bore fit");
        check(support.z>=arbor.parameterNumber("height")-arbor.parameterNumber("supportLength")-1e-8&&support.z+bearing.parameterNumber("height")<=arbor.parameterNumber("height")+1e-8,"bearing width lands on support journal");
      }
      mounts();
      var actual=machine.hobToWork(state),expected=new HobbingAxes(state.joint("X"),state.joint("Y"),state.joint("Z"),state.joint(FlexsplineHobber.A),0,state.joint(FlexsplineHobber.C)).poseValues();
      near(actual.x,expected[9],"CAM/setup tool-work X");near(actual.y,expected[10],"CAM/setup tool-work Y");near(actual.z,expected[11],"CAM/setup tool-work Z");
      for(i in 0...3){var axis=F.transformVector(actual,i==0?1:0,i==1?1:0,i==2?1:0);
        near(axis.x,expected[i],"CAM/setup rotation X");near(axis.y,expected[3+i],"CAM/setup rotation Y");near(axis.z,expected[6+i],"CAM/setup rotation Z");}
      var touches=0,minR=1e30,minZ=1e30,maxZ=-1e30;
      var turns=Std.int(Math.ceil(hob.parameterNumber("height")/(2*hob.design.lead)))+2;
      for(g in 0...hob.design.gashes)for(turn in -turns...turns+1)for(p in hob.design.rakeFaceRegion(g,0,turn)){
        if(Math.abs(p.z)>hob.parameterNumber("height")/2)continue;
        var q=F.transformPoint(actual,p.x,p.y,p.z),r=Math.sqrt(q.x*q.x+q.y*q.y);
        if(q.z>=0&&q.z<=machine.spec.faceWidth){minR=Math.min(minR,r);}
        if(r>=machine.spec.rootRadius()&&r<=machine.spec.tipRadius()&&q.z>=0&&q.z<=machine.spec.faceWidth){touches++;minZ=Math.min(minZ,q.z);maxZ=Math.max(maxZ,q.z);}
      }
      check(touches>0&&minR<machine.spec.tipRadius(),"generating rake edges reach toothed rim at inspection setup");
      var tilt=state.joint(FlexsplineHobber.A),phase=state.joint(FlexsplineHobber.C);
      var poses=0;
      for(x in [machine.travel[0].lower,(machine.travel[0].lower+machine.travel[0].upper)/2,machine.travel[0].upper])for(y in [-10.0,0,10])for(z in [-10.0,90,190])for(a in [-Math.PI/12,0.0,Math.PI/12])for(b in [0.0,0.73,Math.PI]){
        state.setJoint("X",x);state.setJoint("Y",y);state.setJoint("Z",z);state.setJoint(FlexsplineHobber.A,a);state.setJoint(FlexsplineHobber.B,b);state.forwardKinematics();mounts();poses++;
      }
      rows.push({reference:reference,poses:poses,toolSeatStartMm:seatStart,toolSeatEndMm:seatEnd,boreMm:hob.parameterNumber("bore"),nominalDiametralClearanceMm:0,spindleHousingAxialGapMm:nearGap,outboardHousingAxialGapMm:farGap,
        setupTiltRadians:tilt,tablePhaseRadians:phase,rakePointsOnToothedRim:touches,engagedBandMinZMm:minZ,engagedBandMaxZMm:maxZ,
        axialRetention:"OPEN: datum seating only; no physical stop, spacer or retainer modeled",manufacturingFits:"OPEN: no bore/shaft fit tolerances specified"});
    }
    var report={scope:"nominal datums, coaxiality and toothed-rim reach; retention and manufacturing fits remain open",assertions:assertions,configurations:rows};
    var output=Sys.getEnv("HOBBER_INSTALLATION_EVIDENCE");if(output!=null)sys.io.File.saveContent(output+"/installation.json",haxe.Json.stringify(report));
    Sys.println("HOBBER_INSTALLATION_PASS "+haxe.Json.stringify(report));
  }
}
