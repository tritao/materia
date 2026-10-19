import stockkit.hobbing.HobbingErrorSource;
import stockkit.hobbing.RingCompliance;

/** HB11 adapter: amplitudes read from installed parts. Runout fields interpreted
    as TIR, eccentricity fields as offset amplitude. Harmonic shapes/phases assumed.
    Runtime sync and supported blank compliance must be supplied explicitly. */
class FlexsplineHobberErrors {
  public static function sources(machine:FlexsplineHobber,radialForce:Float->Float,
      wall:RingCompliance,sync:HobbingErrorSource):Array<HobbingErrorSource> {
    if(sync.id!="sync")throw "hobber budget needs an explicitly identified runtime/diagnostic sync source";
    if(Math.abs(wall.radius-(machine.spec.rootRadius()-wall.wall/2))>1e-8||Math.abs(wall.width-machine.spec.faceWidth)>1e-8)
      throw "wall compliance does not match blank geometry";
    var table=machine.tableSpec,arbor=machine.spindle.arbor,tool=machine.hob;
    var pivotDepth=machine.mandrel.parameterNumber("workSeat")+table.number("pivotBelowFace");
    var tablePart="table/"+table.errorPart,encoderPart="table/"+table.encoderPart;
    var outboard:machinekit.hobbing.OutboardSupport=cast machine.components().filter(m->m.id=="outboard")[0].component;
    var supportRadial=HobbingErrorSource.hobRadial("outboard",outboard.parameterNumber("radialRunout"));
    var radial=HobbingErrorSource.hobRadial("spindle/arbor",arbor.parameterNumber("radialRunout"));
    var toolRadial=HobbingErrorSource.hobRadial("hob",tool.parameterNumber("radialRunout"));
    var axial=HobbingErrorSource.hobAxial("spindle/arbor",arbor.parameterNumber("axialRunout"));
    var toolAxial=HobbingErrorSource.hobAxial("hob",tool.parameterNumber("axialRunout"));
    if(Math.abs(tool.design.rake)>1e-12&&tool.parameterNumber("profileError")>0)throw "assumed profile waveform supports zero rake only";
    var sources=[HobbingErrorSource.workEccentricity("mandrel","mandrel",machine.mandrel.eccentricity),
      HobbingErrorSource.workEccentricity("table-radial",tablePart,Math.max(0,table.number("radialRunout")/2-table.number("radialNrro"))),
      HobbingErrorSource.tableAxial(tablePart,Math.max(0,table.number("axialRunout")-2*table.number("axialNrro"))),
      HobbingErrorSource.tableTilt(tablePart,table.number("momentStiffness"),pivotDepth,pivotDepth+machine.spec.faceWidth/2,radialForce),
      HobbingErrorSource.encoder(encoderPart,0,50), // net positioning data already includes installed encoder eccentricity

      HobbingErrorSource.graduation(encoderPart,table.number("absoluteAccuracy"),4),
      radial,axial,
      new HobbingErrorSource("tool-radial",toolRadial.part,toolRadial.basis,toolRadial.apply),
      new HobbingErrorSource("tool-axial",toolAxial.part,toolAxial.basis,toolAxial.apply),
      HobbingErrorSource.profile("hob",tool.parameterNumber("profileError"),tool.design.axialPitch),sync,
      new HobbingErrorSource("outboard-radial","outboard",supportRadial.basis+"; parallel-axis approximation, no shaft bending",supportRadial.apply),
      wall.source("blank + mandrel contact",radialForce)];
    // Additional purchased/built table disturbances retain their individual provenance.
    function angular(id:String,field:String,order:Float,amplitude:Float):Void {
      sources.push(new HobbingErrorSource(id,tablePart,field+"="+table.number(field)+" "+table.basis(field)+"; "+table.reason(field)+"; assumed order "+order,(t,a)->
        new stockkit.hobbing.HobbingAxes(a.x,a.y,a.z,a.a,a.b,a.c+amplitude*Math.sin(order*a.c),a.offset)));
    }
    angular("table-interpolation","interpolationError",4096,table.number("interpolationError"));
    angular("table-speed-ripple","speedRipple",1,table.number("speedRipple")); // integrate fractional sinusoidal speed ripple per C radian
    angular("table-cogging","coggingTorque",48,table.number("coggingTorque")/table.number("servoTorsionalStiffness"));
    sources.push(new HobbingErrorSource("table-seal-drag",tablePart,
      "Assumed closed-loop compliance: mean drag "+table.number("sealDrag")+" N m + twice-turn ripple "+table.number("sealDragRipple")+" N m, divided by "+table.number("servoTorsionalStiffness")+" N m/rad; "+table.reason("sealDrag"),
      (t,a)->new stockkit.hobbing.HobbingAxes(a.x,a.y,a.z,a.a,a.b,a.c-(table.number("sealDrag")+table.number("sealDragRipple")*Math.cos(2*a.c))/table.number("servoTorsionalStiffness"),a.offset)));
    sources.push(new HobbingErrorSource("table-nrro-radial",tablePart,"Assumed non-repeating quasiperiodic bounded radial surrogate; periodic share reduced to retain total TIR; "+table.reason("radialNrro"),(t,a)->{
      var offset=a.offset==null?new stockkit.hobbing.HobbingPoseOffset():a.offset.copy();
      offset.workX+=table.number("radialNrro")*Math.cos(a.c*1.41421356237);offset.workY+=table.number("radialNrro")*Math.sin(a.c*1.41421356237);
      return new stockkit.hobbing.HobbingAxes(a.x,a.y,a.z,a.a,a.b,a.c,offset);}));
    sources.push(new HobbingErrorSource("table-nrro-axial",tablePart,"Assumed non-repeatable bounded axial sequence; "+table.reason("axialNrro"),(t,a)->{
      var offset=a.offset==null?new stockkit.hobbing.HobbingPoseOffset():a.offset.copy();
      offset.workZ+=table.number("axialNrro")*Math.sin(a.c*1.41421356237);return new stockkit.hobbing.HobbingAxes(a.x,a.y,a.z,a.a,a.b,a.c,offset);}));
    sources.push(new HobbingErrorSource("table-radial-compliance",tablePart,"Assumed Fr/radialStiffness; "+table.reason("radialStiffness"),(t,a)->{
      var offset=a.offset==null?new stockkit.hobbing.HobbingPoseOffset():a.offset.copy(),d=radialForce(t)/table.number("radialStiffness");
      offset.workX+=d*Math.cos(a.c);offset.workY-=d*Math.sin(a.c);return new stockkit.hobbing.HobbingAxes(a.x,a.y,a.z,a.a,a.b,a.c,offset);}));
    var members=machine.components();
    return [for(source in sources){var basis=source.basis;
      if(StringTools.startsWith(source.id,"table-")||StringTools.startsWith(source.part,"table/"))basis+="; installed rotary-axis field provenance: "+haxe.Json.stringify(table.report());
      for(member in members)if(member.id==source.part){
        var part=machinekit.component.EngineeringMetadata.of(member.component);
        if(part==null)continue;
        basis+='; part errors basis ${part.errorsBasis}; source ${part.source}';
      }
      new HobbingErrorSource(source.id,source.part,basis,source.apply,source.warp);
    }];
  }
  /** at() is the baseline motion. Provide measured C-minus-baseline sync once,
      and an explicitly stated HB10 radial-force history / support model. */
  public static function budget(machine:FlexsplineHobber,at:Float->stockkit.hobbing.HobbingAxes,
      passes:Array<stockkit.hobbing.GashPass>,radialForce:Float->Float,wall:RingCompliance,sync:HobbingErrorSource,
      radialStep:Float=0.005,axialStep:Float=0.1,tolerance:Float=0.000025,
      ?inspection:gearkit.GearInspection,?motionKnots:Array<Float>,threads:Int=8):gearkit.ErrorBudget {
    var span=Std.int(Math.ceil(machine.hob.parameterNumber("height")/(2*machine.hob.design.axialPitch)))+2;
    return new stockkit.hobbing.HobbingErrorJob(machine.spec,machine.hob.design,at,passes,
      sources(machine,radialForce,wall,sync),wall.wall,radialStep,axialStep,-span,span,tolerance,inspection,motionKnots,threads).budget();
  }
}
