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
    var table=machine.table,arbor=machine.spindle.arbor,tool=machine.hob;
    var outboard:machinekit.hobbing.OutboardSupport=cast machine.components().filter(m->m.id=="outboard")[0].component;
    var supportRadial=HobbingErrorSource.hobRadial("outboard",outboard.parameterNumber("radialRunout"));
    var encoderRadius=(table.encoder.parameterNumber("outside")+table.encoder.parameterNumber("bore"))/4;
    var radial=HobbingErrorSource.hobRadial("spindle/arbor",arbor.parameterNumber("radialRunout"));
    var toolRadial=HobbingErrorSource.hobRadial("hob",tool.parameterNumber("radialRunout"));
    var axial=HobbingErrorSource.hobAxial("spindle/arbor",arbor.parameterNumber("axialRunout"));
    var toolAxial=HobbingErrorSource.hobAxial("hob",tool.parameterNumber("axialRunout"));
    if(Math.abs(tool.design.rake)>1e-12&&tool.parameterNumber("profileError")>0)throw "assumed profile waveform supports zero rake only";
    var sources=[HobbingErrorSource.workEccentricity("mandrel","mandrel",machine.mandrel.eccentricity),
      HobbingErrorSource.workEccentricity("table-radial","table/bearing",table.bearing.radialRunout/2),
      HobbingErrorSource.tableAxial("table/bearing",table.bearing.axialRunout),
      HobbingErrorSource.encoder("table/encoder",table.encoder.eccentricity,encoderRadius),
      HobbingErrorSource.graduation("table/encoder",table.encoder.angularAccuracy,4),
      radial,axial,
      new HobbingErrorSource("tool-radial",toolRadial.part,toolRadial.basis,toolRadial.apply),
      new HobbingErrorSource("tool-axial",toolAxial.part,toolAxial.basis,toolAxial.apply),
      HobbingErrorSource.profile("hob",tool.parameterNumber("profileError"),tool.design.axialPitch),sync,
      new HobbingErrorSource("outboard-radial","outboard",supportRadial.basis+"; parallel-axis approximation, no shaft bending",supportRadial.apply),
      wall.source("blank + mandrel contact",radialForce)];
    var members=machine.components();
    return [for(source in sources){var basis=source.basis;
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
