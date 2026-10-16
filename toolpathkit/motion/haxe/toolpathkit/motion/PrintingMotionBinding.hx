package toolpathkit.motion;

import materia.assembly.AssemblyDefinition;
import materia.project.SceneArtifact.SceneArtifactPrinting;
import motionkit.axis.MotionAxisBlueprint;
import motionkit.robot.MotionSystemBlueprint;
import motionkit.trajectory.InputShaper;
import robotkit.model.RobotModel;
import robotkit.model.SteadyLoads;
import robotkit.runtime.RobotRuntimeBlueprint;

/** Assembly placement and execution context; initial positions are in authored assembly units. */
typedef PrintingMotionContext = {
  var definition:AssemblyDefinition;
  var initialPositions:Array<Float>;
  var metresPerUnit:Float;
  var model:RobotModel;
  var runtime:RobotRuntimeBlueprint;
  var fixedTimestepSeconds:Float;
  var channels:ToolpathChannels;
  var steady:SteadyLoads;
}

/** One scene-job adapter for application execution and planning benchmarks.
 * Logical XYZ positions are machine metres; explicit joint offsets account for
 * the assembly's initial placement. Optional overrides describe benchmark variants.
 */
class PrintingMotionBinding {
  public final blueprint:MotionSystemBlueprint;
  public final machine:MachineBinding;
  public final printer:PrinterBinding;
  public final binding:ToolpathMotionBinding;
  public final initialPositions:Array<Float>;

  /** Fresh parser/writer state using the same filament as physical motion lowering. */
  public function gcodeController():cnckit.CncController
    return new cnckit.CncController(Printer,null,new cnckit.PrinterSettings(printer.filamentDiameter));

  public function new(job:SceneArtifactPrinting,context:PrintingMotionContext,
      ?pressureAdvanceOverride:Float,applyShaping:Bool=true) {
    if (job.axes.length!=3 || context.initialPositions.length!=3 ||
        !Math.isFinite(context.metresPerUnit) || context.metresPerUnit<=0)
      throw "Printing motion needs three placed axes and a positive assembly unit scale";
    initialPositions=[];
    var axes:Array<MotionAxisBlueprint> = [],rapid=0.0;
    for (index in 0...job.axes.length) {
      var id=job.axes[index];
      var found=[for (joint in context.definition.joints) if (joint.id==id) joint];
      if (found.length!=1 || Std.string(found[0].type)!="prismatic")
        throw 'Print axis "$id" must be a prismatic joint of the printer';
      var lower=found[0].limits.lower,upper=found[0].limits.upper;
      if (lower==null || upper==null) throw 'Print axis "$id" needs travel limits';
      var initial=context.initialPositions[index]*context.metresPerUnit;
      initialPositions.push(initial);
      var limits=context.model.coupledLimits(id,context.steady,true);
      var velocity=limits.requireVelocity(),acceleration=limits.requireAcceleration();
      if (!(velocity>0) || !(acceleration>0)) throw 'Print axis "$id" needs positive motor limits';
      rapid=Math.max(rapid,velocity);
      axes.push(new MotionAxisBlueprint(id,[id],lower*context.metresPerUnit,upper*context.metresPerUnit,
        velocity,acceleration,initial,[1.0],[-initial]));
    }
    blueprint=new MotionSystemBlueprint(context.model,context.runtime,axes,context.fixedTimestepSeconds);
    machine=new MachineBinding("machine",job.axes[0],job.axes[1],job.axes[2],rapid,context.channels,1e-4);
    machine.setRapidsAtFeed(true);
    printer=new PrinterBinding(job.extruder,job.filamentDiameter,job.gearRadius,job.maxFlow,job.rampLength,
      pressureAdvanceOverride==null ? job.pressureAdvance : pressureAdvanceOverride,job.pressureAdvanceWindow);
    var shaping=job.shaping;
    if (applyShaping && shaping!=null) {
      var soft=robotkit.model.DriveLoads.of(context.model,null,[for (shaper in shaping) shaper.axis]);
      var hard=robotkit.model.DriveLoads.stiffest(context.model,null,[for (shaper in shaping) shaper.axis]);
      for (shaper in shaping) {
        var shaped=InputShaper.named(shaper.kind,shaper.frequency,shaper.damping);
        for (load in soft) if (load.axis==shaper.axis && load.stiffness>0 && load.mass>0) {
          var upper=[for (candidate in hard) if (candidate.axis==load.axis) candidate][0];
          var lowerHz=Math.sqrt(load.stiffness/load.mass)/(2*Math.PI),upperHz=Math.sqrt(upper.stiffness/upper.mass)/(2*Math.PI);
          if (shaped.impulses.residualBound(lowerHz,upperHz,shaper.damping)>0.2)
            throw 'Fixed shaper for ${shaper.axis} does not cover physical travel: $lowerHz..$upperHz Hz';
        }
        printer.shapers.set(shaper.axis,shaped);
      }
    }
    machine.setPrinter(printer);
    binding=new ToolpathMotionBinding(machine,blueprint);
  }
}
