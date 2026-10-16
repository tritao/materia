import cadbridge.AssemblyPhysicalPartView;
import cadbridge.AssemblySimulationBridge;
import cadkit.modeling.AssemblyState;
import materia.project.SceneArtifact;
import robotkit.device.DeviceBinding;
import robotkit.model.SteadyLoads;
import robotkit.runtime.RobotRuntimeCompiler;
import toolpathkit.motion.PrintingMotionBinding;
import toolpathkit.motion.PrintingMotionBinding.PrintingMotionContext;
import toolpathkit.motion.ToolpathChannels;

/** Non-default scene placement and job values must reach the shared app/benchmark adapter. */
class PrintingJobBindingTests {
  static var assertions = 0;
  static function check(ok:Bool,message:String):Void { assertions++;if(!ok)throw message; }
  static function near(actual:Float,expected:Float,message:String):Void
    check(Math.abs(actual-expected)<1e-12,'$message: expected $expected, got $actual');
  public static function run():Void {
    assertions=0;
    var scene=SceneArtifact.decode(CoreXyPrinterPreview.printer());
    var job=scene.printing,definition=scene.assemblyDefinition;
    if(job==null || definition==null)throw "Printer fixture has no scene job";
    var physical=AssemblyPhysicalPartView.fromSceneArtifact(scene);
    var placement=new AssemblyState(definition,scene.assemblyState);
    placement.setJoint(job.axes[0],70);placement.setJoint(job.axes[1],90);placement.setJoint(job.axes[2],20);
    job.controller.stepTickHz=1000;
    job.rampLength=0.0013;job.pressureAdvance=0.031;job.pressureAdvanceWindow=0.06;
    job.filamentDiameter=0.00285;job.maxFlow=2.1e-8;
    var raw=AssemblySimulationBridge.toRobotModel(definition,physical,placement.record()).model;
    var model=DeviceBinding.planningModel(raw,job.controller.stepTickHz);
    var context:PrintingMotionContext={definition:definition,initialPositions:[for(id in job.axes)placement.joint(id)],
      metresPerUnit:physical.metresPerUnit,model:model,runtime:RobotRuntimeCompiler.compile(model,new robotkit.profile.RobotProfile()),
      fixedTimestepSeconds:0.01,channels:new ToolpathChannels(""),steady:new SteadyLoads()};
    var configured=new PrintingMotionBinding(job,context);
    var pose=configured.binding.solver.forward([for(_ in model.joints)0.0]);
    near(pose.x,0.07,"Adapter lost the scene's non-default X placement");
    near(pose.y,0.09,"Adapter lost the scene's non-default Y placement");
    near(pose.z,0.02,"Adapter lost the scene's non-default Z placement");
    // 40 mm belt travel / (200 full steps * 16 microsteps) at a 1 kHz step clock.
    // Deployed steps/radian are rounded once to binary32. Reciprocation gives
    // relative rate error <= u/(1-u), u=2^-24, plus double arithmetic roundoff.
    var wireUnitRoundoff=Math.pow(2.0,-24);
    check(Math.abs(configured.machine.rapidSpeed/0.0125-1)<=wireUnitRoundoff/(1-wireUnitRoundoff)+64*2.220446049250313e-16,
      "Rapid cap did not follow the controller, belt drives and wire calibration");
    check(configured.machine.frameId=="machine" && configured.machine.rapidsAtFeed,"Printer machine-coordinate travel policy was lost");
    near(configured.printer.rampLength,0.0013,"Adapter replaced the job's extrusion ramp");
    near(configured.printer.pressureAdvance,0.031,"Adapter replaced the job's advance");
    near(configured.printer.pressureAdvanceWindow,0.06,"Adapter replaced the job's smoothing window");
    near(configured.printer.filamentDiameter,0.00285,"Adapter replaced the job's filament");
    near(configured.printer.maxFlow,2.1e-8,"Adapter replaced the job's melt limit");
    check(configured.printer.shapers.exists(job.axes[0]) && configured.printer.shapers.exists(job.axes[1]),"Job shapers were not applied");
    var decoded=cnckit.CncCompiler.compileDetailed("G21\nG90\nM83\nG1 X10 F600\nG1 X20 E1 F600\n",configured.gcodeController());
    var expectedVolume=0.001*Math.PI*0.00285*0.00285/4;
    check(Math.abs(decoded.program.depositedVolume()/expectedVolume-1)<1e-12,
      "G-code reader did not use the job's filament cross-section");
    var variant=new PrintingMotionBinding(job,context,0.0,false);
    near(variant.printer.pressureAdvance,0,"Explicit no-advance comparison was ignored");
    check(!variant.printer.shapers.keys().hasNext(),"Explicit unshaped comparison was ignored");
    near(job.pressureAdvance,0.031,"Comparison variant mutated the scene job");
    Sys.println('Printing job binding checks passed ($assertions assertions)');
  }
}
