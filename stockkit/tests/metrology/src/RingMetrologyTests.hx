import gearkit.*;
import stockkit.*;
import stockkit.hobbing.*;

class RingMetrologyTests {
  static var assertions=0;
  static function check(ok:Bool,msg:String):Void {assertions++;if(!ok)throw msg;}
  public static function run():Void {
    var spec=new GearSpec(0.5,100,10,20*Math.PI/180),p=new ToothProfile(spec);
    var grid=new RingGrid(p.rootJoinRadius+0.00001,p.tipJoinRadius-0.00001,-0.025,10.025,
      (p.tipJoinRadius-p.rootJoinRadius)/181,0.05);
    var stock=RingGearTarget.create(grid,spec);
    var cfg=new GearInspection(spec,p.rootJoinRadius+0.02*(p.tipJoinRadius-p.rootJoinRadius));
    var report=GearMeasurement.fromRings(stock,spec,cfg);
    check(report.isoClass==1,'native ideal gear numerical class 1: ${report.warnings}');
    for(k in Iso1328.parameters)check(report.value(k)<1e-6,'native perfect $k');
    check(report.thickness.length==100&&report.profiles.length==200&&report.helices.length==200,"shared report has all traces");
    stock.dispose();
    Sys.println('RingMetrologyTests: $assertions assertions passed');
  }
  /** Actual full-profile/reference-face rake sweep; expensive acceptance mode.
      Source spacing resolves 150 profile samples and 5*b/cutoff axial samples.
      The standard excludes module 0.3, so this must report extrapolated grades. */
  static function reference():Void {
    var spec=new GearSpec(0.3,200,8,20*Math.PI/180),p=new ToothProfile(spec);
    var lo=p.rootJoinRadius+0.002,hi=p.tipJoinRadius-0.002;
    var grid=new RingGrid(lo,hi,-0.02,8.02,(hi-lo)/185,0.04);
    var stock=RingStock.annulus(grid);stock.setThreads(8);
    Sys.println("RING_METROLOGY_SETTINGS "+haxe.Json.stringify({threads:8,pairedDomains:Sys.getEnv("STOCKKIT_RAKE_BATCH_DOMAINS")=="1",maxRefinements:5,sweepToleranceMm:0.00005,radialStepMm:(hi-lo)/185,axialStepMm:0.04,scope:"fine full-profile grading; sweep convergence tolerance unchanged"}));
    var hob=new HobDesign(ConjugateRack.of(p),1,32,12);
    var job=new IdealHobbingJob(spec,hob,3000,1.1,0,4);
    var sweep=new HobbingSweep(hob,-28,28,0.00005),began=Sys.time();
    var cfg=new GearInspection(spec,lo+0.015*(hi-lo),null,4,4,hi-0.015*(hi-lo));
    var cross=InspectionRingStock.create(grid,cfg);
    if(cross==null){stock.dispose();sweep.dispose();throw "exact HB4 inspection sampling unavailable";}
    stock.dispose();
    var ends=new FaceEndpointStock(cross,cross.stocks,grid,spec.faceWidth);
    for(part in ends.stocks)part.setThreads(8);
    var output=Sys.getEnv("HB4_EVIDENCE");if(output==null)output="build/flexspline-hb4-accelerated";
    var comparisons:Array<Dynamic> =[];
    var convergence=sweep.cutMany(ends.stocks,job.axesAt,job.schedule(grid),5,0.001,function(event:Dynamic):Void {
      Sys.println("RING_SWEEP_PROGRESS "+haxe.Json.stringify(event));Sys.stdout().flush();
      if(event.phase=="comparison"){comparisons.push(event);
        sys.io.File.saveContent(output+"/convergence.json",haxe.Json.stringify(comparisons));}
    });cross.assertOverlap();
    var report=GearMeasurement.fromRings(ends,spec,cfg);
    check(report.warnings.indexOf("Face endpoints not sampled; end excess material unchecked")<0,"reference physically samples both face ends");
    sys.io.File.saveContent(output+"/result.json",haxe.Json.stringify({
      threads:8,pairedDomains:Sys.getEnv("STOCKKIT_RAKE_BATCH_DOMAINS")=="1",maxRefinements:5,sweepToleranceMm:0.00005,pathToleranceMm:0.00005/(4*Math.pow(4,convergence.refinement)),
      convergenceMm:convergence.maxArcChange,refinement:convergence.refinement,wallSeconds:Sys.time()-began,
      grades:[for(k in Iso1328.parameters){parameter:k,valueUm:report.value(k),grade:report.grades.get(k)}],
      isoClass:report.isoClass,warnings:report.warnings,extrapolation:report.extrapolation,
      sampling:"exact inspection circles plus physical face endpoints; complete HB4 stroke; unchanged fine grid and convergence tolerance"}));
    for(k in Iso1328.parameters){
      var grade=report.grades.get(k);Sys.println('$k: ${report.value(k)} um, extrapolated grade $grade');
      check(grade!=null&&grade<=3,'reference $k must reach extrapolated grade <=3');
    }
    check(report.overallGrade!=null&&report.overallGrade<=3,"reference overall extrapolated grade <=3");
    check(report.isoClass==null&&report.extrapolation.length>0,"module 0.3 cannot be ISO-certified");
    Sys.println('reference threads=8 sweep_tolerance_mm=0.00005 path_tolerance_mm=${0.00005/(4*Math.pow(4,convergence.refinement))} convergence_mm=${convergence.maxArcChange} time_s=${Sys.time()-began} warnings=${report.warnings}');
    sweep.dispose();for(part in ends.stocks)part.dispose();
  }
  public static function main():Void {
    if(Sys.args().length>0&&Sys.args()[0]=="boundary-probe")HB4BoundaryProbe.run();
    else if(Sys.args().length>0&&Sys.args()[0]=="convergence-probe")HB4ConvergenceProbe.run();
    else if(Sys.args().length>0&&Sys.args()[0]=="reference")reference();else run();
  }
}
