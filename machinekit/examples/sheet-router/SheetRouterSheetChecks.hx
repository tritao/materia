import camkit.*;
import SheetRouterVacuum.RouterVacuumReport;
import camkit.CamSheetOrder.CamSheetOperation;
import stockkit.*;
import toolpathkit.path.Point3;
import toolpathkit.tool.Tool;
import sys.io.File;

/** Both-bed cabinet retention check using generated CAM, native cuts and the SR8 component/pressure model. */
class SheetRouterSheetChecks {
  static var assertions=0;
  static function check(ok:Bool,label:String):Void {assertions++;if(!ok)throw label;}
  static function rectangle(x:Float,y:Float,w:Float,h:Float):CamContour return new CamContour([
    new Point3(x,y,0.018),new Point3(x+w,y,0.018),new Point3(x+w,y+h,0.018),new Point3(x,y+h,0.018)]);
  static function main():Void {
    var reports:Array<Dynamic> = [];
    for(bed in [2600.0,3150.0]) {
      var vacuum=new SheetRouterVacuum(new SheetRouter(new SheetRouterSpec(bed)));
      var length=bed/1000,lattice=StockLattice.covering(0,0,0,1.6,length,0.018,0.001,false);
      var stock=Stock.box(lattice,0,0,0,1.6,length,0.018);stock.setThreads(4);
      var tool=new Tool(3,0,0.006,null,0.04,0.06);
      var small=rectangle(1.53,0.05,0.05,0.05),coverage=vacuum.analyze(stock).coveredAreaByZoneM2;
      var original=stock.snapshot(),unsafeReport:Null<RouterVacuumReport> = null;
      var unsafe:CamSheetOperation={id:"unsafe-small",part:"small",area:0.0025,kind:"outside",requires:[],releases:true};
      var refused=false;
      try CamSheetOrder.order([unsafe],op->{
        var program=new CamJob(0.025,12000,new Point3(0,0,0.025)).sheetProfile(small,tool,-0.001,0.05,{stepDown:0.006}).finish();
        stock.cut(CutMoves.fromProgram(program));unsafeReport=vacuum.analyze(stock,SheetRouterCutting.forceEnvelopeN(),null,coverage);
        var accepted=true;for(warning in unsafeReport.warnings)if(warning.code=="freed_part_slipping")accepted=false;
        if(!accepted)stock.restore(original);
        return accepted;
      })catch(_:Dynamic)refused=true;
      check(refused&&unsafeReport!=null&&unsafeReport.components.length==2,"native full-load holding check refuses the freed 50 mm part");
      check(Math.abs(stock.volume()-1.6*length*0.018)<1e-8,"refused candidate is rolled back");
      var contours:Map<String,CamContour> = ["small"=>small],operations:Array<CamSheetOperation> = [];
      var rows=bed>2800?4:3,height=(length-0.15)/rows-0.05;
      operations.push({id:"small-hole",part:"small",area:0.0025,kind:"drill",requires:[],releases:false});
      operations.push({id:"small-profile",part:"small",area:0.0025,kind:"outside",requires:["small-hole"],releases:false});
      for(row in 0...rows)for(column in 0...2) {
        var id='panel-$row-$column';contours.set(id,rectangle(0.05+column*0.75,0.05+row*(height+0.05),0.70,height));
        operations.push({id:id+"-profile",part:id,area:0.7*height,kind:"outside",requires:[],releases:false});
      }
      var accepted:Array<Dynamic> = [],minimum=Math.POSITIVE_INFINITY;
      var ordered=CamSheetOrder.order(operations,op->{
        var checkpoint=stock.snapshot(),contour=contours.get(op.part);
        var job=new CamJob(0.025,12000,new Point3(0,0,0.025));
        if(op.kind=="drill")job.drill([new Point3(1.555,0.075,0.018)],tool,0.010,0.020,0.005,op.id,0.003);
        else job.sheetProfile(contour,tool,-0.001,0.05,{stepDown:0.006,autoTabs:4,tabLength:0.008,
          tabThickness:0.004,cornerClearance:0.003,onionSkin:0.0015,skinPass:true,spindleRpm:18000,featureRef:op.id});
        var before=stock.volume(),cut=stock.cut(CutMoves.fromProgram(job.finish())),report=vacuum.analyze(stock,SheetRouterCutting.forceEnvelopeN(),null,coverage);
        var ok=true;for(part in report.components)if(!part.passes)ok=false;
        if(!ok){stock.restore(checkpoint);checkpoint.dispose();return false;}
        for(part in report.components)minimum=Math.min(minimum,part.margin);
        accepted.push({operation:op.id,components:report.components.length,holdingMargin:report.components[0].margin,
          retainedWebNeedsForceQualification:report.retainedWebNeedsForceQualification,removedM3:before-stock.volume()});
        checkpoint.dispose();Sys.println('SR11 bed=$bed accepted ${op.id}, holding margin=${report.components[0].margin}');return true;
      });
      check(ordered.operations[0].id=="small-hole"&&ordered.operations[1].id=="small-profile","hole precedes retained small profile; small precedes large");
      check(accepted.length==operations.length&&minimum>=1,"all native cabinet checkpoints pass SR8's connected-component holding screen");
      var finalReport=vacuum.analyze(stock,SheetRouterCutting.forceEnvelopeN(),null,coverage);
      check(finalReport.components.length==1&&finalReport.retainedWebNeedsForceQualification,"tabs retain every part; web strength remains a commissioning/tooling gate");
      reports.push({workY:bed,spacingMm:1,forcePerPartN:SheetRouterCutting.forceEnvelopeN(),safetyFactor:2,unsafe:unsafeReport,
        sequence:accepted,minimumHoldingMargin:minimum,finalReport:finalReport,axes:"Z rays for component/holding check; tri-dexel geometry checked in core fixture"});
      original.dispose();stock.dispose();
    }
    var output={assertions:assertions,reports:reports};
    var args=Sys.args();if(args.length>0)File.saveContent(args[0],haxe.Json.stringify(output,null,"  ")+"\n");
    Sys.println('Router cabinet checks passed ($assertions assertions)');
  }
}
