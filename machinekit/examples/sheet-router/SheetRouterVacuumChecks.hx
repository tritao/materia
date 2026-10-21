import stockkit.*;
import toolpathkit.tool.Tool;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.Point3;
import toolpathkit.path.Provenance;
import sys.io.File;
class SheetRouterVacuumChecks {
    static var assertions=0;
    static function check(ok:Bool,label:String):Void {assertions++;if(!ok)throw 'SR8: $label';}
    static function contour(stock:Stock,bottom:Float,withTab:Bool=false):Void {
        var tool=new Tool(3,0.06,0.006,null,0.04,0.06);
        var xy=[[0.047,0.047],[0.103,0.047],[0.103,0.103],[0.047,0.103],[0.047,0.047]];
        var moves:Array<CutMove> = [];
        for(i in 0...4) {
            if(withTab&&i==3) {
                // Retract over the omitted interval: a 12 mm full-height tab
                // remains after allowing for the 6 mm cutter diameter.
                moves.push(new CutMove(tool,Path(Line(new Point3(0.047,0.103,bottom),new Point3(0.047,0.084,bottom))),Cut,3,new Provenance(43,1,1)));
                moves.push(new CutMove(tool,Path(Line(new Point3(0.047,0.066,bottom),new Point3(0.047,0.047,bottom))),Cut,4,new Provenance(43,1,1)));
            } else moves.push(new CutMove(tool,Path(Line(new Point3(xy[i][0],xy[i][1],bottom),new Point3(xy[i+1][0],xy[i+1][1],bottom))),Cut,i,new Provenance(40+i,1,1)));
        }
        stock.cut(moves);
    }
    public static function main():Void {
        var reports:Array<Dynamic> = [];
        for(bed in [2600.0,3150.0]) {
            var vacuum=new SheetRouterVacuum(new SheetRouter(new SheetRouterSpec(bed)));
            check(Math.abs(vacuum.spoilThicknessM-0.025)<1e-12&&Math.abs(vacuum.spoilPermeabilityM2-1e-12)<1e-24,"vacuum inputs follow the CAD spoilboard recipe in SI");
            var stock=Stock.box(StockLattice.covering(0,0,0,0.4,0.4,0.018,0.001,false),0,0,0,0.4,0.4,0.018);stock.setThreads(1);
            var snapshot=stock.snapshot();
            try {
                var initial=vacuum.analyze(stock,SheetRouterCutting.forceEnvelopeN()),coverage=initial.coveredAreaByZoneM2.copy();
                var allOpen=vacuum.analyze(stock,SheetRouterCutting.forceEnvelopeN(),[for(i in 0...vacuum.zoneCount)true]);
                check(initial.point.zonePressurePa[0]>allOpen.point.zonePressurePa[0],"closing uncovered zones improves active pressure");
                check(initial.point.totalFlowM3PerHour<=300&&initial.point.blowerPressurePa<=25000,"operating point respects assumed blower curve");
                contour(stock,-0.001);var through=vacuum.analyze(stock,SheetRouterCutting.forceEnvelopeN(),null,coverage);
                check(through.components.length==2,"through-cut frees the part");
                var found=false;for(warning in through.warnings)if(warning.code=="freed_part_slipping"&&warning.line==43)found=true;
                check(found,"50 mm part slipping names final cut line 43");
                var small=through.components[0];for(part in through.components)if(part.supportAreaM2<small.supportAreaM2)small=part;
                check(Math.abs(small.supportAreaM2-0.0025)<1e-9&&!small.passes&&small.margin<1,"actual 50 mm support footprint fails 600 N requirement");
                check(through.throughKerfAreaM2>0&&through.equivalentThroughKerfLengthM>0,"open kerf contributes uncovered leakage");
                vacuum.friction=0.25;var lowFriction=vacuum.analyze(stock,SheetRouterCutting.forceEnvelopeN());
                check(Math.abs(lowFriction.components[0].lateralCapacityN-through.components[0].lateralCapacityN/2)<1e-7,"friction sensitivity scales holding force");vacuum.friction=0.5;
                vacuum.spoilPermeabilityM2=2e-12;var permeable=vacuum.analyze(stock,SheetRouterCutting.forceEnvelopeN());
                check(permeable.point.zonePressurePa[0]<through.point.zonePressurePa[0],"more permeable spoilboard lowers pressure");vacuum.spoilPermeabilityM2=1e-12;
                vacuum.spoilThicknessM=0.018;var worn=vacuum.analyze(stock,SheetRouterCutting.forceEnvelopeN(),null,coverage);
                check(worn.point.zonePressurePa[0]<through.point.zonePressurePa[0],"18 mm hydraulic sensitivity gives a more demanding pump screen than new 25 mm spoilboard");vacuum.spoilThicknessM=0.025;
                stock.restore(snapshot);contour(stock,0.0005);var onion=vacuum.analyze(stock,SheetRouterCutting.forceEnvelopeN(),null,coverage);
                check(onion.components.length==1,"0.5 mm onion skin keeps connected stock");
                var freed=false;for(warning in onion.warnings)if(warning.code=="freed_part_slipping")freed=true;
                check(!freed&&onion.retainedWebNeedsForceQualification,"onion skin avoids freed-part warning but needs web force qualification");
                check(onion.minimumSupportThicknessMm>0.49&&onion.minimumSupportThicknessMm<0.51,"remaining skin thickness is measured from stock");
                stock.restore(snapshot);contour(stock,-0.001,true);var tabbed=vacuum.analyze(stock,SheetRouterCutting.forceEnvelopeN(),null,coverage);
                check(tabbed.components.length==1,"full-height tab keeps native stock connected");
                check(tabbed.retainedWebNeedsForceQualification&&tabbed.minimumSupportThicknessMm>17.9,"full-height tabs still need strength qualification");
                reports.push({newSpoilThicknessMm:25,hydraulicSensitivityThicknessMm:18,wornSpoilboard:worn,tabbed:tabbed,workY:bed,initial:initial,allZonesOpen:allOpen,through:through,onion:onion,lowFriction:lowFriction,higherPermeability:permeable});
                stock.dispose();snapshot.dispose();
            }catch(error:Dynamic) {stock.dispose();snapshot.dispose();throw error;}
        }
        File.saveContent("build/sheet-router-work/sr13/vacuum.json",haxe.Json.stringify({status:"PASS",assertions:assertions,reports:reports},null,"  ")+"\n");
        Sys.println('SR8: $assertions assertions passed');
    }
}
