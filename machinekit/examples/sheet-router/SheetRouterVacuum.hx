import stockkit.Stock;
import stockkit.StockComponents;
import stockkit.StockComponents.StockComponent;

typedef RouterVacuumPoint = {
    var headerPressurePa:Float;
    var blowerPressurePa:Float;
    var totalFlowM3PerHour:Float;
    var zonePressurePa:Array<Float>;
    var zoneFlowM3PerHour:Array<Float>;
    var open:Array<Bool>;
}
typedef RouterPartHold = {
    var component:Int;
    var supportAreaM2:Float;
    var supportByZoneM2:Array<Float>;
    var boundsMinM:Array<Float>;
    var boundsMaxM:Array<Float>;
    var normalForceN:Float;
    var lateralCapacityN:Float;
    var requiredLateralN:Float;
    var margin:Float;
    var passes:Bool;
    var independent:Bool;
}
typedef RouterVacuumReport = {
    var point:RouterVacuumPoint;
    var components:Array<RouterPartHold>;
    var coveredAreaByZoneM2:Array<Float>;
    var conductanceByZoneM3PerSPa:Array<Float>;
    var throughKerfAreaM2:Float;
    var equivalentThroughKerfLengthM:Float;
    var minimumSupportThicknessMm:Float;
    var retainedWebNeedsForceQualification:Bool;
    var programLine:Int;
    var warnings:Array<{line:Int,component:Int,code:String}>;
}

/** SR8 low-vacuum, steady-state planning screen. All stock coordinates are SI
 * workpiece coordinates. Pump curve, Darcy permeability, friction and losses
 * are declared assumptions to replace with measured data before acceptance.
 */
class SheetRouterVacuum {
    public final widthM:Float;
    public final lengthM:Float;
    public final columns:Int;
    public final rows:Int;
    public final zoneCount:Int;
    public final zoneAreaM2:Float;
    public final freeFlowM3PerS:Float;
    public final shutoffPa:Float;
    public var spoilThicknessM:Float=0.025;
    public var spoilPermeabilityM2:Float=1e-12;
    /** Zero denotes an impermeable sheet. */
    public var sheetPermeabilityM2:Float=1e-12;
    public var friction:Float=0.5;
    public var effectiveAreaFactor:Float=0.8;
    public var safetyFactor:Float=2;
    public var viscosityPaS:Float=1.8e-5;
    public var airDensityKgM3:Float=1.2;
    public var hoseFriction:Float=0.03;
    public var trunkIdM:Float=0.075;
    public var trunkLengthM:Float=4;
    public var branchIdM:Float=0.05;
    public var branchLengthM:Float=3;
    public var trunkFittingK:Float=6;
    public var branchFittingK:Float=4;
    public var filterLossAtFreeFlowPa:Float=2000;
    public function new(machine:SheetRouter) {
        spoilThicknessM=machine.component("spoilboard").parameterNumber("thickness")/1000;
        spoilPermeabilityM2=machine.component("spoilboard").parameterNumber("permeability")/1000000;
        widthM=machine.routerSpec.workX/1000;lengthM=machine.routerSpec.workY/1000;
        columns=machine.table.parameterCount("columns");rows=machine.table.parameterCount("rows");zoneCount=columns*rows;
        zoneAreaM2=widthM*lengthM/zoneCount;
        freeFlowM3PerS=machine.component("vacuumBlower").parameterNumber("freeFlow")/3600;
        shutoffPa=machine.component("vacuumBlower").parameterNumber("shutoffPressure");
    }
    public function zone(x:Float,y:Float):Int {
        if(x<0||x>=widthM||y<0||y>=lengthM)throw "Supported stock extends outside the vacuum bed";
        return Std.int(Math.floor(x/(widthM/columns)))+columns*Std.int(Math.floor(y/(lengthM/rows)));
    }
    function lossCoefficient(diameter:Float,length:Float,fittings:Float):Float {
        var area=Math.PI*diameter*diameter/4;
        return airDensityKgM3/2*(fittings+hoseFriction*length/diameter)/(area*area);
    }
    public function operatingPoint(open:Array<Bool>,conductance:Array<Float>):RouterVacuumPoint {
        if(open.length!=zoneCount||conductance.length!=zoneCount)throw "Vacuum zone count mismatch";
        for(value in conductance)if(!Math.isFinite(value)||value<0)throw "Invalid zone conductance";
        var branch=lossCoefficient(branchIdM,branchLengthM,branchFittingK);
        var trunk=lossCoefficient(trunkIdM,trunkLengthM,trunkFittingK)+filterLossAtFreeFlowPa/(freeFlowM3PerS*freeFlowM3PerS);
        function at(header:Float):RouterVacuumPoint {
            var pressure:Array<Float> = [],flows:Array<Float> = [],total=0.0;
            for(i in 0...zoneCount) {
                var p=0.0,q=0.0;
                if(open[i]) {
                    var alpha=branch*conductance[i]*conductance[i];
                    p=2*header/(1+Math.sqrt(1+4*alpha*header));q=conductance[i]*p;
                }
                pressure.push(p);flows.push(q*3600);total+=q;
            }
            return {headerPressurePa:header,blowerPressurePa:header+trunk*total*total,totalFlowM3PerHour:total*3600,
                zonePressurePa:pressure,zoneFlowM3PerHour:flows,open:open.copy()};
        }
        var low=0.0,high=shutoffPa;
        for(i in 0...80) {
            var mid=(low+high)/2,p=at(mid);
            var supplied=freeFlowM3PerS*Math.max(0,1-p.blowerPressurePa/shutoffPa);
            if(supplied>p.totalFlowM3PerHour/3600)low=mid;else high=mid;
        }
        return at((low+high)/2);
    }
    public function analyze(stock:Stock,forcePerPartN:Float=300,openMask:Null<Array<Bool>> = null,initialCoverage:Null<Array<Float>> = null):RouterVacuumReport {
        for(value in [spoilThicknessM,spoilPermeabilityM2,viscosityPaS,airDensityKgM3,trunkIdM,branchIdM,freeFlowM3PerS,shutoffPa,forcePerPartN])
            if(!Math.isFinite(value)||value<=0)throw "Vacuum assumptions require finite positive dimensions/ratings";
        for(value in [sheetPermeabilityM2,friction,hoseFriction,trunkLengthM,branchLengthM,trunkFittingK,branchFittingK,filterLossAtFreeFlowPa])
            if(!Math.isFinite(value)||value<0)throw "Vacuum assumptions cannot be negative or nonfinite";
        if(!Math.isFinite(effectiveAreaFactor)||effectiveAreaFactor<0||effectiveAreaFactor>1||!Math.isFinite(safetyFactor)||safetyFactor<1)throw "Invalid hold-down factors";
        var coveredFlow:Array<Float> = [for(i in 0...zoneCount)0.0],minimumThickness=Math.POSITIVE_INFINITY;
        var boardResistance=viscosityPaS*spoilThicknessM/spoilPermeabilityM2,cellArea=stock.grid.spacing*stock.grid.spacing;
        var components=StockComponents.fromRows(stock.grid,row->{
            var rays=stock.rays(0,row,stock.grid.countU,1);
            for(i in 0...rays.length)for(interval in rays[i])if(interval.lo<=1e-12&&interval.hi>1e-12) {
                var r=zone(stock.grid.u(i),stock.grid.v(row)),height=interval.hi;
                minimumThickness=Math.min(minimumThickness,height);
                var resistance=sheetPermeabilityM2==0?Math.POSITIVE_INFINITY:viscosityPaS*height/sheetPermeabilityM2;
                coveredFlow[r]+=cellArea/(boardResistance+resistance);
            }
            return rays;
        },0,1e-12,zoneCount,(x,y)->zone(x,y));
        var covered:Array<Float> = [for(i in 0...zoneCount)0.0];
        for(component in components)for(i in 0...zoneCount)covered[i]+=component.supportByRegion[i];
        var conductance:Array<Float> = [],automatic:Array<Bool> = [];
        for(i in 0...zoneCount) {
            var scale=covered[i]>zoneAreaM2?zoneAreaM2/covered[i]:1.0;
            conductance.push(Math.max(0,zoneAreaM2-covered[i])/boardResistance+coveredFlow[i]*scale);
            automatic.push(covered[i]>1e-12);
        }
        var open=openMask==null?automatic:(cast openMask:Array<Bool>).copy(),point=operatingPoint(open,conductance);
        var line=stock.history.length==0?0:stock.history[stock.history.length-1].provenance.line;
        var holds:Array<RouterPartHold> = [],warnings:Array<{line:Int,component:Int,code:String}> = [];
        for(index in 0...components.length) {
            var component=components[index],normal=0.0;
            for(i in 0...zoneCount)normal+=component.supportByRegion[i]*point.zonePressurePa[i]*effectiveAreaFactor;
            var capacity=friction*normal,required=safetyFactor*forcePerPartN;
            var resolved=Math.min(component.max[0]-component.min[0],component.max[1]-component.min[1])>=10*stock.grid.spacing;
            var passes=capacity>=required&&resolved;
            holds.push({component:index,supportAreaM2:component.supportArea,supportByZoneM2:component.supportByRegion,
                boundsMinM:component.min,boundsMaxM:component.max,normalForceN:normal,lateralCapacityN:capacity,
                requiredLateralN:required,margin:capacity/required,passes:passes,independent:components.length>1});
            if(!passes)warnings.push({line:line,component:index,code:!resolved?"stock_resolution_insufficient":components.length>1?"freed_part_slipping":"stock_hold_insufficient"});
        }
        var kerfArea=0.0;
        if(initialCoverage!=null) {
            var original:Array<Float> = cast initialCoverage;if(original.length!=zoneCount)throw "Initial coverage zone count mismatch";
            for(i in 0...zoneCount)kerfArea+=Math.max(0,original[i]-covered[i]);
        }
        var kerfWidth=stock.history.length==0?0:stock.history[stock.history.length-1].tool.diameter;
        var thin=false;
        if(components.length==1) {
            var c=components[0],rectangle=(c.max[0]-c.min[0])*(c.max[1]-c.min[1]);
            // Connectivity does not establish tab/skin strength. Conservatively
            // flag connected stock with a thin skin or missing support footprint.
            thin=minimumThickness+1e-9<c.max[2]-c.min[2]||rectangle-c.supportArea>cellArea/2;
        }
        return {point:point,components:holds,coveredAreaByZoneM2:covered,conductanceByZoneM3PerSPa:conductance,
            throughKerfAreaM2:kerfArea,equivalentThroughKerfLengthM:kerfWidth>0?kerfArea/kerfWidth:0,
            minimumSupportThicknessMm:Math.isFinite(minimumThickness)?minimumThickness*1000:0,
            retainedWebNeedsForceQualification:thin,programLine:line,warnings:warnings};
    }
}
