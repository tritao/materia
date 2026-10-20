import sys.io.Process;

/** Independent seven-phase estimate checked against native Ruckig samples.
 * Straight moves use a scalar path whose largest axis displacement is length.
 */
class SheetRouterAtcTiming {
    public final start:Array<Float>;
    public final target:Array<Float>;
    public final durationS:Float;
    public final estimatedS:Float;
    public final pathLengthMm:Float;
    public final pathSpeedMmPerS:Float;
    final rows:Array<Array<Float>> = [];

    public static function estimate(distanceMm:Float,speedMmPerS:Float):Float {
        if(distanceMm<=1e-9)return 0;
        var acceleration=3000.0,jerk=20000.0,tj=acceleration/jerk,ta=0.0;
        if(speedMmPerS<acceleration*tj)tj=Math.sqrt(speedMmPerS/jerk);
        else ta=speedMmPerS/acceleration-tj;
        var accelerationTime=2*tj+ta,critical=speedMmPerS*accelerationTime;
        if(distanceMm>=critical)return 2*accelerationTime+(distanceMm-critical)/speedMmPerS;
        tj=acceleration/jerk;
        if(distanceMm<2*jerk*tj*tj*tj) {tj=Math.pow(distanceMm/(2*jerk),1.0/3);ta=0;}
        else ta=(Math.sqrt(tj*tj+4*distanceMm/acceleration)-3*tj)/2;
        return 4*tj+2*ta;
    }
    public function new(start:Array<Float>,target:Array<Float>,feedMmPerS:Float,sampler:String,initialSpeedMmPerS:Float=0) {
        this.start=start.copy();this.target=target.copy();
        var length=0.0;for(i in 0...3)length=Math.max(length,Math.abs(target[i]-start[i]));pathLengthMm=length;
        var speed=feedMmPerS;
        for(i in 0...3)if(Math.abs(target[i]-start[i])>1e-9)speed=Math.min(speed,(i==2?10000/60:40000/60)*length/Math.abs(target[i]-start[i]));
        pathSpeedMmPerS=speed;
        if(initialSpeedMmPerS==0)estimatedS=estimate(length,speed);
        else {
            var stoppingDistance=initialSpeedMmPerS*Math.sqrt(initialSpeedMmPerS/20000);
            if(Math.abs(length-stoppingDistance)>1e-7)throw "Only the minimum-distance probe stop accepts nonzero initial speed";
            estimatedS=2*Math.sqrt(initialSpeedMmPerS/20000);
        }
        if(length<1e-9) {durationS=0;return;}
        // The retained sampler has its own native-library RUNPATH. The host's
        // HashLink provider path must not override it with another ABI.
        var process=Process.run("/usr/bin/env",["-u","LD_LIBRARY_PATH",sampler,"ruckig",Std.string(length/1000),Std.string(speed/1000),Std.string(initialSpeedMmPerS/1000),"0","0.005"]);
        var output=process.readStdout(),errors=process.readStderr(),code=process.exitCode();process.close();
        if(code!=0)throw 'Native ATC planner failed: $errors';
        var duration=Math.NaN;
        for(line in output.split("\n")) {
            line=StringTools.trim(line);
            if(StringTools.startsWith(line,"# duration "))duration=Std.parseFloat(line.substr(11));
            else {
                var values=[for(value in line.split(","))Std.parseFloat(value)];
                if(values.length<5||!Math.isFinite(values[0]))continue;
                if(Math.abs(values[2])>speed/1000+1e-6||Math.abs(values[3])>3.00001||Math.abs(values[4])>20.00001)throw "ATC native planner exceeds configured limits";
                rows.push(values);
            }
        }
        if(!Math.isFinite(duration)||rows.length<2||Math.abs(duration-estimatedS)>1e-6)throw 'ATC estimate/native timing mismatch: $estimatedS / $duration';
        durationS=duration;
    }
    public function at(timeS:Float):Array<Float> {
        if(durationS==0||timeS>=durationS)return target.copy();
        if(timeS<=0)return start.copy();
        var low=0,high=rows.length-1;
        while(high-low>1) {var mid=Std.int((low+high)/2);if(rows[mid][0]<=timeS)low=mid;else high=mid;}
        var a=rows[low],b=rows[high],fraction=(timeS-a[0])/(b[0]-a[0]);
        var position=(a[1]+fraction*(b[1]-a[1]))*1000/pathLengthMm;
        return [for(i in 0...3)start[i]+position*(target[i]-start[i])];
    }
}
