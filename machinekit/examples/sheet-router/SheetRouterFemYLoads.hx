import machinekit.sizing.LinearDriveSizing;
import machinekit.sizing.LinearDriveSizing.LinearDriveInput;
import machinekit.sizing.RackDriveSizing;
import machinekit.transmission.HelicalPinion;
import machinekit.transmission.PreloadedPinionMount;
import machinekit.transmission.HelicalRack;
import machinekit.motion.PlanetaryReducer;
import machinekit.motion.ServoMotor;

/** Feed eccentric FE branch reactions back through the same SR4 drive/tooth screen. */
class SheetRouterFemYLoads {
    public static function main():Void {
        var folder=Sys.getEnv("SHEET_ROUTER_FEM_OUTPUT");if(folder==null)folder="build/sheet-router-sr6";
        var loads:Array<Dynamic> = haxe.Json.parse(sys.io.File.getContent(folder+"/y-loads.json"));
        var output:Array<Dynamic> = [];
        for(length in [2600.0,3150.0]) {
            var worst:Dynamic=null;
            for(row in loads)if(row.workY==length&&(worst==null||row.reservedPeakForceN>worst.reservedPeakForceN))worst=row;
            if(worst==null)throw "Missing FE Y branch reactions";
            var machine=new SheetRouter(new SheetRouterSpec(length));
            var baseline=SheetRouterSizing.report(machine,"y");
            var input:LinearDriveInput=cast Reflect.copy(baseline.input);
            input.massKg=cast worst.effectiveMassKg;input.motors=1;
            input.cuttingForceN=cast worst.cuttingForceN;input.dragForceN=15;
            var motor:ServoMotor=cast machine.component("motorYLeft");
            var demand=LinearDriveSizing.demand(input,motor.rating);
            var pinion:HelicalPinion=cast machine.component("pinionYLeft");
            var preload:PreloadedPinionMount=cast machine.component("pinionYLeftPreload");
            var tooth=RackDriveSizing.strength(pinion,demand.peakForcePerMotorN*input.reserve,1.5,preload.preload());
            var failures=demand.failures.copy();
            if(!preload.resists(tooth.separatingN))failures.push("preload: eccentric separating force at reserved peak");
            var reducer:PlanetaryReducer=cast machine.component("pinionYLeftReducer");
            var radius=pinion.pitchDiameter()/2000;
            var rack:HelicalRack=cast machine.component("rackYLeft");
            var rotary:Float=cast baseline.rotatingOutputInertiaKgM2;
            var outputPeak=demand.peakForcePerMotorN*radius/rack.n("efficiency")+rotary*input.acceleration/radius;
            var outputRated=(input.cuttingForceN+input.dragForceN)*radius/rack.n("efficiency");
            var radial=tooth.radialBearingN+preload.preload()*input.reserve;
            if(outputPeak*input.reserve>reducer.n("peakOutputTorque"))failures.push("reducer: eccentric peak output torque");
            if(outputRated*input.reserve>reducer.n("ratedOutputTorque"))failures.push("reducer: eccentric rated output torque");
            if(tooth.axialN>reducer.n("axialLoad"))failures.push("reducer: eccentric axial load");
            if(radial>reducer.n("radialLoad"))failures.push("reducer: eccentric radial load including preload");
            if(rack.n("faceWidth")<pinion.n("faceWidth"))throw "Tooth screen requires full engaged face width";
            var catalogue=SheetRouterRackRating.check(pinion,rack,demand.peakForcePerMotorN*input.reserve,tooth.flankNormalN);
            if(!(cast catalogue.passes:Bool))failures.push("rack: manufacturer feed-force rating including preload and mounting derating");
            output.push({workY:length,reducer:{peakOutputNm:outputPeak,ratedOutputNm:outputRated,radialN:radial,axialN:tooth.axialN},
                catalogueRack:catalogue,source:worst,input:input,demand:demand,tooth:tooth,preloadN:preload.preload(),failures:failures});
            Sys.println("SR5 eccentric Y="+length+" reserved force="+demand.peakForcePerMotorN*input.reserve+" N; contact="+tooth.contactMPa+" MPa; failures="+haxe.Json.stringify(failures));
        }
        sys.io.File.saveContent(folder+"/eccentric-drives.json",haxe.Json.stringify({basis:"FE 300 N vector norm and 3 m/s2 Y inertia; 15 N drag per side; SR4 1.5 reserve and tooth load factor",variants:output},null,"  ")+"\n");
    }
}
