import machinekit.motion.NemaStepper;
import machinekit.component.ComponentValues;

class OpenBuildsMotorChecks {
    static function check(ok:Bool,message:String):Void {if(!ok)throw message;}
    static function main():Void {
        var motor=NemaStepper.model("OPENBUILDS-NEMA23-56"),part=motor.geometry();
        var raw=machinekit.motion.OpenBuildsNema23Geometry.build(56,false);
        check(part.shape.subshapeCount(CadKit.ShapeKind.Face)<raw.shape.subshapeCount(CadKit.ShapeKind.Face),"Refinement removes redundant construction faces");
        check(Math.abs(part.volume()-raw.volume())<0.00001,"Refinement preserves motor volume");
        trace("Motor faces before/after refinement: "+raw.shape.subshapeCount(CadKit.ShapeKind.Face)+"/"+part.shape.subshapeCount(CadKit.ShapeKind.Face));
        raw.close();
        trace("Native motor volume: "+part.volume());
        check(part.valid() && part.solidCount()==1,"Native motor is one valid fused solid");
        // Independent oracle: execute pinned bd_warehouse StepperMotor and fuse its body/shaft.
        check(Math.abs(part.volume()-160912.08568962364)<0.01,"Native motor matches upstream fused volume");
        var restored=NemaStepper.namedRecipeType().create(new ComponentValues().setToken("model","OPENBUILDS-NEMA23-56"));
        check(restored.geometryKey()==motor.geometryKey(),"Named recipe retains OpenBuilds identity");
        check(motor.rating()==null,"Do not inherit StepperOnline electrical ratings");
        check(NemaStepper.model("23HS22-2804S").variant.housingProfile!="openbuilds-nema23-v1","Do not relabel OpenBuilds construction as StepperOnline");
        part.close();trace("OpenBuilds motor geometry and recipe checks passed");
    }
}
