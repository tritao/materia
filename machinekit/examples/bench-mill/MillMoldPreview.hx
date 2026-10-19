import haxe.Json;
import materia.tooling.casting.CastingMoldPlanner;
import materia.tooling.casting.CastingMoldCodec;
class MillMoldPreview {
    public static function main():Void {
        var frame=MillMoldSetup.frame(),input=MillMoldSetup.input(frame);
        var cwd=Sys.getCwd(),root=sys.FileSystem.exists(cwd+"/machinekit")?cwd+"/machinekit/examples/bench-mill":cwd;
        var dir=root+"/build/tooling/base";sys.FileSystem.createDirectory(root+"/build/tooling");sys.FileSystem.createDirectory(dir);
        sys.io.File.saveContent(dir+"/source-input.json",Json.stringify(input,null,"\t")+"\n");
        var plan=CastingMoldPlanner.plan(MillMoldSetup.document(frame),MillMoldSetup.options(frame));
        var review=machinekit.tooling.CastingMoldCad.review(plan);
        sys.io.File.saveContent(dir+"/review.json",Json.stringify(review,null,"\t")+"\n");
        if(Sys.args().indexOf("--checks")>=0)MillMoldChecks.run(frame,plan,review);
        machinekit.tooling.CastingMoldExport.write(plan,dir,review);
        var encoded=CastingMoldCodec.encode(plan);CastingMoldCodec.decode(encoded);
        sys.io.File.saveContent(dir+"/mold-plan.json",encoded);
        Sys.println("Base mold plan: "+plan.cores.length+" cores, "+plan.formers.length+" formers, "+plan.fixtures.length+" datum fixtures");
    }
}
