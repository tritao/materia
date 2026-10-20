import machinekit.assembly.MachineAssembly;
import machinekit.gantry.Gantry;
import machinekit.gantry.GantrySpec;
import machinekit.gantry.GantrySpec.GantryDrive;
import machinekit.component.*;
import machinekit.motion.*;
import machinekit.transmission.*;
import materia.assembly.AssemblyFrames as F;

/** SR2 baseline and focused shared-builder regression gate. */
class SheetRouterAxesTests {
    static var assertions=0;
    static function check(ok:Bool,message:String):Void {assertions++;if(!ok)throw message;}
    static function near(a:Float,b:Float,label:String):Void check(Math.isFinite(a)&&Math.abs(a-b)<1e-8,label+': '+a+' vs '+b);
    static function baseline():Void {
        var machines:Array<{name:String,machine:MachineAssembly}> = [
            {name:'desktop-router',machine:new CncRouter()},
            {name:'gantry-picker',machine:new GantryPicker()},
            {name:'gantry-welder',machine:new GantryWelder()},
            {name:'overhead-arm',machine:new OverheadArm()},
            {name:'corexy',machine:new CoreXyPlotter()}
        ];
        var expected=["0aaf594dfe25b262b80f7ffb5e517cf522c117fac85d847aee5ce694e8dca885",
            "adb50775b3201f71869ea8785ee202088270ec11a8d5c8e28e6c48499c98f987",
            "44aa511fc721eae19d8fea0276c06fb7e8d6a55c234aa83b75c49a1be150e74d",
            "acee2d2e103a25553df532e52288af8a036bddd41d3fae843ff6dce69893509c",
            "182a495da133b49be557accdc15f39f78a055388f2f3a0963428629bb36f7180"];
        var index=0;
        for(entry in machines) {
            var encoded=materia.assembly.AssemblyDefinitionCodec.encode(entry.machine.definition())+haxe.Json.stringify([for(member in entry.machine.components()) {id:member.id,designation:member.component.designation,key:member.component.geometryKey(Preview)}]);
            var digest=haxe.crypto.Sha256.encode(encoded);
            Sys.println('SR2 baseline '+entry.name+' '+digest);
            check(digest==expected[index++],entry.name+' baseline geometry and definition changed');
        }
    }
    public static function main():Void {
        if(Sys.args().indexOf("--legacy")>=0) {
            GantryChecks.run();CncRouterPreview.CncRouterChecks.run();
            Sys.println("SR2 existing gantry and router checks passed");return;
        }
        baseline();
        newAxes();
        Sys.println('Sheet router SR2 axes: '+assertions+' assertions passed');
    }
    static function newAxes():Void {assertions+=SheetRouterNewAxesChecks.run();}
}
