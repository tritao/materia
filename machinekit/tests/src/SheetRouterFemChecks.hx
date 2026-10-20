import machinekit.structural.FrameAssembly;
import machinekit.structural.RectTube;
import machinekit.structural.fem.AssemblyFemExport;
import cadkit.modeling.Vector;

class SheetRouterFemChecks {
    static var assertions=0;
    static function check(ok:Bool,label:String):Void {assertions++;if(!ok)throw "SR5: "+label;}
    static function checks():Void {
        var frame=new FrameAssembly();frame.point("a",0,0,0);frame.point("b",0,0,500);
        frame.member("post","a","b",new RectTube(100,80,4));
        var rows=frame.analysisMembers();
        check(rows.length==1&&rows[0].end.z==frame.length("post"),"analysis length follows authored stock");
        check(rows[0].reference.y==1,"vertical stock reference matches CAD orientation");
        rows[0].start=new Vector(9,9,9);rows.pop();
        check(frame.analysisMembers()[0].start.x==0,"analysis data does not mutate frame coordinates");
        var text="FILE_NAME('2026-10-20T12:34:56');\nDATA;\nPRODUCT('Product 7','Product 7','');\n";
        var normalized=AssemblyFemExport.stepText(text,"a'b/c");
        check(normalized.indexOf("1970-01-01T00:00:00")>=0,"STEP timestamps normalize");
        check(normalized.indexOf("a''b/c")>=0,"STEP product label quotes escape");
        check(AssemblyFemExport.stepText(normalized,"a'b/c")==normalized,"STEP normalization is idempotent");
        var rejected=false;try AssemblyFemExport.stepText("no data","part") catch(_:Dynamic) rejected=true;
        check(rejected,"truncated STEP rejected");
        MillFemChecks.main();
        Sys.println("Sheet router SR5: "+assertions+" exporter/centreline assertions passed");
    }
    public static function main():Void {
        if(Sys.args().indexOf("--export-only")>=0) {SheetRouterFemExport.main();return;}
        if(Sys.args().indexOf("--baseline")>=0) {SheetRouterAxesTests.main();return;}
        if(Sys.args().indexOf("--layout")>=0) {SheetRouterGuideChecks.main();return;}
        if(Sys.args().indexOf("--name")>=0) {MillFemExport.main();return;}
        if(Sys.args().indexOf("--y-loads")>=0) {SheetRouterFemYLoads.main();return;}
        checks();
        if(Sys.args().indexOf("--checks-only")<0)SheetRouterFemExport.main();
    }
}
