import sys.FileSystem;
import sys.io.File;

/** Initial SR6 shop packet, authored stock/joints plus geometry-derived seam candidates. */
class SheetRouterFabrication {
    static function point(v:cadkit.modeling.Vector):Array<Float> return [v.x,v.y,v.z];
    public static function main():Void {
        var root=Sys.getEnv("SHEET_ROUTER_FABRICATION_OUTPUT");
        if(root==null)root="build/sheet-router-freeze/fabrication";
        if(!FileSystem.exists(root))FileSystem.createDirectory(root);
        for(length in [2600.0,3150.0]) {
            var machine=new SheetRouter(new SheetRouterSpec(length)),base=machine.base;
            var members=[for(row in base.frame.analysisMembers()) {
                id:row.name,profile:row.profile.profileDesignation(),startMm:point(row.start),endMm:point(row.end),
                centrelineLengthMm:base.frame.length(row.name),stockLengthMm:base.frame.cutLength(row.name)
            }];
            var found=base.seams();
            var seams=[for(seam in found.seams) {
                id:seam.name(),a:seam.a,b:seam.b,startMm:point(seam.start),endMm:point(seam.stop),
                legMm:seam.legSize,joint:Std.string(seam.joint)
            }];
            var feet=[for(side in 0...2)for(station in 0...4) {
                id:'foot$side$station',x:machine.zeroPose('foot$side$station').x,
                y:machine.zeroPose('foot$side$station').y,z:machine.zeroPose('foot$side$station').z
            }];
            var result={status:"SR6 draft; seam candidates require access, fit-up and WPS review",workY:length,
                members:members,cutList:base.cutList(),declaredWelds:base.welds,seams:seams,
                seamDiagnostics:found.diagnostics.items,jigFootDatumsMm:feet,
                railPadDatumsMm:[machine.zeroPose("railPadYLeft"),machine.zeroPose("railPadYRight")],
                source:"SheetRouter buildBase; unchanged raised-datum structure plus Y1575 legs"};
            File.saveContent(root+'/y'+Std.int(length)+'.json',haxe.Json.stringify(result,null,"  ")+"\n");
            Sys.println('SR6 Y=$length: '+members.length+' base stock members, '+base.welds.length+' weld declarations, '+seams.length+' seam candidates, '+found.diagnostics.items.length+' findings');
        }
    }
}
