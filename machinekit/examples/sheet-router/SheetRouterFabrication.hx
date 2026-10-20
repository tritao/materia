import sys.FileSystem;
import sys.io.File;

/** Initial SR6 shop packet, authored stock/joints plus geometry-derived seam candidates. */
class SheetRouterFabrication {
    static function point(v:cadkit.modeling.Vector):Array<Float> return [v.x,v.y,v.z];
    public static function main():Void {
        var root=Sys.getEnv("SHEET_ROUTER_FABRICATION_OUTPUT");
        if(root==null)root="build/sheet-router-work/sr6/native";
        if(!FileSystem.exists(root))FileSystem.createDirectory(root);
        for(length in [2600.0,3150.0]) {
            var report=root+'/y'+Std.int(length)+'.json';
            if(FileSystem.exists(report))FileSystem.deleteFile(report);
            var machine=new SheetRouter(new SheetRouterSpec(length)),base=machine.base;
            var prep=new SheetRouterFramePreparation(machine);prep.findSeams();
            var service=SheetRouterServiceChecks.check(machine,prep);
            var folder=root+'/y'+Std.int(length);if(!FileSystem.exists(folder))FileSystem.createDirectory(folder);
            var members:Array<Dynamic> = [];
            for(piece in prep.pieces) {
                var path=folder+'/'+piece.id+'.step';piece.part.shape.exportStep(path);
                members.push({id:piece.id,sourceStock:piece.stock.id,profile:piece.stock.profile.profileDesignation(),
                    startMm:piece.startMm,endMm:piece.endMm,centrelineLengthMm:piece.stockLengthMm,
                    stockLengthMm:piece.stockLengthMm,volumeMm3:piece.part.volume(),stepFile:path});
            }
            var groups:Map<String,{designation:String,description:String,quantity:Int,totalLength:Float}> = new Map();
            for(m in members) {
                if(!groups.exists(m.profile))groups.set(m.profile,{designation:m.profile,description:m.profile,quantity:0,totalLength:0.0});
                var g=groups.get(m.profile);Reflect.setField(g,"quantity",(cast Reflect.field(g,"quantity"):Int)+1);Reflect.setField(g,"totalLength",(cast Reflect.field(g,"totalLength"):Float)+(cast m.stockLengthMm:Float));
            }
            var feet=[for(side in 0...2)for(station in 0...4) {
                id:'foot$side$station',x:machine.zeroPose('foot$side$station').x,
                y:machine.zeroPose('foot$side$station').y,z:machine.zeroPose('foot$side$station').z
            }];
            var result={status:prep.failures.length==0?"Prepared contacts and seam graph pass; shop checklist still gates release":"Prepared frame checks failed; no release",workY:length,
                members:members,cutList:[for(g in groups)g],declaredWelds:prep.joints,seams:prep.seams,
                seamDiagnostics:prep.failures,serviceAccess:service,preparations:prep.preparations,removedInteriorVolumeMm3:prep.removedVolumeMm3,
                jigFootDatumsMm:feet,railPadDatumsMm:[machine.zeroPose("railPadYLeft"),machine.zeroPose("railPadYRight")],
                source:"Frozen SheetRouter envelopes; continuous stringers, coped/intersection-trimmed pieces"};
            File.saveContent(report,haxe.Json.stringify(result,null,"  ")+"\n");
            Sys.println('SR6 Y=$length: '+members.length+' prepared pieces, '+prep.joints.length+' joints, '+prep.seams.length+' seams, '+prep.failures.length+' findings');
            for(failure in prep.failures)Sys.println("SR6: "+failure);
            Sys.println('SR6 service: '+service.status);
            for(failure in (cast service.errors:Array<String>))Sys.println('SR6 service: '+failure);
            prep.close();
        }
    }
}
