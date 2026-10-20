import CadKit;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.Plane;
import cadkit.modeling.Location;
import machinekit.structural.RectTube;
import machinekit.structural.RectTubeMember;
import machinekit.welding.WeldSeams;
import materia.assembly.AssemblyFrames as F;

typedef SheetFrameStock = {
    var id:String;
    var start:Vector;
    var end:Vector;
    var reference:Vector;
    var profile:RectTube;
    var raw:Part;
    var envelope:Part;
}
typedef SheetFramePiece = {
    var id:String;
    var stock:SheetFrameStock;
    var part:Part;
    var stockLengthMm:Float;
    var startMm:Array<Float>;
    var endMm:Array<Float>;
}

/** Shop partition of the frozen frame envelopes. Continuous stringers precede coped posts/braces. */
class SheetRouterFramePreparation {
    public final stocks:Array<SheetFrameStock> = [];
    public final pieces:Array<SheetFramePiece> = [];
    public final preparations:Array<Dynamic> = [];
    public final joints:Array<Dynamic> = [];
    public final seams:Array<Dynamic> = [];
    public final failures:Array<String> = [];
    public var removedVolumeMm3(default,null):Float=0;
    static function point(v:Vector):Array<Float> return [v.x,v.y,v.z];
    static function rank(id:String):Int {
        if(id=="frameLeft"||id=="frameRight")return 0;
        if(StringTools.startsWith(id,"lower"))return 1;
        if(StringTools.startsWith(id,"tableSide"))return 2;
        if(id=="frameFront"||id=="frameBack")return 3;
        if(StringTools.startsWith(id,"post"))return 4;
        if(id.indexOf("Brace")<0)return 5;
        return 6;
    }
    static function overlap(a:Part,b:Part):Bool {
        var x=a.shape.bounds(),y=b.shape.bounds(),lo=x.get_min(),hi=x.get_max(),bl=y.get_min(),bh=y.get_max();
        return lo.get_x()<=bh.get_x()+1e-5&&hi.get_x()>=bl.get_x()-1e-5&&lo.get_y()<=bh.get_y()+1e-5&&hi.get_y()>=bl.get_y()-1e-5&&lo.get_z()<=bh.get_z()+1e-5&&hi.get_z()>=bl.get_z()-1e-5;
    }
    public function new(machine:SheetRouter) {
        for(row in machine.base.frame.analysisMembers()) {
            var profile:RectTube=cast row.profile;
            add(row.name,row.start,row.end,row.reference,profile,machine.base.frame.geometry(row.name));
        }
        for(id in ["frameLeft","frameRight","frameFront","frameBack"]) {
            var member:RectTubeMember=cast machine.component(id),pose=machine.zeroPose(id);
            var a=F.transformPoint(pose,0,0,0),b=F.transformPoint(pose,0,0,member.length),ref=F.transformVector(pose,0,1,0);
            var original=member.geometry(),placed=machinekit.assembly.PosedParts.moved(original,pose);original.close();
            add(id,new Vector(a.x,a.y,a.z),new Vector(b.x,b.y,b.z),new Vector(ref.x,ref.y,ref.z),member.profile,placed);
        }
        stocks.sort((a,b)->rank(a.id)==rank(b.id)?Reflect.compare(a.id,b.id):rank(a.id)-rank(b.id));
        for(i in 0...stocks.length) {
            var row=stocks[i],body=new Part(row.raw.shape.cloneShape()),cutters:Array<String> = [];
            for(j in 0...i) {
                var mate=stocks[j];if(!overlap(body,mate.envelope))continue;
                var common=body.shape.common(mate.envelope.shape),volume=common.volume();common.close();
                if(volume<0.01)continue;
                var cut=body.subtract(mate.envelope);body.close();body=cut;cutters.push(mate.id);removedVolumeMm3+=volume;
            }
            if(body.isEmpty())failures.push(row.id+": preparation removed the entire stock");
            var count=body.solidCount();
            preparations.push({stock:row.id,copeTo:cutters,pieces:count,blankLengthMm:row.end.subtract(row.start).length()});
            for(k in 0...count) {
                var raw=new Part(body.shape.subshape(CadKit.ShapeKind.Solid,k)),part=raw.named(row.id+"-p"+k);raw.close();
                var axis=row.end.subtract(row.start).normalized(),lo=Math.POSITIVE_INFINITY,hi=Math.NEGATIVE_INFINITY;
                for(edge in 0...part.shape.subshapeCount(CadKit.ShapeKind.Edge)) {
                    var e=part.shape.subshape(CadKit.ShapeKind.Edge,edge);
                    for(t in [0.0,1.0]) {var d=Vector.fromNative(e.positionAt(t)).subtract(row.start).dot(axis);lo=Math.min(lo,d);hi=Math.max(hi,d);}
                    e.close();
                }
                if(part.volume()<0.01)failures.push(row.id+": sliver solid after coping");
                pieces.push({id:row.id+"-p"+k,stock:row,part:part,stockLengthMm:hi-lo,startMm:point(row.start.add(axis.scale(lo))),endMm:point(row.start.add(axis.scale(hi)))});
            }
            body.close();
        }
    }
    function add(id:String,a:Vector,b:Vector,reference:Vector,profile:RectTube,raw:Part):Void {
        var direction=b.subtract(a).normalized(),x=reference.cross(direction).normalized();
        var blank=Part.box(profile.width,profile.height,b.subtract(a).length());
        var envelope=blank.placed(new Location(new Plane(a,x,direction)));blank.close();
        stocks.push({id:id,start:a,end:b,reference:reference,profile:profile,raw:raw,envelope:envelope});
    }
    /** Every positive-area prepared contact gets a real B-rep seam and a connected load path. */
    public function findSeams():Void {
        var links:Map<String,Array<String>> = new Map();for(p in pieces)links.set(p.id,[]);
        for(i in 0...pieces.length)for(j in i+1...pieces.length) {
            var a=pieces[i],b=pieces[j];if(!overlap(a.part,b.part))continue;
            if(a.part.shape.distance(b.part.shape)>1e-4)continue;
            var common=a.part.shape.common(b.part.shape),volume=common.volume(),area=common.area();common.close();
            if(volume>0.1)failures.push('Overlapping prepared solids: ${a.id}/${b.id}: $volume mm3');
            var leg=5.0;
            var found=WeldSeams.find({id:a.id,part:a.part},{id:b.id,part:b.part},leg);
            if(found.length==0) {
                var contactArea=0.0;
                for(k in 0...a.part.shape.subshapeCount(CadKit.ShapeKind.Face)) {
                    var face=a.part.shape.subshape(CadKit.ShapeKind.Face,k),shared=face.common(b.part.shape);
                    contactArea+=shared.area();shared.close();face.close();
                }
                if(contactArea>0.01)failures.push('Prepared contact without seam: ${a.id}/${b.id}, area=$contactArea');
                continue;
            }
            var jointId='J'+(joints.length+1),length=0.0;
            for(s in found) {
                var len=s.stop.subtract(s.start).length();length+=len;
                seams.push({id:s.name(),joint:jointId,a:s.a,b:s.b,startMm:point(s.start),endMm:point(s.stop),legMm:leg,lengthMm:len,type:Std.string(s.joint)});
            }
            joints.push({id:jointId,a:a.id,b:b.id,legMm:leg,availableSeamMm:length,seams:found.length,contactAreaMm2:area,sequenceGroup:rank(a.stock.id)>rank(b.stock.id)?rank(a.stock.id):rank(b.stock.id)});
            links.get(a.id).push(b.id);links.get(b.id).push(a.id);
        }
        var seen:Map<String,Bool> = new Map(),todo=[pieces[0].id];
        while(todo.length>0) {var id=todo.pop();if(seen.exists(id))continue;seen.set(id,true);for(n in links.get(id))todo.push(n);}
        for(p in pieces)if(!seen.exists(p.id))failures.push('Disconnected prepared piece: '+p.id);
    }
    public function close():Void {
        for(p in pieces)p.part.close();for(s in stocks){s.raw.close();s.envelope.close();}
    }
}
