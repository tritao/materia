import cadkit.modeling.Part;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentDetail;
import machinekit.structural.FrameAssembly;
import machinekit.structural.RectTube;

/** Purchased floor service-rack envelope; detachable valve clamps are removed for withdrawal. */
class SheetValveServiceStand extends MachineComponent {
    final frame:FrameAssembly;
    final ids:Array<String> = [];
    public function new(x:Float,ys:Array<Float>) {
        super("SHEET-VALVE-SERVICE-RACK","Assumed floor valve service rack","steel",false);
        frame=new FrameAssembly();var tube=new RectTube(25,25,2);
        function member(id:String,a:Array<Float>,b:Array<Float>):Void {
            frame.point(id+"a",a[0],a[1],a[2]);frame.point(id+"b",b[0],b[1],b[2]);frame.member(id,id+"a",id+"b",tube);ids.push(id);
        }
        var first=ys[0]-100,last=ys[ys.length-1]+100;
        // Keep the longitudinal bar below the manifold outlet/trunk envelope.
        var barZ=75.0;
        member("bar",[x,first,barZ],[x,last,barZ]);
        for(i in 0...2) {
            var y=i==0?first:last;
            member('leg$i',[x,y,25],[x,y,barZ]);member('foot$i',[x-150,y,12.5],[x+150,y,12.5]);
        }
        for(i in 0...ys.length) {
            var y=ys[i];
            member('arch${i}a',[x,y-80,barZ],[x,y-80,470]);member('arch${i}b',[x,y+80,barZ],[x,y+80,470]);
            member('arch${i}top',[x,y-80,470],[x,y+80,470]);
        }
    }
    override public function hasGeometry():Bool return true;
    override public function geometry(detail:ComponentDetail=Preview):Part {
        var parts=[for(id in ids)frame.geometry(id)],result=Part.fuseAll(parts);for(p in parts)p.close();return result;
    }
}
