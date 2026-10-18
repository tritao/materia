import cadkit.modeling.Part;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.PosedParts;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentDetail;
import machinekit.motion.BallNut;
import machinekit.motion.BallScrewRetainer;
import machinekit.motion.LeadScrewNut;

/** Check the newly added material at stops and mid-travel, without claiming a full swept-volume audit. */
class BallScrewDetailChecks {
    public static function check(assembly:MachineAssembly, xy:machinekit.milling.MillXYStageLayout, zLayout:machinekit.milling.MillZAxisLayout):Void {
        var model = new AssemblyModel("mm"); assembly.addTo(model, "");
        var state = new AssemblyState(model.definition("ball-screw-details"));
        var members = assembly.components(), cache = new PosedParts();
        var additions:Array<{id:String,component:MachineComponent}> = [];
        var owned:Array<Part> = [];
        try {
            for (member in members) {
                if (Std.isOfType(member.component,BallScrewRetainer)) additions.push(member);
                else if (Std.isOfType(member.component,BallNut)) {
                    var nut:BallNut = cast member.component;
                    var baseline = new LeadScrewNut(nut.thread,6,true,nut.kind,
                        {bodyDiameter:28,bodyLength:35,flangeDiameter:48,flangeThickness:10,boltCircleDiameter:38,mountScrew:"M5"}).geometry();
                    var detailed = nut.geometry(), added = detailed.subtract(baseline);
                    detailed.close(); baseline.close(); owned.push(added);
                    additions.push({id:member.id,component:new AddedMaterial(added,nut.designation)});
                }
            }
            if (additions.length != 6) throw 'Expected three retainers and three detailed nuts, got ${additions.length}';
            var pairs = 0;
            for (x in [-xy.xHalfTravel,0.0,xy.xHalfTravel]) for (y in [-xy.yHalfTravel,0.0,xy.yHalfTravel]) for (z in [zLayout.lower,zLayout.lower/2,0.0]) {
                state.setJoint("xyStage/x",x); state.setJoint("xyStage/y",y); state.setJoint("zAxis/z",z); state.forwardKinematics();
                for (added in additions) for (other in members) if (other.id != added.id) {
                    var volume = cache.overlap(added.component,state.worldPose(added.id),other.component,state.worldPose(other.id));
                    pairs++;
                    if (volume > 0.01) throw 'New screw detail ${added.id} overlaps ${other.id}: $volume mm3 at $x/$y/$z';
                }
            }
            cache.close(); for (part in owned) part.close();
            Sys.println('Ball screw details: $pairs pairs clear at 27 XYZ stop/midpoint combinations');
        } catch (error:Dynamic) { cache.close(); for (part in owned) part.close(); throw error; }
    }
}
private class AddedMaterial extends MachineComponent {
    final part:Part;
    public function new(part:Part,name:String) { super(name+"-ADDED-EXTERIOR","Added nut detail","steel",true);this.part=part; }
    override public function hasGeometry():Bool return true;
    override public function geometry(detail:ComponentDetail=Preview):Part return new Part(part.shape.cloneShape());
}
