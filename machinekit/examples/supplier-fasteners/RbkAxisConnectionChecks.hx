import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentValues;
import machinekit.motion.HiwinHgh15ca;
import machinekit.motion.LinearRailBlock;
import machinekit.standard.HexNut;
import cadkit.threads.Threads;

/** Manufacturing, persistence and block-reference checks for the revised connections. */
class RbkAxisConnectionChecks {
    static var checks=0;
    static function check(ok:Bool,message:String):Void {checks++;if(!ok)throw message;}
    static function main():Void {
        var legacy:RbkAxisPart=cast RbkAxisPart.recipeType().create(new ComponentValues().setToken("kind","base"));
        check(legacy.revision=="v1-clearance","Old saved recipes keep their clearance-hole design");
        var legacyPart=legacy.geometry();
        check(legacyPart.shape.threadedFaces().length==0,"Legacy base has no new tap annotations");legacyPart.close();
        for(kind in ["base","riserLeft","riserRight","nutBracket"]) {
            var component=new RbkAxisPart(kind),part=component.geometry();
            var expected=kind=="base" ? 12 : kind=="nutBracket" ? 10 : 5;
            check(part.valid() && part.solidCount()==1,"Valid tapped fabrication part "+kind);
            check(part.shape.threadedFaces().length==expected,"Distinct named tap bores on "+kind);
            check(Threads.renderData(part.shape).length==expected,"All tap annotations resolve on "+kind);
            for(thread in part.shape.threadedFaces())check(thread.spec.internal,"Fabricated hole carries an internal thread");
            part.close();
        }
        var oldPlate=new RbkAxisPart("carriage","v1-clearance").geometry(),plate=new RbkAxisPart("carriage").geometry();
        check(Math.abs(oldPlate.volume()-plate.volume()-8*Math.PI*(3.75*3.75-2.25*2.25)*2.5)<0.001,
            "Only the eight 2.5 mm-deep block counterbores change carriage geometry");
        oldPlate.close();plate.close();
        var block=new HiwinHgh15ca(),reference=block.geometry(),box=LinearRailBlock.metric("HGR15").geometry();
        check(reference.valid() && reference.shape.bounds().get_min().get_y()<-10,"Detailed block retains lower sides missing from the envelope");
        check(HiwinHgh15ca.MOUNT_THREAD_DEPTH==5,"HGH15CA uses M4x5 mounting depth");
        check(block.componentType()!=null && block.designation!=LinearRailBlock.metric("HGR15").designation,"Detailed block has a distinct saveable identity");
        reference.close();box.close();block.close();
        var explicit=new HiwinHgh15ca(HiwinHgh15ca.referencePath());
        check(explicit.componentType()==null,"Explicit-path reference remains code-only for existing callers");explicit.close();
        var axis=new RbkSfuAxis(),saved=axis.encode(),loaded=MachineAssembly.decode(saved,RbkSfuAxis.registry());
        var blocks=0,nuts=0;
        for(entry in loaded.components()) {
            if(Std.isOfType(entry.component,HexNut))nuts++;
            if(Std.isOfType(entry.component,HiwinHgh15ca)) {
                blocks++;var restored:HiwinHgh15ca=cast entry.component,p=restored.geometry();
                check(p.valid() && p.shape.bounds().get_min().get_y()<-10,"Reload retains detailed block geometry");p.close();restored.close();
            }
        }
        check(blocks==2 && nuts==0,"Reloaded axis has two detailed blocks and no external hex nuts");
        var blockBolts=0;
        for(h in axis.hardware)if(h.interfacePair=="carriage/block") {
            blockBolts++;check(h.length==12 && h.grip==7.5 && h.engagement==4.5 && h.availableDepth==5,
                "Standard M4x12 has 4.5 mm engagement and 0.5 mm bottom clearance");
        }
        check(blockBolts==8,"Eight corrected block screws");
        Sys.println('Axis connection checks passed: $checks');
    }
}
