import materia.tooling.casting.BoxMoldPlanner;
import materia.tooling.ToolingCodec;
import materia.tooling.casting.BoxMoldCodec;
import machinekit.assembly.PosedParts;
import machinekit.tooling.BoxMoldCad;
import machinekit.tooling.ToolingBlock;
import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentValues;
import materia.assembly.AssemblyFrames as F;

class BoxMoldTests {
    static var assertions=0;
    static function check(ok:Bool,label:String):Void {assertions++;if(!ok)throw label;}
    static function rejects(fn:Void->Void,label:String):Void {
        var caught=false;try fn()catch(e:Dynamic){caught=true;Sys.println("Expected rejection: "+Std.string(e));}check(caught,label);
    }
    public static function run():Void {
        for(t in [18.0,17.8])for(rotated in [false,true]) {
            var d=SinglePocketSetup.document(t,rotated),p=BoxMoldPlanner.plan(d),report=BoxMoldCad.review(p);
            var encoded=BoxMoldCodec.encode(p);
            check(BoxMoldCodec.encode(BoxMoldCodec.decode(encoded))==encoded,"mold canonical recipe restoration");
            var local=SinglePocketSetup.foundation().body.geometry(),original=PosedParts.moved(local,d.setup.sourceToMold);local.close();
            var actual=BoxMoldCad.reconstructed(p),missing=original.subtractAll([actual]),excess=actual.subtractAll([original]);
            check(missing.volume()+excess.volume()<1e-5,"actual foundation CAD matches mold reconstruction");
            missing.close();excess.close();original.close();actual.close();
            check(report.symmetricDifferenceMm3<1e-5,"casting symmetric difference");
            check(p.panels.members.length==5 && p.cores.length==1 && p.fixtures.length==1,"complete box kit");
            check(Math.abs(p.cores[0].cavity.height-60)<1e-6,"cut tool clipped to source envelope");
            var c=p.cores[0];
            check(Math.abs(c.foam.width+2*c.coveringThickness-c.cavity.width)<1e-8,"covering is included in cavity size");
            check(c.cutting.wireDirection.y==1 && c.cutting.block.depth==c.foam.depth,"ruled block wire definition");
            check(p.panels.members[0].panel.stock.measuredThickness==t,"measured thickness preserved");
            check(Math.abs(p.cavity.width-(rotated?180:240))<1e-6,"setup rotates cavity and all tooling");
            check(report.assembly.assemblyOrder[0]=="bed","bed installs before supported members");
            check(report.assembly.removalOrder.length==7 && report.demolding.removalOrder.length==6,"full geometric graphs");
            var saved=BoxMoldCad.assembly(p).encode(),restored=MachineAssembly.decode(saved);restored.validate();
            check(restored.encode()==saved,"saved recipes restore complete assembly");
            check(report.pending.length>0,"physical qualification pending");
        }
        var p=BoxMoldPlanner.plan(SinglePocketSetup.document());
        p.penetrations=p.penetrations.filter(x->x.operationId!="insert-fixing");
        rejects(()->BoxMoldCad.penetrations(p),"unclassified hole fails");
        p=BoxMoldPlanner.plan(SinglePocketSetup.document());p.penetrations.filter(x->x.operationId=="insert-fixing")[0].sealMargin=19;
        rejects(()->BoxMoldCad.penetrations(p),"seal margin fails");
        p=BoxMoldPlanner.plan(SinglePocketSetup.document());
        p.panels.members.filter(m->m.panel.id=="south")[0].withdrawal={x:0,y:1,z:0};
        p.panels.members.filter(m->m.panel.id=="north")[0].withdrawal={x:0,y:-1,z:0};
        rejects(()->BoxMoldCad.review(p),"trapped opposite walls rejected");
        var d=SinglePocketSetup.document();d.setup.sourceToMold=F.identity();
        rejects(()->BoxMoldPlanner.plan(d),"closed-bottom core cannot break through fill");
        p=BoxMoldPlanner.plan(SinglePocketSetup.document());p.cores[0].cavity.width-=2;p.cores[0].foam.width-=2;p.cores[0].cutting.block.width-=2;
        rejects(()->BoxMoldCad.review(p),"missing core geometry rejected");
        p=BoxMoldPlanner.plan(SinglePocketSetup.document());
        var wall=p.panels.members.filter(m->m.panel.id=="south")[0];wall.pose.y-=0.5;
        var joint=p.panels.joints.filter(j->j.movingId=="south")[0];
        joint.fixedFrame=F.compose(F.inverse(p.panels.members[0].pose),wall.pose);
        rejects(()->BoxMoldCad.review(p),"open wall seam rejected by solid coverage");
        var block=new ToolingBlock(new ComponentValues().setNumber("width",20).setNumber("depth",30).setNumber("height",40).setToken("material","EPS tooling"));
        var native=block.geometry();check(Math.abs(native.volume()-24000)<1e-6,"core block CAD volume");native.close();
        Sys.println('BoxMoldTests: $assertions assertions passed');
    }
}
