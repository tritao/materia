import machinekit.milling.MillFrame;
import machinekit.tooling.CastingMoldCad;
import machinekit.tooling.ToolingVolume;
import machinekit.tooling.SweptRemoval;
import machinekit.assembly.PosedParts;
import cadkit.modeling.Vector;
import materia.assembly.AssemblyFrames as F;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingMoldPlanner;
import materia.tooling.casting.CastingMoldCodec;

/** Real mill acceptance cases; source CAD is an independent reference to the planner. */
class MillMoldChecks {
    static var assertions=0;
    static function check(ok:Bool,label:String):Void {assertions++;if(!ok)throw label;}
    static function rejects(fn:Void->Void,label:String):Void {var failed=false;try fn()catch(e:Dynamic){failed=true;Sys.println("Expected mold rejection: "+Std.string(e));}check(failed,label);}
    static function equivalent(frame:MillFrame,p:CastingMoldPlan):Void {
        var local=frame.base.body.geometry(),original=PosedParts.moved(local,p.document.setup.sourceToMold);local.close();
        var rebuilt=CastingMoldCad.reconstructed(p),a=original.subtractAll([rebuilt]),b=rebuilt.subtractAll([original]);
        check(a.volume()+b.volume()<1e-3,"actual foundation CAD symmetric difference");a.close();b.close();rebuilt.close();original.close();
    }
    public static function run(frame:MillFrame,p:CastingMoldPlan,report:machinekit.tooling.CastingMoldCad.CastingMoldReview):Void {
        check(p.cores.length==6 && p.fixtures.length==12,"actual mill pocket and insert inventory");
        check(p.features.length==p.document.input.features.length && p.features.length==31,"every full-frame feature classified");
        check(report.symmetricDifferenceMm3<1e-3 && !report.fabricationReady,"review geometry passes without fabricating readiness");
        check(report.assemblyGroups.length==2 && report.permanentGroups.length==2,"two shared row fixtures");
        check(report.assembly.assemblyOrder[0]=="bed","bed installed first");
        check(p.changeRequests.filter(c->c.kind=="lifting-points").length==1,"handling lift/flip request");
        check(p.options.precision.filter(s->s.route=="replicated").length==4 && p.options.groutedJoints.length==1,"reviewed finishing routes retained");
        equivalent(frame,p);
        for(c in p.cores) {
            check(c.foamLayers.length==2,"EPS layers fit 100 mm stock");
            var first=c.foamLayers[0],last=c.foamLayers[c.foamLayers.length-1];
            check(Math.abs(last.z+last.height+c.adhesiveThickness-c.cavity.z-c.cavity.height)<1e-7,"glue line included in foam height");
            check(Math.abs(c.foamLayers[1].z-first.z-first.height-c.adhesiveThickness)<1e-7,"interlayer glue line included");
        }
        for(f in p.formers) {
            var solid=ToolingVolume.geometryOf(f.volume);
            check(solid.valid() && solid.shape.subshapeCount(CadKit.ShapeKind.Solid)==1,"each former is one physical connected solid");solid.close();
        }
        var changed=CastingMoldPlanner.plan(MillMoldSetup.document(frame,17.8),MillMoldSetup.options(frame));
        check(CastingMoldCodec.fingerprint(changed)!=CastingMoldCodec.fingerprint(p),"measured stock invalidates package");equivalent(frame,changed);
        var bare=new MillFrame(frame.layout),simple=CastingMoldPlanner.plan(MillMoldSetup.document(bare),MillMoldSetup.options(bare));
        check(simple.features.length<p.features.length && p.features.filter(f->f.featureId=="rear-y-drive-seat").length==1,"optional XY interfaces come from full source instance");equivalent(bare,simple);
        var encoded=CastingMoldCodec.encode(p),restored=CastingMoldCodec.decode(encoded);
        check(CastingMoldCodec.encode(restored)==encoded,"complete plan recipe regeneration");
        restored.openings.pop();rejects(()->CastingMoldPlanner.validate(restored),"unclassified contact opening");
        restored=CastingMoldCodec.decode(encoded);restored.cores[0].cavity.height-=2;
        rejects(()->CastingMoldCad.review(restored),"missing pocket geometry");
        restored=CastingMoldCodec.decode(encoded);restored.fixtures[0].datum.pieces[0].z+=30;
        rejects(()->CastingMoldCad.review(restored),"displaced insert datum fixture");
        var f=p.formers.filter(f->f.featureIds.length>0 && StringTools.startsWith(f.featureIds[0],"foot-access-"))[0];
        var former=ToolingVolume.geometryOf(f.volume),castingPart=CastingMoldCad.source(p);
        rejects(()->SweptRemoval.analyze([{id:f.volume.id,part:former,direction:new Vector(-f.withdrawal.x,0,0)},{id:"casting",part:castingPart,direction:new Vector(0,0,1)}]),"inward foot former trapped against actual casting");
        former.close();castingPart.close();
        Sys.println('MillMoldChecks: $assertions assertions passed');
    }
}
