import haxe.Json;
import materia.tooling.ToolingDocument;
import materia.tooling.ToolingCodec;
import materia.tooling.casting.BoxMoldPlanner;
import materia.tooling.casting.BoxMoldCodec;
import materia.tooling.panel.PanelCodec;
import materia.tooling.casting.CoreSupports;
import materia.tooling.casting.CoreSupport.CoreSupportMode;
import materia.tooling.casting.CoreSupport.CoreSupportKind;

/** Planning/persistence remain buildable with ProjectKit alone, without native CAD. */
class BoxPlannerTests {
    static var assertions=0;
    static function check(ok:Bool,label:String):Void {assertions++;if(!ok)throw label;}
    static function rejects(fn:Void->Void,label:String):Void {var caught=false;try fn()catch(e:Dynamic)caught=true;check(caught,label);}
    public static function run(d:ToolingDocument):Void {
        d=ToolingCodec.snapshot(d);d.recipe.minimumLigament=2;
        d.input.inserts[0].height=10;d.input.inserts[0].boreDiameter=5;
        var recess=d.input.features.filter(f->f.kind=="insert-recess")[0];recess.height=10;
        d.setup.sourceFingerprint=ToolingCodec.inputFingerprint(d.input);
        var p=BoxMoldPlanner.plan(d),encoded=BoxMoldCodec.encode(p);
        check(BoxMoldCodec.encode(BoxMoldCodec.decode(encoded))==encoded,"mold canonical persistence");
        check(p.panels.members.length==5 && p.penetrations.length==9,"all sockets/lands and fixing hole classified");
        check(p.documentFingerprint==ToolingCodec.documentFingerprint(d),"mold provenance");
        var raw:Dynamic=Json.parse(encoded);Reflect.setField(raw,"version",99);
        rejects(()->BoxMoldCodec.decode(Json.stringify(raw)),"unknown mold version");
        raw=Json.parse(encoded);Reflect.setField(Reflect.field(raw,"resolved"),"id","corrupt");
        rejects(()->BoxMoldCodec.decode(Json.stringify(raw)),"generated records cannot be edited behind source");
        p.cores[0].cutting.wireDirection={x:1,y:1,z:0};
        rejects(()->BoxMoldPlanner.validate(p),"unsupported wire direction rejected");
        p=BoxMoldPlanner.plan(d);p.penetrations.pop();
        rejects(()->BoxMoldPlanner.validate(p),"missing penetration classification");
        var panel:Dynamic=Json.parse(PanelCodec.encode(p.panels.members[0].panel));Reflect.setField(panel,"version",1);
        rejects(()->PanelCodec.decode(Json.stringify(panel)),"old panel encoding rejected");
        panel=Json.parse(PanelCodec.encode(p.panels.members[0].panel));Reflect.setField(Reflect.field(panel,"panel"),"contactOpenings",null);
        rejects(()->PanelCodec.decode(Json.stringify(panel)),"null opening list rejected");
        d.recipe.panel.measuredThickness=18.2;
        check(BoxMoldCodec.fingerprint(BoxMoldPlanner.plan(d))!=BoxMoldCodec.fingerprint(BoxMoldCodec.decode(encoded)),"stock changes invalidate package");
        var bridge=BoxMoldPlanner.plan(d);
        check(bridge.cores[0].support.kind==CoreSupportKind.Bridge && bridge.supportPanels.length==4,"suspended core receives carrier and pinned bridge");
        var old:Dynamic=Json.parse(BoxMoldCodec.encode(bridge));Reflect.setField(old,"version",1);
        rejects(()->BoxMoldCodec.decode(Json.stringify(old)),"old mold encoding rejected");
        var o=CoreSupports.defaults(d.recipe);o.mode=CoreSupportMode.BedSocket;
        rejects(()->BoxMoldPlanner.plan(d,o),"bed socket cannot alter a solid casting floor");
        var bedDoc=ToolingCodec.snapshot(d);bedDoc.input.features.filter(f->f.kind=="void")[0].pose.z=40;
        bedDoc.setup.sourceFingerprint=ToolingCodec.inputFingerprint(bedDoc.input);
        var bed=BoxMoldPlanner.plan(bedDoc),core=bed.cores[0];
        check(core.support.kind==CoreSupportKind.BedSocket && bed.supportPanels.length==0,"bed opening selects routed socket");
        check(bed.penetrations.length==10 && core.support.foot!=null,"bed socket classified and separate foot defined");
        var foot=core.support.foot;if(foot==null)throw "Missing foot";
        check(Math.abs(core.foam.z-bed.cavity.z)<1e-7 && foot.foam.z<bed.cavity.z,"main core height preserved and foot buried");
        check(BoxMoldCodec.encode(BoxMoldCodec.decode(BoxMoldCodec.encode(bed)))==BoxMoldCodec.encode(bed),"bed support roundtrip");
        o=CoreSupports.defaults(bedDoc.recipe);o.mode=CoreSupportMode.Bridge;
        rejects(()->BoxMoldPlanner.plan(bedDoc,o),"bridge cannot pass a closed top");
        o=CoreSupports.defaults(bedDoc.recipe);o.fitClearance=30;
        rejects(()->BoxMoldPlanner.plan(bedDoc,o),"socket clearance cannot escape core sealing land");
        o=CoreSupports.defaults(bedDoc.recipe);o.adhesiveThickness=0;
        rejects(()->BoxMoldPlanner.plan(bedDoc,o),"zero adhesive thickness rejected");
        rejects(()->CoreSupports.options({mode:"guess",embedDepth:6,fitClearance:0.2,adhesiveThickness:0.2}),"unknown support mode rejected");
        foot.cutting.wireDirection={x:1,y:0,z:0};
        rejects(()->BoxMoldPlanner.validate(bed),"foot cutting direction validated");
        Sys.println('BoxPlannerTests: $assertions assertions passed');
    }
}
