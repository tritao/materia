import haxe.Json;
import materia.tooling.ToolingDocument;
import materia.tooling.ToolingCodec;
import materia.tooling.casting.BoxMoldPlanner;
import materia.tooling.casting.BoxMoldCodec;
import materia.tooling.panel.PanelCodec;

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
        Sys.println('BoxPlannerTests: $assertions assertions passed');
    }
}
