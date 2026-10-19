import materia.tooling.panel.PanelJoints;
import materia.tooling.panel.PanelCodec;
import materia.tooling.panel.PanelContours;
import materia.tooling.panel.PanelGeometry;
import materia.tooling.panel.PanelCoupon;
import materia.tooling.panel.PanelAssemblyPlan;
import materia.tooling.ToolingEnums.ContactFace;
import materia.tooling.ToolingEnums.MachiningSide;
import materia.tooling.panel.PanelPlan.PanelOperationKind;
import materia.assembly.AssemblyFrames as F;

class PanelTests {
    static var count=0;
    static function check(ok:Bool,label:String):Void {count++;if(!ok)throw label;}
    static function rejects(fn:Void->Void,label:String):Void {var bad=false;try fn()catch(_:Dynamic)bad=true;check(bad,label);}
    public static function run():Void {
        for(t in [17.6,18.0,18.3]) {
            var stock=PanelJoints.stock(t);
            var pair=PanelJoints.tabSlot({id:"keyed",fixedStock:stock,movingStock:stock,
                fixedWidth:320,fixedHeight:220,movingWidth:280,bodyHeight:100,offsetX:20,slotY:150,
                tabs:[{center:55.0,width:30.0},{center:210.0,width:44.0}],clearance:0.2,routerDiameter:6,minimumLigament:6});
            PanelAssemblyPlan.validate(PanelAssemblyPlan.pair(pair));
            for(p in [pair.fixed,pair.moving]) {
                check(PanelCodec.encode(PanelCodec.decode(PanelCodec.encode(p)))==PanelCodec.encode(p),"exact panel codec roundtrip");
                check(PanelContours.outside(p).length>0,"exact outside operation");
            }
            var slot=pair.fixed.operations.filter(o->o.id=="slot-0")[0],bounds=PanelGeometry.bounds(PanelContours.operation(slot));
            check(Math.abs(bounds.height-(t+0.2+6))<1e-6,"measured stock controls slot and corner overcut");
            check(pair.moving.operations[0].contour.filter(s->s.kind=="arc").length==4,"root relief arcs in exact outside contour");
        }
        var stock=PanelJoints.stock(17.8);
        for(pair in [PanelJoints.rebate("rebate",stock),PanelJoints.pinBolt("pins",stock)]) {
            PanelAssemblyPlan.validate(PanelAssemblyPlan.pair(pair));
            check(PanelCodec.decode(PanelCodec.encode(pair.fixed)).id==pair.fixed.id,"joint panel recipes");
        }
        var coupon=PanelCoupon.make(stock,[0.1,0.2,0.3,0.4]);
        check(coupon.receiver.operations.length==5,"one outside and four coupon slot operations");
        rejects(()->PanelCoupon.make(stock,[0.1,0.1]),"duplicate coupon clearance");
        var bad=PanelJoints.blank("bad",100,100,stock);
        bad.operations[0].kind=cast "inferred";
        rejects(()->PanelCodec.snapshot(bad),"unknown operation kind");
        var circle=PanelContours.circle(20,20,10);
        PanelGeometry.validate(circle);
        check(Math.abs(PanelGeometry.area(circle)-Math.PI*100)<1e-7,"exact arc area");
        check(PanelGeometry.inside(circle,20,20) && !PanelGeometry.inside(circle,40,20),"exact circle containment");
        var relief=PanelContours.relievedRectangle(50,50,30,20,3);
        PanelGeometry.validate(relief);
        check(Math.abs(PanelGeometry.area(relief)-(600+3*Math.PI*9))<1e-6,"single relieved contour equals rectangle and exterior corner sectors");
        var mismatch=PanelJoints.pinBolt("wrong",stock),plan=PanelAssemblyPlan.pair(mismatch);
        plan.members[1].pose.x+=1;
        rejects(()->PanelAssemblyPlan.validate(plan),"wrong alignment rejected");
        var valid=PanelAssemblyPlan.pair(mismatch);
        var independent=PanelAssemblyPlan.pair(mismatch);
        independent.members[1].withdrawal.z=-1;
        check(independent.members[0].withdrawal.z==1,"members own independent removal directions");
        rejects(()->PanelAssemblyPlan.removalOrder(valid,[{movingId:mismatch.fixed.id,blockerId:mismatch.moving.id},
            {movingId:mismatch.moving.id,blockerId:mismatch.fixed.id}]),"blocking cycle");
        var reversed=PanelAssemblyPlan.pair(mismatch);
        reversed.members[1].pose=F.compose(reversed.members[1].pose,F.turnY(Math.PI));
        rejects(()->PanelAssemblyPlan.validate(reversed),"reversed joint rejected");
        var wrongSide=PanelJoints.blank("wrong-side",100,100,stock);
        wrongSide.operations[0].side=cast "sideways";
        rejects(()->PanelCodec.encode(wrongSide),"unknown machining side");
        var skin=PanelJoints.blank("contact",100,100,stock);
        skin.contactFace=ContactFace.PositiveZ;
        rejects(()->PanelCodec.encode(skin),"labels stay off contact face");
        var wrongTool=PanelCodec.snapshot(coupon.receiver);
        wrongTool.operations.filter(o->o.kind==InsideProfile)[0].toolDiameter=8;
        rejects(()->PanelCodec.encode(wrongTool),"relief and CAM tool must agree");
        var arc=PanelCodec.snapshot(coupon.plug);
        arc.operations[0].contour.filter(s->s.kind=="arc")[0].end.x+=1;
        rejects(()->PanelCodec.encode(arc),"arc endpoints must match exact sweep");
        var pocket=PanelJoints.blank("pocket",100,100,stock);
        pocket.operations.push(PanelJoints.operation("pocket",Pocket,PanelContours.circle(30,30,10),4,6,MachiningSide.NegativeZ,false));
        check(PanelCodec.decode(PanelCodec.encode(pocket)).operations.length==2,"explicit back-side pocket and setup");
        pocket.operations[1].setupId="front";
        rejects(()->PanelCodec.encode(pocket),"one setup cannot mix opposite machining faces");
        rejects(()->PanelJoints.rebate("thin",PanelJoints.stock(8)),"rebate preserves shelf thickness");
        rejects(()->PanelGeometry.validate(PanelContours.rectangle(0,0,1e200,1e200)),"overflowing contour area rejected");
        var diagonal=PanelJoints.pinBolt("diagonal",stock,8,0.2,2,7,6,6,
            {width:220,height:160,primary:{x:40,y:40},secondary:{x:140,y:80},bolt:{x:90,y:110}});
        PanelAssemblyPlan.validate(PanelAssemblyPlan.pair(diagonal));
        check(diagonal.moving.operations.filter(o->o.id=="secondary-pin")[0].contour[0].start.y!=
            diagonal.moving.operations.filter(o->o.id=="secondary-pin")[0].contour[0].end.y,"secondary relief follows arbitrary pin baseline");
        Sys.println('PanelTests: $count assertions passed');
    }
}
