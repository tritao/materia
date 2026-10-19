import haxe.Json;
import cadkit.modeling.Part;
import cadkit.drawing.Svg;
import cadkit.drawing.Drawing2D;
import cadkit.drawing.DrawingEntity;
import cadkit.sheetmetal.Point2;
import materia.tooling.panel.PanelGeometry;
import cadkit.drawing.Dxf;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.AssemblyPreview;
import machinekit.assembly.PosedParts;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentValues;
import machinekit.tooling.PanelKit;
import machinekit.tooling.PanelDrawing;
import machinekit.tooling.ToolingPanel;
import machinekit.tooling.ToolingPin;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.standard.FlatWasher;
import machinekit.standard.HexNut;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact;
import materia.tooling.panel.PanelJoints;
import materia.tooling.panel.PanelJoints.PanelJointPair;
import materia.tooling.panel.PanelAssemblyPlan;
import materia.tooling.panel.PanelCodec;
import materia.tooling.panel.PanelCoupon;
import materia.tooling.panel.PanelPlan;

/** Generic joinery demonstrator: no mill or casting-specific dependencies. */
class PanelToolingPreview {
    static function writePanel(dir:String,p:PanelPlan,?clearances:Array<Float>):Void {
        var drawing=PanelDrawing.drawing(p);
        if(clearances!=null) {
            var entities=drawing.entities();
            for(i in 0...clearances.length) {
                var op=p.operations.filter(o->o.id=='fit-$i')[0],bounds=PanelGeometry.bounds(op.contour);
                entities.push(Label(Text,new Point2(bounds.x+bounds.width/2-12,bounds.y-8),2.8,"C="+clearances[i]+" mm"));
            }
            drawing=new Drawing2D(entities);
        }
        sys.io.File.saveContent(dir+"/"+p.id+".svg",Svg.write(drawing));
        sys.io.File.saveContent(dir+"/"+p.id+".dxf",Dxf.write(drawing));
        sys.io.File.saveContent(dir+"/"+p.id+".panel.json",PanelCodec.encode(p));
        var shape=ToolingPanel.fromPlan(p).geometry();
        try shape.shape.exportStep(dir+"/"+p.id+".step")catch(e:Dynamic){shape.close();throw e;}
        shape.close();
    }
    static function hardware(a:MachineAssembly,root:String,id:String,part:MachineComponent,local:AssemblyFrame,base:AssemblyFrame):Void {
        a.addComponent(id,part,F.compose(base,local));
        a.addMemberConnector(root,id+"-seat",local);a.addMemberConnector(id,"tooling-seat",F.identity());
        a.addMate(id+"-fixed","fixed",root,id+"-seat",id,"tooling-seat");
    }
    static function addPinHardware(a:MachineAssembly,pair:PanelJointPair,base:AssemblyFrame):Void {
        var t=pair.fixed.stock.measuredThickness,washer=FlatWasher.metric("M6"),nut=HexNut.metric("M6");
        var boltLength=5*Math.ceil((2*t+2*washer.thickness+nut.height+1)/5);
        for(x in [40.0,140.0])hardware(a,pair.fixed.id,x==40?"primary-pin":"secondary-pin",
            new ToolingPin(new ComponentValues().setNumber("diameter",6).setNumber("length",2*t+4)),F.translation(x,35,-2),base);
        hardware(a,pair.fixed.id,"bolt",SocketHeadCapScrew.metric("M6",boltLength),F.translation(90,65,2*t+washer.thickness),base);
        hardware(a,pair.fixed.id,"top-washer",washer,F.translation(90,65,2*t),base);
        hardware(a,pair.fixed.id,"bottom-washer",FlatWasher.metric("M6"),F.translation(90,65,-washer.thickness),base);
        hardware(a,pair.fixed.id,"nut",nut,F.translation(90,65,-washer.thickness-nut.height),base);
    }
    public static function main():Void {
        var t=18.0;
        for(arg in Sys.args())if(StringTools.startsWith(arg,"--thickness="))t=Std.parseFloat(arg.substr(12));
        var stock=PanelJoints.stock(t),cwd=Sys.getCwd();
        var dir=sys.FileSystem.exists(cwd+"/machinekit")?cwd+"/machinekit/examples/panel-tooling/build/review":cwd+"/build/review";
        sys.FileSystem.createDirectory(dir);
        var keyed=PanelJoints.tabSlot({id:"keyed",fixedStock:stock,movingStock:stock,fixedWidth:320,fixedHeight:220,
            movingWidth:280,bodyHeight:100,offsetX:20,slotY:150,tabs:[{center:55.0,width:30.0},{center:210.0,width:44.0}],
            clearance:0.2,routerDiameter:6,minimumLigament:6},true);
        var pairs=[keyed,PanelJoints.rebate("rebate",stock),PanelJoints.pinBolt("pins",stock)];
        var assembled=new MachineAssembly(),exploded=new MachineAssembly(),reports:Array<Dynamic> = [];
        for(i in 0...pairs.length) {
            var pair=pairs[i],base=F.translation(i*450,0,0),plan=PanelAssemblyPlan.pair(pair,base),report=PanelKit.check(plan);
            var kit=PanelKit.assembly(plan),apart=PanelKit.assembly(plan,true);
            if(pair.joint.id=="pins") {
                addPinHardware(kit,pair,base);
                var driver=Part.cylinderSpan(5,2*t+10,2*t+70,90,65),wrench=Part.cylinderSpan(8,-80,-10,90,65);
                var worldDriver=PosedParts.moved(driver,base),worldWrench=PosedParts.moved(wrench,base);driver.close();wrench.close();
                try PanelKit.checkToolAccess(plan,[{id:"bolt-driver",clearance:worldDriver},{id:"nut-wrench",clearance:worldWrench}])
                catch(e:Dynamic){worldDriver.close();worldWrench.close();throw e;}
                worldDriver.close();worldWrench.close();
            }
            assembled.include(pair.joint.id,kit);exploded.include(pair.joint.id,apart);
            writePanel(dir,pair.fixed);writePanel(dir,pair.moving);
            reports.push({id:pair.joint.id,report:report,closure:pair.joint.closure,
                panelFingerprints:[{id:pair.fixed.id,sha256:PanelCodec.fingerprint(pair.fixed)},{id:pair.moving.id,sha256:PanelCodec.fingerprint(pair.moving)}],
                note:"Blocking report covers panels after loose closure hardware is removed; threads are semantic."});
        }
        var coupon=PanelCoupon.make(stock,[0.1,0.2,0.3,0.4]);
        writePanel(dir,coupon.receiver,coupon.clearances);writePanel(dir,coupon.plug);
        sys.io.File.saveBytes(dir+"/panel-joinery.mtrg",SceneArtifact.encode(AssemblyPreview.scene(assembled,"panel-joinery")));
        sys.io.File.saveBytes(dir+"/panel-joinery-exploded.mtrg",SceneArtifact.encode(AssemblyPreview.scene(exploded,"panel-joinery-exploded")));
        var saved=assembled.encode();MachineAssembly.decode(saved).validate();
        sys.io.File.saveContent(dir+"/panel-joinery.assembly.json",saved);
        sys.io.File.saveContent(dir+"/review.json",Json.stringify({phase:2,stock:stock,panels:reports,couponClearances:coupon.clearances,
            qualification:"Geometry review and calibration specimens; physical fit, material and load qualification pending."},null,"\t")+"\n");
        Sys.println("Panel tooling review: "+dir);
    }
}
