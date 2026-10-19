import haxe.Json;
import cadkit.drawing.Svg;
import cadkit.drawing.Dxf;
import cadkit.drawing.Drawing2D;
import cadkit.drawing.DrawingEntity;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;
import machinekit.tooling.BoxMoldCad;
import machinekit.tooling.PanelDrawing;
import machinekit.tooling.ToolingPanel;
import machinekit.tooling.ToolingBlock;
import machinekit.component.ComponentValues;
import machinekit.assembly.AssemblyPreview;
import machinekit.assembly.MachineAssembly;
import materia.project.SceneArtifact;
import materia.tooling.casting.BoxMoldPlanner;
import materia.tooling.casting.BoxMoldCodec;
import materia.tooling.panel.PanelCodec;

class BoxMoldPreview {
    static function meshes(a:MachineAssembly,dir:String,prefix:String):Void {
        var rows:Array<Dynamic> = [],poses=a.solvedPoses();
        for(row in a.components()) {
            var mesh=AssemblyPreview.part(row.id,row.id,row.component.geometry(),row.component.materialId);
            var stem=prefix+"-"+row.id;
            sys.io.File.saveBytes(dir+"/"+stem+".vertices",mesh.vertices);
            sys.io.File.saveBytes(dir+"/"+stem+".indices",mesh.indices);
            rows.push({id:stem,pose:poses.get(row.id),color:[mesh.red,mesh.green,mesh.blue]});
        }
        sys.io.File.saveContent(dir+"/"+prefix+"-meshes.json",Json.stringify(rows));
    }
    public static function main():Void {
        var thickness=18.0,turn=false;
        for(arg in Sys.args()) {
            if(StringTools.startsWith(arg,"--thickness="))thickness=Std.parseFloat(arg.substr(12));
            if(arg=="--quarter-turn")turn=true;
        }
        var p=BoxMoldPlanner.plan(SinglePocketSetup.document(thickness,turn)),report=BoxMoldCad.review(p),cwd=Sys.getCwd();
        var dir=sys.FileSystem.exists(cwd+"/machinekit")?cwd+"/machinekit/examples/box-mold/build/review":cwd+"/build/review";
        sys.FileSystem.createDirectory(dir);
        var encoded=BoxMoldCodec.encode(p);BoxMoldCodec.decode(encoded);
        sys.io.File.saveContent(dir+"/mold.json",encoded);
        sys.io.File.saveContent(dir+"/review.json",Json.stringify({phase:3,fingerprint:BoxMoldCodec.fingerprint(p),geometry:report,
            qualification:"REVIEW ONLY: physical restraint, datum accuracy, anchorage, compaction, loads and handling pending",
            graphScope:"Assembly includes cores/inserts. Demolding begins after screw/clamp release and EPS breakout; cured casting includes steel. Straight translation sweeps only."},null,"\t")+"\n");
        for(m in p.panels.members) {
            var panel=m.panel,base=PanelDrawing.drawing(panel),entities=base.entities();
            entities.push(Label(Text,new Point2(0,-22),3,"REVIEW ONLY: physical qualification pending"));
            var drawing=new Drawing2D(entities);
            sys.io.File.saveContent(dir+"/"+panel.id+".svg",Svg.write(drawing));
            sys.io.File.saveContent(dir+"/"+panel.id+".dxf",Dxf.write(drawing));
            sys.io.File.saveContent(dir+"/"+panel.id+".panel.json",PanelCodec.encode(panel));
            var part=ToolingPanel.fromPlan(panel,"coated particleboard").geometry();
            part.shape.exportStep(dir+"/"+panel.id+".step");part.close();
        }
        for(c in p.cores) {
            var b=c.cutting.block,component=new ToolingBlock(new ComponentValues().setNumber("width",b.width).setNumber("depth",b.depth).setNumber("height",b.height).setToken("material","EPS tooling"));
            var part=component.geometry();part.shape.exportStep(dir+"/"+c.id+".step");part.close();
            sys.io.File.saveContent(dir+"/"+c.id+".cutting.json",Json.stringify(c,null,"\t")+"\n");
            var lines=[Curve(Outline,Segment.line(new Point2(0,0),new Point2(b.width,0))),Curve(Outline,Segment.line(new Point2(b.width,0),new Point2(b.width,b.height))),
                Curve(Outline,Segment.line(new Point2(b.width,b.height),new Point2(0,b.height))),Curve(Outline,Segment.line(new Point2(0,b.height),new Point2(0,0))),
                Label(Text,new Point2(0,b.height+8),3,c.id+" finished EPS profile; wire +Y, span "+b.depth+" mm; no kerf allowance"),
                Label(Text,new Point2(0,-8),3,"REVIEW ONLY; covering "+c.coveringThickness+" mm; restraint and thermal process pending")];
            var drawing=new Drawing2D(lines);sys.io.File.saveContent(dir+"/"+c.id+".svg",Svg.write(drawing));sys.io.File.saveContent(dir+"/"+c.id+".dxf",Dxf.write(drawing));
        }
        var casting=BoxMoldCad.reconstructed(p);casting.shape.exportStep(dir+"/reconstructed-casting.step");casting.close();
        var assembled=BoxMoldCad.assembly(p),exploded=BoxMoldCad.assembly(p,true),saved=assembled.encode();MachineAssembly.decode(saved).validate();
        meshes(assembled,dir,"assembled");meshes(exploded,dir,"exploded");
        sys.io.File.saveContent(dir+"/mold.assembly.json",saved);
        sys.io.File.saveBytes(dir+"/mold.mtrg",SceneArtifact.encode(AssemblyPreview.scene(assembled,"box-mold")));
        sys.io.File.saveBytes(dir+"/mold-exploded.mtrg",SceneArtifact.encode(AssemblyPreview.scene(exploded,"box-mold-exploded")));
        sys.io.File.saveContent(dir+"/procedure.md","# Box mold geometry review\n\nREVIEW ONLY. Physical qualification remains pending.\n\n"+
            "1. Measure stock and generate a fit coupon before manufacture. Seal cut edges and seams using the selected process.\n"+
            "2. Locate the insert independently against qualified datums; bolt through the bed with sealing washer/contact seal and plug the resin-facing bore end and protect unused threads. Nominal M6 hardware is a study envelope; thread engagement and finishing stock require confirmation.\n"+
            "3. Panel/core/insert installation order from continuous sweeps: "+report.assembly.assemblyOrder.join(" -> ")+". Apply external closure clamps and a qualified broad-area core restraint before filling.\n"+
            "4. Check resin/EPS/covering compatibility, cure limits, compaction and mold/bridge loads before pouring. These checks are pending.\n"+
            "5. After cure, remove external closures and the insert fixing screw from below. Break out the sacrificial EPS through the open fill face.\n"+
            "6. Geometric demolding order: "+report.demolding.removalOrder.join(" -> ")+". The bed can translate downward, or the released casting can lift upward. Actual lifting points and equipment require handling qualification.\n");
        Sys.println("Box mold review: "+dir);
    }
}
