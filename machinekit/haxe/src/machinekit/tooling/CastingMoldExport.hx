package machinekit.tooling;

import haxe.Json;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingMoldCodec;
import materia.tooling.panel.PanelCodec;
import materia.tooling.panel.PanelContours;
import materia.tooling.panel.PanelGeometry;
import materia.tooling.casting.CoreSupports;
import machinekit.assembly.AssemblyPreview;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.PosedParts;
import materia.project.SceneArtifact;
import cadkit.drawing.Svg;
import cadkit.drawing.Dxf;
import cadkit.drawing.Drawing2D;
import cadkit.drawing.DrawingEntity;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;

/** Review outputs only. Existing CadKit writers render the exact panel operations. */
class CastingMoldExport {
    static function step(path:String,part:cadkit.modeling.Part):Void {
        try part.shape.exportStep(path)catch(e:Dynamic){part.close();throw e;}part.close();
    }
    static function text(path:String,value:String):Void sys.io.File.saveContent(path,value);
    static function data(path:String,value:Dynamic):Void text(path,Json.stringify(value,null,"\t")+"\n");
    public static function write(p:CastingMoldPlan,dir:String,review:machinekit.tooling.CastingMoldCad.CastingMoldReview):Void {
        sys.FileSystem.createDirectory(dir);
        var encoded=CastingMoldCodec.encode(p);CastingMoldCodec.decode(encoded);text(dir+"/mold-plan.json",encoded);
        data(dir+"/review.json",{phase:4,fingerprint:CastingMoldCodec.fingerprint(p),geometry:review,changeRequests:p.changeRequests,
            limitations:"REVIEW ONLY. Datum fixtures require qualified attachments; bores are protected envelopes in removal checks. Release fasteners, masks and adhesive before the corresponding straight withdrawals. Compaction, anchorage, precision replication/grouting, loads and handling remain pending."});
        data(dir+"/feature-inventory.json",p.features);data(dir+"/design-change-requests.json",p.changeRequests);
        data(dir+"/process-options.json",p.options);
        for(m in p.panels.members.concat(p.supportPanels)) {
            var base=PanelDrawing.drawing(m.panel),entities=base.entities();entities.push(Label(Text,new Point2(0,-22),3,"REVIEW ONLY: geometry, pending physical qualification"));
            var drawing=new Drawing2D(entities);text(dir+"/"+m.panel.id+".svg",Svg.write(drawing));text(dir+"/"+m.panel.id+".dxf",Dxf.write(drawing));
            text(dir+"/"+m.panel.id+".panel.json",PanelCodec.encode(m.panel));
            var part=ToolingPanel.fromPlan(m.panel).geometry();step(dir+"/"+m.panel.id+".step",part);
        }
        for(c in p.cores)for(index in 0...c.foamLayers.length) {
            var b=c.foamLayers[index],id=c.id+"-layer-"+index,cutting=CoreSupports.cutting(b);
            data(dir+"/"+id+".cutting.json",{featureId:c.featureId,coreId:c.id,cutting:cutting,adhesiveThickness:c.adhesiveThickness,coveringThickness:c.coveringThickness});
            var local=cutting.block,part=ToolingVolume.geometryOf({id:id,material:EPSStock,pieces:[local]});step(dir+"/"+id+".step",part);
            var drawing=new Drawing2D([Curve(Outline,Segment.line(new Point2(0,0),new Point2(b.width,0))),Curve(Outline,Segment.line(new Point2(b.width,0),new Point2(b.width,b.height))),
                Curve(Outline,Segment.line(new Point2(b.width,b.height),new Point2(0,b.height))),Curve(Outline,Segment.line(new Point2(0,b.height),new Point2(0,0))),
                Label(Text,new Point2(0,b.height+8),3,id+" wire +Y; span "+b.depth+" mm; no kerf compensation"),Label(Text,new Point2(0,-8),3,"REVIEW ONLY: EPS grade and adhesive unqualified")]);
            text(dir+"/"+id+".svg",Svg.write(drawing));text(dir+"/"+id+".dxf",Dxf.write(drawing));
        }
        for(f in p.formers) {
            var part=ToolingVolume.geometryOf(f.volume);step(dir+"/"+f.volume.id+".step",part);
            text(dir+"/"+f.volume.id+".volume.json",materia.tooling.casting.MoldVolumeCodec.encode(f.volume));
        }
        for(f in p.fixtures) {
            var part=ToolingVolume.geometryOf(f.datum);step(dir+"/"+f.datum.id+".step",part);
            text(dir+"/"+f.datum.id+".volume.json",materia.tooling.casting.MoldVolumeCodec.encode(f.datum));
        }
        var casting=CastingMoldCad.reconstructed(p);step(dir+"/reconstructed-casting.step",casting);
        var assembled=CastingMoldCad.assembly(p),exploded=CastingMoldCad.assembly(p,true),saved=assembled.encode();
        var restored=MachineAssembly.decode(saved);restored.validate();if(restored.encode()!=saved)throw "Assembly recipe did not restore exactly";
        text(dir+"/mold.assembly.json",saved);
        scene(dir,"mold",assembled);scene(dir,"mold-exploded",exploded);
        var coreList=[for(c in p.cores)c.id+": "+c.foamLayers.length+" layers, carrier "+c.carrierId+", shared "+c.bridgeId];
        text(dir+"/procedure.md","# Casting mold review — "+p.document.input.id+"\n\nREVIEW ONLY. Not approved for fabrication.\n\n"+
            "1. Qualify stock, EPS, adhesive/covering/release compatibility, compaction, closure and brace loads before pouring.\n"+
            "2. Resolve all design-change requests, especially fixture attachments, precision allowances and lifting/flip points. Datum fixtures place inserts nominally; they do not establish final precision.\n"+
            "3. Cut/laminate EPS with the specified interlayer glue lines. Permanently bond carriers to each shared bridge; prebond the row's cores to that fixture.\n\n"+coreList.join("\n")+"\n\n"+
            "4. Geometric installation order: "+review.assembly.assemblyOrder.join(" -> ")+". Install pins and clamp bolts afterwards. Clamp support rails and outer walls with qualified external closures.\n"+
            "5. Independently locate and secure every insert to its datum fixture. Resolve pending temporary threads/clamps and mask/protect bores. Seams and square router-corner cleanup need a declared process.\n"+
            "6. After cure, release loose hardware, masks and EPS/carrier adhesive as applicable. Geometric removal order: "+review.demolding.removalOrder.join(" -> ")+". These are full straight-translation sweeps; mechanical release and EPS breakup are manual prerequisites.\n"+
            "7. Lift/flip only with qualified points/equipment. Replicate the designated rail/bearing seats against qualified masters; grout the column joint using its approved gap, jacking and cure-before-torque process. These allowances and processes are pending, and source geometry was not altered.\n");
    }
    static function scene(dir:String,id:String,a:MachineAssembly):Void {
        sys.io.File.saveBytes(dir+"/"+id+".mtrg",SceneArtifact.encode(AssemblyPreview.scene(a,id)));
        var rows:Array<Dynamic> = [],poses=a.solvedPoses();
        for(row in a.components()) {
            var mesh=AssemblyPreview.part(row.id,row.id,row.component.geometry(),row.component.materialId),stem=id+"-"+row.id;
            sys.io.File.saveBytes(dir+"/"+stem+".vertices",mesh.vertices);sys.io.File.saveBytes(dir+"/"+stem+".indices",mesh.indices);
            rows.push({id:stem,pose:poses.get(row.id),color:[mesh.red,mesh.green,mesh.blue]});
        }
        data(dir+"/"+id+"-meshes.json",rows);
    }
}
