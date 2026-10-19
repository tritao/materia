package machinekit.tooling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.PosedParts;
import machinekit.component.ComponentValues;
import machinekit.component.Solids;
import machinekit.structural.CastInsert;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.standard.FlatWasher;
import machinekit.standard.HexNut;
import materia.assembly.AssemblyFrames as F;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingMoldPlanner;
import materia.tooling.casting.CastingMoldCodec;
import materia.tooling.casting.MoldPlan.MoldBox;
import materia.tooling.casting.CoreSupport.SupportHardwareKind;
import machinekit.tooling.SweptRemoval.RemovalMember;
import machinekit.tooling.SweptRemoval.RemovalReport;

typedef CastingMoldReview = {
    var symmetricDifferenceMm3:Float;var sourceVolumeMm3:Float;var reconstructedVolumeMm3:Float;
    var assembly:RemovalReport;var demolding:RemovalReport;
    var permanentGroups:Array<{id:String,members:Array<String>}>;var assemblyGroups:Array<{id:String,members:Array<String>}>;
    var pending:Array<String>;var fabricationReady:Bool;
    var assemblyMotions:Array<{id:String,x:Float,y:Float,z:Float}>;var demoldingMotions:Array<{id:String,x:Float,y:Float,z:Float}>;
}

/** CAD bridge for neutral multi-core box plans. Frames and exact operations remain authoritative. */
class CastingMoldCad {
    static function box(b:MoldBox):Part return ToolingVolume.geometryOf({id:"temporary",material:FormerStock,pieces:[b]});
    public static function source(p:CastingMoldPlan):Part {
        var pieces:Array<Part> = [box(p.envelope)];
        return Solids.building(pieces,parts->{
            for(f in p.document.input.features)parts.push(box(CastingMoldPlanner.featureBox(p.document,f)));
            return Solids.cut(parts[0],parts.slice(1));
        });
    }
    public static function reconstructed(p:CastingMoldPlan):Part {
        var pieces:Array<Part> = [box(p.cavity)];
        return Solids.building(pieces,parts->{
            for(c in p.cores)parts.push(box(c.cavity));
            for(f in p.formers)parts.push(ToolingVolume.geometryOf(f.volume));
            for(f in p.fixtures)parts.push(ToolingVolume.geometryOf(f.datum));
            // Full envelopes include the protected insert bores; thread masks are a
            // declared release prerequisite, not permission for resin to enter bores.
            for(i in p.document.input.inserts)parts.push(box(insertBox(p,i.id)));
            return Solids.cut(parts[0],parts.slice(1));
        });
    }
    static function insertBox(p:CastingMoldPlan,id:String):MoldBox {
        var i=p.document.input.inserts.filter(i->i.id==id)[0];
        return materia.tooling.casting.BoxMoldPlanner.box(i.width,i.depth,i.height,F.compose(p.document.setup.sourceToMold,i.pose));
    }
    static function panelWorld(m:materia.tooling.panel.PanelAssemblyPlan.PanelPlacement):Part {
        var local=ToolingPanel.fromPlan(m.panel).geometry();
        try {var out=PosedParts.moved(local,m.pose);local.close();return out;}catch(e:Dynamic){local.close();throw e;}
    }
    public static function assembly(p:CastingMoldPlan,exploded:Bool=false):MachineAssembly {
        CastingMoldPlanner.validate(p);
        var a=exploded?new MachineAssembly():PanelKit.assembly(p.panels,false,"coated particleboard"),b=p.cavity;
        if(exploded)for(m in p.panels.members) {
            var pose=F.compose(F.identity(),m.pose);
            switch m.panel.id {case "bed":pose.z-=b.height*0.7;case "east":pose.x+=b.width;case "west":pose.x-=b.width;case "north":pose.y+=b.depth*0.4;case "south":pose.y-=b.depth*0.4;}
            a.addComponent(m.panel.id,ToolingPanel.fromPlan(m.panel,"coated particleboard"),pose);
        }
        for(i in 0...p.supportPanels.length) {
            var m=p.supportPanels[i],pose=F.compose(F.identity(),m.pose);if(exploded)pose.z+=b.height*(0.5+i*0.15);
            a.addComponent(m.panel.id,ToolingPanel.fromPlan(m.panel),pose);
        }
        for(i in 0...p.cores.length) {
            var c=p.cores[i],pose=F.identity();if(exploded)pose.z+=b.height*(0.8+i*0.12);
            a.addComponent(c.id,ToolingVolume.fromVolume({id:c.id,material:EPSStock,pieces:c.foamLayers}),pose);
        }
        for(f in p.formers) {
            var pose=F.identity();if(exploded){pose.x+=f.withdrawal.x*b.width*0.7;pose.y+=f.withdrawal.y*b.depth*0.4;pose.z+=f.withdrawal.z*b.height*0.7;}
            a.addComponent(f.volume.id,ToolingVolume.fromVolume(f.volume),pose);
        }
        for(f in p.fixtures) {
            var pose=F.identity();if(exploded){pose.x+=f.withdrawal.x*b.width;pose.z+=f.withdrawal.z*b.height*1.4;}
            a.addComponent(f.datum.id,ToolingVolume.fromVolume(f.datum),pose);
        }
        for(i in p.document.input.inserts) {
            var pose=F.compose(p.document.setup.sourceToMold,i.pose);if(exploded)pose.x+=b.width*1.8;
            var part=new CastInsert(new ComponentValues().setNumber("width",i.width).setNumber("depth",i.depth).setNumber("height",i.height).setNumber("boreDiameter",i.boreDiameter).setNumber("positionTolerance",i.positionTolerance).setToken("role",i.sourceRole));
            a.addComponent("insert-"+i.id,part,pose);
        }
        if(!exploded) {
            for(h in p.hardware) {
                if(h.kind==SupportHardwareKind.LocatingPin)a.addComponent(h.id,new ToolingPin(new ComponentValues().setNumber("diameter",h.diameter).setNumber("length",h.length)),h.pose);
                else {
                    var w=FlatWasher.metric("M6"),n=HexNut.metric("M6"),length=5*Math.ceil((h.length+2*w.thickness+n.height+1)/5);
                    a.addComponent(h.id,SocketHeadCapScrew.metric("M6",length),F.compose(h.pose,F.translation(0,0,w.thickness)));
                    a.addComponent(h.id+"-top-washer",w,h.pose);
                    a.addComponent(h.id+"-bottom-washer",FlatWasher.metric("M6"),F.compose(h.pose,F.translation(0,0,-h.length-w.thickness)));
                    a.addComponent(h.id+"-nut",n,F.compose(h.pose,F.translation(0,0,-h.length-w.thickness-n.height)));
                }
            }
            var poses=a.solvedPoses(),inverse=F.inverse(poses.get("bed"));
            for(row in a.components())if(p.panels.members.filter(m->m.panel.id==row.id).length==0) {
                a.addMemberConnector("bed",row.id+"-datum",F.compose(inverse,poses.get(row.id)));
                a.addMemberConnector(row.id,"tooling-datum",F.identity());a.addMate(row.id+"-positioned","fixed","bed",row.id+"-datum",row.id,"tooling-datum");
            }
        }
        return a;
    }
    static function coverage(p:CastingMoldPlan,tooling:Array<Part>):Void {
        var b=p.cavity,s=0.1,faces:Array<MoldBox> = [
            {x:b.x-s,y:b.y,z:b.z,width:s,depth:b.depth,height:b.height},
            {x:b.x+b.width,y:b.y,z:b.z,width:s,depth:b.depth,height:b.height},
            {x:b.x,y:b.y-s,z:b.z,width:b.width,depth:s,height:b.height},
            {x:b.x,y:b.y+b.depth,z:b.z,width:b.width,depth:s,height:b.height},
            {x:b.x,y:b.y,z:b.z-s,width:b.width,depth:b.depth,height:s}];
        for(index in 0...faces.length) {
            var slab=box(faces[index]);
            try {
                var uncovered=slab.subtractAll(tooling),volume=uncovered.volume();uncovered.close();
                if(volume>1e-3)throw "Uncovered mold boundary "+index+": "+volume+" mm3";
                slab.close();
            }catch(e:Dynamic){slab.close();throw e;}
        }
    }
    public static function review(p:CastingMoldPlan):CastingMoldReview {
        CastingMoldPlanner.validate(p);
        var references:Array<Part> = [],difference=0.0,ov=0.0,rv=0.0;
        try {
            var original=source(p);references.push(original);
            var rebuilt=reconstructed(p);references.push(rebuilt);
            var missing=original.subtractAll([rebuilt]);references.push(missing);
            var excess=rebuilt.subtractAll([original]);references.push(excess);
            difference=missing.volume()+excess.volume();ov=original.volume();rv=rebuilt.volume();
            if(difference>1e-3)throw "Casting reconstruction differs by "+difference+" mm3";
            for(part in references)part.close();references=[];
        }catch(e:Dynamic){for(part in references)if(!part.shape.isClosed())part.close();throw e;}
        var parts:Array<Part> = [],members:Array<RemovalMember> = [],permanent:Array<{id:String,members:Array<String>}> = [],groups:Array<{id:String,members:Array<String>}> = [];
        try {
            for(m in p.panels.members) {
                var shape=panelWorld(m);parts.push(shape);members.push({id:m.panel.id,part:shape,direction:new Vector(m.withdrawal.x,m.withdrawal.y,m.withdrawal.z)});
            }
            var rowIds:Array<String> = [];for(c in p.cores)if(rowIds.indexOf(c.bridgeId)<0)rowIds.push(c.bridgeId);
            for(id in rowIds) {
                var panelIds=[id].concat([for(c in p.cores)if(c.bridgeId==id)c.carrierId]),shapes:Array<Part> = [];
                var shape=Solids.building(shapes,owned->{
                    for(m in p.supportPanels)if(panelIds.indexOf(m.panel.id)>=0)owned.push(panelWorld(m));
                    return Solids.union(owned);
                });parts.push(shape);members.push({id:id+"-fixture",part:shape,direction:new Vector(0,0,1)});permanent.push({id:id+"-fixture",members:panelIds});
            }
            for(m in p.supportPanels)if(rowIds.indexOf(m.panel.id)<0 && p.cores.filter(c->c.carrierId==m.panel.id).length==0) {
                var shape=panelWorld(m);parts.push(shape);members.push({id:m.panel.id,part:shape,direction:new Vector(0,0,1)});
            }
            for(f in p.formers) {var shape=ToolingVolume.geometryOf(f.volume);parts.push(shape);members.push({id:f.volume.id,part:shape,direction:new Vector(f.withdrawal.x,f.withdrawal.y,f.withdrawal.z)});}
            for(f in p.fixtures) {var shape=ToolingVolume.geometryOf(f.datum);parts.push(shape);members.push({id:f.datum.id,part:shape,direction:new Vector(f.withdrawal.x,f.withdrawal.y,f.withdrawal.z)});}
            for(c in p.cores) {var shape=box(c.cavity);parts.push(shape);members.push({id:c.id,part:shape,direction:new Vector(0,0,1)});}
            coverage(p,parts);
            var castingPieces=[source(p)];for(i in p.document.input.inserts)castingPieces.push(box(insertBox(p,i.id)));
            var cured=Solids.union(castingPieces);parts.push(cured);members.push({id:"cured-casting",part:cured,direction:new Vector(0,0,1)});
            Sys.println("Checking full demolding sweeps for "+members.length+" members");
            var demoldingMotions=[for(m in members){id:m.id,x:m.direction.x,y:m.direction.y,z:m.direction.z}];
            var demolding=SweptRemoval.analyze(members,1e-4);members.pop();
            for(id in rowIds) {
                var ids=[id+"-fixture"].concat([for(c in p.cores)if(c.bridgeId==id)c.id]),pieces:Array<Part> = [];
                var n=members.length-1;while(n>=0){if(ids.indexOf(members[n].id)>=0){pieces.push(new Part(members[n].part.shape.cloneShape()));members.splice(n,1);}n--;}
                var shape=Solids.union(pieces);parts.push(shape);members.push({id:id+"-loaded",part:shape,direction:new Vector(0,0,1)});groups.push({id:id+"-loaded",members:ids});
            }
            for(i in p.document.input.inserts){var shape=box(insertBox(p,i.id));parts.push(shape);members.push({id:"insert-"+i.id,part:shape,direction:new Vector(0,0,1)});}
            members[0].direction=new Vector(0,0,1);
            // Before casting, bed-facing tooling can approach downward from the open
            // fill side. After cure those same pieces leave below the released bed.
            for(m in members)if(m.direction.z< -0.9)m.direction=new Vector(0,0,1);
            var assemblyMotions=[for(m in members){id:m.id,x:m.direction.x,y:m.direction.y,z:m.direction.z}];
            Sys.println("Checking full assembly sweeps for "+members.length+" members");
            var assembly=SweptRemoval.analyze(members,1e-4);
            for(part in parts)part.close();
            return {symmetricDifferenceMm3:difference,sourceVolumeMm3:ov,reconstructedVolumeMm3:rv,assembly:assembly,demolding:demolding,permanentGroups:permanent,assemblyGroups:groups,pending:p.pending.copy(),fabricationReady:false,assemblyMotions:assemblyMotions,demoldingMotions:demoldingMotions};
        }catch(e:Dynamic){for(part in parts)part.close();throw e;}
    }
}
