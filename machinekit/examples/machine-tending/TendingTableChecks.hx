import machinekit.structural.TSlotTable;
import machinekit.structural.TSlotTableHardware;
import machinekit.structural.TSlotTableParts.TableAngle;
import machinekit.structural.TSlotTableParts.TableTop;
import machinekit.structural.TSlotTableParts.TableSlotNut;
import machinekit.structural.LevelingFoot;
import machinekit.structural.TSlotExtrusion;
import machinekit.component.ComponentDetail;
import machinekit.assembly.AssemblyPreview;
import machinekit.assembly.MachineAssembly;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import cadkit.modeling.AssemblyState;

/** Size-independent construction, hardware fit, machining and preview integration checks. */
class TendingTableChecks {
	static var assertions = 0;
	static function check(ok:Bool,message:String):Void { assertions++; if (!ok) throw message; }
	static function near(a:Float,b:Float,message:String):Void check(Math.abs(a-b)<0.001,message+': $a != $b');
	static function rejects(build:Void->Void,message:String):Void {
		var failed=false;
		try build() catch (_:Dynamic) failed=true;
		check(failed,message);
	}

	static function placed(table:TSlotTable,id:String):Part {
		id=table.partPath(id);
		var matches=table.components().filter(entry -> entry.id==id);
		check(matches.length==1,'Unique assembly part $id');
		var part=matches[0].component.geometry(Preview);
		var f=new AssemblyState(table.definition()).worldPose(id);
		var x=AssemblyFrames.transformVector(f,1,0,0), z=AssemblyFrames.transformVector(f,0,0,1);
		var result=part.placed(new Location(new Plane(new Vector(f.x,f.y,f.z),new Vector(x.x,x.y,x.z),new Vector(z.x,z.y,z.z))));
		part.close(); return result;
	}
	static function noClash(table:TSlotTable,a:String,b:String,contact:Bool=false):Void {
		var first=placed(table,a), second=placed(table,b);
		var intersection=first.intersect(second);
		near(intersection.volume(),0,'Mating CAD has no penetration: $a / $b');
		if (contact) near(first.distance(second),0,'Mating CAD seats against its bearing face: $a / $b');
		intersection.close(); first.close(); second.close();
	}

	static function verify(table:TSlotTable):Void {
		var s=table.profile.size, h=table.hardware;
		var baseCuts=table.memberCuts().filter(cut -> StringTools.startsWith(cut.name,"base-"));
		var crossCuts=baseCuts.filter(cut -> StringTools.startsWith(cut.name,"base-cross-"));
		var angleCount=16+(table.hasOutriggers ? 4+2*crossCuts.length : 0);
		check(table.memberCuts().length==12+baseCuts.length,"Upper frame and independently sized base rails");
		near(table.cutList()[0].totalLength,4*(table.height-table.topThickness-table.postBottom)+
			4*(table.width-2*s)+4*(table.depth-table.frontInset-2*s)+(table.hasOutriggers ?
			2*table.baseDepth+crossCuts.length*(table.baseWidth-2*s) : 0),"Butt-joint cut stock");
		check(table.cutList().length==1 && table.cutList()[0].quantity==12+baseCuts.length,"Profile stock aggregation");
		var components=table.components(), bolts=0, nuts=0, feet=0, angles=0;
		check(table.subassemblies().length==(4+(table.hasOutriggers ? 1 : 0)+(table.rearBrace ? 1 : 0)),"Semantic table subassemblies");
		for (entry in components) check(entry.id.indexOf("/")>0,"Every table part has assembly ownership");
		var connectionParts=0;
		for (occurrence in table.definition().occurrences)
			if (occurrence.includePath=="upper-frame/connections/upper-bracket-left-front-0") connectionParts++;
		check(connectionParts==5,"Connection owns its angle and four fastener parts");
		var bom=table.billOfMaterials();
		var total=0;
		for (line in bom.lines()) total+=line.quantity;
		check(total==components.length,"Every placed part is counted once in the BOM");
		for (entry in components) {
			if (StringTools.endsWith(entry.id,"-screw")) bolts++;
			if (StringTools.endsWith(entry.id,"-nut")) nuts++;
			if (Std.isOfType(entry.component,LevelingFoot)) {
				feet++;
				var foot:LevelingFoot = cast entry.component;
				switch foot.specification() {
					case Foot(_, _, thread, _, _, mount, ThreadRoot, NoCollar):
						near(thread.majorDiameter,h.footSpec().diameter,"Source foot thread diameter");
						near(thread.pitch,h.footSpec().pitch,"Source foot thread pitch");
						near(mount,table.footHeight,"Source foot mounting datum");
					default: throw "T-slot table must use the shared thread-root foot variant";
				}
			}
			if (Std.isOfType(entry.component,TableAngle)) angles++;
		}
		check(bolts==2*angleCount+4+(table.rearBrace ? 2 : 0)+(table.hasOutriggers ? 8 : 0) && bolts==nuts,"Two fasteners per bracket, four top fasteners and optional brace pair");
		check(feet==4 && angles==angleCount,"Four threaded feet and sixteen drilled angles");
		var state=new AssemblyState(table.definition());
		for (entry in components) if (StringTools.endsWith(entry.id,"-screw")) {
			var nutId=entry.id.substr(0,entry.id.length-6)+"-nut";
			var head=state.worldPose(entry.id), nut=state.worldPose(nutId);
			var local=AssemblyFrames.transformPoint(AssemblyFrames.inverse(head),nut.x,nut.y,nut.z);
			near(local.x,0,"Nut and screw share an axis"); near(local.y,0,"Nut and screw share an axis");
			check(local.z<0,"Nut is behind the bearing face");
		}
		check(components.length==12+baseCuts.length+angleCount+4+4+1+2*bolts+(table.rearBrace ? 1 : 0)+(table.hasOutriggers ? 4 : 0),"Complete hardware instances");
		check(table.collisionPieces().length==components.length+angleCount,"Collision envelopes cover all parts and both angle arms");
		for (piece in table.collisionPieces()) {
			check(piece.vertices.length==24,"Box hull vertices");
			for (i in 0...8) {
				var p=AssemblyFrames.transformPoint(piece.frame,piece.vertices[i*3],piece.vertices[i*3+1],piece.vertices[i*3+2]);
				var rearAllowance=table.rearBrace ? h.bracketThickness+h.screwSpec().headHeight : 0;
				check(p.x>=Math.min(-table.width/2,table.baseOffsetX-table.baseWidth/2)-0.001 &&
					p.x<=Math.max(table.width/2,table.baseOffsetX+table.baseWidth/2)+0.001 &&
					p.y>=Math.min(-table.depth/2,table.baseOffsetY-table.baseDepth/2)-0.001 &&
					p.y<=Math.max(table.depth/2+rearAllowance,table.baseOffsetY+table.baseDepth/2)+0.001 &&
					p.z>=-0.001 && p.z<=table.height+0.001,
					'Placed collision envelope stays within construction bounds: ${piece.id}');
			}
		}
		// Exercise each unique manufactured definition, not every copy of the same bolt.
		var seen:Map<String,Bool> = [];
		for (entry in components) if (!seen.exists(entry.component.designation)) {
			seen.set(entry.component.designation,true);
			var part=entry.component.geometry(Preview), volume=part.volume();
			check(volume>0,'Positive CAD volume: ${entry.id}');
			if (Std.isOfType(entry.component,TableAngle)) {
				var angle:TableAngle=cast entry.component;
				near(volume,(2*s*angle.width*angle.thickness-angle.width*angle.thickness*angle.thickness)-
					2*Math.PI*Math.pow(angle.hole/2,2)*angle.thickness,"Both angle mounting holes cut through the plates");
			}
			if (Std.isOfType(entry.component,TableTop)) {
				var expected=table.width*table.depth*table.topThickness;
				for (m in table.topMounts()) expected-=Math.PI*(m.clearance*m.clearance*table.topThickness+
					(m.counterbore*m.counterbore-m.clearance*m.clearance)*m.depth)/4;
				near(volume,expected,"Four through holes with flush screw counterbores");
			}
			if (Std.isOfType(entry.component,TableSlotNut))
				near(volume,(h.nutLength*h.nutWidth-Math.PI*Math.pow(h.screwSpec().diameter/2,2))*h.nutThickness,"Slot nut is threaded through");
			part.close();
		}
		var adjustment=table.footAdjustment();
		check(adjustment.minimum<adjustment.nominal && adjustment.maximum>adjustment.nominal,"Foot adjustment retains tapped engagement");
		var bracket="lower-bracket--1--1-0";
		noClash(table,bracket+"-leg-screw",bracket,true);
		noClash(table,bracket+"-rail-screw",bracket,true);
		noClash(table,bracket+"-leg-nut","leg--1--1",true);
		noClash(table,bracket+"-rail-nut","lower-cross--1",true);
		noClash(table,bracket+"-rail-screw","lower-cross--1");
		noClash(table,bracket+"-rail-screw",bracket+"-rail-nut");
		noClash(table,"top-mount--1--1-screw","top",true);
		noClash(table,"top-mount--1--1-nut","upper-cross--1",true);
		if (!table.hasOutriggers) noClash(table,"foot--1--1","leg--1--1");
		else {
			noClash(table,"foot--1--1","foot-plate--1--1");
			noClash(table,"foot-locknut--1--1","foot-plate--1--1",true);
			noClash(table,"foot-plate--1--1","base-side--1",true);
			noClash(table,"foot-plate--1--1-mount--1-screw","foot-plate--1--1",true);
			noClash(table,"foot-plate--1--1-mount--1-nut","base-side--1",true);
			for (cut in crossCuts) for (ix in [-1,1]) {
				var joint='base-joint-${cut.name.substr("base-cross-".length)}-$ix';
				noClash(table,joint+"-leg-nut",'base-side-$ix',true);
				noClash(table,joint+"-rail-nut",cut.name,true);
			}
			for (ix in [-1,1]) for (iy in [-1,1]) {
				var joint='post-base-$ix-$iy';
				noClash(table,joint+"-leg-nut",'leg-$ix-$iy',true);
				noClash(table,joint+"-rail-nut",iy<0 ? "base-cross-support-front" : "base-cross-support-back",true);
			}
		}
		if (table.rearBrace) {
			noClash(table,"rear-brace-0-screw","rear-brace",true);
			noClash(table,"rear-brace-0-nut","leg--1-1",true);
		}
		check(table.machining().length==4,"Foot thread machining instructions");
		for (op in table.machining()) check(op.thread==h.footThread && op.depth>h.footEngagement,"Tapped support matches foot thread");
		var mounts=table.topMounts();
		check(mounts.length==4,"Top machining pattern");
		for (m in mounts) check(m.depth>=h.screwSpec().headHeight && table.topThickness-m.depth>=2,"Recess head while retaining plate thickness");
		mounts[0].x=1e9;
		check(table.topMounts()[0].x!=1e9,"Machining data is an owned copy");
	}

	public static function main():Void {
		near(TSlotExtrusion.forProfile("HFS5-2020").modeledSlotDepth(),3.5,"2020 modeled corner web");
		near(TSlotExtrusion.forProfile("HFS5-4040").modeledSlotDepth(),6,"4040 modeled slot");
		var tables=[
			new TSlotTable(320,240,650,"HFS5-2020",10,0,25,150),
			new TSlotTable(540,330,1020,"HFS5-4040",10,100),
			new TSlotTable(1200,700,900,"HFS5-4040",18,0,40,200,null,true),
			new TSlotTable(600,400,750,"HFS5-2020",12,40,35,180,null,true),
			new TSlotTable(540,330,1020,"HFS5-4040",10,100,30,220,null,true,
				{width:720,depth:670,offsetX:90,offsetY:-170}),
			new TSlotTable(320,240,650,"HFS5-2020",10,0,25,150,null,true,
				{width:500,depth:450})
		];
		for (table in tables) verify(table);
		// A supplied hardware specification follows the same fitting rules as a preset.
		var custom=new TSlotTableHardware("M3",2.7,9,12,1.6,"M5",10);
		verify(new TSlotTable(400,300,700,"HFS5-2020",10,0,30,150,custom));
		rejects(() -> { new TSlotTable(100,100,200); },"Reject undersized frame");
		rejects(() -> { new TSlotTable(320,240,650,"HFS5-2020",3); },"Reject thin counterbored top");
		rejects(() -> { new TSlotTable(320,240,650,"HFS5-2020",10,0,10); },"Reject foot without locknut access");
		rejects(() -> { new TSlotTable(320,240,650,"HFS5-2040"); },"Reject unsupported nonsquare profile");
		rejects(() -> { new TSlotTableHardware("M8",6,9,12,1.6,"M5",10).validate(TSlotExtrusion.forProfile("HFS5-2020")); },"Reject oversized bolt");
		rejects(() -> { new TSlotTableHardware("M5",4,5,12,1.6,"M5",10).validate(TSlotExtrusion.forProfile("HFS5-2020")); },"Reject nut that pulls through slot");
		rejects(() -> { new TSlotTableHardware("M5",4,9,12,3,"M5",10).validate(TSlotExtrusion.forProfile("HFS5-2020")); },"Reject nut too thick for undercut");
		rejects(() -> { new TSlotTableHardware("M5",7,9,12,1.6,"M5",10).validate(TSlotExtrusion.forProfile("HFS5-2020")); },"Reject blocked angle driver access");
		rejects(() -> { new TSlotTable(540,330,1020,"HFS5-4040",10,100,30,220,null,true,
			{width:400,depth:600}); },"Reject base narrower than supported upper frame");
		rejects(() -> { new TSlotTable(540,330,1020,"HFS5-4040",10,100,30,220,null,true,
			{width:720,depth:670,offsetX:200}); },"Reject offset that leaves a post outside base");
		var layout=TendingReachLayout.decode(TendingReachCertificate.JSON).result;
		var banks=new TendingCellTable(layout.tableWidth,layout.tableDepth,layout.tableTop,layout.tableOffset);
		check(banks.definition().occurrences.length==378,"Both wide braced banks flatten into cell");
		check(banks.collisionPieces().length==430,"Both base collision sets include all connections");
		for (i in 0...banks.tables.length) {
			var table=banks.tables[i], sign=i==0 ? -1 : 1;
			near(table.baseOffsetY+table.baseDepth/2,table.depth/2,"Rear base edge remains flush with top");
			near(sign*table.baseOffsetX-table.baseWidth/2,-table.width/2,"Widening stays outside robot corridor");
			check(table.baseWidth>=0.7*table.height && table.baseDepth>=0.65*table.height,"Footprint scales with support height");
			check(table.rearBrace && table.hasOutriggers,"Both banks have bracing and wide bases");
			for (piece in table.collisionPieces()) for (v in 0...8) {
				var p=AssemblyFrames.transformPoint(piece.frame,piece.vertices[3*v],piece.vertices[3*v+1],piece.vertices[3*v+2]);
				if (piece.id!=table.partPath("top") && p.y < -table.depth/2+100-0.001)
					check(p.z<=table.postBottom+0.001,"Only low base members extend into the front overhang");
			}
		}
		var root=Sys.getCwd();
		while (!sys.FileSystem.exists(root+"/machinekit/examples/machine-tending")) {
			var parent=haxe.io.Path.directory(root);
			if (parent==root || parent.length==0) throw "Could not locate repository";
			root=parent;
		}
		var output=root+"/machinekit/examples/machine-tending/build/";
		sys.io.File.saveBytes(output+"tslot-tables.mtrg",SceneArtifact.encode(AssemblyPreview.scene(banks,"tslot-tray-tables")));
		var sizing=new MachineAssembly();
		for (i in 0...tables.length) sizing.include('size-$i',tables[i],AssemblyFrames.translation(i*2000,0,0));
		sys.io.File.saveBytes(output+"tslot-table-sizing.mtrg",SceneArtifact.encode(AssemblyPreview.scene(sizing,"tslot-table-sizing")));
		sys.io.File.saveContent(output+"tslot-table-manufacturing.json",haxe.Json.stringify([for (table in tables)
			{profile:table.profile.designation,width:table.width,depth:table.depth,height:table.height,
			 base:{width:table.baseWidth,depth:table.baseDepth,offsetX:table.baseOffsetX,offsetY:table.baseOffsetY},
			 cuts:table.memberCuts(),topHoles:table.topMounts(),footMountMachining:table.machining(),footAdjustment:table.footAdjustment(),bom:table.billOfMaterials().lines()}],null,"  "));
		Sys.println('TendingTableChecks: $assertions assertions passed; seven size/footprint/hardware variants');
	}
}
