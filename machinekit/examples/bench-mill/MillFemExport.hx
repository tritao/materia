import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import haxe.crypto.Sha256;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.PosedParts;
import machinekit.milling.*;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import sys.FileSystem;
import sys.io.File;

/** Posed geometry and physical datums only: no mesh, contacts or structural solution. */
class MillFemExport {
	public static inline var DIRECTORY = "machinekit/examples/bench-mill";

	public static function root():String {
		var path = FileSystem.fullPath(Sys.getCwd());
		while (!FileSystem.exists(path + "/machinekit/examples/bench-mill")) {
			var parent = haxe.io.Path.directory(path);
			if (parent == path || parent.length == 0) throw "Run the FEM exporter inside the materia repository";
			path = parent;
		}
		return path;
	}

	public static function build():{assembly:MachineAssembly, frame:MillFrame} {
		var reference = new BenchMill(false, false), layout = MillFramePreview.layout(reference);
		var z = MillFramePreview.zLayout(reference, layout);
		var xy = new MillXYStageLayout(layout, z.spindleY, BenchMill.RAIL);
		var frame = new MillFrame(layout, z, xy);
		return {assembly:MillFramePreview.concept(reference, frame), frame:frame};
	}

	public static function state(assembly:MachineAssembly, x:Float, y:Float, z:Float):AssemblyState {
		var model = new AssemblyModel("mm");
		assembly.addTo(model, "");
		var result = new AssemblyState(model.definition("mill-fem"));
		result.setJoint("xyStage/x", x);
		result.setJoint("xyStage/y", y);
		result.setJoint("zAxis/z", z);
		result.forwardKinematics();
		return result;
	}

	public static function datums(state:AssemblyState):Dynamic {
		var table = state.worldConnector("xyStage/table", "top");
		// Connector frames use mate conventions; their local Z is not the table's upward normal.
		var height = F.transformVector(state.worldPose("xyStage/table"),0,0,50);
		return {toolTipMm:state.worldConnector("spindle/tool", "tip"), tableTopMm:table,
			workpieceReferenceMm:F.translation(table.x+height.x,table.y+height.y,table.z+height.z),
			workpieceReferenceNote:"20 kg payload centre, assumed 50 mm above table; not a qualified cutting contact"};
	}

	public static function jointDirections(state:AssemblyState):Array<Dynamic> {
		var result:Array<Dynamic> = [];
		for (entry in [{joint:"xyStage/x",member:"xyStage/table"},{joint:"xyStage/y",member:"xyStage/table"},
			{joint:"zAxis/z",member:"spindle/tool"}]) {
			var original = state.joint(entry.joint), before = state.worldPose(entry.member);
			var joint = state.definition.joints.filter(j -> j.id == entry.joint)[0];
			var lower = joint.limits.lower, upper = joint.limits.upper;
			if (lower == null || upper == null) throw 'FEM joint direction needs finite travel limits: ${entry.joint}';
			var delta = Math.min(0.1,(upper-lower)/4) * (original < (lower+upper)/2 ? 1 : -1);
			state.setJoint(entry.joint,original+delta);
			var after = state.worldPose(entry.member);
			state.setJoint(entry.joint,original);
			result.push({joint:entry.joint,positiveDirectionWorld:{x:(after.x-before.x)/delta,
				y:(after.y-before.y)/delta,z:(after.z-before.z)/delta}});
		}
		return result;
	}

	static function exportSolid(id:String):Bool
		return StringTools.startsWith(id, "frame/") ||
			["xyStage/saddle", "xyStage/table", "zAxis/head", "zAxis/carriage"].indexOf(id) >= 0;

	public static function coordinate(text:String):Float {
		if (!~/^[+-]?([0-9]+(\.[0-9]*)?|\.[0-9]+)([eE][+-]?[0-9]+)?$/.match(text))
			throw "FEM coordinates must be numeric millimetres without a unit suffix";
		var value = Std.parseFloat(text);
		if (!Math.isFinite(value)) throw "FEM coordinate must be finite";
		return value;
	}

	static function command(root:String, args:Array<String>):String {
		var process = sys.io.Process.run("git", ["-C", root].concat(args));
		var output = process.readStdout(), code = process.exitCode();
		process.close();
		if (code != 0) throw 'Cannot obtain FEM source provenance: git ${args[0]}';
		return output;
	}

	static function ensure(path:String):Void {
		if (FileSystem.exists(path)) return;
		ensure(haxe.io.Path.directory(path));
		FileSystem.createDirectory(path);
	}

	static function sourceBuildProvenance(repository:String,folder:String):Dynamic {
		ensure(folder+"/provenance");
		var paths=[FileSystem.fullPath(Sys.programPath()),FileSystem.fullPath(Sys.executablePath())];
		var reader=sys.io.Process.run("cat",[FileSystem.fullPath("/proc/self/maps")]);
		var mappings=reader.readStdout(),status=reader.exitCode();reader.close();
		if(status!=0)throw "Cannot read loaded geometry-generation build identities";
		for(line in mappings.split("\n")) {
			var start=line.indexOf("/");
			if(start<0)continue;
			var path=line.substr(start);
			if((path.indexOf(".so")>=0 || path.indexOf(".hdll")>=0) && paths.indexOf(path)<0)paths.push(path);
		}
		var runtime=folder+"/provenance/runtime.json",file=folder+"/provenance/source-build.json";
		File.saveContent(runtime,haxe.Json.stringify(paths));
		var process=sys.io.Process.run("python3",[repository+"/"+DIRECTORY+"/fem/provenance.py",repository,runtime,file]);
		var output=process.readStdout(),code=process.exitCode();process.close();
		if(code!=0)throw 'Cannot capture source/build provenance: $output';
		return haxe.Json.parse(File.getContent(file));
	}


	public static function write(assembly:MachineAssembly, frame:MillFrame, state:AssemblyState, folder:String,?capturedProvenance:Dynamic,?conceptPath:String):Dynamic {
        var zLayout:MillZAxisLayout=frame.zAxis;
        var referencePath=conceptPath==null?root()+"/"+DIRECTORY+"/build/mill-frame-concept.json":conceptPath;
        var referenceText=File.getContent(referencePath);
		var repository = root();
		ensure(folder);
		var manifestPath = folder + "/manifest.json";
		// A failed export must not leave a previous manifest referring to partly replaced files.
		if (FileSystem.exists(manifestPath)) FileSystem.deleteFile(manifestPath);
        var exported=machinekit.structural.fem.AssemblyFemExport.bodies(assembly,state,folder,exportSolid);
        var parts=exported.parts,materials=exported.materials,totalMass=exported.totalMassKg,solidMass=exported.solidMassKg;
        var poses=assembly.solvedPoses(state),members=assembly.components(),interfaces:Array<Dynamic> = [],supports:Array<Dynamic> = [];
        PosedParts.scope(cache -> {
			for (foundation in [{id:"base",value:frame.base},{id:"column",value:frame.column}])
				for (insert in foundation.value.insertLocations()) {
					var casting = 'frame/${foundation.id}/casting', steel = 'frame/${foundation.id}/${insert.id}';
					var a = members.filter(m -> m.id == casting)[0], b = members.filter(m -> m.id == steel)[0];
					var overlap = cache.overlap(a.component,poses.get(casting),b.component,poses.get(steel));
					if (overlap > 1e-3) throw 'FEM casting/insert overlap at $steel: $overlap mm3';
					interfaces.push({kind:"cast-in", first:casting, second:steel, overlapMm3:overlap,
						connection:"unassigned", datumMm:state.worldConnector(steel,"top")});
				}
		});
		for (side in ["Left","Right"]) for (end in ["Front","Rear"]) {
			var corner = side + end, base = 'frame/base/column$corner', column = 'frame/column/foot$corner';
			interfaces.push({kind:"column-clamp", first:base, second:column, connection:"unassigned",
				datumMm:state.worldConnector(base,"top")});
			var foot = 'frame/levelingFoot$corner';
			supports.push({id:corner, member:foot, floorMm:state.worldConnector(foot,"floor"),
				seatMm:state.worldConnector('frame/base/foot$corner',"top"), restraint:"unassigned"});
		}
		var guides:Array<Dynamic> = [];
		for (group in [{axis:"x",ids:["xyStage/blockX0","xyStage/blockX1","xyStage/blockX2","xyStage/blockX3"]},
			{axis:"y",ids:["xyStage/blockY0","xyStage/blockY1","xyStage/blockY2","xyStage/blockY3"]},
			{axis:"z",ids:["zAxis/blockLeftLower","zAxis/blockLeftUpper","zAxis/blockRightLower","zAxis/blockRightUpper"]}])
			for (id in group.ids) {
				var block:machinekit.motion.LinearRailBlock = cast members.filter(m -> m.id == id)[0].component;
				guides.push({axis:group.axis,member:id,topMm:state.worldConnector(id,"top"),stiffness:"unassigned",
					railWidthMm:block.spec.railWidth,blockLengthMm:block.spec.blockLength});
			}
		materials.sort((a,b) -> Reflect.compare(a.id,b.id));
		var identity:Dynamic=capturedProvenance;
        if(identity==null)identity={revision:StringTools.trim(command(repository,["rev-parse","HEAD"])),
				trackedDiffSha256:Sha256.encode(command(repository,["diff","--binary","HEAD","--","machinekit","cadkit","projectkit","haxeon"])),
				exporterSha256:Sha256.encode(File.getContent(repository+"/"+DIRECTORY+"/MillFemExport.hx")),
				sourceBuild:sourceBuildProvenance(repository,folder)};
		var result = {schemaVersion:1, status:"geometry packet; not solver-ready", geometryFrame:"world", poseApplication:"already-applied",
			units:{length:"mm",force:"N",mass:"kg",time:"s",modulus:"Pa",density:"kg/m3"},
			provenance:identity,
			jointsMm:{x:state.joint("xyStage/x"),y:state.joint("xyStage/y"),z:state.joint("zAxis/z")},
			jointDirections:jointDirections(state),
			parts:parts, materials:materials, interfaces:interfaces, supports:supports, guides:guides,
            frameLayout:{wall:frame.layout.wall,roof:frame.layout.roof,baseHeight:frame.layout.baseHeight,
                baseWidth:frame.layout.baseWidth,columnWidth:frame.layout.columnWidth,columnDepth:frame.layout.columnDepth,
                columnHeight:frame.layout.columnHeight,columnCentreY:frame.layout.columnCentreY,railPlane:frame.layout.railPlane,
                front:frame.layout.front,columnFront:frame.layout.columnFront,footPockets:frame.layout.footPockets,zChannelBack:zLayout.channelBack,
                yRailSpacing:frame.layout.yRailSpacing,seatWidth:frame.layout.seatWidth,insertThickness:frame.layout.insertThickness},
			conceptReference:{file:"mill-frame-concept.json",sha256:Sha256.encode(referenceText)},
			datums:datums(state), totalComponentMassKg:totalMass, solidMassKg:solidMass,
			castingMassKg:{base:frame.base.castingByWeight().kg,column:frame.column.castingByWeight().kg},
			studyEnvelope:{cuttingForcePerAxisN:500,payloadKg:20,payloadHeightMm:50,accelerationMPerS2:1,
				gravityMPerS2:9.80665,cuttingContactAssigned:false},
			pending:["mesh and elastic material inputs including Poisson ratio", "guide and drive load-path stiffness", "insert bonds and column clamp contact/preload", "foot/floor restraints", "cutting contact and workpiece attachment"]};
		File.saveContent(manifestPath,haxe.Json.stringify(result,null,"  ")+"\n");
		File.saveContent(folder+"/mill-frame-concept.json",referenceText);
		return result;
	}

	public static function main():Void {
		var mill = build(), zLayout:MillZAxisLayout = mill.frame.zAxis;
		var name = "center", x = 0.0, y = 0.0, z = zLayout.lower / 2;
        var verifyRepeat=false, poseMatrix=false;
		var args = Sys.args(), i = 0;
		while (i < args.length) {
			var flag = args[i++];
			if (i >= args.length) throw 'Missing value for $flag';
			var value = args[i++];
			switch flag {
				case "--name": name = value;
				case "--x": x = coordinate(value);
				case "--y": y = coordinate(value);
				case "--z": z = coordinate(value);
                case "--pose-matrix":
                    if(value!="true" && value!="false")throw "--pose-matrix requires true or false";
                    poseMatrix=value=="true";
                case "--verify-repeat":
                    if(value!="true" && value!="false")throw "--verify-repeat requires true or false";
                    verifyRepeat=value=="true";
				default: throw 'Unknown FEM option $flag';
			}
		}
		if (!~/^[A-Za-z0-9_-]+$/.match(name)) throw "FEM packet name must contain letters, digits, underscores or hyphens";
		var folder = root()+"/"+DIRECTORY+"/build/fem/"+name;
		if(poseMatrix) {
            ensure(folder);
            MillFramePreview.writeReport(mill.assembly,mill.frame,folder,false);
            var entries:Array<Dynamic> =[{id:"center",x:0.0,y:0.0,z:zLayout.lower/2}];
            for(px in [-125.0,125.0])for(py in [-80.0,80.0])for(pz in [zLayout.lower,0.0])
                entries.push({id:"x"+(px<0?"minus":"plus")+"_y"+(py<0?"minus":"plus")+"_z"+(pz<0?"lower":"upper"),x:px,y:py,z:pz});
            for(entry in entries) {
                var target=folder+"/"+entry.id;
                var record=write(mill.assembly,mill.frame,state(mill.assembly,entry.x,entry.y,entry.z),target,null,folder+"/mill-frame-concept.json");
                Sys.println('FEM pose packet: $target/manifest.json');
            }
            File.saveContent(folder+"/poses.json",haxe.Json.stringify(entries,null,"  ")+"\n");
            return;
        }
        var repeatPath=folder+"/repeat-verification.json";
        if(FileSystem.exists(repeatPath))FileSystem.deleteFile(repeatPath);
        var pose=state(mill.assembly,x,y,z);
        ensure(folder);
        MillFramePreview.writeReport(mill.assembly,mill.frame,folder,false,{x:x,y:y,z:z});
        var conceptPath=folder+"/mill-frame-concept.json";
        var result = write(mill.assembly,mill.frame,pose,folder,null,conceptPath);
        if(verifyRepeat) {
            var before=File.getContent(folder+"/manifest.json");
            File.saveContent(folder+"/repeat-first-manifest.json",before);
            // Both exports execute the same loaded build; preserve its captured identity
            // even if another session changes unrelated working-tree files meanwhile.
            result=write(mill.assembly,mill.frame,pose,folder,result.provenance,conceptPath);
            var after=File.getContent(folder+"/manifest.json");
            if(before!=after)throw "Repeat export changed: inspect source/build provenance and geometry before qualifying the packet";
            File.saveContent(repeatPath,haxe.Json.stringify({passed:true,manifestSha256:Sha256.encode(after),
                sourceRevision:result.provenance.revision,method:"Two exports from the same loaded build and posed assembly; complete manifest and STEP hashes identical"},null,"  ")+"\n");
            Sys.println("Complete manifest and STEP hash repeat verification passed");
        }
		var parts:Array<Dynamic> = result.parts;
		Sys.println('FEM packet: $folder/manifest.json (${parts.length} components; no structural solution)');
	}
}
