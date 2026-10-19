import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import haxe.crypto.Sha256;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.PosedParts;
import machinekit.milling.*;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.MaterialLibrary;
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

	/** Preserve STEP geometry but fix the writer's volatile header timestamp. */
	static function stepText(text:String):String {
		var end = text.indexOf("DATA;");
		if (end < 0) throw "STEP export has no DATA section";
		var timestamp = ~/[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}/g;
		return timestamp.replace(text.substr(0, end), "1970-01-01T00:00:00") + text.substr(end);
	}

	public static function write(assembly:MachineAssembly, frame:MillFrame, state:AssemblyState, folder:String):Dynamic {
		var repository = root();
		ensure(folder);
		var manifestPath = folder + "/manifest.json";
		// A failed export must not leave a previous manifest referring to partly replaced files.
		if (FileSystem.exists(manifestPath)) FileSystem.deleteFile(manifestPath);
		var poses = assembly.solvedPoses(state), parts:Array<Dynamic> = [], interfaces:Array<Dynamic> = [];
		var supports:Array<Dynamic> = [], materials:Array<Dynamic> = [], materialIds:Array<String> = [];
		var members = assembly.components();
		members.sort((a,b) -> Reflect.compare(a.id,b.id));
		var totalMass = 0.0, solidMass = 0.0;
		PosedParts.scope(cache -> {
			for (member in members) {
				var id = member.id, component = member.component, pose = poses.get(id);
				if (pose == null) throw 'No FEM pose for $id';
				materia.assembly.AssemblyCodec.validateFrame(pose);
				var material = MaterialLibrary.require(component.materialId);
				if (materialIds.indexOf(material.id) < 0) {
					materialIds.push(material.id);
					materials.push({id:material.id, densityKgPerM3:material.physical.density,
						youngsModulusPa:material.physical.youngsModulusPa,
						propertiesAssumed:material.physical.propertiesAssumed == true,
						poissonRatio:null, source:"projectkit/src/materia/project/MaterialLibrary.hx"});
				}
				var mass = component.massProperties(), c = mass.centreOfMass;
				if (!Math.isFinite(mass.mass) || mass.mass <= 0) throw 'Invalid FEM mass: $id';
				var centre = F.transformPoint(pose,c.x,c.y,c.z);
				var solid = exportSolid(id), file:Null<String> = null, digest:Null<String> = null;
				var volume:Null<Float> = null, bounds:Dynamic = null;
				if (solid) {
					var part = cache.posed(component,pose);
					try {
						volume = part.volume();
						if (!part.valid() || part.solidCount() != 1 || !Math.isFinite(volume) || volume <= 0)
							throw 'FEM solid must be valid, connected and positive-volume: $id';
						bounds = PosedParts.boxOf(part);
						file = StringTools.replace(id,"/","__") + ".step";
						part.shape.exportStep(folder + "/" + file);
						var text = stepText(File.getContent(folder + "/" + file));
						File.saveContent(folder + "/" + file,text);
						digest = Sha256.encode(text);
					} catch (error:Dynamic) {part.close(); throw error;}
					part.close();
					solidMass += mass.mass;
				}
				totalMass += mass.mass;
				parts.push({id:id, materialId:material.id, representation:solid?"solid":"mass-only",
					poseMm:pose, massKg:mass.mass, centreOfMassMm:centre, volumeMm3:volume,
					boundsMm:bounds, stepFile:file, stepSha256:digest});
			}
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
			for (id in group.ids) guides.push({axis:group.axis,member:id,topMm:state.worldConnector(id,"top"),stiffness:"unassigned"});
		materials.sort((a,b) -> Reflect.compare(a.id,b.id));
		var result = {schemaVersion:1, status:"geometry packet; not solver-ready", geometryFrame:"world", poseApplication:"already-applied",
			units:{length:"mm",force:"N",mass:"kg",time:"s",modulus:"Pa",density:"kg/m3"},
			provenance:{revision:StringTools.trim(command(repository,["rev-parse","HEAD"])),
				trackedDiffSha256:Sha256.encode(command(repository,["diff","--binary","HEAD","--","machinekit","cadkit","projectkit","haxeon"])),
				exporterSha256:Sha256.encode(File.getContent(repository+"/"+DIRECTORY+"/MillFemExport.hx"))},
			jointsMm:{x:state.joint("xyStage/x"),y:state.joint("xyStage/y"),z:state.joint("zAxis/z")},
			jointDirections:jointDirections(state),
			parts:parts, materials:materials, interfaces:interfaces, supports:supports, guides:guides,
			datums:datums(state), totalComponentMassKg:totalMass, solidMassKg:solidMass,
			castingMassKg:{base:frame.base.castingByWeight().kg,column:frame.column.castingByWeight().kg},
			studyEnvelope:{cuttingForcePerAxisN:500,payloadKg:20,payloadHeightMm:50,accelerationMPerS2:1,
				gravityMPerS2:9.80665,cuttingContactAssigned:false},
			pending:["mesh and elastic material inputs including Poisson ratio", "guide and drive load-path stiffness", "insert bonds and column clamp contact/preload", "foot/floor restraints", "cutting contact and workpiece attachment", "mass-only component attachment and inertia for modal study"]};
		File.saveContent(manifestPath,haxe.Json.stringify(result,null,"  ")+"\n");
		return result;
	}

	public static function main():Void {
		var mill = build(), zLayout:MillZAxisLayout = mill.frame.zAxis;
		var name = "center", x = 0.0, y = 0.0, z = zLayout.lower / 2;
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
				default: throw 'Unknown FEM option $flag';
			}
		}
		if (!~/^[A-Za-z0-9_-]+$/.match(name)) throw "FEM packet name must contain letters, digits, underscores or hyphens";
		var folder = root()+"/"+DIRECTORY+"/build/fem/"+name;
		var result = write(mill.assembly,mill.frame,state(mill.assembly,x,y,z),folder);
		var parts:Array<Dynamic> = result.parts;
		Sys.println('FEM packet: $folder/manifest.json (${parts.length} components; no structural solution)');
	}
}
