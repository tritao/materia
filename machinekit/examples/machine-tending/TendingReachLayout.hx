import TendingCyclePlan.TendingTransferDoor;
import TendingCyclePlan.TendingTransferRequest;
import TendingCyclePlan.TendingTransferRole;
import TendingReach.TendingReachPose;
import TendingReach.TendingReachResult;
import TendingReach.TendingReachTransfer;
import TendingReach.TendingTrayLayout;
import haxe.Json;
import haxe.crypto.Sha256;
import haxe.io.Bytes;
import haxe.io.BytesBuffer;
import machinekit.assembly.AssemblyPreview;
import machinekit.robotics.CobotArm;
import machinekit.robotics.CobotClass;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact;
import robotkit.collision.CollisionClearance;

/**
 * The tending cell's certified reach layout, kept as data instead of solved on every cell build.
 *
 * `TendingReach.study` searches for the arm placement, poses and complete joint paths; that takes
 * minutes, and every build of the cell, its controllers and its mission used to repeat it. The
 * result is stored in `TendingReachCertificate` together with a fingerprint of everything the study
 * reads: the loaded mill and arm assemblies as scene artifacts, the production's transfer requests
 * and the study's clearance constants. A cell build checks that fingerprint and uses the stored
 * layout; when the inputs change it refuses and names the command that solves and stores it again.
 * Live planning still checks every executed edge against the actual scene.
 */
class TendingReachLayout {
	/** Bump when the study's acceptance rules change without any of its fingerprinted inputs changing. */
	public static inline var STUDY_VERSION:Int = 1;
	public static inline var FORMAT:String = "materia.tending-reach";
	public static inline var REGENERATE:String =
		"./haxeon/scripts/haxeon run --project=machinekit/examples/machine-tending/haxeon.json -- --store";

	/** The stored layout for `production`, after checking that the study's inputs are unchanged. */
	public static function certified(production:TendingCyclePlan):TendingReachResult {
		var root:Dynamic = Json.parse(TendingReachCertificate.JSON);
		var storedFingerprint:String = field(root, "fingerprint"), current = fingerprint(production);
		if (storedFingerprint != current)
			throw 'The tending reach layout was certified for other cell inputs ($storedFingerprint, now $current); ' +
				'solve and store it again with `$REGENERATE`';
		return decode(TendingReachCertificate.JSON).result;
	}

	/** A digest of every input `TendingReach.study` reads for `production`. */
	public static function fingerprint(production:TendingCyclePlan):String {
		var buffer = new BytesBuffer();
		function text(value:String):Void {
			var bytes = Bytes.ofString(value);
			buffer.addInt32(bytes.length);
			buffer.add(bytes);
		}
		text(FORMAT + "/" + STUDY_VERSION);
		// The study reads the loaded reference mill, whatever the cell's own vise holds.
		buffer.add(SceneArtifact.encode(AssemblyPreview.scene(new EnclosedBenchMill(), "reach-mill")));
		buffer.add(SceneArtifact.encode(AssemblyPreview.scene(TendingReach.armAssembly(
			new CobotArm(Reach1300, TendingReach.tendingTool())), "reach-arm")));
		text('${production.rows}x${production.columns}@${production.firstSlot}');
		for (request in production.transferRequests()) text(requestKey(request));
		text('margin=${CollisionClearance.MARGIN} ' + TendingReach.studyConstants());
		var digest = Sha256.make(buffer.getBytes()), hex = new StringBuf();
		for (index in 0...digest.length) hex.add(StringTools.hex(digest.get(index), 2).toLowerCase());
		return hex.toString();
	}

	public static function encode(result:TendingReachResult, fingerprint:String):String {
		var document = {
			format: FORMAT,
			fingerprint: fingerprint,
			cls: Std.string(result.cls),
			riser: result.riser,
			armPlacement: frame(result.armPlacement),
			armBaseFlange: frame(result.armBaseFlange),
			jointMargin: result.jointMargin,
			wristSine: result.wristSine,
			handlingApproachMm: result.handlingApproachMm,
			parkPose: result.parkPose,
			trayClearanceMm: result.trayClearanceMm,
			poses: [for (pose in result.poses) {name: pose.name, tool: pose.tool, position: pose.position,
				target: frame(pose.target), toolYaw: pose.toolYaw}],
			transfers: [for (transfer in result.transfers) {station: transfer.request.station, tool: transfer.request.tool,
				fromTool: transfer.request.fromTool, fromPose: transfer.request.fromPose, toPose: transfer.request.toPose,
				door: Std.string(transfer.request.door), role: Std.string(transfer.request.role), positions: transfer.positions}],
			trays: [for (tray in result.trays) {id: tray.id, frame: frame(tray.frame),
				slots: [for (slot in tray.slots) {name: slot.name, frame: frame(slot.frame)}],
				pitchX: tray.pitchX, pitchY: tray.pitchY, width: tray.width, depth: tray.depth}],
			tableFrame: frame(result.tableFrame),
			tableOffset: result.tableOffset,
			tableWidth: result.tableWidth,
			tableDepth: result.tableDepth,
			tableTop: result.tableTop
		};
		return Json.stringify(document, null, "\t");
	}

	public static function decode(json:String):{fingerprint:String, result:TendingReachResult} {
		var root:Dynamic = Json.parse(json);
		if (field(root, "format") != FORMAT) throw "Tending reach certificate has the wrong format";
		var result:TendingReachResult = {
			cls: cobotClass(field(root, "cls")),
			riser: number(root, "riser"),
			armPlacement: frameOf(field(root, "armPlacement")),
			armBaseFlange: frameOf(field(root, "armBaseFlange")),
			jointMargin: number(root, "jointMargin"),
			wristSine: number(root, "wristSine"),
			handlingApproachMm: number(root, "handlingApproachMm"),
			parkPose: numbers(field(root, "parkPose")),
			trayClearanceMm: number(root, "trayClearanceMm"),
			poses: [for (pose in list(root, "poses")) ({name: field(pose, "name"), tool: field(pose, "tool"),
				position: numbers(field(pose, "position")), target: frameOf(field(pose, "target")),
				toolYaw: number(pose, "toolYaw")} : TendingReachPose)],
			transfers: [for (transfer in list(root, "transfers")) ({request: {station: field(transfer, "station"),
				tool: field(transfer, "tool"), fromTool: field(transfer, "fromTool"), fromPose: field(transfer, "fromPose"),
				toPose: field(transfer, "toPose"), door: door(field(transfer, "door")), role: role(field(transfer, "role"))},
				positions: [for (position in (field(transfer, "positions") : Array<Dynamic>)) numbers(position)]}
				: TendingReachTransfer)],
			trays: [for (tray in list(root, "trays")) ({id: field(tray, "id"), frame: frameOf(field(tray, "frame")),
				slots: [for (slot in (field(tray, "slots") : Array<Dynamic>)) {name: field(slot, "name"),
					frame: frameOf(field(slot, "frame"))}],
				pitchX: number(tray, "pitchX"), pitchY: number(tray, "pitchY"), width: number(tray, "width"),
				depth: number(tray, "depth")} : TendingTrayLayout)],
			tableFrame: frameOf(field(root, "tableFrame")),
			tableOffset: number(root, "tableOffset"),
			tableWidth: number(root, "tableWidth"),
			tableDepth: number(root, "tableDepth"),
			tableTop: number(root, "tableTop")
		};
		var fingerprint:String = field(root, "fingerprint");
		return {fingerprint: fingerprint, result: result};
	}

	/** The certificate source for `result`: one string constant, so every build compiles the layout in. */
	public static function certificateSource(result:TendingReachResult, fingerprint:String):String {
		var json = encode(result, fingerprint);
		if (json.indexOf("'") >= 0 || json.indexOf("$") >= 0) throw "Tending reach certificate cannot quote its JSON";
		return "/** Generated by `" + REGENERATE + "`; see TendingReachLayout. Do not edit. */\n" +
			"class TendingReachCertificate {\n\tpublic static final JSON:String = '" + json + "';\n}\n";
	}

	static function requestKey(request:TendingTransferRequest):String
		return '${request.station}|${request.tool}|${request.fromTool}|${request.fromPose}|${request.toPose}|' +
			'${request.door}|${request.role}';

	static function frame(value:AssemblyFrame):Array<Float>
		return [value.x, value.y, value.z, value.qx, value.qy, value.qz, value.qw];

	static function frameOf(value:Dynamic):AssemblyFrame {
		var values = numbers(value);
		if (values.length != 7) throw "Tending reach certificate has a malformed frame";
		return {x: values[0], y: values[1], z: values[2], qx: values[3], qy: values[4], qz: values[5], qw: values[6]};
	}

	static function cobotClass(name:String):CobotClass {
		for (value in [Reach500, Reach850, Reach900, Reach1300]) if (Std.string(value) == name) return value;
		throw 'Tending reach certificate names an unknown cobot class "$name"';
	}

	static function door(name:String):TendingTransferDoor {
		for (value in [TendingTransferDoor.Open, TendingTransferDoor.Closed]) if (Std.string(value) == name) return value;
		throw 'Tending reach certificate names an unknown door state "$name"';
	}

	static function role(name:String):TendingTransferRole {
		for (value in [Inventory, TrayApproach, Startup, ViseEntry, Exchange]) if (Std.string(value) == name) return value;
		throw 'Tending reach certificate names an unknown transfer role "$name"';
	}

	static function field(record:Dynamic, name:String):Dynamic {
		if (record == null || !Reflect.hasField(record, name)) throw 'Tending reach certificate lacks "$name"';
		return Reflect.field(record, name);
	}

	static function list(record:Dynamic, name:String):Array<Dynamic> {
		var value:Dynamic = field(record, name);
		if (!Std.isOfType(value, Array)) throw 'Tending reach certificate field "$name" is not a list';
		return value;
	}

	static function number(record:Dynamic, name:String):Float {
		var value:Dynamic = field(record, name);
		if (!(Std.isOfType(value, Float) || Std.isOfType(value, Int))) throw 'Tending reach certificate field "$name" is not a number';
		return value;
	}

	static function numbers(value:Dynamic):Array<Float> {
		if (!Std.isOfType(value, Array)) throw "Tending reach certificate has a malformed number list";
		return [for (entry in (value : Array<Dynamic>))
			if (Std.isOfType(entry, Float) || Std.isOfType(entry, Int)) (entry : Float)
			else throw "Tending reach certificate has a malformed number list"];
	}
}
