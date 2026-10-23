import machinekit.gantry.Gantry;
import machinekit.assembly.Transmission;
import machinekit.component.ComponentDetail;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyBodies;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.Part;
import machinekit.assembly.PosedParts;
import machinekit.assembly.PosedParts.PartBox;

/** OCCT checks across moving bodies; fixed-body contacts and declared drive meshes are interfaces. */
class GantryClearanceChecks {
	final localParts = new Map<String, Part>();
	final localBoxes = new Map<String, PartBox>();
	final body = new Map<String, String>();
	final interfaces = new Map<String, Bool>();
	final sensors = new Map<String, Bool>();
	final rackMeshes:Array<{pinion:String, rack:String}> = [];
	final shaftSeatings:Array<{parent:String, child:String}> = [];
    final chains:Array<{id:String,axis:String,chain:machinekit.motion.CableChain}> = [];
    final chainPositions:Map<String,Float> = [];
    final clearPairs:Map<String,Bool> = [];
    final clearMeshPhases:Map<String,Bool> = [];
    final shaftJoints:Map<String,String> = [];
    final pinions:Map<String,machinekit.component.MachineComponent> = [];

	public function new(gantry:Gantry, definition:AssemblyDefinition, ?seatedInterfaces:Array<{a:String,b:String}>) {
        if(seatedInterfaces!=null)for(pair in seatedInterfaces)allow(pair.a,pair.b);
		for (group in AssemblyBodies.of(definition)) for (member in group.occurrences) body.set(member, group.id);
        for(binding in gantry.cableChainBindings) {
            chains.push({id:binding.member,axis:binding.axis,chain:binding.chain});chainPositions.set(binding.member,0.0);
            body.set(binding.member,binding.member);
        }
		// A shaft is seated inside the motor or journal on its moving mate.
		for (joint in definition.joints) if (joint.type == materia.assembly.AssemblyDefinition.AssemblyJointType.Continuous) {
			shaftJoints.set(joint.child,joint.id);
            allow(joint.parent, joint.child); shaftSeatings.push({parent: joint.parent, child: joint.child});
		}
		var description = gantry.describe().machine;
		if (definition.switches != null) for (contact in definition.switches) sensors.set(contact.part, true);
		var transmissions = description.transmissions;
		if (transmissions != null) for (relation in transmissions) switch relation.source {
			case LeadScrew(screw, nut): allow(screw, nut);
			case TimingBelt(belt, pulley) | BeltIdler(belt, pulley): allow(belt, pulley);
			case RackAndPinion(pinion, rack):
				if (rack != null) { allow(pinion, rack); rackMeshes.push({pinion: pinion, rack: rack});pinions.set(pinion,gantry.component(pinion)); }
			case _: throw "Unexpected drive in the translational gantry";
		}
		var paths = description.beltPaths;
		if (paths != null) for (path in paths) {
			var clamp = path.clamp;
			if (clamp != null) allow(path.belt, clamp.instanceId);
		}
		try {
			for (member in gantry.components()) {
				if (!body.exists(member.id)) throw 'Gantry body ownership is missing for ${member.id}';
				var part = member.component.geometry(ComponentDetail.Preview);
				localParts.set(member.id, part);
				localBoxes.set(member.id, PosedParts.boxOf(part));
			}
		} catch (error:Dynamic) { close(); throw error; }
	}

	function allow(a:String, b:String):Void {
		interfaces.set(a + "|" + b, true); interfaces.set(b + "|" + a, true);
	}

	public function close():Void {
		for (part in localParts) if (!part.shape.isClosed()) part.close();
	}

	static function boxesOverlap(a:PartBox, b:PartBox):Bool
		return Math.min(a.maxX, b.maxX) - Math.max(a.minX, b.minX) > 1e-7 &&
			Math.min(a.maxY, b.maxY) - Math.max(a.minY, b.minY) > 1e-7 &&
			Math.min(a.maxZ, b.maxZ) - Math.max(a.minZ, b.minZ) > 1e-7;

	/** Members are moved to their poses only for pairs whose boxes, the local boxes carried to the poses, overlap. */
	public function check(state:AssemblyState):Void {
		var placed = new Map<String, Part>();
		var boxes = new Map<String, PartBox>();
		var ids:Array<String> = [];
		function dispose():Void { for (part in placed) if (!part.shape.isClosed()) part.close(); }
		function at(id:String):Part {
			var world = placed.get(id);
			if (world != null) return world;
			var part = localParts.get(id);
			if (part == null) throw "Gantry clearance member is missing";
			var moved = PosedParts.moved(part, state.worldPose(id));
			placed.set(id, moved);
			return moved;
		}
        function signature(a:String,b:String):String {
            var relative=materia.assembly.AssemblyFrames.compose(materia.assembly.AssemblyFrames.inverse(state.worldPose(a)),state.worldPose(b));
            var rotation=[relative.qx,relative.qy,relative.qz,relative.qw],largest=0;
            for(i in 1...4)if(Math.abs(rotation[i])>Math.abs(rotation[largest]))largest=i;
            if(rotation[largest]<0)rotation=[for(value in rotation)-value];
            var values=[relative.x,relative.y,relative.z].concat(rotation);
            return a+"|"+b+"|"+[for(value in values)Std.string(Math.ffloor(value*1e12+0.5))].join(",")+"|"+
                Std.string(chainPositions.get(a))+"|"+Std.string(chainPositions.get(b));
        }
		function overlap(a:String, b:String):Float {
			var first = at(a), second = at(b);
			var common = first.intersect(second);
			try { var volume = common.volume(); common.close(); return volume; }
			catch (error:Dynamic) { common.close(); throw error; }
		}
		try {
            for(binding in chains) {
                var position=state.joint(binding.axis);
                if(chainPositions.get(binding.id)!=position) {
                    var old=localParts.get(binding.id);if(old!=null)old.close();
                    var routed=binding.chain.geometryAt(position);localParts.set(binding.id,routed);localBoxes.set(binding.id,PosedParts.boxOf(routed));
                    chainPositions.set(binding.id,position);
                }
            }
			for (id in localParts.keys()) {
				var local = localBoxes.get(id);
				if (local == null) throw "Missing gantry geometry";
				boxes.set(id, PosedParts.carried(local, state.worldPose(id))); ids.push(id);
			}
			var clashes:Array<String> = [];
			for (i in 0...ids.length) for (j in i + 1...ids.length) {
				var a = ids[i], b = ids[j];
				// Separate sensor bodies cannot occupy the same space even when
				// both mounts belong to one fixed structural body.
				var sensorPair = sensors.exists(a) && sensors.exists(b);
				if ((!sensorPair && body.get(a) == body.get(b)) || interfaces.exists(a + "|" + b)) continue;
				var boxA = boxes.get(a), boxB = boxes.get(b);
				if (boxA == null || boxB == null) throw "Missing gantry bounds";
				if (!boxesOverlap(boxA, boxB)) continue;
				var key=signature(a,b);if(clearPairs.exists(key))continue;
				var volume = overlap(a, b);
                if(volume<=1e-3)clearPairs.set(key,true);
				if (volume > 1e-3) clashes.push('$a / $b: $volume mm³');
			}
			if (clashes.length > 0) throw "Gantry interference at " +
				[state.joint("x"), state.joint("y"), state.joint("z")].join(", ") + ":\n" + clashes.join("\n");
			for (pair in shaftSeatings) {
                var parentBox=boxes.get(pair.parent),childBox=boxes.get(pair.child);
                if(parentBox==null||childBox==null)throw "Missing shaft seating bounds";
                if(!boxesOverlap(parentBox,childBox))continue;
                var key=signature(pair.parent,pair.child);if(clearPairs.exists(key))continue;
				var volume = overlap(pair.parent, pair.child);
                if(volume<=1e-3)clearPairs.set(key,true);
				if (volume > 1e-3) throw 'Shaft does not fit ${pair.child}: $volume mm³';
			}
			// These teeth must actually mesh, unlike the belt's nominal band envelope.
			for (pair in rackMeshes) {
                // A rack repeats every pitch and a pinion every tooth. Reuse only
                // congruent relative poses; the separate tooth sweep covers the period.
                var part=pinions.get(pair.pinion);if(part==null)throw "Missing rack pinion";
                var wheel=machinekit.transmission.RackGeometry.pinion(part);
                var relative=materia.assembly.AssemblyFrames.compose(materia.assembly.AssemblyFrames.inverse(state.worldPose(pair.rack)),state.worldPose(pair.pinion));
                var joint=shaftJoints.get(pair.pinion);if(joint==null)throw "Rack pinion has no shaft joint";
                function periodic(value:Float,period:Float):Float {var phase=value-period*Math.ffloor(value/period);return phase<1e-9||period-phase<1e-9?0:phase;}
                var key=pair.pinion+"|"+[for(value in [relative.x,relative.y,periodic(relative.z,Math.PI*wheel.moduleSize),periodic(state.joint(joint),2*Math.PI/wheel.teeth)])Std.string(Math.ffloor(value*1e9+0.5))].join(",");
                if(clearMeshPhases.exists(key))continue;
				var volume = overlap(pair.pinion, pair.rack);
                if(volume<=0.05)clearMeshPhases.set(key,true);
				if (volume > 0.05) throw 'Rack/pinion tooth interference ${pair.pinion}: $volume mm³';
			}
			dispose();
		} catch (error:Dynamic) { dispose(); throw error; }
	}
}
