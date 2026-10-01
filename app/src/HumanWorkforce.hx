package app;

import animkit.AnimationAsset;
import app.editor.RobotLinkBounds;
import app.editor.WorkerAssetPath;
import app.editor.WorkerSceneTargets;
import humankit.HumanBodyProxy;
import humankit.HumanCharacter;
import humankit.HumanDescription;
import humankit.HumanJob;
import humankit.HumanJobSpec;
import humankit.rig.HumanoidRig;
import humankit.action.Wait;
import humankit.sim.HumanWorker;
import humankit.sim.HumanWorkerSignals;
import humankit.sim.HumanZone;
import nativekit.scene.Transform;
import nativekit.sim.SimObject;
import nativekit.sim.SimPose;
import nativekit.sim.SimSession;
import robotkit.model.RobotModel;
import robotkit.runtime.Simulation;

/** One worker with the character and asset it animates; the worker is set once it is built. */
class WorkforceEntry {
	public final id:String;
	public var worker:Null<HumanWorker>;
	public final character:HumanCharacter;
	public final asset:AnimationAsset;

	public function new(id:String, character:HumanCharacter, asset:AnimationAsset) {
		this.id = id;
		this.character = character;
		this.asset = asset;
	}
}

/**
 * Every authored worker of one simulation session: built from the document's
 * worker objects together with the session, fed before each tick, presented to
 * the scene once per frame, and reset with the session.
 */
class HumanWorkforce implements SessionMember {
	public final scene:EditorScene;
	/** One line per problem, tagged with its worker, for the simulation's warning list. */
	public final warnings:Array<String> = [];
	final entries:Array<WorkforceEntry> = [];
	final warningsById:Map<String, Array<String>> = new Map();

	function new(scene:EditorScene) {
		this.scene = scene;
	}

	/**
	 * Builds a worker for every worker object in the scene, placed in `session`
	 * among the environment objects. Throws when a worker cannot be built; nothing
	 * made for the workers outlives a throw.
	 */
	public static function build(session:SimSession, scene:EditorScene, environment:Array<SceneObjectData>,
			objectsById:Map<String, SimObject>, robotIds:Array<String>, robotModels:Array<RobotModel>,
			simulation:Simulation):HumanWorkforce {
		var workforce = new HumanWorkforce(scene);
		try workforce.populate(session, environment, objectsById, robotIds, robotModels, simulation)
		catch (failure:Dynamic) {
			workforce.dispose();
			throw failure;
		}
		return workforce;
	}

	function populate(session:SimSession, environment:Array<SceneObjectData>,
			objectsById:Map<String, SimObject>, robotIds:Array<String>, robotModels:Array<RobotModel>,
			simulation:Simulation):Void {
		var targets = new WorkerSceneTargets(environment, scene);
		for (record in scene.records()) if (record.type == "human-worker") {
			var data = record.worker;
			if (data == null) throw 'Worker "${record.id}" has no worker data';
			var asset = AnimationAsset.load(WorkerAssetPath.resolve(data.asset));
			var character:HumanCharacter;
			try {
				var rig = HumanoidRig.detect(asset);
				character = new HumanCharacter(scene.runtimeContentScene(), asset, rig, null, record.id);
			} catch (error:Dynamic) {
				asset.dispose();
				throw error;
			}
			// Register before construction so a failure removes this worker's scene nodes.
			var entry = new WorkforceEntry(record.id, character, asset);
			entries.push(entry);
			character.advance(0.0);
			character.publish();
			var proxy = HumanBodyProxy.standard(character.pose,
				HumanDescription.measure(character.pose, character.height()));
			var rotation = record.rotation == null ? [0.0, 0.0, 0.0, 1.0] : record.rotation;
			var worker = new HumanWorker(session, character, proxy,
				new SimPose(record.x, record.y, 0.0, rotation[0], rotation[1], rotation[2], rotation[3]));
			entry.worker = worker;
			for (zoneId in data.zones) {
				var box = targets.box(zoneId);
				if (box == null) {
					var warning = 'Missing zone "$zoneId"';
					warnings.push('Worker "${record.id}": $warning');
					var perWorker = warningsById.get(record.id);
					if (perWorker == null) { perWorker = []; warningsById.set(record.id, perWorker); }
					perWorker.push(warning);
					continue;
				}
				var c = Math.cos(box.yaw), t = Math.sin(box.yaw);
				var polygon:Array<Array<Float>> = [];
				for (corner in [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]]) {
					var x = corner[0] * box.halfExtents[0], y = corner[1] * box.halfExtents[1];
					polygon.push([box.center[0] + c * x - t * y, box.center[1] + t * x + c * y]);
				}
				worker.addZone(new HumanZone(zoneId, polygon));
			}
			for (robotIndex in 0...robotModels.length)
				for (bound in RobotLinkBounds.shapes(robotModels[robotIndex])) {
					var linkIndex = bound.link, offset = bound.offset, radius = bound.radius;
					worker.addRobotLinkPose(robotIds[robotIndex], function() {
						var link = simulation.linkPose(robotIndex, linkIndex);
						return RobotLinkBounds.worldCenter(link.position, link.rotation, offset);
					}, radius);
				}
			try worker.runSpec(HumanJobSpec.parse(data.job), targets, objectsById)
			catch (failure:Dynamic) {
				var failed = new HumanJob().add(new Wait(1.0));
				worker.run(failed);
				failed.abort('Invalid worker job: $failure');
			}
		}
	}

	public function ids():Array<String> return [for (entry in entries) entry.id];

	public function worker(id:String):Null<HumanWorker> {
		for (entry in entries) if (entry.id == id) return entry.worker;
		return null;
	}

	/** The worker's readings as of the last tick, taken when asked: the telemetry panel draws them, the job does not use them. */
	public function signals(id:String):Null<HumanWorkerSignals> {
		var found = worker(id);
		return found == null ? null : found.readSignals();
	}

	public function warningsFor(id:String):Array<String> {
		var found = warningsById.get(id);
		return found == null ? [] : found.copy();
	}

	public function feed():Void {
		for (entry in entries) {
			var worker = entry.worker;
			if (worker != null) worker.advance();
		}
	}

	public function reset():Void {
		for (entry in entries) {
			var worker = entry.worker;
			if (worker != null) {
				worker.reset();
				entry.character.publish();
			}
		}
	}

	public function present():Void {
		for (entry in entries) {
			var worker = entry.worker;
			if (worker == null) continue;
			entry.character.publish();
			var matrix = worker.body.rootTransform();
			var transform = Transform.identity();
			for (index in 0...16) transform.set(index, matrix[index]);
			var transaction = scene.runtimeContentScene().beginTransaction();
			transaction.setTransform(entry.character.root, transform);
			transaction.commit();
			scene.publishRuntimeNodes(entry.character.changedNodes());
		}
	}

	/** Releases every worker and removes its character from the scene. */
	public function dispose():Void {
		if (entries.length == 0) return;
		for (entry in entries) if (entry.worker != null) entry.worker.dispose();
		var transaction = scene.runtimeContentScene().beginTransaction();
		for (entry in entries) {
			var nodes = entry.character.changedNodes();
			nodes.reverse();
			for (node in nodes) transaction.destroyNode(node);
		}
		transaction.commit();
		scene.publishRuntimeNodes([]);
		for (entry in entries) { entry.character.dispose(); entry.asset.dispose(); }
		entries.resize(0);
	}
}
