package processkit.screwdriving;

import collisionkit.CollisionPairRule;
import collisionkit.CollisionPairStatus;
import robotkit.collision.CollisionSceneBuilder;
import processkit.ToolWorkCollisionPolicy;

/** Exact thread, clamp and driver contacts from one named CAD fastening. */
class ScrewCollisionPolicy {
	static function set(scene:CollisionSceneBuilder, screw:String, member:String,
		rule:CollisionPairRule, reason:CollisionPairStatus):Void {
		var screws = scene.memberObjects([screw]);
		if (screws.length == 0) throw 'Fastening contact has no CAD geometry for screw "$screw"';
		var targets = scene.memberObjects([member]);
		if (targets.length == 0) throw 'Fastening contact for screw "$screw" has no CAD geometry for member "$member"';
		for (screwObject in screws) for (target in targets) {
			var first = scene.description.objects[screwObject], second = scene.description.objects[target];
			if (first.body == second.body) continue;
			scene.pair(first.name, second.name, rule, reason, 0.0, 0.0);
		}
	}

	public static function apply(scene:CollisionSceneBuilder, connections:Array<{screw:String, member:String}>,
		activeScrew:Null<String>, activeBitMember:Null<String>, sceneMargin:Float):Void {
		if (scene == null || connections == null || !Math.isFinite(sceneMargin) || sceneMargin < 0.0)
			throw "Screw collision policy needs named connections and a finite scene margin";
		var activeFound = activeScrew == null;
		for (connection in connections) {
			if (connection.screw == activeScrew) {
				activeFound = true;
				set(scene, connection.screw, connection.member, CollisionPairRule.Touch, CollisionPairStatus.DeclaredContact);
			} else {
				// Other CAD-seated screws stay fixed while this fastening is planned.
				set(scene, connection.screw, connection.member, CollisionPairRule.Allow, CollisionPairStatus.DeclaredContact);
			}
		}
		if (activeScrew != null) {
			if (!activeFound) throw 'Fastening collision has no declared CAD connection for active screw "$activeScrew"';
			var target = scene.memberObjects([activeScrew]);
			if (target.length == 0) throw 'Fastening collision has no CAD geometry for active screw "$activeScrew"';
			if (activeBitMember == null || activeBitMember.length == 0)
				throw 'Fastening collision has no named CAD bit for active screw "$activeScrew"';
			var bit = scene.memberObjects([activeBitMember]);
			if (bit.length == 0) throw 'Fastening collision has no CAD geometry for screwdriver bit "$activeBitMember"';
			for (object in bit) if (!scene.description.objects[object].tool)
				throw 'Fastening screwdriver bit "$activeBitMember" is not a robot tool member';
			ToolWorkCollisionPolicy.apply(scene, target, sceneMargin, 0.0, 0.0,
				CollisionPairRule.Touch, CollisionPairStatus.DeclaredContact);
			// The bit's exact CAD profile is designed to engage the screw drive socket.
			// Preserve collision checks for every other spindle member and the screw head.
			set(scene, activeScrew, activeBitMember,
				CollisionPairRule.Allow, CollisionPairStatus.DeclaredContact);
		}
	}
}
