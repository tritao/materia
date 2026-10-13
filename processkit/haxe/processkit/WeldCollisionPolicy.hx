package processkit;

import collisionkit.CollisionPairRule;
import collisionkit.CollisionPairStatus;
import robotkit.collision.CollisionSceneBuilder;
import robotkit.collision.CollisionSceneBuilder.CollisionOwnership;

/** Welding margins apply to installed mechanism and named CAD work interfaces. */
class WeldCollisionPolicy {
	public static function apply(scene:CollisionSceneBuilder, work:Array<Int>, airMargin:Float, contactMargin:Float):Void {
		function installed(metadata:CollisionOwnership):Bool return switch metadata.owner {
			case Mechanism(controller, _): controller == scene.owner;
			case _: false;
		};
		var objects = scene.description.objects;
		for (i in 0...objects.length) for (j in i + 1...objects.length) {
			var first = objects[i].name, second = objects[j].name;
			var a = scene.ownership.get(first), b = scene.ownership.get(second);
			if (a == null || b == null) continue;
			var contact = (a.tool && work.indexOf(j) >= 0) || (b.tool && work.indexOf(i) >= 0);
			if ((installed(a) && installed(b)) || contact)
				scene.pair(first, second, CollisionPairRule.Default, CollisionPairStatus.Checked,
					airMargin, contact ? contactMargin : null);
		}
	}
}
