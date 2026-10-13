package processkit;

import collisionkit.CollisionMarginClass;
import collisionkit.CollisionPairRule;
import collisionkit.CollisionPairStatus;
import robotkit.collision.CollisionSceneBuilder;

/** Installed interfaces share class margins; named work keeps sparse tool contact policy. */
class WeldCollisionPolicy {
	public static function apply(scene:CollisionSceneBuilder, work:Array<Int>, airMargin:Float, contactMargin:Float):Void {
		scene.description.setGroupMargins(CollisionMarginClass.Installed, CollisionMarginClass.Installed, airMargin);
		scene.description.setGroupMargins(CollisionMarginClass.Tool, CollisionMarginClass.Installed, airMargin);
		scene.description.setGroupMargins(CollisionMarginClass.Tool, CollisionMarginClass.Tool, airMargin);
		for (tool in scene.toolObjects()) for (target in work) if (tool != target)
			scene.pair(scene.description.objects[tool].name, scene.description.objects[target].name,
				CollisionPairRule.Default, CollisionPairStatus.Checked, airMargin, contactMargin);
	}
}
