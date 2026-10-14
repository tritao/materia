package processkit;

import collisionkit.CollisionMarginClass;
import collisionkit.CollisionPairRule;
import collisionkit.CollisionPairStatus;
import robotkit.collision.CollisionSceneBuilder;

/** Tool clearance keeps the scene margin, with separate margins for named work. */
class ToolWorkCollisionPolicy {
	public static function apply(scene:CollisionSceneBuilder, work:Array<Int>, sceneMargin:Float, workAirMargin:Float,
		contactMargin:Float, rule:CollisionPairRule = CollisionPairRule.Default,
		reason:CollisionPairStatus = CollisionPairStatus.Checked):Void {
		scene.description.setGroupMargins(CollisionMarginClass.Installed, CollisionMarginClass.Installed, sceneMargin);
		scene.description.setGroupMargins(CollisionMarginClass.Tool, CollisionMarginClass.Installed, sceneMargin);
		scene.description.setGroupMargins(CollisionMarginClass.Tool, CollisionMarginClass.Tool, sceneMargin);
		for (tool in scene.toolObjects()) for (target in work) if (tool != target)
			scene.pair(scene.description.objects[tool].name, scene.description.objects[target].name,
				rule, reason, workAirMargin, contactMargin);
	}
}
