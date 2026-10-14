package processkit;

import robotkit.collision.CollisionSceneBuilder;

/** Installed interfaces share class margins; named work keeps sparse tool contact policy. */
class WeldCollisionPolicy {
	public static function apply(scene:CollisionSceneBuilder, work:Array<Int>, airMargin:Float, contactMargin:Float):Void {
		ToolWorkCollisionPolicy.apply(scene, work, airMargin, airMargin, contactMargin);
	}
}
