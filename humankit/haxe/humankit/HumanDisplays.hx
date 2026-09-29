package humankit;

class HumanDisplays {
	/** Parses "mesh", "capsules", or "skeleton". */
	public static function parse(name:String):HumanDisplay
		return switch name {
			case "mesh": Mesh;
			case "capsules": Capsules;
			case "skeleton": Skeleton;
			default: throw 'Unknown human display "$name"; use mesh, capsules, or skeleton';
		};
}
