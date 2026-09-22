package cadkit.parametric;

import cadkit.modeling.Location;
import cadkit.modeling.Plane;

/** Immutable rigid transform used by document elements. */
class Placement {
	public final location:Location;

	public function new(?plane:Plane) {
		location = new Location(plane == null ? Plane.XY() : plane);
	}

	public static function identity():Placement return new Placement();
	public function compose(child:Placement):Placement return new Placement(location.compose(child.location).plane);
	public function inverse():Placement return new Placement(location.inverse().plane);
	public function isIdentity():Bool {
		var p=location.plane;
		return p.origin.x==0 && p.origin.y==0 && p.origin.z==0 && p.xDirection.x==1 && p.xDirection.y==0 && p.xDirection.z==0 && p.normal.x==0 && p.normal.y==0 && p.normal.z==1;
	}
}
