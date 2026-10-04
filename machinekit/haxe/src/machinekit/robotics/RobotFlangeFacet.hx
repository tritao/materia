package machinekit.robotics;

import machinekit.component.ComponentFacet;
import machinekit.component.MachineComponent;

/** Physical tool flange connector, independent of the component core. */
class RobotFlangeFacet implements ComponentFacet {
	public final connector:String;
	public function new(connector:String) this.connector = connector;
	public function check(component:MachineComponent):Void { component.connector(connector); }
	public function describe():String return 'robot flange $connector';
	public static function of(component:MachineComponent):Null<RobotFlangeFacet> {
		for (facet in component.facets()) if (Std.isOfType(facet, RobotFlangeFacet)) return cast facet;
		return null;
	}
}
