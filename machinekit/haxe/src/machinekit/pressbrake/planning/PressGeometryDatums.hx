package machinekit.pressbrake.planning;

/** Member identities, not copied dimensions. Bed owns left/right, ram tool-mount,
 * side frame throat, and fingers/tool segments contact connectors.
 */
typedef PressGeometryDatums = {
	final bed:String;
	final ram:String;
	final throat:String;
	final fingers:Array<String>;
	final ramJoint:String;
	final gaugeXJoint:String;
	final gaugeRJoint:Null<String>;
	final tools:Array<PressToolDatum>;
}
