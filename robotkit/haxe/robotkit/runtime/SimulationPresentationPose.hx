package robotkit.runtime;

/** One immutable body pose copied from a shared simulation presentation. */
typedef SimulationPresentationPose = {
	var kind:Int;
	var robotIndex:Int;
	var linkIndex:Int;
	var objectId:Int;
	var position:Array<Float>;
	var rotation:Array<Float>;
}
