package machinekit.component;

/** Raised by components that do not define a shape for mass estimation. */
class NoGeometry {
	public final designation:String;
	public function new(designation:String) this.designation = designation;
}
