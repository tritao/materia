package machinekit.component;

/** Descriptive tag for a connector frame; mating always uses the frame itself. */
enum ConnectorRole {
	/** Rotation or symmetry axis along the frame's +Y. */
	Axis;
	/** Planar contact face whose outward normal is the frame's +Y. */
	Face;
	/** Mounting interface, such as a flange face or bolt position. */
	Mount;
	/** End of a drive shaft that receives a coupling, pulley, or gear. */
	Shaft;
}
