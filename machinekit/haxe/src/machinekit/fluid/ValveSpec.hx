package machinekit.fluid;

/** Typed spool behavior exposed by a directional valve component. */
interface ValveSpec {
	function controlSpec():ValveControlSpec;
}
