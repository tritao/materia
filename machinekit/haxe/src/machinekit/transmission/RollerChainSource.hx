package machinekit.transmission;

/** A chain member states the pitch and roller size that its sprockets must accept. */
interface RollerChainSource {
	public function chainSpec():RollerChainSpec;
}
