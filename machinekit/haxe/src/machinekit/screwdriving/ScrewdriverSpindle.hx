package machinekit.screwdriving;

import machinekit.assembly.MachineAssembly;
import machinekit.motion.LinearScale;
import materia.assembly.AssemblyFrames;

/** Assumed 20 Nm, 600 rpm tool on a 20 mm compliant feed. Rates are design inputs, not catalog claims. */
class ScrewdriverSpindle extends MachineAssembly {
	public final bit:ScrewdriverBit;
	public final capability:ScrewdriverFacet;

	public function new(bit:ScrewdriverBit, maxTorque:Float = 20, maxSpeed:Float = 62.83185307179586,
		strokeMm:Float = 20, springRate:Float = 2) {
		super(); this.bit = bit;
		var carrier = new ScrewdriverCarrier(bit, maxTorque, maxSpeed, strokeMm, springRate);
		capability = cast ScrewdriverFacet.of(carrier);
		addComponent("body", new ScrewdriverHousing('Screwdriver motor gearbox ${machinekit.component.Dimension.format(maxTorque)}Nm-${machinekit.component.Dimension.format(maxSpeed)}radps', 120, 25, true));
		addComponent("slide", new ScrewdriverHousing("Screwdriver feed slide", 25, 12));
		addComponent("carrier", carrier);
		addComponent("bit", bit);
		addMateOnAxis("feed", "prismatic", "body", "end", "slide", "mount", {x: 0, y: 1, z: 0}, 0,
			{lower: -strokeMm, upper: 0, velocity: 100, effort: springRate * strokeMm});
		addMateOnAxis("spindle", "continuous", "slide", "end", "carrier", "mount", {x: 0, y: 1, z: 0}, 0,
			{lower: null, upper: null, velocity: maxSpeed, effort: maxTorque});
		addMate("bit-mount", "fixed", "carrier", "bit", "bit", "mount");
		addComponent("scale", new LinearScale(strokeMm, 100, true), AssemblyFrames.translation(28, 0, 100));
		addMemberConnector("body", "scaleMount", machinekit.component.Solids.axial(28, 0, 100));
		addMate("scale-mount", "fixed", "body", "scaleMount", "scale", "mount");
		addEncoder("stroke", "feed", "scale");
		connectPorts("slide-control", "body", "bitControl", "slide", "control");
		connectPorts("carrier-control", "slide", "bitControl", "carrier", "control");
		connectPorts("slide-vacuum", "body", "bitVacuum", "slide", "vacuum");
		connectPorts("bit-vacuum", "slide", "bitVacuum", "bit", "vacuum");
		exposeConnector("mount", "body", "mount"); exposeConnector("tcp", "carrier", "tcp");
		exposePort("power", "body", "power"); exposePort("control", "body", "control"); exposePort("vacuum", "body", "vacuum");
	}
}
