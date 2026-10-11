package machinekit.screwdriving;

import machinekit.standard.SocketHeadCapScrew;
import materia.assembly.AssemblyFrames;

/** Existing supported pickup nest with an explicit rear inlet for oriented feed. */
class AutomaticScrewPresenter extends ScrewPresenter {
	public function new(screw:SocketHeadCapScrew) {
		super(screw);
		addMemberConnector("nest","feed",AssemblyFrames.translation(0,-25,screw.length+15));
		exposeConnector("feed","nest","feed");
	}
}
