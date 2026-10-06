package machinekit.structural;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.structural.FrameAssembly.StructuralProfile;

/** Cut structural stock along local +Z. Hollow sections retain their actual material and mass. */
class ProfileMember extends MachineComponent {
	public final profile:StructuralProfile;
	public final length:Float;

	public function new(profile:StructuralProfile, length:Float, material:String = "steel") {
		if (profile == null || !(length > 0) || !Math.isFinite(length))
			throw "Structural member needs a profile and a finite positive length";
		super('${profile.profileDesignation()}-L${Dimension.format(length)}', profile.profileDescription(), material);
		this.profile = profile;
		this.length = length;
		addConnector("start", Mount, Solids.axial(0, 0, 0));
		addConnector("end", Mount, Solids.axial(0, 0, length));
	}

	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part return profile.geometry(length);
}
