package machinekit.screwdriving;

import machinekit.component.MachineComponent;
import machinekit.component.ComponentDetail;
import materia.assembly.AssemblyFrames;
import cadkit.modeling.Part;

/** Custom metering/air control envelope; valve timing and pressure are commissioned process inputs. */
class BlowFeedMeter extends MachineComponent {
	public function new(tubeOuterDiameter:Float) {
		if (!Math.isFinite(tubeOuterDiameter) || tubeOuterDiameter <= 0) throw "Meter needs the actual delivery tube diameter";
		super("CUSTOM-BLOW-FEED-METER","Screw escapement and air valve","steel",true);
		addConnector("inlet",Mount,AssemblyFrames.translation(0,0,0));
		addConnector("outlet",Mount,AssemblyFrames.translation(0,0,20));
		addPort({name:"screw",kind:ScrewFeed,role:Supply,iface:PushIn(tubeOuterDiameter),required:true,connector:"outlet"});
		addPort({name:"air",kind:Pneumatic,role:Consumer,iface:PushIn(6),required:true});
		addPort({name:"control",kind:Signal,role:Consumer,iface:ScrewFeederInterfaces.control(),required:true});
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part return Part.box(30,30,20);
}
