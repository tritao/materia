package machinekit.screwdriving;

import machinekit.robotics.RobotFlange;
import machinekit.robotics.EndEffector;
import machinekit.assembly.MachineAssembly;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentDetail;
import machinekit.component.Solids;
import materia.assembly.AssemblyFrames;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;

/** Custom receiving envelope and local tube route; external dress-pack routing is commissioned separately. */
class BlowFeedTool extends ArmScrewdrivingTool {
	public final screw:SocketHeadCapScrew;
	public final innerDiameter:Float;
	public final outerDiameter:Float;
	public final minimumBendRadius:Float;
	public final externalTubeLength:Float;
	public function new(bit:ScrewdriverBit, screw:SocketHeadCapScrew, innerDiameter:Float,
			outerDiameter:Float, minimumBendRadius:Float, externalTubeLength:Float) {
		if (screw == null || bit == null || !bit.matches(screw.screwDrive())) throw "Blow-feed tooling needs the actual compatible screw drive";
		for (value in [innerDiameter,outerDiameter,minimumBendRadius,externalTubeLength])
			if (!Math.isFinite(value) || value <= 0) throw "Blow-feed tube dimensions must be finite and positive";
		var gap = (innerDiameter-screw.spec.headDiameter)/2, span = screw.length+screw.spec.headHeight;
		if (externalTubeLength <= 2*Math.PI*minimumBendRadius || gap <= 0 || outerDiameter <= innerDiameter || innerDiameter/2 <= bit.sleeveRadius
			|| minimumBendRadius <= span/2 || minimumBendRadius-Math.sqrt(minimumBendRadius*minimumBendRadius-span*span/4) > gap)
			throw "Blow-feed tube cannot clear the commissioned screw envelope, sleeve or rigid-span bend";
		super(bit); this.screw = screw; this.innerDiameter = innerDiameter; this.outerDiameter = outerDiameter;
		this.minimumBendRadius = minimumBendRadius; this.externalTubeLength = externalTubeLength;
	}
	override public function build(flange:RobotFlange):EndEffector {
		var result = super.build(flange), nose = new ReceivingNose(screw,innerDiameter,outerDiameter);
		result.addComponent("receiver",nose);
		result.addMemberConnector("driver/slide","receiverMount",AssemblyFrames.translation(0,0,15+spindle.bit.length-20));
		result.addMate("receiver-mount","fixed","driver/slide","receiverMount","receiver","mount");
		// This straight local section is carried by the non-rotating feed slide.
		result.addComponent("feedTube",new machinekit.pneumatic.RoutedHose("CUSTOM-SCREW-FEED-TUBE",
			[new Vector(50,0,-100),new Vector(50,0,10)],outerDiameter,innerDiameter,0.08,machinekit.component.PortKind.ScrewFeed));
		result.addMate("tube-mount","fixed","receiver","mount","feedTube","mount");
		result.connectPorts("tube-to-nose","feedTube","output","receiver","feed");
		result.addPresence("screw-receipt","receiver/receipt",screw.spec.headHeight+4);
		result.exposePort("screwFeed","feedTube","input");
		return result;
	}
	override public function expose(arm:MachineAssembly):Void {
		super.expose(arm); arm.exposePort("screwFeed","tool/feedTube","input");
	}
}
private class ReceivingNose extends MachineComponent {
	final inner:Float; final outer:Float;
	public function new(screw:SocketHeadCapScrew, inner:Float, outer:Float) {
		super("CUSTOM-BLOW-FEED-NOSE-"+screw.designation,"Receiving nosepiece","steel",true);
		this.inner = inner; this.outer = outer;
		addConnector("mount",Mount,AssemblyFrames.translation(0,0,0));
		addConnector("receipt",Mount,AssemblyFrames.translation(0,0,20));
		addConnector("inlet",Mount,AssemblyFrames.translation(50,0,10));
		addPort({name:"feed",kind:ScrewFeed,role:Consumer,iface:PushIn(outer),required:true,connector:"inlet"});
		var r = inner/2, o = outer/2;
		addFacet(machinekit.component.CollisionHullFacet.fromBoxes([
			[-o,-o,0.0,-r,o,12.0],[r,-o,0.0,o,o,12.0],
			[-r,-o,0.0,r,-r,12.0],[-r,r,0.0,r,o,12.0],
			[o,-o,0.0,58.0,o,12.0]]));
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.cut(Part.box(58+outer/2,outer,12).translated(new Vector(29-outer/4,0,0)),[Part.cylinderSpan(inner/2,-0.1,12.1)]);
}
