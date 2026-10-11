package machinekit.screwdriving;

import machinekit.assembly.MachineAssembly;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentDetail;
import machinekit.component.Solids;
import machinekit.standard.SocketHeadCapScrew;
import materia.assembly.AssemblyFrames;
import cadkit.modeling.Part;

/** Custom bowl/track equipment; dimensions are explicit design assumptions, not vendor specifications. */
class ScrewBowlFeeder extends MachineAssembly {
	public final screw:SocketHeadCapScrew;
	public final bowlDiameter:Float;
	public final trackGap:Float;
	public final guideHeight:Float;
	public final source:String = "Custom SD9 bowl and track design; no vendor performance or dimensional claim";
	public function new(screw:SocketHeadCapScrew, bowlDiameter:Float = 120, bowlHeight:Float = 60,
			trackLength:Float = 115, clearance:Float = 0.5) {
		if (screw == null || !Math.isFinite(clearance) || clearance <= 0 || clearance >= (screw.spec.headDiameter-screw.diameter)/2
			|| !Math.isFinite(bowlDiameter) || bowlDiameter < 4*screw.spec.headDiameter
			|| !Math.isFinite(bowlHeight) || bowlHeight <= screw.length+screw.spec.headHeight
			|| !Math.isFinite(trackLength) || trackLength <= screw.spec.headDiameter)
			throw "Bowl tooling needs a compatible cap-screw recipe, head-supported track and positive envelope";
		super(); this.screw = screw; this.bowlDiameter = bowlDiameter;
		trackGap = screw.diameter+2*clearance; guideHeight = screw.spec.headHeight+clearance;
		addComponent("drive",new FeederSolid("Vibration drive",80,80,15));
		addComponent("bowl",new FeederBowl(bowlDiameter,bowlHeight),AssemblyFrames.translation(0,0,15));
		var railWidth = 8.0, headSeat = 15+bowlHeight;
		var drop = bowlHeight-screw.length;
		var tilt = -Math.atan2(drop,trackLength), distance = Math.sqrt(trackLength*trackLength+drop*drop);
		var qx = Math.sin(tilt/2), qw = Math.cos(tilt/2);
		var centerY = bowlDiameter/2+trackLength/2;
		var centerZ = headSeat-drop/2;
		for (side in [-1,1]) addComponent(side < 0 ? "trackLeft" : "trackRight",
			new FeederSolid("Head-supported guide rail",railWidth,distance,8),
			{x:side*(trackGap+railWidth)/2,y:centerY+8*Math.sin(tilt),z:centerZ-8*Math.cos(tilt),qx:qx,qy:0,qz:0,qw:qw});
		addComponent("cover",new FeederSolid("Single-layer track guide",trackGap+2*railWidth,distance,3),
			{x:0,y:centerY-guideHeight*Math.sin(tilt),z:centerZ+guideHeight*Math.cos(tilt),qx:qx,qy:0,qz:0,qw:qw});
		exposeConnector("mount","drive","mount");
		addMemberConnector("trackLeft","outlet",AssemblyFrames.translation((trackGap+railWidth)/2,distance/2,8));
		exposeConnector("outlet","trackLeft","outlet");
		exposePort("control","drive","control");
	}
}
private class FeederSolid extends MachineComponent {
	final width:Float; final length:Float; final height:Float;
	public function new(label:String,width:Float,length:Float,height:Float) {
		super('CUSTOM-FEEDER-$label-$width-$length-$height',label,"steel",true);
		this.width = width; this.length = length; this.height = height;
		addConnector("mount",Mount,AssemblyFrames.translation(0,0,0));
		addPort({name:"control",kind:Signal,role:Consumer,iface:ScrewFeederInterfaces.control(),required:false});
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part return Part.box(width,length,height);
}
private class FeederBowl extends MachineComponent {
	final diameter:Float; final height:Float;
	public function new(diameter:Float,height:Float) {
		super('CUSTOM-FEEDER-BOWL-$diameter-$height',"Bulk screw bowl","steel",true);
		this.diameter = diameter; this.height = height;
		var radius = diameter/2;
		addFacet(machinekit.component.CollisionHullFacet.fromBoxes([
			[-radius,-radius,0.0,radius,radius,4.0],
			[-radius,-radius,4.0,-radius+3,radius,height],
			[radius-3,-radius,4.0,radius,radius,height],
			[-radius+3,-radius,4.0,radius-3,-radius+3,height],
			[-radius+3,radius-3,4.0,radius-3,radius,height]]));
		addConnector("mount",Mount,AssemblyFrames.translation(0,0,0));
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.cut(Part.cylinderSpan(diameter/2,0,height),[Part.cylinderSpan(diameter/2-3,4,height+0.1)]);
}
