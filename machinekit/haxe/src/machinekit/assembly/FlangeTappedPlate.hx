package machinekit.assembly;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.threads.ThreadSpec;
import cadkit.threads.Threads;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.Solids;
import machinekit.motion.FlangeBearingHousing;
import machinekit.standard.SocketHeadCapScrew;

/** Aluminium fixture plate with four independently named taps on the housing's bolt square. */
class FlangeTappedPlate extends MachineComponent {
	public final housing:FlangeBearingHousing;
	public final screw:SocketHeadCapScrew;
	public final thickness:Float;
	public final width:Float;
	public final tapDepth:Float;
	public final holeShift:Float;
	public final thread:ThreadSpec;

	public function new(housing:FlangeBearingHousing, screw:SocketHeadCapScrew, thickness:Float = 20,
		tapDepth:Float = 16, holeShift:Float = 0, thread:Null<ThreadSpec> = null, edgeMargin:Float = 10) {
		if (!Math.isFinite(thickness) || !Math.isFinite(tapDepth) || !Math.isFinite(holeShift)
			|| !Math.isFinite(edgeMargin) || thickness <= 0 || tapDepth <= 0 || tapDepth > thickness || edgeMargin < 10)
			throw "Fixture tap depth and edge margin must fit its plate";
		var nominal = thread == null ? screw.thread.mating() : cast thread;
		if (!nominal.internal) throw "Fixture plate requires an internal thread";
		super('FLANGEPLATE-${housing.designation}-T${Dimension.format(thickness)}-D${Dimension.format(tapDepth)}-X${Dimension.format(holeShift)}'
			+ (edgeMargin == 10 ? "" : '-E${Dimension.format(edgeMargin)}')
			+ '-${nominal.designation}-${nominal.majorDiameter}-${nominal.pitch}-${nominal.starts}-${nominal.hand}-${nominal.profile}-${nominal.toleranceClass}-DRILL${screw.spec.tapDrill}',
			"Tapped flange fixture plate", "aluminium 6061", true);
		this.housing = housing; this.screw = screw; this.thickness = thickness; this.width = housing.face + 2 * edgeMargin;
		this.tapDepth = tapDepth; this.holeShift = holeShift;
		this.thread = nominal;
		addConnector("mount", Mount, Solids.axial(0, 0, thickness));
		// Keep each tapped opening clear of the simulation hull. Circumscribed
		// square cutouts are a compact proxy that slightly enlarges each round
		// drill opening; the lower unthreaded web remains a solid box.
		var radialClearance = (screw.spec.tapDrill - screw.thread.externalMinorDiameter()) / 2;
		if (!Math.isFinite(radialClearance) || radialClearance <= 0)
			throw "Tapped-plate collision needs positive screw-to-bore clearance";
		var centers = [for (point in housing.boltPattern()) new Vector(point.x + holeShift,point.y)];
		var pieces = machinekit.component.CollisionHullFacet.aroundCircularOpenings(width,width,
			thickness - tapDepth,thickness,centers,screw.spec.tapDrill/2).hulls;
		if (thickness > tapDepth) pieces = machinekit.component.CollisionHullFacet.fromBoxes(
			[[-width/2,-width/2,0,width/2,width/2,thickness-tapDepth]]).hulls.concat(pieces);
		addFacet(new machinekit.component.CollisionHullFacet(pieces));
	}

	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Part.box(width, width, thickness);
		if (detail == Envelope) return body;
		var tools:Array<Part> = [], index = 1;
		return Solids.building([body], tracked -> {
			for (point in housing.boltPattern()) {
				var tap = Threads.tappedBore(thread, screw.spec.tapDrill, tapDepth);
				tracked.push(tap);
				var placed = tap.translated(new Vector(point.x + holeShift, point.y, thickness));
				tap.close(); tracked.push(placed);
				var named = Solids.named(placed, "bolt" + index++);
				tracked.push(named); tools.push(named);
			}
			return Solids.cut(body, tools);
		});
	}
}
