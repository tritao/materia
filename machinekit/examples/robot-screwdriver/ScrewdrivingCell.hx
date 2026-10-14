import machinekit.assembly.MachineAssembly;
import machinekit.screwdriving.ArmScrewdrivingTool;
import machinekit.screwdriving.ScrewdriverBit;
import machinekit.screwdriving.TighteningController;
import machinekit.screwdriving.ScrewPresenter;
import machinekit.screwdriving.FlangeBearingMount;
import machinekit.standard.DeepGrooveBearing;
import materia.assembly.AssemblyFrames;
import cadkit.modeling.Part;
import cadkit.modeling.Sketch;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import machinekit.component.Solids;
import machinekit.component.ComponentDetail;
import machinekit.pneumatic.VacuumGenerator;
import RobotArm.ArmTable;
import RobotArm.ArmBlock;
/** Equipment cell for the SD3 flange workpiece; screws remain seated until the driving step exists. */
class ScrewdrivingCell extends MachineAssembly {
	public final work:FlangeBearingMount;
	public final tool:ArmScrewdrivingTool;
	public final arm:RobotArm;
	public final controller:TighteningController;
	public final presenter:ScrewPresenter;
	static inline final DATUM_STOP_WIDTH = 80.0;
	static inline final DATUM_BEVEL_RUN = 60.0;
	static inline final DATUM_BEVEL_DROP = 35.0;
	public function new(?bit:ScrewdriverBit, workX:Float = 0, ?bearing:DeepGrooveBearing, automaticSupply:Bool = false,
		blowSupply:Bool = false, plateEdgeMargin:Float = 10, fixtureHeight:Float = 15,
		presenterX:Float = 180, presenterY:Float = -450, registrationDatums:Bool = false) {
		super();
		work = new FlangeBearingMount(bearing == null ? DeepGrooveBearing.metric("608") : bearing, plateEdgeMargin);
		var fitted = bit == null ? new ScrewdriverBit(work.flange.screw.screwDrive()) : bit;
		tool = blowSupply ? new machinekit.screwdriving.BlowFeedTool(fitted,work.flange.screw,14,18,200,2500) : new ArmScrewdrivingTool(fitted);
		arm = new RobotArm(false, tool); controller = new TighteningController();
		presenter = automaticSupply || blowSupply ? new machinekit.screwdriving.AutomaticScrewPresenter(work.flange.screw) : new ScrewPresenter(work.flange.screw);
		include("arm", arm);
		addComponent("table", new ArmTable(700, 400, RobotArm.TABLE_TOP), AssemblyFrames.translation(0, -520, 0));
		addComponent("controller", controller, AssemblyFrames.translation(-800, 200, 0));
		addComponent("vacuum", new VacuumGenerator(60, null, null, machinekit.screwdriving.ScrewdrivingInterfaces.vacuum()),
			AssemblyFrames.translation(-800, 350, 0));
		include("work", work);
		// The table constrains the tapped plate, not any screw or housing inferred from radius.
		addMemberConnector("table", "workSeat", Solids.axial(workX, 20, RobotArm.TABLE_TOP));
		// Included members are snapshots, so add the cell's base connector explicitly.
		addMemberConnector("work/plate", "base", Solids.axial(0, 0, 0));
		addMate("work-mount", "fixed", "table", "workSeat", "work/plate", "base");
		if (!Math.isFinite(fixtureHeight) || fixtureHeight <= 0) throw "Screwdriving fixture stops need a positive finite height";
		var fixtureWidth = registrationDatums ? DATUM_STOP_WIDTH : 20.0;
	var stop = work.plate.width / 2 + fixtureWidth / 2 + (registrationDatums ? 10.0 : 0.0);
		for (side in [-1, 1]) {
			var name = side < 0 ? "fixtureLeft" : "fixtureRight";
			var fixture = registrationDatums
				? new ScrewdrivingDatumStop(fixtureWidth, work.plate.width, fixtureHeight, "steel", "Fixture", side < 0)
				: new ArmBlock(fixtureWidth, work.plate.width, fixtureHeight, "steel", "Fixture");
			addComponent(name, fixture);
			addMemberConnector("table", name, Solids.axial(workX + side * stop, 20, RobotArm.TABLE_TOP));
			addMate(name + "-mount", "fixed", "table", name, name, "base");
		}
		include("presenter", presenter, AssemblyFrames.translation(presenterX, presenterY, RobotArm.TABLE_TOP));
		// One cable carries distinct electrical and signal services.
		connectPorts("tool-power", "controller", "toolPower", "arm/tool/driver/body", "power");
		connectPorts("tool-cable", "controller", "tool", "arm/tool/driver/body", "control");
		connectPorts("vacuum-hose", "vacuum", "vacuum", "arm/tool/driver/body", "vacuum");
		exposePort("mains", "controller", "mains"); exposePort("control", "controller", "control");
		exposePort("air", "vacuum", "air"); exposePort("presenterControl", "presenter/escapement", "control");
	}
	public function readyPose():Array<Float> return [0.0, 0.25, Math.PI / 2 - 1.4, 0.0, Math.PI / 2 - 0.25 + Math.PI / 2 - 1.4, 0.0];
}

/** Registered examples use broad, top-accessible angled faces as explicit CAD datums. */
private class ScrewdrivingDatumStop extends ArmBlock {
	final innerX:Bool;
	public function new(width:Float, depth:Float, height:Float, material:String, name:String, innerX:Bool) {
		super(width, depth, height, material, name);
		if (width <= ScrewdrivingCell.DATUM_BEVEL_RUN || height <= ScrewdrivingCell.DATUM_BEVEL_DROP)
			throw "Screwdriver datum stop is too small for its chamfered reference face";
		this.innerX = innerX;
	}
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var block = Part.box(width, depth, height);
		var plane = innerX
			? new Plane(new Vector(0, depth / 2, 0), Vector.X(), new Vector(0, -1, 0))
			: new Plane(new Vector(-width / 2, 0, 0), Vector.Y(), Vector.X());
		var run = ScrewdrivingCell.DATUM_BEVEL_RUN, drop = ScrewdrivingCell.DATUM_BEVEL_DROP;
		var points = innerX
			? [new Vector(width / 2, height), new Vector(width / 2, height - drop), new Vector(width / 2 - run, height)]
			: [new Vector(depth / 2, height), new Vector(depth / 2, height - drop), new Vector(depth / 2 - run, height)];
		var sketch:Null<Sketch> = null, wedge:Null<Part> = null;
		try {
			sketch = Sketch.polygon(points, plane);
			wedge = sketch.extrude(innerX ? depth : width);
			sketch.close(); sketch = null;
			var result = block.subtract(wedge);
			block.close(); wedge.close();
			return result;
		} catch (error:Dynamic) {
			block.close();
			if (sketch != null) sketch.close();
			if (wedge != null) wedge.close();
			throw error;
		}
	}
}
