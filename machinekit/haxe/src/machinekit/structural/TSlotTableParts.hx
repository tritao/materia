package machinekit.structural;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import cadkit.threads.Threads;
import cadkit.threads.ThreadSpec;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.structural.TSlotTable.TableTopMount;

/** Shared primitives for the generator; all sizes are mm. */
class TSlotTableParts {
	public static function bore(radius:Float, from:Float, to:Float, x:Float = 0, y:Float = 0):Part {
		var raw = Part.cylinderSpan(radius, from, to);
		var result = raw.translated(new Vector(x,y,0)); raw.close(); return result;
	}
	public static function threadBore(radius:Float, from:Float, to:Float, thread:ThreadSpec):Part {
		var raw = Solids.named(Part.cylinderSpan(radius,from,to), "bore");
		try {
			var result = new Part(Threads.attach(raw.shape,"bore:cyl.side",thread,AxisMinimum));
			raw.close(); return result;
		} catch (error:Dynamic) { raw.close(); throw error; }
	}
}

/** Local X extends away from the leg, Z=0 touches the rail underside. */
class TableAngle extends MachineComponent {
	public final span:Float;
	public final thickness:Float;
	public final width:Float;
	public final hole:Float;
	public function new(span:Float, thickness:Float, width:Float, hole:Float) {
		super('TABLE-ANGLE-$span-$thickness-$width-D$hole', "Drilled table angle", "aluminium 6061", true);
		this.span=span; this.thickness=thickness; this.width=width; this.hole=hole;
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part return Solids.building([], owned -> {
		var horizontal = Part.box(span,width,thickness,Min,Center,Max); owned.push(horizontal);
		var vertical = Part.box(thickness,width,span,Min,Center,Max); owned.push(vertical);
		var body = Solids.union([horizontal,vertical]); owned.push(body);
		var bottom = TSlotTableParts.bore(hole/2,-thickness-0.1,0.1,span/2); owned.push(bottom);
		var tool = Part.cylinderSpan(hole/2,-0.1,thickness+0.1); owned.push(tool);
		var side = tool.placed(new Location(new Plane(new Vector(0,0,-span/2),Vector.Y(),Vector.X()))); owned.push(side);
		tool.close();
		return Solids.cut(body,[bottom,side]);
	});
}

/** An end-loaded plate nut occupies only the undercut, not the slot throat. */
class TableSlotNut extends MachineComponent {
	final hardware:TSlotTableHardware;
	public function new(hardware:TSlotTableHardware) {
		this.hardware=hardware;
		super('TABLE-SLOT-NUT-${hardware.boltSize}-${hardware.nutLength}x${hardware.nutWidth}x${hardware.nutThickness}',
			"End-loaded threaded slot plate nut", "steel", true);
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var screw = SocketHeadCapScrew.metric(hardware.boltSize,10);
		return Solids.building([], owned -> {
			var body=Part.box(hardware.nutLength,hardware.nutWidth,hardware.nutThickness); owned.push(body);
			var bore=TSlotTableParts.threadBore(screw.diameter/2,-0.1,hardware.nutThickness+0.1,screw.thread.mating()); owned.push(bore);
			return Solids.cut(body,[bore]);
		});
	}
}

class TableTop extends MachineComponent {
	final width:Float; final depth:Float; final thickness:Float;
	final mounts:Array<TableTopMount>;
	public function new(width:Float, depth:Float, thickness:Float, mounts:Array<TableTopMount>) {
		this.width=width; this.depth=depth; this.thickness=thickness;
		this.mounts=[for (m in mounts) {x:m.x,y:m.y,clearance:m.clearance,counterbore:m.counterbore,depth:m.depth}];
		super('TABLE-TOP-$width-$depth-$thickness-' + haxe.Json.stringify(this.mounts), "Drilled steel table top", "steel", true);
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part return Solids.building([], owned -> {
		var body=Part.box(width,depth,thickness); owned.push(body);
		var tools:Array<Part> = [];
		for (m in mounts) {
			var through=TSlotTableParts.bore(m.clearance/2,-0.1,thickness+0.1,m.x,m.y); owned.push(through); tools.push(through);
			var recess=TSlotTableParts.bore(m.counterbore/2,thickness-m.depth,thickness+0.1,m.x,m.y); owned.push(recess); tools.push(recess);
		}
		return Solids.cut(body,tools);
	});
}

/** Foot stem uses the thread-root CAD envelope; its mating thread is carried by the leg bore. */
class TableLevellingFoot extends MachineComponent {
	final diameter:Float; final height:Float; final engagement:Float; final size:String;
	public function new(diameter:Float, height:Float, engagement:Float, size:String) {
		this.diameter=diameter; this.height=height; this.engagement=engagement; this.size=size;
		super('TABLE-FOOT-$size-D$diameter-H$height-E$engagement', "Threaded levelling foot", "steel", true);
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var screw=SocketHeadCapScrew.metric(size,engagement);
		return Solids.union([Part.cylinderSpan(diameter/2,0,8),
			Part.cylinderSpan(screw.thread.externalMinorDiameter()/2,7,height+engagement)]);
	}
}

class TableExtrusion extends MachineComponent {
	final profile:TSlotExtrusion; final length:Float; final foot:Null<TSlotTableHardware>;
	public function new(profile:TSlotExtrusion,length:Float,?foot:TSlotTableHardware) {
		this.profile=profile; this.length=length; this.foot=foot;
		var footHardware:TSlotTableHardware=cast foot;
		super('TABLE-${profile.designation}-L$length' + (foot==null ? "" : '-TAP-${footHardware.footThread}-${footHardware.footEngagement}'),
			foot==null ? "Table extrusion" : "Table post with tapped foot bore", "aluminium 6061",true);
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		if (detail==Envelope) return Part.box(profile.size,profile.height,length);
		if (foot==null) return profile.geometry(length);
		var footHardware:TSlotTableHardware=cast foot;
		return Solids.building([], owned -> {
			var body=profile.geometry(length); owned.push(body);
			var screw=SocketHeadCapScrew.metric(footHardware.footThread,footHardware.footEngagement);
			var bore=TSlotTableParts.threadBore(screw.spec.tapDrill/2,-0.1,footHardware.footEngagement+2,screw.thread.mating()); owned.push(bore);
			return Solids.cut(body,[bore]);
		});
	}
}

/** Rear strap: local Y joins the hole centres; Z points outward from the table. */
class TableRearBrace extends MachineComponent {
	final length:Float; final width:Float; final thickness:Float; final hole:Float;
	public function new(length:Float,width:Float,thickness:Float,hole:Float) {
		this.length=length; this.width=width; this.thickness=thickness; this.hole=hole;
		super('TABLE-REAR-BRACE-$length-$width-$thickness-D$hole', "Drilled rear diagonal strap", "steel",true);
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part return Solids.building([], owned -> {
		var raw=Part.box(width,length+width,thickness,Center,Min,Min); owned.push(raw);
		var body=raw.translated(new Vector(0,-width/2,0)); owned.push(body); raw.close();
		var a=TSlotTableParts.bore(hole/2,-0.1,thickness+0.1); owned.push(a);
		var b=TSlotTableParts.bore(hole/2,-0.1,thickness+0.1,0,length); owned.push(b);
		return Solids.cut(body,[a,b]);
	});
}

/** Foot adapter below a horizontal base rail. Central tapped foot bore and two recessed
 * slot screws avoid trying to tap the open extrusion slot for a vertical levelling stem. */
class TableFootPlate extends MachineComponent {
	public final width:Float;
	public final length:Float;
	public final thickness:Float;
	public final recess:Float;
	final hardware:TSlotTableHardware;
	public function new(size:Float, hardware:TSlotTableHardware, recess:Float) {
		width=size; length=2*size; thickness=hardware.footEngagement+2;
		this.hardware=hardware; this.recess=recess;
		super('TABLE-FOOT-PLATE-$size-$thickness-${hardware.footThread}-${hardware.boltSize}-CB$recess',
			"Drilled and tapped base foot adapter", "steel", true);
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part return Solids.building([], owned -> {
		var body=Part.box(width,length,thickness); owned.push(body);
		var foot=SocketHeadCapScrew.metric(hardware.footThread,hardware.footEngagement);
		var tap=TSlotTableParts.threadBore(foot.spec.tapDrill/2,-0.1,thickness+0.1,foot.thread.mating()); owned.push(tap);
		var tools=[tap];
		for (side in [-1,1]) {
			var bore=TSlotTableParts.bore(hardware.screwSpec().clearanceMedium/2,-0.1,thickness+0.1,0,side*width/2);
			owned.push(bore); tools.push(bore);
			var counter=TSlotTableParts.bore(hardware.screwSpec().counterboreDiameter/2,-0.1,recess,0,side*width/2);
			owned.push(counter); tools.push(counter);
		}
		return Solids.cut(body,tools);
	});
}
