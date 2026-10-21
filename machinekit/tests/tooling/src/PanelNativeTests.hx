import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.drawing.Svg;
import cadkit.drawing.Dxf;
import cadkit.drawing.DrawingEntity;
import cadkit.drawing.Drawing2D;
import machinekit.tooling.ToolingPanel;
import machinekit.tooling.PanelKit;
import machinekit.tooling.PanelCad;
import machinekit.tooling.PanelDrawing;
import machinekit.tooling.SweptRemoval;
import machinekit.assembly.MachineAssembly;
import machinekit.component.MachineKitComponents;
import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingEnums.MachiningSide;
import materia.tooling.panel.PanelJoints;
import materia.tooling.panel.PanelContours;
import materia.casting.CastingProfileGeometry;
import materia.tooling.panel.PanelAssemblyPlan;
import materia.tooling.panel.PanelCoupon;
import materia.tooling.panel.PanelCodec;
import materia.tooling.panel.PanelPlan.PanelOperationKind;

class PanelNativeTests {
	static var count = 0;

	static function check(ok:Bool, label:String):Void {
		count++;
		if (!ok)
			throw label;
	}

	static function near(a:Float, b:Float, label:String):Void
		check(Math.abs(a - b) < 1e-4, '$label: $a vs $b');

	static function rejects(fn:Void->Void, expected:String):Void {
		var message = "";
		try
			fn()
		catch (e:Dynamic)
			message = Std.string(e);
		check(message.indexOf(expected) >= 0 && message.length > 0, 'Expected $expected, got $message');
	}

	public static function run():Void {
		var stock = PanelJoints.stock(17.8);
		var keyed = PanelJoints.tabSlot({
			id: "keyed",
			fixedStock: stock,
			movingStock: stock,
			fixedWidth: 320,
			fixedHeight: 220,
			movingWidth: 280,
			bodyHeight: 100,
			offsetX: 20,
			slotY: 150,
			tabs: [
				{
					center: 55.0,
					width: 30.0
				},
				{
					center: 210.0,
					width: 44.0
				}
			],
			clearance: 0.2,
			routerDiameter: 6,
			minimumLigament: 6
		});
		var pairs = [
			keyed,
			PanelJoints.rebate("rebate", stock),
			PanelJoints.pinBolt("pins", stock),
			PanelJoints.pinBolt("diagonal", stock, 8, 0.2, 2, 7, 6, 6, {
				width: 220,
				height: 160,
				primary: {
					x: 40,
					y: 40
				},
				secondary: {
					x: 140,
					y: 80
				},
				bolt: {
					x: 90,
					y: 110
				}
			})
		];
		for (pair in pairs) {
			Sys.println("Checking exact panel pair: " + pair.joint.id);
			for (p in [pair.fixed, pair.moving]) {
				var component = ToolingPanel.fromPlan(p),
					solid = component.geometry();
				check(solid.valid() && solid.solidCount() == 1, "connected valid panel solid");
				var expected = 0.0;
				for (op in p.operations)
					expected += (op.kind == OutsideProfile ? 1 : -1) * CastingProfileGeometry.area(PanelContours.operation(op)) * op.depth;
				near(solid.volume(), expected, "CAD uses exact operation areas and depths");
				solid.close();
				var drawing = PanelDrawing.drawing(p), curveCount = 0;
				for (e in drawing.entities())
					switch e {
						case Curve(_, _):
							curveCount++;
						case _:
					}
				var expectedCount = 0;
				for (op in p.operations)
					expectedCount += PanelContours.operation(op).length;
				check(curveCount == expectedCount, "drawing curves match CAD operation records");
				var svg = Svg.write(drawing), dxf = Dxf.write(drawing);
				for (op in p.operations) {
					var layer = (op.kind : String).toUpperCase().split("-").join("_");
					check(svg.indexOf('id="' + layer + '"') >= 0 && dxf.indexOf(layer) >= 0, "SVG/DXF operation layer");
				}
				if (p.operations.filter(o -> o.contour.filter(s -> s.kind == "arc").length > 0).length > 0)
					check(svg.indexOf("<path") >= 0 && dxf.indexOf("ARC") >= 0, "exact arcs in both existing writers");
				var recipe = component.componentType();
				if (recipe == null)
					throw "Missing panel recipe";
				var rebuilt = MachineKitComponents.standard().byId(recipe.id).create(component.values());
				var restored = rebuilt.geometry(),
					source = ToolingPanel.fromPlan(p).geometry();
				var missing = source.subtractAll([restored]),
					excess = restored.subtractAll([source]);
				near(missing.volume() + excess.volume(), 0, "recipe symmetric difference");
				missing.close();
				excess.close();
				source.close();
				restored.close();
				check(rebuilt.designation == component.designation && rebuilt.materialSpec() == component.materialSpec(),
					"part identity and material survive recipe");
				for (connector in component.connectors()) {
					var actual = rebuilt.connector(connector.name),
						delta = F.compose(F.inverse(connector.frame), actual.frame);
					check(actual.role == connector.role
						&& Math.abs(delta.x)
						+ Math.abs(delta.y)
						+ Math.abs(delta.z)
						+ Math.abs(delta.qx)
						+ Math.abs(delta.qy)
						+ Math.abs(delta.qz) < 1e-8,
						"connector frame and role survive recipe");
				}
			}
			var plan = PanelAssemblyPlan.pair(pair),
				report = PanelKit.check(plan);
			check(report.removalOrder.indexOf(pair.moving.id) < report.removalOrder.indexOf(pair.fixed.id), "geometric blocking order");
			var saved = PanelKit.assembly(plan).encode(),
				restored = MachineAssembly.decode(saved);
			check(restored.components().length == 2, "panel assembly reconstruction");
			var restoredPose = restored.solvedPoses().get(pair.moving.id);
			if (restoredPose == null)
				throw "Missing restored pose";
			near(restoredPose.z, plan.members[1].pose.z, "assembly placement survives reconstruction");
			var definition=machinekit.assembly.AssemblyPreview.scene(restored,"panel-presentation").assemblyDefinition;
			var presentation=PanelKit.presentation(plan,definition);
			var cursor=materia.presentation.PresentationEvaluator.reset(presentation);cursor.explode=1;
			var shown=materia.presentation.PresentationEvaluator.evaluate(definition,restored.solvedPoses(),presentation,cursor,"");
			check(presentation.explode.length==2,"Both panels have explode offsets");
			check(restored.encode()==saved,"Presentation evaluation preserves the jointed engineering recipe");
			var first=shown.poses.get(pair.fixed.id),second=shown.poses.get(pair.moving.id);
			if(first==null||second==null)throw "Missing exploded panel poses";
			check(Math.abs(first.x-plan.members[0].pose.x)+Math.abs(first.y-plan.members[0].pose.y)+Math.abs(first.z-plan.members[0].pose.z)>1,
				"Explode displaces the fixed panel for review without deleting its joint");
		}
		var floor = PanelJoints.blank("fixture-floor", 120, 120, stock);
		var seat = PanelJoints.operation("seat", Pocket, PanelContours.relievedRectangle(50, 50, 40, 40, 3), 8, 6, MachiningSide.PositiveZ, false, 6);
		var fixing = PanelJoints.operation("fixing", Drill, PanelContours.circle(50, 50, 3.5), stock.measuredThickness, 7);
		fixing.accessVia = seat.id;
		floor.operations.push(seat);
		floor.operations.push(fixing);
		var floorSolid = ToolingPanel.fromPlan(floor).geometry();
		near(floorSolid.volume(),
			120 * 120 * stock.measuredThickness - CastingProfileGeometry.area(seat.contour) * 8 - Math.PI * 3.5 * 3.5 * (stock.measuredThickness - 8),
			"drill through pocket floor removes shared air once");
		floorSolid.close();
		fixing.accessVia = "missing";
		rejects(() -> PanelCodec.encode(floor), "Unknown operation access pocket");
		// Include the physical pins; panel-only pairs cannot test pin blocking.
		for (zeroClearance in [false, true]) {
			var pair = PanelJoints.pinBolt("physical-pins", stock);
			var localLower = ToolingPanel.fromPlan(pair.fixed).geometry();
			var localUpper = ToolingPanel.fromPlan(pair.moving).geometry();
			var upper = machinekit.assembly.PosedParts.moved(localUpper, PanelJoints.movingPose(pair.joint, F.identity()));
			localUpper.close();
			var diameter = zeroClearance ? 6.15 : 6.0;
			var pinStock = Part.cylinderSpan(diameter / 2, 0, 2 * stock.measuredThickness);
			var primary = pinStock.translated(new Vector(40, 35, 0));
			var secondary = pinStock.translated(new Vector(140, 35, 0));
			pinStock.close();
			var members:Array<machinekit.tooling.SweptRemoval.RemovalMember> = [
				{
					id: pair.fixed.id,
					part: localLower,
					direction: new Vector(0, 0, -1)
				},
				{
					id: pair.moving.id,
					part: upper,
					direction: Vector.X()
				},
				{
					id: "primary-pin",
					part: primary,
					direction: Vector.Z()
				},
				{
					id: "secondary-pin",
					part: secondary,
					direction: Vector.Z()
				}
			];
			var report = SweptRemoval.analyze(members);
			var edges = [for (e in report.blocking) e.movingId + "->" + e.blockerId];
			check(edges.join(",") == pair.moving.id + "->primary-pin," + pair.moving.id + "->secondary-pin",
				"physical pin pair exact blocking edges, including sliding contact");
			members.reverse();
			var permuted = SweptRemoval.analyze(members);
			check(haxe.Json.stringify(report) == haxe.Json.stringify(permuted), "member permutation preserves complete removal report");
			localLower.close();
			upper.close();
			primary.close();
			secondary.close();
		}
		var a = Part.box(10, 10, 10),
			base = Part.box(10, 10, 10),
			b = base.translated(new Vector(50, 0, 0));
		base.close();
		var endpoint = a.translated(new Vector(100, 0, 0)),
			hit = endpoint.intersect(b);
		near(hit.volume(), 0, "endpoint-only check misses middle obstacle");
		hit.close();
		endpoint.close();
		var report = SweptRemoval.analyze([
			{id: "A", part: a, direction: Vector.X()},
			{
				id: "B",
				part: b,
				direction: Vector.Z()
			}
		]);
		check(report.blocking.filter(e -> e.movingId == "A" && e.blockerId == "B").length == 1, "swept volume catches middle obstacle");
		var endB = b.translated(new Vector(-report.travelMm, 0, 0)),
			endHit = endB.intersect(a);
		near(endHit.volume(), 0, "both trapped-case endpoints are clear");
		endHit.close();
		endB.close();
		rejects(() -> SweptRemoval.analyze([
			{id: "A", part: a, direction: Vector.X()},
			{
				id: "B",
				part: b,
				direction: Vector.X().scale(-1)
			}
		]), "A, B");
		a.close();
		b.close();
		var cylinder = Part.cylinder(5, 8),
			sweep = SweptRemoval.sweep(cylinder, new Vector(20, 0, 0));
		near(sweep.volume(), (Math.PI * 25 + 200) * 8, "exact cylinder translation sweep");
		sweep.close();
		cylinder.close();
		var outer = Part.cylinder(10, 10),
			bore = Part.cylinder(4, 10),
			tube = outer.subtractAll([bore]);
		outer.close();
		bore.close();
		var pinBase = Part.cylinder(3, 10),
			pin = pinBase.translated(new Vector(0, 0, -20));
		pinBase.close();
		var pinSweep = SweptRemoval.sweep(pin, new Vector(0, 0, 50)),
			clearance = pinSweep.intersect(tube);
		near(clearance.volume(), 0, "exact sweep preserves an open bore that a bounding box would block");
		clearance.close();
		pinSweep.close();
		pin.close();
		tube.close();
		var reversed = PanelAssemblyPlan.pair(keyed), flip = F.turnY(Math.PI);
		flip.x = 280;
		flip.z = stock.measuredThickness;
		reversed.members[1].pose = F.compose(reversed.members[1].pose, flip);
		reversed.joints = [];
		rejects(() -> PanelKit.check(reversed), "Initial overlap");
		var pinPlan = PanelAssemblyPlan.pair(pairs[2]);
		var driver = Part.cylinderSpan(5, 2 * stock.measuredThickness + 10, 2 * stock.measuredThickness + 70, 90, 65);
		PanelKit.checkToolAccess(pinPlan, [{id: "bolt-driver", clearance: driver}]);
		driver.close();
		var tool = Part.box(20, 20, 10),
			blocked = tool.translated(new Vector(60, 30, stock.measuredThickness + 2));
		tool.close();
		rejects(() -> PanelKit.checkToolAccess(pinPlan, [{id: "blocked-driver", clearance: blocked}]), "Blocked tool access");
		blocked.close();
		var pocket = PanelJoints.blank("two-sided", 100, 100, stock);
		pocket.operations.push(PanelJoints.operation("pocket", Pocket, PanelContours.circle(30, 30, 10), 4, 6, MachiningSide.PositiveZ, false));
		var shape = ToolingPanel.fromPlan(pocket).geometry();
		near(shape.volume(), 100 * 100 * 17.8 - Math.PI * 100 * 4, "explicit blind pocket depth");
		shape.close();
		pocket.operations[1].side = MachiningSide.NegativeZ;
		pocket.operations[1].setupId = "back";
		var back = ToolingPanel.fromPlan(pocket).geometry(),
			probe = Part.cylinderSpan(3, 0.1, 3.9, 30, 30),
			bottom = back.intersect(probe);
		near(bottom.volume(), 0, "negative-side pocket opens on the bottom face");
		bottom.close();
		probe.close();
		back.close();
		var coupon = PanelCoupon.make(stock, [0.1, 0.2, 0.3, 0.4]);
		for (p in [coupon.receiver, coupon.plug]) {
			var shape = ToolingPanel.fromPlan(p).geometry();
			check(shape.valid(), "coupon CAD");
			shape.close();
			check(Dxf.write(PanelDrawing.drawing(p)).indexOf("ARC") >= 0, "coupon DXF retains relief arcs");
		}
		Sys.println('PanelNativeTests: $count assertions passed');
	}
}
