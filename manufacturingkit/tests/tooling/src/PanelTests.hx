import materia.tooling.panel.PanelJoints;
import materia.tooling.panel.PanelCodec;
import materia.tooling.panel.PanelContours;
import materia.tooling.panel.PanelGeometry;
import materia.tooling.panel.PanelCoupon;
import materia.tooling.panel.PanelAssemblyPlan;
import materia.tooling.ToolingEnums.ContactFace;
import materia.tooling.ToolingEnums.MachiningSide;
import materia.tooling.panel.PanelPlan.PanelOperationKind;
import materia.assembly.AssemblyFrames as F;

class PanelTests {
	static var count = 0;

	static function check(ok:Bool, label:String):Void {
		count++;
		if (!ok)
			throw label;
	}

	static function rejects(fn:Void->Void, expected:String):Void {
		var message = "";
		try {
			fn();
		} catch (e:Dynamic) {
			message = Std.string(e);
		}
		check(message.length > 0 && message.indexOf(expected) >= 0, 'Expected $expected, got $message');
	}

	public static function run():Void {
		var uprightSlot = PanelContours.obround(12, 20, 3.2, 5.2);
		PanelGeometry.validate(uprightSlot);
		var uprightBounds = PanelGeometry.bounds(uprightSlot);
		check(Math.abs(uprightBounds.width - 3.2) + Math.abs(uprightBounds.height - 5.2) < 1e-7,
			"vertical locator slot retains exact extents");
		check(Math.abs(PanelGeometry.area(uprightSlot) - (3.2 * 2 + Math.PI * 1.6 * 1.6)) < 1e-7,
			"vertical locator slot retains exact line/arc area");
		for (t in [17.6, 18.0, 18.3]) {
			var stock = PanelJoints.stock(t);
			var pair = PanelJoints.tabSlot({
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
			PanelAssemblyPlan.validate(PanelAssemblyPlan.pair(pair));
			for (p in [pair.fixed, pair.moving]) {
				check(PanelCodec.encode(PanelCodec.decode(PanelCodec.encode(p))) == PanelCodec.encode(p), "exact panel codec roundtrip");
				check(PanelContours.outside(p).length > 0, "exact outside operation");
			}
			var slot = pair.fixed.operations.filter(o -> o.id == "slot-0")[0],
				bounds = PanelGeometry.bounds(PanelContours.operation(slot));
			check(Math.abs(bounds.height - (t + 0.2 + 6)) < 1e-6, "measured stock controls slot and corner overcut");
			check(pair.moving.operations[0].contour.filter(s -> s.kind == "arc").length == 4, "root relief arcs in exact outside contour");
		}
		var stock = PanelJoints.stock(17.8);
		for (pair in [PanelJoints.rebate("rebate", stock), PanelJoints.pinBolt("pins", stock)]) {
			PanelAssemblyPlan.validate(PanelAssemblyPlan.pair(pair));
			check(PanelCodec.decode(PanelCodec.encode(pair.fixed)).id == pair.fixed.id, "joint panel recipes");
		}
		var coupon = PanelCoupon.make(stock, [0.1, 0.2, 0.3, 0.4]);
		check(coupon.receiver.operations.length == 5, "one outside and four coupon slot operations");
		rejects(() -> PanelCoupon.make(stock, [0.1, 0.1]), "Invalid or duplicate coupon clearance");
		var bad = PanelJoints.blank("bad", 100, 100, stock);
		bad.operations[0].kind = cast "inferred";
		rejects(() -> PanelCodec.snapshot(bad), "Unsupported panel operation kind");
		var circle = PanelContours.circle(20, 20, 10);
		PanelGeometry.validate(circle);
		check(Math.abs(PanelGeometry.area(circle) - Math.PI * 100) < 1e-7, "exact arc area");
		check(PanelGeometry.inside(circle, 20, 20) && !PanelGeometry.inside(circle, 40, 20), "exact circle containment");
		var relief = PanelContours.relievedRectangle(50, 50, 30, 20, 3);
		PanelGeometry.validate(relief);
		check(Math.abs(PanelGeometry.area(relief) - (600 + 3 * Math.PI * 9)) < 1e-6, "single relieved contour equals rectangle and exterior corner sectors");
		var mismatch = PanelJoints.pinBolt("wrong", stock),
			plan = PanelAssemblyPlan.pair(mismatch);
		plan.members[1].pose.x += 1;
		rejects(() -> PanelAssemblyPlan.validate(plan), "Misaligned or reversed joint");
		var valid = PanelAssemblyPlan.pair(mismatch);
		var independent = PanelAssemblyPlan.pair(mismatch);
		independent.members[1].withdrawal.z = -1;
		check(independent.members[0].withdrawal.z == 1, "members own independent removal directions");
		rejects(() -> PanelAssemblyPlan.removalOrder(valid, [
			{movingId: mismatch.fixed.id, blockerId: mismatch.moving.id},
			{
				movingId: mismatch.moving.id,
				blockerId: mismatch.fixed.id
			}
		]), "Cyclic panel blocking graph");
		var reversed = PanelAssemblyPlan.pair(mismatch);
		reversed.members[1].pose = F.compose(reversed.members[1].pose, F.turnY(Math.PI));
		rejects(() -> PanelAssemblyPlan.validate(reversed), "Misaligned or reversed joint");
		var wrongSide = PanelJoints.blank("wrong-side", 100, 100, stock);
		wrongSide.operations[0].side = cast "sideways";
		rejects(() -> PanelCodec.encode(wrongSide), "Invalid machining side");
		var skin = PanelJoints.blank("contact", 100, 100, stock);
		skin.contactFace = ContactFace.PositiveZ;
		rejects(() -> PanelCodec.encode(skin), "Label zone on contact face");
		var wrongTool = PanelCodec.snapshot(coupon.receiver);
		wrongTool.operations.filter(o -> o.kind == InsideProfile)[0].toolDiameter = 8;
		rejects(() -> PanelCodec.encode(wrongTool), "Relief cutter must match operation tool");
		var arc = PanelCodec.snapshot(coupon.plug);
		var broken = arc.operations[0].contour.filter(s -> s.kind == "arc")[0];
		// Replace the point: adjacent segments can share the original endpoint.
		broken.end = {x: broken.end.x + 1, y: broken.end.y};
		rejects(() -> PanelCodec.encode(arc), "Invalid panel arc");
		var pocket = PanelJoints.blank("pocket", 100, 100, stock);
		pocket.operations.push(PanelJoints.operation("pocket", Pocket, PanelContours.circle(30, 30, 10), 4, 6, MachiningSide.NegativeZ, false));
		check(PanelCodec.decode(PanelCodec.encode(pocket)).operations.length == 2, "explicit back-side pocket and setup");
		pocket.operations[1].setupId = "front";
		rejects(() -> PanelCodec.encode(pocket), "Setup contains opposite machining faces");
		rejects(() -> PanelJoints.rebate("thin", PanelJoints.stock(8)), "Rebate leaves insufficient shelf");
		rejects(() -> PanelGeometry.validate(PanelContours.rectangle(0, 0, 1e200, 1e200)), "Panel contour must have finite positive CCW area");
		var diagonal = PanelJoints.pinBolt("diagonal", stock, 8, 0.2, 2, 7, 6, 6, {
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
		});
		PanelAssemblyPlan.validate(PanelAssemblyPlan.pair(diagonal));
		check(diagonal.moving.operations.filter(o -> o.id == "secondary-pin")[0].contour[0].start.y != diagonal.moving.operations.filter(o ->
			o.id == "secondary-pin")[0].contour[0].end.y,
			"secondary relief follows arbitrary pin baseline");
		var narrow = PanelJoints.blank("narrow", 120, 120, stock);
		narrow.operations.push(PanelJoints.operation("slot", InsideProfile, PanelContours.obround(40, 40, 20, 5), stock.measuredThickness, 6));
		rejects(() -> PanelCodec.encode(narrow), "Cutter does not fit slot");
		var sharp = PanelJoints.blank("sharp", 120, 120, stock);
		sharp.operations.push(PanelJoints.operation("square", Pocket, PanelContours.rectangle(30, 30, 20, 20), 4, 6, MachiningSide.NegativeZ, false));
		rejects(() -> PanelCodec.encode(sharp), "Cutter cannot cut sharp internal corner");
		rejects(() -> PanelJoints.pinBolt("oversized", stock, 4, 0.1, 2, 7, 6), "Cutter does not fit slot");
		var rebateLigament = PanelJoints.blank("rebate-ligament", 120, 120, stock);
		rebateLigament.operations.push(PanelJoints.operation("edge", Rebate, PanelContours.rectangle(0, 0, 10, 120), 4, 6, MachiningSide.NegativeZ, false));
		rebateLigament.operations.push(PanelJoints.operation("nearby", Pocket, PanelContours.circle(15, 40, 3), 4, 6, MachiningSide.NegativeZ, false));
		rejects(() -> PanelCodec.encode(rebateLigament), "Insufficient ligament between cuts");
		var reliefRadius = PanelJoints.blank("small-radius", 120, 120, stock);
		reliefRadius.operations.push(PanelJoints.operation("small-radius", Pocket, PanelContours.relievedRectangle(40, 40, 30, 20, 2), 4, 6,
			MachiningSide.NegativeZ, false));
		rejects(() -> PanelCodec.encode(reliefRadius), "Cutter exceeds internal radius");
		var smallArc = PanelJoints.blank("small-arc", 120, 120, stock);
		smallArc.operations.push(PanelJoints.operation("circle", Pocket, PanelContours.circle(40, 40, 2), 4, 6, MachiningSide.NegativeZ, false));
		rejects(() -> PanelCodec.encode(smallArc), "Cutter does not fit slot");
		var fit = PanelJoints.tabSlot({
			id: "fit",
			fixedStock: stock,
			movingStock: stock,
			fixedWidth: 200,
			fixedHeight: 150,
			movingWidth: 120,
			bodyHeight: 70,
			offsetX: 20,
			slotY: 80,
			tabs: [
				{
					center: 60.0,
					width: 30.0
				}
			],
			clearance: 0.2,
			routerDiameter: 6,
			minimumLigament: 6
		});
		fit.joint.fits[0].tabWidth += 1;
		rejects(() -> PanelAssemblyPlan.validate(PanelAssemblyPlan.pair(fit)), "Joint slot/tab dimensions disagree");
		rejects(() -> PanelJoints.validateClearance(0.2, 0.1, 0.1, 0.05), "Joint clearance below stock variation and allowances");
		var missing:Dynamic = PanelJoints.blank("missing", 120, 120, stock);
		Reflect.deleteField(missing, "contactOpenings");
		rejects(() -> PanelCodec.snapshot(cast missing), "Expected panel array contactOpenings");
		var nul = PanelJoints.blank("nul", 120, 120, stock);
		var nulJson = PanelCodec.encode(nul).split("birch plywood").join("board" + "\\u0000");
		rejects(() -> PanelCodec.decode(nulJson), "NUL");
		var cyclicMessage = "";
		try
			PanelAssemblyPlan.topological(["dependent", "a", "b"], [
				{movingId: "dependent", blockerId: "a"},
				{
					movingId: "a",
					blockerId: "b"
				},
				{
					movingId: "b",
					blockerId: "a"
				}
			])
		catch (e:Dynamic)
			cyclicMessage = Std.string(e);
		check(cyclicMessage == "Cyclic panel blocking graph: a, b; edges: a -> b, b -> a", "cycle diagnostic excludes downstream dependents");
		Sys.println('PanelTests: $count assertions passed');
	}
}
