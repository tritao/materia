package materia.tooling.casting;

import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingRecipe;
import materia.tooling.ToolingEnums.MachiningSide;
import materia.tooling.panel.PanelJoints;
import materia.tooling.panel.PanelContours;
import materia.tooling.panel.PanelCodec;
import materia.tooling.panel.PanelPlan;
import materia.tooling.panel.PanelAssemblyPlan.PanelPlacement;
import materia.tooling.casting.CoreSupport;
import materia.tooling.casting.CoreSupport.CoreSupportOptions;
import materia.tooling.casting.CoreSupport.CoreSupportMode;
import materia.tooling.casting.CoreSupport.CoreSupportKind;
import materia.tooling.casting.MoldPlan.CorePlan;
import materia.tooling.casting.MoldPlan.MoldBox;
import materia.tooling.casting.MoldPlan.PenetrationPlan;

private typedef BridgeRegistration = {
	var receiver:PanelPlan;
	var bridge:PanelPlan;
	var receiverOrigin:{x:Float, y:Float};
	var bridgeOrigin:{x:Float, y:Float};
	var pin:{x:Float, y:Float};
	var bolt:{x:Float, y:Float};
	var secondary:Bool;
	var angle:Float;
	var receiverPinId:String;
	var bridgePinId:String;
	var receiverBoltId:String;
	var bridgeBoltId:String;
}

/** Two explicit support templates; neither establishes adhesive or structural qualification. */
class CoreSupports {
	public static function defaults(r:ToolingRecipe):CoreSupportOptions
		return {
			mode: Auto,
			embedDepth: r.panel.measuredThickness / 3,
			fitClearance: r.jointClearance,
			adhesiveThickness: r.adhesiveLineThickness
		};

	public static function options(raw:Dynamic):CoreSupportOptions {
		if (raw == null || Reflect.fields(raw).length != 4)
			throw "Invalid core support options";
		for (k in [
			        "mode",      "embedDepth",
			"fitClearance", "adhesiveThickness"
		])
			if (!Reflect.hasField(raw, k))
				throw "Missing core support option " + k;
		var token:Dynamic = Reflect.field(raw, "mode");
		if (!Std.isOfType(token, String))
			throw "Invalid core support mode";
		var text:String = cast token;
		var mode:CoreSupportMode = switch text {
			case "auto":
				Auto;
			case "bridge":
				Bridge;
			case "bed-socket":
				BedSocket;
			default:
				throw "Invalid core support mode";
		};
		function number(k:String, zero:Bool):Float {
			var v:Dynamic = Reflect.field(raw, k);
			if ((!Std.isOfType(v, Float) && !Std.isOfType(v, Int)) || !Math.isFinite(v) || (zero ? v < 0 : v <= 0))
				throw "Invalid core support dimension " + k;
			return v;
		}
		return {
			mode: mode,
			embedDepth: number("embedDepth", false),
			fitClearance: number("fitClearance", true),
			adhesiveThickness: number("adhesiveThickness", false)
		};
	}

	public static function cutting(b:MoldBox, axis:materia.tooling.ToolingRecipe.CoreWireAxis):materia.tooling.casting.CoreSupport.CoreCutting {
		var v = switch axis {
			case X:
				{x: 1.0, y: 0.0, z: 0.0};
			case Y:
				{x: 0.0, y: 1.0, z: 0.0};
			case Z:
				{x: 0.0, y: 0.0, z: 1.0};
			default:
				throw "Invalid core wire axis";
		};
		return {
			block: {
				x: 0,
				y: 0,
				z: 0,
				width: b.width,
				depth: b.depth,
				height: b.height
			},
			stockToMold: F.translation(b.x, b.y, b.z),
			wireDirection: v,
			entryFace: {
				origin: {
					x: 0,
					y: 0,
					z: 0
				},
				normal: {
					x: -v.x,
					y: -v.y,
					z: -v.z
				}
			},
			exitFace: {
				origin: {x: v.x * b.width, y: v.y * b.depth, z: v.z * b.height},
				normal: {x: v.x, y: v.y, z: v.z}
			}
		};
	}

	public static function validateCutting(actual:materia.tooling.casting.CoreSupport.CoreCutting, b:MoldBox, r:ToolingRecipe):Void {
		if (materia.tooling.ToolingCodec.canonical(actual) != materia.tooling.ToolingCodec.canonical(cutting(b, r.coreWireAxis)))
			throw "Invalid ruled core cutting definition";
	}

	public static function kind(o:CoreSupportOptions, cb:MoldBox, b:MoldBox):CoreSupportKind {
		var bottom = Math.abs(cb.z - b.z) < 1e-7,
			top = Math.abs(cb.z + cb.height - b.z - b.height) < 1e-7;
		if (o.mode == BedSocket || (o.mode == Auto && bottom)) {
			if (!bottom)
				throw "Bed socket requires a cavity opening onto the bed";
			return BedSocket;
		}
		if (!top)
			throw "Bridge requires a cavity opening onto the fill face";
		return Bridge;
	}

	public static function build(c:CorePlan, b:MoldBox, r:ToolingRecipe, o:CoreSupportOptions, bed:PanelPlacement, panels:Array<PanelPlacement>,
			penetrations:Array<PenetrationPlan>):Void {
		var s = c.support, t = r.panel.measuredThickness, l = r.minimumLigament, tool = r.routerDiameter, g = o.adhesiveThickness, hw = r.hardware;
		PanelJoints.validateClearance(o.fitClearance, r.panel.thicknessVariation, r.jointLiningAllowance, r.jointMachiningAllowance);
		if (s.kind == BedSocket) {
			if (o.embedDepth > t - l || o.embedDepth <= 2 * g)
				throw "Core socket leaves insufficient bed stock or glue space";
			var inset = tool + l, foot:MoldBox = {
				x: c.foam.x + inset,
				y: c.foam.y + inset,
				z: b.z - o.embedDepth + g,
				width: c.foam.width - 2 * inset,
				depth: c.foam.depth - 2 * inset,
				height: o.embedDepth - 2 * g
			};
			if (foot.width <= tool || foot.depth <= tool)
				throw "Core too small for locating foot";
			s.foot = {foam: foot, cutting: cutting(foot, r.coreWireAxis)};
			s.embedDepth = o.embedDepth;
			s.bondAreaMm2 = foot.width * foot.depth;
			s.socketOperationId = materia.tooling.ToolingIds.derive("", c.id, "-socket");
			var op = PanelJoints.operation(s.socketOperationId, Pocket,
				PanelContours.relievedRectangle(foot.x
					+ foot.width / 2
					- bed.pose.x, foot.y
					+ foot.depth / 2
					- bed.pose.y, foot.width
					+ o.fitClearance,
					foot.depth
					+ o.fitClearance, tool / 2),
				o.embedDepth, tool, MachiningSide.PositiveZ, false, tool);
			var bounds = materia.casting.CastingProfileGeometry.bounds(op.contour), cb = c.cavity;
			if (bounds.x + bed.pose.x < cb.x + l
				|| bounds.y + bed.pose.y < cb.y + l
				|| bounds.x + bounds.width + bed.pose.x > cb.x + cb.width - l
				|| bounds.y + bounds.height + bed.pose.y > cb.y + cb.depth - l)
				throw "Core socket escapes sealing footprint";
			bed.panel.operations.push(op);
			bed.panel.contactOpenings.push(op.id);
			penetrations.push({
				panelId: bed.panel.id,
				operationId: op.id,
				kind: CoreSealed,
				insertId: null,
				coreId: c.id,
				sealMargin: l,
				sealMaterial: hw.sealMaterial,
				sealingFace: F.translation(cb.x + cb.width / 2, cb.y + cb.depth / 2, b.z),
				plug: null
			});
			return;
		}
		// The carrier is bonded to the foam; the bridge is bonded to the carrier.
		// Removable top pads register over the entire wall top in blind sockets.
		// Their locating pins and separate bolts sit outside the casting boundary.
		var edge = 2 * l + tool,
			padDepth = hw.padDepth,
			padWidth = b.width + 2 * t + 2 * edge,
			px = b.x - t - edge;
		var cx = c.foam.x + c.foam.width / 2,
			bw = Math.max(c.foam.width, hw.minimumBridgeWidth),
			bridgeX = cx - bw / 2;
		var southY = b.y - padDepth - edge,
			northY = b.y + b.depth,
			bridgeY = southY,
			bridgeDepth = b.depth + 2 * (padDepth + edge);
		var carrier = PanelJoints.blank(materia.tooling.ToolingIds.derive("", c.id, "-carrier"), c.foam.width, c.foam.depth, r.panel, tool, l);
		var bridge = PanelJoints.blank(materia.tooling.ToolingIds.derive("", c.id, "-bridge"), bw, bridgeDepth, r.panel, tool, l);
		function placed(p:PanelPlan, x:Float, y:Float, z:Float):Void {
			panels.push({
				panel: PanelCodec.snapshot(p),
				pose: F.translation(x, y, z),
				withdrawal: {x: 0, y: 0, z: 1}
			});
			s.panelIds.push(p.id);
		}
		for (north in [false, true]) {
			var id = materia.tooling.ToolingIds.derive("", c.id, north ? "-north-pad" : "-south-pad"),
				py = north ? northY : southY;
			var pad = PanelJoints.blank(id, padWidth, padDepth + edge, r.panel, tool, l),
				slotY = north ? t / 2 : padDepth + edge - t / 2;
			pad.labelY = north ? padDepth + edge - hw.labelOffset : hw.labelOffset;
			pad.operations.push(PanelJoints.operation("wall-top-socket", Rebate,
				PanelContours.relievedRectangle(padWidth / 2, slotY, b.width + 2 * t + o.fitClearance, t + o.fitClearance, tool / 2), t / 2, tool,
				MachiningSide.NegativeZ, false, tool));
			var y = north ? b.y + b.depth + 2 * t : b.y - 2 * t,
				pinx = cx - bw / 4,
				boltx = cx + bw / 4;
			registration({
				receiver: pad,
				bridge: bridge,
				receiverOrigin: {x: px, y: py},
				bridgeOrigin: {x: bridgeX, y: bridgeY},
				pin: {x: pinx, y: y},
				bolt: {x: boltx, y: y},
				secondary: north,
				angle: Math.PI / 2,
				receiverPinId: "locating-pin",
				bridgePinId: north ? "secondary-slot" : "primary-pin",
				receiverBoltId: "clamp-bolt",
				bridgeBoltId: north ? "north-bolt" : "south-bolt"
			}, r, o.fitClearance);
			var prefix = materia.tooling.ToolingIds.derive("", c.id, north ? "-north" : "-south");
			supportHardware(s.hardware, materia.tooling.ToolingIds.derive("", prefix, "-pin"), materia.tooling.ToolingIds.derive("", prefix, "-bolt"),
				{x: pinx, y: y}, {x: boltx, y: y}, b.z + b.height, r);
			placed(pad, px, py, b.z + b.height);
		}
		placed(carrier, c.foam.x, c.foam.y, b.z + b.height);
		placed(bridge, bridgeX, bridgeY, b.z + b.height + t);
		s.bondedPanelIds = [carrier.id, bridge.id];
		s.bondAreaMm2 = c.foam.width * c.foam.depth;
	}

	public static function validateFoot(c:CorePlan, r:ToolingRecipe, o:CoreSupportOptions, bed:MoldBox):Void {
		var s = c.support, foot = s.foot;
		if (s.kind != CoreSupportKind.BedSocket)
			return;
		if (foot == null)
			throw "Missing locating foot";
		var f = foot.foam, b = foot.cutting.block, p = foot.cutting.stockToMold, v = foot.cutting.wireDirection;
		for (n in [
			f.x, f.y, f.z, f.width, f.depth, f.height, b.x, b.y, b.z, b.width, b.depth, b.height, p.x, p.y, p.z, p.qx, p.qy, p.qz, p.qw
		])
			if (!Math.isFinite(n))
				throw "Invalid foot geometry";
		validateCutting(foot.cutting, f, r);
		var inset = r.routerDiameter + r.minimumLigament;
		if (f.width <= 0
			|| f.depth <= 0
			|| f.height <= 0
			|| f.width > r.core.width
			|| f.depth > r.core.depth
			|| f.height > r.core.height
			|| Math.abs(f.x - c.foam.x - inset)
			+ Math.abs(f.y - c.foam.y - inset)
			+ Math.abs(f.width - c.foam.width + 2 * inset)
			+ Math.abs(f.depth - c.foam.depth + 2 * inset)
			+ Math.abs(f.z - bed.z + o.embedDepth - o.adhesiveThickness)
			+ Math.abs(f.height - o.embedDepth + 2 * o.adhesiveThickness) > 1e-7)
			throw "Locating foot does not match support";
		if (b.x != 0
			|| b.y != 0
			|| b.z != 0
			|| Math.abs(b.width - f.width) + Math.abs(b.depth - f.depth) + Math.abs(b.height - f.height) > 1e-7
			|| Math.abs(p.x - f.x)
			+ Math.abs(p.y - f.y)
			+ Math.abs(p.z - f.z)
			+ Math.abs(p.qx)
			+ Math.abs(p.qy)
			+ Math.abs(p.qz) > 1e-7 || Math.abs(Math.abs(p.qw) - 1) > 1e-8)
			throw "Invalid foot cutting definition";
	}

	public static function sharedBridges(b:MoldBox, r:ToolingRecipe, cores:Array<materia.tooling.casting.CastingMoldPlan.CastingMoldCore>,
			panels:Array<PanelPlacement>, hardware:Array<materia.tooling.casting.CoreSupport.SupportHardware>):Void {
		if (cores.length == 0)
			return;
		var t = r.panel.measuredThickness,
			l = r.minimumLigament,
			edge = 2 * l + r.routerDiameter,
			pad = r.hardware.padDepth;
		var rows:Array<Float> = [];
		for (c in cores) {
			var y = c.cavity.y + c.cavity.depth / 2;
			if (rows.filter(v -> Math.abs(v - y) < 1e-7).length == 0)
				rows.push(y);
		}
		rows.sort(Reflect.compare);
		function place(panel:materia.tooling.panel.PanelPlan, x:Float, y:Float, z:Float):Void
			panels.push({
				panel: PanelCodec.snapshot(panel),
				pose: F.translation(x, y, z),
				withdrawal: {
					x: 0,
					y: 0,
					z: 1
				}
			});
		var rails = [];
		for (right in [false, true]) {
			var x = right ? b.x + b.width : b.x - pad - edge,
				y = b.y - t - edge;
			var rail = PanelJoints.blank(right ? "bridge-right-rail" : "bridge-left-rail", pad + edge, b.depth + 2 * t + 2 * edge, r.panel, r.routerDiameter,
				l);
			rail.labelX = right ? pad + edge - 2 * r.hardware.labelOffset : 2 * r.hardware.labelOffset;
			rail.operations.push(PanelJoints.operation("wall-top-socket", Rebate,
				PanelContours.relievedRectangle(right ? t / 2 : pad + edge - t / 2, (b.depth + 2 * t + 2 * edge) / 2, t + r.jointClearance,
					b.depth + r.jointClearance, r.routerDiameter / 2),
				t / 2, r.routerDiameter, MachiningSide.NegativeZ, false, r.routerDiameter));
			rails.push({
				panel: rail,
				x: x,
				y: y,
				right: right
			});
		}
		for (index in 0...rows.length) {
			var y = rows[index], width = 0.0;
			for (core in cores)
				if (Math.abs(core.cavity.y + core.cavity.depth / 2 - y) < 1e-7)
					width = Math.max(width, core.cavity.depth / 2);
			if (width < r.hardware.minimumBridgeWidth)
				throw "Core row cannot provide the minimum bridge width";
			var id = "core-bridge-" + index,
				beam = PanelJoints.blank(id, b.width + 2 * (pad + edge), width, r.panel, r.routerDiameter, l),
				bx = b.x - pad - edge,
				by = y - width / 2;
			beam.labelY = width / 2;
			for (rail in rails) {
				var x = rail.right ? b.x + b.width + 2 * t : b.x - 2 * t,
					py = y - width / 4,
					byy = y + width / 4,
					d = r.hardware.locatingPinDiameter + r.jointClearance;
				registration({
					receiver: rail.panel,
					bridge: beam,
					receiverOrigin: {x: rail.x, y: rail.y},
					bridgeOrigin: {x: bx, y: by},
					pin: {x: x, y: py},
					bolt: {x: x, y: byy},
					secondary: rail.right,
					angle: 0,
					receiverPinId: "pin-" + index,
					bridgePinId: rail.right ? "secondary-slot" : "primary-pin",
					receiverBoltId: "bolt-" + index,
					bridgeBoltId: rail.right ? "right-bolt" : "left-bolt"
				}, r, r.jointClearance);
				supportHardware(hardware, id
					+ (rail.right ? "-right-pin" : "-left-pin"), id
					+ (rail.right ? "-right-bolt" : "-left-bolt"), {x: x, y: py},
					{x: x, y: byy}, b.z
					+ b.height, r);
			}
			place(beam, bx, by, b.z + b.height + t);
			for (c in cores)
				if (Math.abs(c.cavity.y + c.cavity.depth / 2 - y) < 1e-7) {
					c.bridgeId = id;
					var f = c.foamLayers[0];
					c.bridgeBondAreaMm2 = f.width * Math.max(0, Math.min(y + width / 2, f.y + f.depth) - Math.max(y - width / 2, f.y));
					place(PanelJoints.blank(c.carrierId, f.width, f.depth, r.panel, r.routerDiameter, l), f.x, f.y, b.z + b.height);
				}
		}
		for (rail in rails)
			place(rail.panel, rail.x, rail.y, b.z + b.height);
	}

	static function registration(s:BridgeRegistration, r:ToolingRecipe, clearance:Float):Void {
		var h = r.hardware,
			d = h.locatingPinDiameter + clearance,
			t = r.panel.measuredThickness;
		var pinX = s.pin.x - s.receiverOrigin.x,
			pinY = s.pin.y - s.receiverOrigin.y;
		s.receiver.operations.push(PanelJoints.operation(s.receiverPinId, Drill, PanelContours.circle(pinX, pinY, d / 2), t, d));
		s.receiver.operations.push(PanelJoints.operation(s.receiverBoltId, Drill,
			PanelContours.circle(s.bolt.x - s.receiverOrigin.x, s.bolt.y - s.receiverOrigin.y, h.supportClearanceDiameter / 2), t, h.supportClearanceDiameter));
		var x = s.pin.x - s.bridgeOrigin.x, y = s.pin.y - s.bridgeOrigin.y;
		var contour = s.secondary ? PanelContours.placed(PanelContours.obround(0, 0, d + h.secondaryPinTravel, d), s.angle, x,
			y) : PanelContours.circle(x, y, d / 2);
		s.bridge.operations.push(PanelJoints.operation(s.bridgePinId, s.secondary ? InsideProfile : Drill, contour, t, s.secondary ? r.routerDiameter : d));
		s.bridge.operations.push(PanelJoints.operation(s.bridgeBoltId, Drill,
			PanelContours.circle(s.bolt.x - s.bridgeOrigin.x, s.bolt.y - s.bridgeOrigin.y, h.supportClearanceDiameter / 2), t, h.supportClearanceDiameter));
	}

	static function supportHardware(out:Array<SupportHardware>, pinId:String, boltId:String, pin:{x:Float, y:Float}, bolt:{x:Float, y:Float}, z:Float,
			r:ToolingRecipe):Void {
		var t = r.panel.measuredThickness;
		out.push({
			id: pinId,
			kind: LocatingPin,
			pose: F.translation(pin.x, pin.y, z),
			diameter: r.hardware.locatingPinDiameter,
			length: 2 * t
		});
		out.push({
			id: boltId,
			kind: ClampBolt,
			pose: F.translation(bolt.x, bolt.y, z + 2 * t),
			diameter: r.hardware.supportScrewDiameter,
			length: 2 * t
		});
	}
}
