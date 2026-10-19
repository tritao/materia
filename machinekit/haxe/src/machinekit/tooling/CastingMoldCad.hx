package machinekit.tooling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.PosedParts;
import machinekit.component.ComponentValues;
import machinekit.component.Solids;
import machinekit.structural.CastInsert;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.standard.FlatWasher;
import machinekit.standard.HexNut;
import materia.assembly.AssemblyFrames as F;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingMoldPlanner;
import materia.tooling.casting.CastingMoldCodec;
import materia.tooling.casting.MoldPlan.MoldBox;
import materia.tooling.casting.CoreSupport.SupportHardwareKind;
import machinekit.tooling.SweptRemoval.RemovalMember;
import machinekit.tooling.SweptRemoval.RemovalReport;

enum abstract DemoldingStepKind(String) to String {
	var ReleaseHardware = "release-hardware";
	var Withdraw = "withdraw";
	var FlipToService = "flip-to-service";
}

typedef MoldDemoldingStep = {var id:String; var kind:DemoldingStepKind; var memberIds:Array<String>; var dependsOn:Array<String>; var status:materia.tooling.ToolingDiagnostic.ToolingCheckStatus;}
typedef CastingMoldAssessment = {var diagnostics:machinekit.assembly.Diagnostics; var geometry:Null<CastingMoldReview>;}

typedef CastingMoldReview = {
	var handling:HandlingPlan;
	var demoldingSteps:Array<MoldDemoldingStep>;
	var symmetricDifferenceMm3:Float;
	var sourceVolumeMm3:Float;
	var reconstructedVolumeMm3:Float;
	var assembly:RemovalReport;
	var demolding:RemovalReport;
	var permanentGroups:Array<{
		id:String,
		members:Array<String>
	}>;
	var assemblyGroups:Array<{
		id:String,
		members:Array<String>
	}>;
	var validation:Array<materia.tooling.ToolingValidation>;
	var fabrication:materia.tooling.ToolingValidation.FabricationReadiness;
	var assemblyMotions:Array<{
		id:String,
		x:Float,
		y:Float,
		z:Float
	}>;
	var demoldingMotions:Array<{
		id:String,
		x:Float,
		y:Float,
		z:Float
	}>;
}

/** CAD bridge for neutral multi-core box plans. Frames and exact operations remain authoritative. */
class CastingMoldCad {
	static function box(b:MoldBox):Part
		return ToolingBlock.placedBox(b);

	public static function source(p:CastingMoldPlan):Part {
		var pieces:Array<Part> = [box(p.envelope)];
		return Solids.building(pieces, parts -> {
			for (f in p.document.input.features)
				parts.push(box(CastingMoldPlanner.featureBox(p.document, f)));
			return Solids.cut(parts[0], parts.slice(1));
		});
	}

	public static function reconstructed(p:CastingMoldPlan):Part {
		var pieces:Array<Part> = [box(p.cavity)];
		return Solids.building(pieces, parts -> {
			for (c in p.cores)
				parts.push(box(c.cavity));
			for (f in p.formers)
				parts.push(ToolingVolume.geometryOf(f.volume));
			for (f in p.fixtures)
				parts.push(ToolingVolume.geometryOf(f.datum));
			// Full envelopes include bores protected during pouring. Their actual
			// masks are separate members in the assembly and demolding graphs.
			for (i in p.document.input.inserts)
				parts.push(box(insertBox(p, i.id)));
			return Solids.cut(parts[0], parts.slice(1));
		});
	}

	public static function steel(p:CastingMoldPlan, id:String):Part {
		var i = p.document.input.inserts.filter(i -> i.id == id)[0];
		var component = new CastInsert(new ComponentValues().setNumber("width", i.width)
			.setNumber("depth", i.depth)
			.setNumber("height", i.height)
			.setNumber("boreDiameter", i.boreDiameter));
		var local = component.geometry();
		try {
			var out = PosedParts.moved(local, F.compose(p.document.setup.sourceToMold, i.pose));
			local.close();
			return out;
		} catch (e:Dynamic) {
			local.close();
			throw e;
		}
	}

	public static function assess(p:CastingMoldPlan):CastingMoldAssessment {
		CastingMoldPlanner.validate(p);
		var diagnostics = ToolingDiagnostics.fromValidation(p.validation);
		if (p.features.filter(f -> f.kind == UnsupportedFeature).length > 0)
			return {diagnostics: diagnostics, geometry: null};
		return {diagnostics: diagnostics, geometry: review(p)};
	}

	static function steps(p:CastingMoldPlan, order:Array<String>):Array<MoldDemoldingStep> {
		var result:Array<MoldDemoldingStep> = [
			{
				id: "release-loose-tooling",
				kind: ReleaseHardware,
				memberIds: [
					for (h in p.hardware)
						if (h.kind != SupportHardwareKind.FixturePin && h.kind != SupportHardwareKind.BoreMask) h.id
				],
				dependsOn: [],
				status: materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending
			}
		];
		var previous = "release-loose-tooling";
		for (id in order) {
			var stepId = materia.tooling.ToolingIds.derive("withdraw-", id);
			result.push({
				id: stepId,
				kind: Withdraw,
				memberIds: [id],
				dependsOn: [previous],
				status: materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass
			});
			previous = stepId;
		}
		if (p.handling.requiresFlip)
			result.push({
				id: "flip-to-service",
				kind: FlipToService,
				memberIds: ["cured-casting"],
				dependsOn: [previous],
				status: materia.tooling.casting.CastHandling.qualified(p.document, p.options,
					p.handling) ? materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass : materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending
			});
		return result;
	}

	static function maskId(f:MoldInsertFixture):Null<String> {
		var mask = f.boreMask;
		return mask == null ? null : mask.id;
	}

	static function insertBox(p:CastingMoldPlan, id:String):MoldBox {
		var i = p.document.input.inserts.filter(i -> i.id == id)[0];
		return materia.tooling.casting.BoxMoldPlanner.box(i.width, i.depth, i.height, F.compose(p.document.setup.sourceToMold, i.pose));
	}

	static function panelWorld(m:materia.tooling.panel.PanelAssemblyPlan.PanelPlacement):Part {
		var local = ToolingPanel.fromPlan(m.panel).geometry();
		try {
			var out = PosedParts.moved(local, m.pose);
			local.close();
			return out;
		} catch (e:Dynamic) {
			local.close();
			throw e;
		}
	}

	public static function assembly(p:CastingMoldPlan, exploded:Bool = false):MachineAssembly {
		CastingMoldPlanner.validate(p);
		var a = exploded ? new MachineAssembly() : PanelKit.assembly(p.panels, false, "coated particleboard"), b = p.cavity;
		if (exploded)
			for (m in p.panels.members) {
				var pose = F.compose(F.identity(), m.pose);
				switch m.panel.id {
					case "bed":
						pose.z -= b.height * 0.7;
					case "east":
						pose.x += b.width;
					case "west":
						pose.x -= b.width;
					case "north":
						pose.y += b.depth * 0.4;
					case "south":
						pose.y -= b.depth * 0.4;
				}
				a.addComponent(m.panel.id, ToolingPanel.fromPlan(m.panel, "coated particleboard"), pose);
			}
		for (i in 0...p.supportPanels.length) {
			var m = p.supportPanels[i], pose = F.compose(F.identity(), m.pose);
			if (exploded)
				pose.z += b.height * (0.5 + i * 0.15);
			a.addComponent(m.panel.id, ToolingPanel.fromPlan(m.panel), pose);
		}
		for (i in 0...p.cores.length) {
			var c = p.cores[i], pose = F.identity();
			if (exploded)
				pose.z += b.height * (0.8 + i * 0.12);
			a.addComponent(c.id, ToolingVolume.fromVolume({
				id: c.id,
				material: EPSStock,
				cuts: [],
				profiles: [],
				pieces: c.foamLayers
			}), pose);
		}
		for (f in p.formers) {
			var pose = F.identity();
			if (exploded) {
				pose.x += f.withdrawal.x * b.width * 0.7;
				pose.y += f.withdrawal.y * b.depth * 0.4;
				pose.z += f.withdrawal.z * b.height * 0.7;
			}
			a.addComponent(f.volume.id, ToolingVolume.fromVolume(f.volume), pose);
		}
		for (f in p.fixtures) {
			var pose = F.identity();
			if (exploded) {
				pose.x += f.withdrawal.x * b.width;
				pose.z += f.withdrawal.z * b.height * 1.4;
			}
			a.addComponent(f.datum.id, ToolingVolume.fromVolume(f.datum), pose);
		}
		for (i in p.document.input.inserts) {
			var pose = F.compose(p.document.setup.sourceToMold, i.pose);
			if (exploded)
				pose.x += b.width * 1.8;
			var part = new CastInsert(new ComponentValues().setNumber("width", i.width)
				.setNumber("depth", i.depth)
				.setNumber("height", i.height)
				.setNumber("boreDiameter", i.boreDiameter)
				.setNumber("positionTolerance", i.positionTolerance)
				.setToken("role", i.sourceRole));
			a.addComponent(materia.tooling.ToolingIds.derive("insert-", i.id), part, pose);
		}
		if (!exploded) {
			for (h in p.hardware) {
				if (h.kind == SupportHardwareKind.LocatingPin
					|| h.kind == SupportHardwareKind.BoreMask
					|| h.kind == SupportHardwareKind.FixturePin)
					a.addComponent(h.id, new ToolingPin(new ComponentValues().setNumber("diameter", h.diameter).setNumber("length", h.length)), h.pose);
				else if (h.kind == SupportHardwareKind.FixtureScrew)
					ToolingHardwareCad.fixtureScrew(a, h.id, h.diameter, h.length, h.pose);
				else {
					ToolingHardwareCad.throughBolt(a, h.id, h.diameter, h.length, h.pose);
				}
			}
			var poses = a.solvedPoses(), inverse = F.inverse(poses.get("bed"));
			a.addMemberConnector("bed", "tooling-origin", F.identity());
			for (row in a.components())
				if (p.panels.members.filter(m -> m.panel.id == row.id).length == 0) {
					a.addMemberConnector(row.id, "tooling-datum", F.inverse(F.compose(inverse, poses.get(row.id))));
					a.addMate(materia.tooling.ToolingIds.derive("", row.id, "-positioned"), "fixed", "bed", "tooling-origin", row.id, "tooling-datum");
				}
		}
		return a;
	}

	static function coverage(p:CastingMoldPlan, tooling:Array<Part>):Void {
		var b = p.cavity, s = 0.1, faces:Array<MoldBox> = [
			{
				x: b.x - s,
				y: b.y,
				z: b.z,
				width: s,
				depth: b.depth,
				height: b.height
			},
			{
				x: b.x + b.width,
				y: b.y,
				z: b.z,
				width: s,
				depth: b.depth,
				height: b.height
			},
			{
				x: b.x,
				y: b.y - s,
				z: b.z,
				width: b.width,
				depth: s,
				height: b.height
			},
			{
				x: b.x,
				y: b.y + b.depth,
				z: b.z,
				width: b.width,
				depth: s,
				height: b.height
			},
			{
				x: b.x,
				y: b.y,
				z: b.z - s,
				width: b.width,
				depth: b.depth,
				height: s
			}
		];
		var directions = [
			new Vector(-s, 0, 0),
			new Vector(s, 0, 0),
			new Vector(0, -s, 0),
			new Vector(0, s, 0),
			new Vector(0, 0, -s)
		];
		for (index in 0...faces.length) {
			var slab = box(faces[index]);
			var owned:Array<Part> = [slab];
			try {
				// Check actual resin contact on the closed outer face. A former's
				// shoulder can seal on the face without filling a positive-depth
				// slab behind it. Voids are not resin contact surfaces.
				var casting = source(p);
				owned.push(casting);
				var translated = casting.translated(directions[index]);
				owned.push(translated);
				var contact = slab.intersect(translated);
				owned.push(contact);
				var uncovered = contact.subtractAll(tooling);
				owned.push(uncovered);
				var volume = uncovered.volume();
				if (volume > 1e-3)
					throw "Uncovered mold boundary " + index + ": " + volume + " mm3";
				for (part in owned)
					part.close();
			} catch (e:Dynamic) {
				for (part in owned)
					if (!part.shape.isClosed())
						part.close();
				throw e;
			}
		}
	}

	public static function review(p:CastingMoldPlan):CastingMoldReview {
		CastingMoldPlanner.validate(p);
		var references:Array<Part> = [], difference = 0.0, ov = 0.0, rv = 0.0;
		try {
			var original = source(p);
			references.push(original);
			var rebuilt = reconstructed(p);
			references.push(rebuilt);
			var missing = original.subtractAll([rebuilt]);
			references.push(missing);
			var excess = rebuilt.subtractAll([original]);
			references.push(excess);
			difference = missing.volume() + excess.volume();
			ov = original.volume();
			var physical = original.massProperties(),
				mass = physical.volume * p.document.recipe.mixDensityKgM3 * 1e-9;
			var mx = mass * physical.centerOfMass.x,
				my = mass * physical.centerOfMass.y,
				mz = mass * physical.centerOfMass.z;
			for (i in p.document.input.inserts) {
				var shape = steel(p, i.id);
				references.push(shape);
				var properties = shape.massProperties(),
					kg = properties.volume * p.document.recipe.steelDensityKgM3 * 1e-9;
				mass += kg;
				mx += kg * properties.centerOfMass.x;
				my += kg * properties.centerOfMass.y;
				mz += kg * properties.centerOfMass.z;
			}
			var center = p.handling.centerOfMassMoldMm;
			if (Math.abs(mass - p.handling.castingMassKg) > 1e-5
				|| Math.abs(mx / mass - center.x) + Math.abs(my / mass - center.y) + Math.abs(mz / mass - center.z) > 1e-5)
				throw "Casting mass/CoM differs from exact CAD with steel bores";
			rv = rebuilt.volume();
			if (difference > 1e-3)
				throw "Casting reconstruction differs by " + difference + " mm3";
			for (part in references)
				part.close();
			references = [];
		} catch (e:Dynamic) {
			for (part in references)
				if (!part.shape.isClosed())
					part.close();
			throw e;
		}
		var parts:Array<Part> = [], members:Array<RemovalMember> = [], permanent:Array<{
			id:String,
			members:Array<String>
		}> = [], groups:Array<{
			id:String,
			members:Array<String>
		}> = [];
		try {
			for (m in p.panels.members) {
				var shape = panelWorld(m);
				parts.push(shape);
				members.push({
					id: m.panel.id,
					part: shape,
					direction: new Vector(m.withdrawal.x, m.withdrawal.y, m.withdrawal.z)
				});
			}
			var rowIds:Array<String> = [];
			for (c in p.cores)
				if (rowIds.indexOf(c.bridgeId) < 0)
					rowIds.push(c.bridgeId);
			for (id in rowIds) {
				var panelIds = [id].concat([for (c in p.cores) if (c.bridgeId == id) c.carrierId]),
					shapes:Array<Part> = [];
				var shape = Solids.building(shapes, owned -> {
					for (m in p.supportPanels)
						if (panelIds.indexOf(m.panel.id) >= 0)
							owned.push(panelWorld(m));
					return Solids.union(owned);
				});
				parts.push(shape);
				members.push({
					id: materia.tooling.ToolingIds.derive("", id, "-fixture"),
					part: shape,
					direction: new Vector(0, 0, 1)
				});
				permanent.push({id: materia.tooling.ToolingIds.derive("", id, "-fixture"), members: panelIds});
			}
			for (m in p.supportPanels)
				if (rowIds.indexOf(m.panel.id) < 0 && p.cores.filter(c -> c.carrierId == m.panel.id).length == 0) {
					var shape = panelWorld(m);
					parts.push(shape);
					members.push({id: m.panel.id, part: shape, direction: new Vector(0, 0, 1)});
				}
			for (f in p.formers) {
				var shape = ToolingVolume.geometryOf(f.volume);
				parts.push(shape);
				members.push({
					id: f.volume.id,
					part: shape,
					direction: new Vector(f.withdrawal.x, f.withdrawal.y, f.withdrawal.z)
				});
			}
			for (f in p.fixtures) {
				var shape = ToolingVolume.geometryOf(f.datum);
				parts.push(shape);
				members.push({
					id: f.datum.id,
					part: shape,
					direction: new Vector(f.withdrawal.x, f.withdrawal.y, f.withdrawal.z)
				});
			}
			for (c in p.cores) {
				var shape = box(c.cavity);
				parts.push(shape);
				members.push({id: c.id, part: shape, direction: new Vector(0, 0, 1)});
			}
			for (h in p.hardware)
				if (h.kind == SupportHardwareKind.FixturePin || h.kind == SupportHardwareKind.BoreMask) {
					var local = new ToolingPin(new ComponentValues().setNumber("diameter", h.diameter).setNumber("length", h.length)).geometry();
					parts.push(local);
					var shape = PosedParts.moved(local, h.pose);
					parts.push(shape);
					parts.remove(local);
					local.close();
					var direction = new Vector(0, 0, -1);
					if (h.kind == SupportHardwareKind.BoreMask) {
						var fixture = p.fixtures.filter(f -> maskId(f) == h.id)[0],
							v = fixture.boreMaskWithdrawal;
						if (v == null)
							throw "Missing bore-mask removal direction";
						direction = new Vector(v.x, v.y, v.z);
					}
					members.push({id: h.id, part: shape, direction: direction});
				}
			coverage(p, parts);
			var castingPieces = [source(p)];
			for (i in p.document.input.inserts)
				castingPieces.push(steel(p, i.id));
			var cured = Solids.union(castingPieces);
			parts.push(cured);
			members.push({id: "cured-casting", part: cured, direction: new Vector(0, 0, 1)});
			Sys.println("Checking full demolding sweeps for " + members.length + " members");
			var demoldingMotions = [
				for (m in members)
					{
						id: m.id,
						x: m.direction.x,
						y: m.direction.y,
						z: m.direction.z
					}
			];
			var demolding = SweptRemoval.analyze(members, 1e-4);
			members.pop();
			for (id in rowIds) {
				var ids = [materia.tooling.ToolingIds.derive("", id, "-fixture")].concat([for (c in p.cores) if (c.bridgeId == id) c.id]),
					pieces:Array<Part> = [];
				var n = members.length - 1;
				while (n >= 0) {
					if (ids.indexOf(members[n].id) >= 0) {
						pieces.push(new Part(members[n].part.shape.cloneShape()));
						members.splice(n, 1);
					}
					n--;
				}
				var shape = Solids.union(pieces);
				parts.push(shape);
				members.push({
					id: materia.tooling.ToolingIds.derive("", id, "-loaded"),
					part: shape,
					direction: new Vector(0, 0, 1)
				});
				groups.push({id: materia.tooling.ToolingIds.derive("", id, "-loaded"), members: ids});
			}
			for (i in p.document.input.inserts) {
				var shape = steel(p, i.id);
				parts.push(shape);
				members.push({
					id: materia.tooling.ToolingIds.derive("insert-", i.id),
					part: shape,
					direction: new Vector(0, 0, 1)
				});
			}
			members[0].direction = new Vector(0, 0, 1);
			for (member in members) {
				var fixtures = p.fixtures.filter(f -> f.datum.id == member.id);
				if (fixtures.length == 1) {
					var v = fixtures[0].assemblyClearance;
					member.direction = new Vector(v.x, v.y, v.z);
				}
				if (p.hardware.filter(h -> h.id == member.id
					&& (h.kind == SupportHardwareKind.FixturePin || h.kind == SupportHardwareKind.BoreMask))
					.length == 1)
					member.direction = new Vector(0, 0, p.hardware.filter(h -> h.id == member.id)[0].kind == SupportHardwareKind.FixturePin ? -1 : 1);
			}
			// Before casting, bed-facing tooling can approach downward from the open
			// fill side. After cure those same pieces leave below the released bed.
			for (m in members)
				if (m.direction.z < -0.9 && p.hardware.filter(h -> h.id == m.id && h.kind == SupportHardwareKind.FixturePin).length == 0)
					m.direction = new Vector(0, 0, 1);
			var assemblyMotions = [
				for (m in members)
					{
						id: m.id,
						x: m.direction.x,
						y: m.direction.y,
						z: m.direction.z
					}
			];
			Sys.println("Checking full assembly sweeps for " + members.length + " members");
			var assembly = SweptRemoval.analyze(members, 1e-4);
			for (part in parts)
				if (!part.shape.isClosed())
					part.close();
			return {
				handling: p.handling,
				demoldingSteps: steps(p, demolding.removalOrder),
				symmetricDifferenceMm3: difference,
				sourceVolumeMm3: ov,
				reconstructedVolumeMm3: rv,
				assembly: assembly,
				demolding: demolding,
				permanentGroups: permanent,
				assemblyGroups: groups,
				validation: p.validation.copy(),
				fabrication: p.fabrication,
				assemblyMotions: assemblyMotions,
				demoldingMotions: demoldingMotions
			};
		} catch (e:Dynamic) {
			for (part in parts)
				if (!part.shape.isClosed())
					part.close();
			throw e;
		}
	}
}
