package machinekit.tooling;

import materia.casting.InsertSpec.InsertBodyCodec as InsertBodies;

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
	var fingerprint:String;
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
		var component = ToolingInsertCad.component(i);
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
		if (p.features.filter(f -> f.kind == UnsupportedFeature).length > 0
			|| p.validation.filter(v -> StringTools.startsWith(v.rule, "tooling.fixture.hole-protection.") && v.status == materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported).length > 0)
			return {diagnostics: diagnostics, geometry: null};
		return {diagnostics: diagnostics, geometry: review(p)};
	}

	public static function threadedMask(p:CastingMoldPlan, h:materia.tooling.casting.CoreSupport.SupportHardware):Bool
		return h.kind == SupportHardwareKind.BoreMask && boreMaskRecord(p, h.id).hole.thread != null;

	public static function looseHardware(p:CastingMoldPlan, h:materia.tooling.casting.CoreSupport.SupportHardware):Bool
		return h.kind != SupportHardwareKind.FixturePin && (h.kind != SupportHardwareKind.BoreMask || threadedMask(p, h));

	static function steps(p:CastingMoldPlan, order:Array<String>):Array<MoldDemoldingStep> {
		var result:Array<MoldDemoldingStep> = [
			{
				id: "release-loose-tooling",
				kind: ReleaseHardware,
				memberIds: [
					for (h in p.hardware)
						if (looseHardware(p, h) && h.kind != SupportHardwareKind.BorePlug && !threadedMask(p, h)) h.id
				],
				dependsOn: [],
				status: materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending
			}
		];
		var previous = "release-loose-tooling";
		var entryMasks = [for (h in p.hardware) if (threadedMask(p, h)) h.id];
		if (entryMasks.length > 0) {
			result.push({id: "release-entry-thread-masks", kind: ReleaseHardware, memberIds: entryMasks,
				dependsOn: [previous], status: materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending});
			previous = "release-entry-thread-masks";
		}
		var farPlugs = [for (h in p.hardware) if (h.kind == SupportHardwareKind.BorePlug) h.id];
		if (farPlugs.length > 0) {
			result.push({id: "release-far-end-thread-plugs", kind: ReleaseHardware, memberIds: farPlugs,
				dependsOn: [previous], status: materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending});
			previous = "release-far-end-thread-plugs";
		}
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

	static function boreMaskRecord(p:CastingMoldPlan, id:String):{
		protection:BoreMaskProtection, hole:materia.casting.InsertHole
	} {
		for (fixture in p.fixtures)
			for (protection in fixture.boreMasks)
				if (protection.hardwareId == id) {
					var insert = p.document.input.inserts.filter(source -> source.id == fixture.insertId)[0];
					var holes = materia.tooling.casting.InsertFixturePlanner.holes(insert).filter(hole -> hole.id == protection.sourceHoleId);
					if (holes.length != 1)
						throw "Bore mask has no unique source hole: " + id;
					return {protection: protection, hole: holes[0]};
				}
		throw "Unknown bore mask: " + id;
	}

	static function insertBox(p:CastingMoldPlan, id:String):MoldBox {
		var i = p.document.input.inserts.filter(i -> i.id == id)[0];
		var dimensions = InsertBodies.box(i.body, i.id);
		return materia.tooling.casting.BoxMoldPlanner.box(dimensions.width, dimensions.depth, dimensions.height, F.compose(p.document.setup.sourceToMold, i.pose));
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
			var part = ToolingInsertCad.component(i);
			a.addComponent(materia.tooling.ToolingIds.derive("insert-", i.id), part, pose);
		}
		if (!exploded) {
			for (h in p.hardware) {
				if (h.kind == SupportHardwareKind.BoreMask)
					a.addComponent(h.id, ToolingPin.boreMask(boreMaskRecord(p, h.id).hole, boreMaskRecord(p, h.id).protection.plugLength), h.pose);
				else if (h.kind == SupportHardwareKind.BorePlug) {
					var protections = [for (fixture in p.fixtures) for (pen in fixture.penetrations)
						if (pen.sourceHoleId != null && materia.tooling.ToolingIds.derive("resin-plug-", fixture.insertId + "-" + pen.sourceHoleId) == h.id)
						{insertId: fixture.insertId, holeId: pen.sourceHoleId}];
					for (fixture in p.fixtures) for (mask in fixture.boreMasks)
						if (mask.farEndId == h.id)
							protections.push({insertId: fixture.insertId, holeId: mask.sourceHoleId});
					if (protections.length != 1)
						throw "Resin-side plug has no unique source hole: " + h.id;
					var source = p.document.input.inserts.filter(i -> i.id == protections[0].insertId)[0];
					var hole = materia.tooling.casting.InsertFixturePlanner.holes(source).filter(hole -> hole.id == protections[0].holeId)[0];
					a.addComponent(h.id, machinekit.standard.SocketSetScrew.forThread(
						machinekit.structural.InsertHoles.threadSpec(hole.thread).mating(), h.length), h.pose);
				}
				else if (h.kind == SupportHardwareKind.LocatingPin
					|| h.kind == SupportHardwareKind.FixturePin)
					a.addComponent(h.id, new ToolingPin(new ComponentValues().setNumber("diameter", h.diameter).setNumber("length", h.length)), h.pose);
				else if (h.kind == SupportHardwareKind.FixtureScrew) {
					var fixings = [for (fixture in p.fixtures) for (fixing in fixture.fixings)
						if (fixing.hardwareId == h.id) {sourceId:fixture.insertId,fixing:fixing}];
					if (fixings.length != 1)
						throw "Fixture screw has no unique source thread: " + h.id;
					var source=p.document.input.inserts.filter(insert -> insert.id==fixings[0].sourceId)[0];
					var fixing=fixings[0].fixing;
					var hole=materia.tooling.casting.InsertFixturePlanner.holes(source).filter(hole -> hole.id==fixing.sourceHoleId)[0];
					ToolingHardwareCad.fixtureScrew(a,h.id,hole,h.length,h.pose,fixing.entry);
				}
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
				if (h.kind == SupportHardwareKind.FixturePin || (h.kind == SupportHardwareKind.BoreMask && !threadedMask(p, h))) {
					var component = h.kind == SupportHardwareKind.BoreMask ? ToolingPin.boreMask(boreMaskRecord(p, h.id).hole, boreMaskRecord(p, h.id).protection.plugLength)
						: new ToolingPin(new ComponentValues().setNumber("diameter", h.diameter).setNumber("length", h.length));
					var local = component.geometry(machinekit.component.ComponentDetail.Envelope);
					parts.push(local);
					var shape = PosedParts.moved(local, h.pose);
					parts.push(shape);
					parts.remove(local);
					local.close();
					var direction = new Vector(0, 0, -1);
					if (h.kind == SupportHardwareKind.BoreMask) {
						var v = boreMaskRecord(p, h.id).protection.withdrawal;
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
					&& (h.kind == SupportHardwareKind.FixturePin || (h.kind == SupportHardwareKind.BoreMask && !threadedMask(p, h))))
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
				fingerprint: materia.tooling.casting.CastingMoldCodec.fingerprint(p),
				handling: p.handling,
				demoldingSteps: steps(p, demolding.removalOrder),
				symmetricDifferenceMm3: difference,
				sourceVolumeMm3: ov,
				reconstructedVolumeMm3: rv,
				assembly: assembly,
				demolding: demolding,
				permanentGroups: permanent,
				assemblyGroups: groups,
				validation: p.validation.concat([
					for (name in ["reconstruction", "coverage", "assembly-graph", "demolding-graph"])
						materia.tooling.export.ToolingLoads.check("tooling.cad."
							+ name, [p.document.input.id],
							materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass, "Exact CAD "
							+ name
							+ " check passed.",
							[materia.tooling.casting.CastingMoldCodec.fingerprint(p)])
				]),
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
