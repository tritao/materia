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
import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingEnums.MachiningSide;
import materia.tooling.casting.MoldPlan;
import materia.tooling.casting.MoldPlan.MoldBox;
import materia.tooling.casting.BoxMoldPlanner;
import materia.tooling.casting.CoreSupport.CoreSupportKind;
import materia.tooling.casting.CoreSupport.SupportHardwareKind;
import machinekit.standard.HexNut;
import machinekit.tooling.SweptRemoval.RemovalMember;
import machinekit.tooling.SweptRemoval.RemovalReport;

typedef BoxMoldReview = {
	var assembly:RemovalReport;
	var demolding:RemovalReport;
	var symmetricDifferenceMm3:Float;
	var validation:Array<materia.tooling.ToolingValidation>;
	var fabrication:materia.tooling.ToolingValidation.FabricationReadiness;
	var rigidGroups:Array<{
		id:String,
		members:Array<String>
	}>;
	var assemblyGroups:Array<{
		id:String,
		members:Array<String>
	}>;
}

/** CAD realization consumes the neutral plan. Loose screws/clamps are released first;
 * adhesive joints are released explicitly. Cores and supports participate in the
 * appropriate bed/fill withdrawal graph; EPS breakup remains a manual operation.
 */
class BoxMoldCad {
	static function placedBox(b:MoldBox):Part {
		return ToolingBlock.placedBox(b);
	}

	public static function reconstructed(p:MoldPlan):Part {
		var parts:Array<Part> = [placedBox(p.cavity)];
		return Solids.building(parts, items -> {
			for (c in p.cores)
				items.push(placedBox(c.cavity));
			for (f in p.fixtures)
				items.push(placedBox(f.body));
			return Solids.cut(items[0], items.slice(1));
		});
	}

	public static function source(p:MoldPlan):Part {
		var envelope:Array<Part> = [];
		return Solids.building(envelope, items -> {
			for (e in p.document.input.envelope) {
				var local = Part.prism([for (v in e.profile) new Vector(v.x, v.y)], 0, e.height);
				var world = PosedParts.moved(local, F.compose(p.document.setup.sourceToMold, e.pose));
				local.close();
				items.push(world);
			}
			var body = Solids.union(items);
			items.resize(0);
			items.push(body);
			for(c in materia.casting.CastingCavities.derive(p.document.input.envelope,p.document.input.features))for(piece in materia.casting.CastingCavities.prisms(c)) {
				var local=Part.prism([for(v in piece.profile)new Vector(v.x,v.y)],0,piece.height);
				try items.push(PosedParts.moved(local,F.compose(p.document.setup.sourceToMold,piece.pose))) catch(e:Dynamic){local.close();throw e;}
				local.close();
			}
			return Solids.cut(items[0], items.slice(1));
		});
	}

	static function insert(p:MoldPlan):CastInsert
		return ToolingInsertCad.component(p.document.input.inserts[0]);

	public static function assembly(p:MoldPlan, exploded:Bool = false):MachineAssembly {
		BoxMoldPlanner.validate(p);
		var b = p.cavity, a = exploded ? new MachineAssembly() : PanelKit.assembly(p.panels, false, "coated particleboard");
		if (exploded)
			for (m in p.panels.members) {
				var pose = F.compose(F.identity(), m.pose);
				switch m.panel.id {
					case "bed":
						pose.z -= b.height * 0.6;
					case "south":
						pose.y -= b.depth * 0.9;
					case "north":
						pose.y += b.depth * 0.9;
					case "east":
						pose.x += b.width * 0.9;
					case "west":
						pose.x -= b.width * 0.9;
					default:
						throw "Unknown box template panel";
				}
				a.addComponent(m.panel.id, ToolingPanel.fromPlan(m.panel, "coated particleboard"), pose);
			}
		for (index in 0...p.supportPanels.length) {
			var m = p.supportPanels[index],
				pose = F.compose(F.identity(), m.pose);
			if (exploded)
				pose.z += (index + 1) * b.height * 0.5;
			a.addComponent(m.panel.id, ToolingPanel.fromPlan(m.panel), pose);
		}
		for (c in p.cores) {
			var block = c.cutting.block,
				pose = F.compose(F.identity(), c.cutting.stockToMold);
			if (exploded)
				pose.z += b.height * 1.3;
			var foot = c.support.foot;
			if (foot != null) {
				var fb = foot.cutting.block,
					fp = F.compose(F.identity(), foot.cutting.stockToMold);
				if (exploded)
					fp.z -= b.height;
				a.addComponent(materia.tooling.ToolingIds.derive("", c.id, "-foot"), ToolingBlock.fromBox(fb, "EPS tooling"), fp);
			}
			if (!exploded)
				for (h in c.support.hardware) {
					if (h.kind == SupportHardwareKind.LocatingPin) {
						a.addComponent(h.id, new ToolingPin(new ComponentValues().setNumber("diameter", h.diameter).setNumber("length", h.length)), h.pose);
					} else {
						ToolingHardwareCad.throughBolt(a, h.id, h.diameter, h.length, h.pose);
					}
				}
			a.addComponent(c.id, ToolingBlock.fromBox(block, "EPS tooling"), pose);
		}
		var i = p.document.input.inserts[0],
			pose = F.compose(p.document.setup.sourceToMold, i.pose);
		if (exploded) {
			pose.x += b.width * 1.4;
			pose.y -= b.depth * 1.4;
		}
		a.addComponent(i.id, insert(p), pose);
		if (!exploded) {
			var fixture = p.fixtures[0],
				f = fixture.mountingFace,
				t = p.document.recipe.panel.measuredThickness;
			for (fixing in fixture.fixings) {
				var hole = materia.tooling.casting.InsertFixturePlanner.holes(i).filter(h -> h.id == fixing.sourceHoleId)[0];
				var thread:materia.casting.InsertHole.InsertThread = cast hole.thread;
				ToolingHardwareCad.fixtureScrew(a, fixing.hardwareId, hole, t + materia.tooling.casting.InsertFixturePlanner.minimumInsertion(hole),
					F.fromRotationMatrix(fixing.entry.x, fixing.entry.y, b.z - t, [1, 0, 0, 0, -1, 0, 0, 0, -1]),fixing.entry);
				var penetration = p.penetrations.filter(v -> v.sourceHoleId == hole.id && v.insertId == i.id)[0];
				var plug = penetration.plug;
				if (plug != null) {
					if (plug.materialSpec != "steel")
						throw "Unsupported nominal bore plug material";
					a.addComponent(materia.tooling.ToolingIds.derive("resin-plug-", hole.id),
						machinekit.standard.SocketSetScrew.forThread(machinekit.structural.InsertHoles.threadSpec(thread).mating(), plug.length),
						F.compose(fixing.entry, F.translation(0, 0, hole.depth - plug.length)));
				}
			}
			for (mask in fixture.boreMasks) {
				var hole = materia.tooling.casting.InsertFixturePlanner.holes(i).filter(h -> h.id == mask.sourceHoleId)[0];
				a.addComponent(mask.hardwareId, ToolingPin.boreMask(hole, mask.plugLength), F.compose(pose, hole.frame));
				if (mask.farEndId != null)
					a.addComponent(mask.farEndId, ToolingPin.boreMask(hole, mask.plugLength),
						F.compose(F.compose(pose, hole.frame), F.translation(0, 0, hole.depth - (cast mask.plugLength : Float))));
			}

			var poses = a.solvedPoses(), bedPose = poses.get("bed");
			for (row in a.components())
				if (p.panels.members.filter(m -> m.panel.id == row.id).length == 0) {
					var local = F.compose(F.inverse(bedPose), poses.get(row.id));
					a.addMemberConnector("bed", materia.tooling.ToolingIds.derive("", row.id, "-datum"), local);
					a.addMemberConnector(row.id, "tooling-datum", F.identity());
					a.addMate(materia.tooling.ToolingIds.derive("", row.id, "-positioned"), "fixed", "bed",
						materia.tooling.ToolingIds.derive("", row.id, "-datum"), row.id, "tooling-datum");
				}
		}
		return a;
	}

	static function cutVolume(p:MoldPlan, panelId:String, opId:String):Part {
		var m = p.panels.members.filter(m -> m.panel.id == panelId)[0],
			op = m.panel.operations.filter(o -> o.id == opId)[0],
			t = m.panel.stock.measuredThickness;
		var z0 = op.side == MachiningSide.PositiveZ ? t - op.depth : 0.0,
			z1 = op.side == MachiningSide.PositiveZ ? t : op.depth;
		var local = PanelCad.prism(op, z0, z1),
			world = PosedParts.moved(local, m.pose);
		local.close();
		return world;
	}

	public static function penetrations(p:MoldPlan):Void {
		BoxMoldPlanner.validate(p);
		var envelope = placedBox(p.cavity);
		try {
			for (record in p.penetrations) {
				var cut = cutVolume(p, record.panelId, record.operationId);
				try {
					if (record.kind == OutsideCasting) {
						var overlap = cut.intersect(envelope),
							v = overlap.volume();
						overlap.close();
						if (v > 1e-5)
							throw "Outside-casting cut enters cavity: " + record.operationId;
					} else if (record.kind == CoreSealed) {
						var core = p.cores.filter(c -> c.id == record.coreId)[0],
							foot = core.support.foot;
						if (foot == null || core.support.kind != CoreSupportKind.BedSocket)
							throw "Missing bed locating foot";
						var bounds = cut.shape.bounds(),
							lo = Vector.fromNative(bounds.get_min()),
							hi = Vector.fromNative(bounds.get_max()),
							cb = core.cavity;
						if (record.panelId != "bed"
							|| core.support.socketOperationId != record.operationId
							|| lo.x < cb.x + record.sealMargin
							|| hi.x > cb.x + cb.width - record.sealMargin
							|| lo.y < cb.y + record.sealMargin
							|| hi.y > cb.y + cb.depth - record.sealMargin
							|| Math.abs(hi.z - p.cavity.z) > 1e-7
							|| Math.abs(lo.z - p.cavity.z + core.support.embedDepth) > 1e-7)
							throw "Core socket escapes sealing footprint";
					} else {
						BoxMoldPlanner.validateInsertSeal(p, record);
					}
					cut.close();
				} catch (e:Dynamic) {
					cut.close();
					throw e;
				}
			}
			envelope.close();
		} catch (e:Dynamic) {
			envelope.close();
			throw e;
		}
	}

	/** Cover every intended closed face with actual panels; the single fixing seal is
	 * represented only by its explicitly checked insert footprint. Fill +Z remains open. */
	static function coverage(p:MoldPlan, panels:Array<Part>):Void {
		var b = p.cavity, s = 0.1;
		var slabs:Array<MoldBox> = [
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
		for (k in 0...slabs.length) {
			var slab = placedBox(slabs[k]),
				tools = panels.copy(),
				seals:Array<Part> = [];
			try {
				if (k == 4) {
					var insert = p.fixtures[0].body;
					var seal = placedBox({
						x: insert.x,
						y: insert.y,
						z: b.z - s,
						width: insert.width,
						depth: insert.depth,
						height: s
					});
					seals.push(seal);
					tools.push(seal);
					for (core in p.cores)
						if (core.support.kind == CoreSupportKind.BedSocket) {
							var cb = core.cavity, coreSeal = placedBox({
								x: cb.x,
								y: cb.y,
								z: b.z - s,
								width: cb.width,
								depth: cb.depth,
								height: s
							});
							seals.push(coreSeal);
							tools.push(coreSeal);
						}
				}
				var missing = slab.subtractAll(tools),
					volume = missing.volume();
				missing.close();
				if (volume > 1e-5)
					throw "Uncovered mold boundary face " + k;
				for (seal in seals)
					seal.close();
				slab.close();
			} catch (e:Dynamic) {
				for (seal in seals)
					seal.close();
				slab.close();
				throw e;
			}
		}
	}

	public static function review(p:MoldPlan):BoxMoldReview {
		penetrations(p);
		var original = source(p), rebuilt = reconstructed(p), difference = 0.0;
		try {
			var missing = original.subtractAll([rebuilt]),
				excess = rebuilt.subtractAll([original]);
			difference = missing.volume() + excess.volume();
			missing.close();
			excess.close();
			if (difference > 1e-5)
				throw "Reconstructed casting differs from source";
			original.close();
			rebuilt.close();
		} catch (e:Dynamic) {
			original.close();
			rebuilt.close();
			throw e;
		}
		var parts:Array<Part> = [], members:Array<RemovalMember> = [];
		try {
			for (m in p.panels.members) {
				var local = ToolingPanel.fromPlan(m.panel).geometry(),
					world = PosedParts.moved(local, m.pose);
				local.close();
				parts.push(world);
				members.push({
					id: m.panel.id,
					part: world,
					direction: new Vector(m.withdrawal.x, m.withdrawal.y, m.withdrawal.z)
				});
			}
			coverage(p, parts);
			var rigidGroups:Array<{
				id:String,
				members:Array<String>
			}> = [];
			for (core in p.cores) {
				var bonded:Array<Part> = [];
				for (m in p.supportPanels)
					if (core.support.panelIds.indexOf(m.panel.id) >= 0) {
						var local = ToolingPanel.fromPlan(m.panel).geometry(),
							world = PosedParts.moved(local, m.pose);
						local.close();
						parts.push(world);
						if (core.support.bondedPanelIds.indexOf(m.panel.id) >= 0)
							bonded.push(new Part(world.shape.cloneShape()));
						else
							members.push({id: m.panel.id, part: world, direction: new Vector(0, 0, 1)});
					}
				if (bonded.length > 0) {
					var group = Solids.union(bonded),
						id = materia.tooling.ToolingIds.derive("", core.id, "-bridge-carrier");
					parts.push(group);
					members.push({id: id, part: group, direction: new Vector(0, 0, 1)});
					rigidGroups.push({id: id, members: core.support.bondedPanelIds.copy()});
				}
			}
			var casting = reconstructed(p),
				steel = insert(p).geometry(),
				worldSteel = PosedParts.moved(steel, F.compose(p.document.setup.sourceToMold, p.document.input.inserts[0].pose));
			steel.close();
			var cured = Solids.union([casting, worldSteel]);
			parts.push(cured);
			var core = p.cores[0],
				covered = placedBox(core.cavity),
				foot = core.support.foot;
			if (foot != null) {
				var fb = foot.foam;
				// Occupancy includes the two declared adhesive lines, with the finished
				// EPS foot between them. Both bonds must be released before withdrawal.
				covered = Solids.union([
					covered,
					placedBox({
						x: fb.x,
						y: fb.y,
						z: p.cavity.z - core.support.embedDepth,
						width: fb.width,
						depth: fb.depth,
						height: core.support.embedDepth
					})
				]);
			}
			parts.push(covered);
			var direction = core.support.kind == CoreSupportKind.BedSocket ? -1.0 : 1.0;
			members.push({id: core.id, part: covered, direction: new Vector(0, 0, direction)});
			members.push({id: "cured-casting", part: cured, direction: new Vector(0, 0, 1)});
			var demolding = SweptRemoval.analyze(members);
			members.pop();
			var assemblyGroups:Array<{
				id:String,
				members:Array<String>
			}> = [];
			if (core.support.kind == CoreSupportKind.Bridge) {
				// Bond the EPS to the reusable fixture before installing the hanging unit.
				members.pop();
				var id = materia.tooling.ToolingIds.derive("", core.id, "-bridge-carrier"), index = -1;
				for (k in 0...members.length)
					if (members[k].id == id)
						index = k;
				if (index < 0)
					throw "Missing hanging fixture group";
				var fixture = members[index];
				members.splice(index, 1);
				var unit = Solids.union([new Part(fixture.part.shape.cloneShape()), new Part(covered.shape.cloneShape())]);
				parts.push(unit);
				members.push({
					id: materia.tooling.ToolingIds.derive("", core.id, "-suspended-unit"),
					part: unit,
					direction: new Vector(0, 0, 1)
				});
				assemblyGroups.push({
					id: materia.tooling.ToolingIds.derive("", core.id, "-suspended-unit"),
					members: [core.id].concat(core.support.bondedPanelIds)
				});
			} else
				members[members.length - 1].direction = new Vector(0, 0, 1);
			var steelBody = placedBox(p.fixtures[0].body);
			parts.push(steelBody);
			members.push({id: p.fixtures[0].insertId, part: steelBody, direction: new Vector(0, 0, 1)});
			// For installation the bed approaches downward first, unlike its downward
			// demolding withdrawal. Reverse the corresponding +Z clearance motion.
			members[0].direction = new Vector(0, 0, 1);
			var assemblyReport = SweptRemoval.analyze(members);
			for (part in parts)
				part.close();
			return {
				assembly: assemblyReport,
				demolding: demolding,
				symmetricDifferenceMm3: difference,
				validation: p.validation.copy(),
				fabrication: p.fabrication,
				rigidGroups: rigidGroups,
				assemblyGroups: assemblyGroups
			};
		} catch (e:Dynamic) {
			for (part in parts)
				part.close();
			throw e;
		}
	}
}
