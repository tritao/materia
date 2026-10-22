package machinekit.milling;

import machinekit.component.ComponentValues;
import machinekit.structural.CastInsert;
import machinekit.structural.MineralFoundation;
import machinekit.structural.MineralFoundation.CastInsertPlacement;
import machinekit.structural.InsertAnchorPattern;
import machinekit.structural.InsertHoles;
import materia.assembly.AssemblyFrames as F;
import materia.casting.CastingPatterns;

/** Four source connections enclose the foundation's nominal centre of mass.
 * The attachment footprint is a design declaration, never a capacity approval. */
class MillLiftingPattern {
	/** Native nominal mass in the pour state: transferred holes and finishing
	 * removal do not yet exist. Embedded anchor hardware remains real components. */
	public static function asCastMass(foundation:MineralFoundation):machinekit.assembly.MachineAssembly.MachineAssemblyMassProperties {
		var raw=new machinekit.assembly.MachineAssembly(),poses=foundation.solvedPoses();
		for (member in foundation.components()) {
			var part=member.component;
			if (Std.isOfType(part,CastInsert)) {
				var steel:CastInsert=cast part;
				part=new CastInsert(steel.values().copy().setToken("buildStage",materia.casting.InsertStages.InsertStage.BeforeCasting));
			}
			raw.addComponent(member.id,part,poses.get(member.id));
		}
		return raw.massProperties();
	}

	public static function placements(body:ComponentValues,existing:Array<CastInsertPlacement>,
		details:MillCastingDetails):Array<CastInsertPlacement> {
		var foundation=new MineralFoundation(body,existing),mass=asCastMass(foundation);
		if (mass.mass<=0 || !Math.isFinite(mass.mass) || mass.unaccounted.length>0)
			throw "Lifting layout needs the source foundation mass and centre of mass";
		var footprints=[{width:details.liftingPlateWidthMm,depth:details.liftingPlateDepthMm}]
			.concat(details.liftingAlternativeFootprintsMm);
		for (footprint in footprints)
			if (!Math.isFinite(footprint.width+footprint.depth) || footprint.width<=0 || footprint.depth<=0)
				throw "Invalid source lifting plate footprint";
		footprints.sort((a,b) -> {
			var area=Reflect.compare(a.width*a.depth,b.width*b.depth);
			return area!=0 ? area : Reflect.compare(a.width,b.width);
		});
		var share=mass.mass*details.anchorRule.gravityMps2/Math.min(details.anchorRule.liftingLoadPaths,4);
		var service=InsertHoles.records(InsertHoles.tappedBlind(details.liftingThread,details.liftingPlateThicknessMm,C45Bright))[0];
		var wall=details.liftingThread.thread.majorDiameter/2+service.limits.minWall;
		var maximum={x:details.liftingServiceInsetX-wall,y:details.liftingServiceInsetY-wall};
		if (maximum.x<0 || maximum.y<0) throw "Fixed lifting service position cannot retain its policy steel wall";
		var seeds:Array<{x:Float,y:Float}> = [];
		for (x in [0.0,maximum.x/2,maximum.x]) for (y in [0.0,maximum.y/2,maximum.y]) seeds.push({x:x,y:y});
		seeds.sort((a,b) -> Reflect.compare(a.x+a.y,b.x+b.y));
		var resin=foundation.body.geometry(),result:Array<CastInsertPlacement> = [];
		try {
			for (sx in [-1,1]) for (sy in [-1,1]) {
				var id="lift"+(sx<0 ? "Left" : "Right")+(sy<0 ? "Front" : "Rear");
				var failures:Array<String> = [],chosen:CastInsertPlacement=null;
				for (footprint in footprints) {
					for (seed in seeds) {
						try {
							var candidate=placement(id,body,details,footprint.width,footprint.depth,sx,sy,share,seed);
							// Use the source contract to reject a new side breakout as
							// well as volume outside the resin. Only the top is open.
							new MineralFoundation(body,existing.concat([candidate])).body.finishedCavities();
							var blank=machinekit.structural.InsertBodies.part(candidate.part.insertSpec().body);
							var placed:cadkit.modeling.Part=null,outside:cadkit.modeling.Part=null;
							var missing=0.0;
							try {
								placed=machinekit.assembly.PosedParts.moved(blank,candidate.pose);
								outside=placed.subtract(resin);missing=outside.volume();
							} catch (error:Dynamic) {
								if (outside!=null) outside.close();
								if (placed!=null) placed.close();
								blank.close();throw error;
							}
							outside.close();placed.close();blank.close();
							if (missing>1e-6) throw "blank intersects an existing cavity or leaves the casting: "+missing+" mm3";
							var steel=new machinekit.assembly.MachineAssembly();steel.addComponent(id,candidate.part,F.identity());
							var checked=machinekit.design.AssemblyDesignChecks.check(steel);
							if (checked.hasErrors()) throw checked.items.map(item -> item.message).join("; ");
							InsertAnchorPattern.derive(id,candidate.part,details.anchorRule,Casting(resin,candidate.pose),share);
							chosen=candidate;break;
						} catch (error:Dynamic) {
							failures.push(footprint.width+"x"+footprint.depth+" @ "+seed.x+","+seed.y+": "+Std.string(error));
						}
					}
					if (chosen!=null) break;
				}
				if (chosen==null) throw "No source lifting footprint fits "+id+": "+failures.join("; ");
				var com=mass.centreOfMass;
				if (Math.abs(com.x)>=Math.abs(chosen.pose.x) || Math.abs(com.y)>=Math.abs(chosen.pose.y))
					throw "Source lifting footprint cannot enclose the foundation centre of mass";
				result.push(chosen);
			}
		} catch (error:Dynamic) {resin.close();throw error;}
		resin.close();return result;
	}

	static function placement(id:String,body:ComponentValues,details:MillCastingDetails,width:Float,depth:Float,
		sx:Int,sy:Int,share:Float,seed:{x:Float,y:Float}):CastInsertPlacement {
		var height=details.liftingPlateThicknessMm;
		if (!Math.isFinite(height) || height<=0) throw "Invalid source lifting plate thickness";
		var land=seed;
		var holes=InsertHoles.records(InsertHoles.tappedBlind(details.liftingThread,height,C45Bright));
		holes[0].frame.x=sx*(width/2+land.x-details.liftingServiceInsetX);
		holes[0].frame.y=sy*(depth/2+land.y-details.liftingServiceInsetY);
		var wall=details.liftingThread.thread.majorDiameter/2+holes[0].limits.minWall;
		if (Math.abs(holes[0].frame.x)+wall>width/2+1e-7
			|| Math.abs(holes[0].frame.y)+wall>depth/2+1e-7)
			throw "Lifting source blank cannot retain the service thread's policy wall";
		var part=new CastInsert(CastInsert.precastHoles(CastInsert.boxValues(width,depth,height),
			InsertHoles.fromNeutral(holes))
			.setToken("role",materia.casting.InsertSpec.InsertRole.LiftingMount)
			.setToken("label","foundation lifting connection"));
		var x=body.number("width")/2-width/2-land.x,y=body.number("depth")/2-depth/2-land.y;
		var z=body.number("height")-height;
		if (x<=0 || y<=0 || z<0) throw "Source lifting blank exceeds the foundation envelope";
		return {id:id,part:part,pose:F.translation(sx*x,sy*y,z),openings:[CastingPatterns.face("body.z-max")]};
	}
}
