package materia.casting;

import materia.assembly.AssemblyFrames as F;
import materia.casting.InsertFinishing.InsertFinishResult;
import materia.casting.InsertFinishing.InsertFinishedSteel;
import materia.casting.InsertHole.HoleExtent;

/** Physical construction state. The source body contains its declared stock;
 * hole frames and usable depths describe the finished steel.
 */
enum abstract InsertStage(String) to String {
	var BeforeCasting = "before-casting";
	var Service = "service";
}

class InsertStages {
	public static function tokens():Array<String> return [InsertStage.BeforeCasting,InsertStage.Service];

	public static function holes(authored:InsertSpec,stage:InsertStage):Array<InsertHole> {
		if (stage!=InsertStage.BeforeCasting && stage!=InsertStage.Service) throw "Unknown insert manufacturing stage";
		var spec=InsertSpec.InsertSpecCodec.snapshot(InsertSpec.InsertSpecCodec.record(authored));
		if (stage==InsertStage.Service) return spec.holes;
		var selected=switch spec.origin {
			case Catalogue(_):spec.holes;
			case Fabricated(manufacturing):spec.holes.filter(hole -> manufacturing.holeRoutes.filter(choice -> choice.holeId==hole.id
				&& choice.route==InsertManufacturing.InsertHoleRoute.BeforeCasting).length==1);
		};
		return rawGeometry(spec,selected);
	}

	/** Geometric projection only, with no manufacturing permission implied.
	 * Fixture feasibility may inspect an existing un-staged service hole without
	 * inventing a source route or changing its budget-controlled declaration. */
	public static function rawGeometryForServiceHoles(authored:InsertSpec,ids:Array<String>):Array<InsertHole> {
		var spec=InsertSpec.InsertSpecCodec.snapshot(InsertSpec.InsertSpecCodec.record(authored));
		var selected:Array<InsertHole> = [],seen:Map<String,Bool> = [];
		for (id in ids) {
			var matches=spec.holes.filter(hole -> hole.id==id);
			if (seen.exists(id) || matches.length!=1) throw "Unknown or duplicate source hole projection: "+id;
			seen.set(id,true);selected.push(matches[0]);
		}
		return rawGeometry(spec,selected);
	}

	static function rawGeometry(spec:InsertSpec,selected:Array<InsertHole>):Array<InsertHole> {
		var steel=switch InsertFinishing.assess(spec.body,spec.interfaces) {
			case Resolved(steel):steel;
			case Invalid(_),Unsupported(_):return selected; // Unresolved stock remains an unqualified preview.
		};
		if (steel.removedSteel.length==0) return selected;
		var service=InsertFinishing.box(steel);
		if (service==null) throw "Unsupported non-box finished insert drilling";
		var raw=InsertSpec.InsertBodyCodec.box(spec.body,"pre-cast insert");
		var rawLow=[-raw.width/2,-raw.depth/2,0.0],rawHigh=[raw.width/2,raw.depth/2,raw.height];
		for (hole in selected) {
			validateServiceHole(hole,service);
			var frame=F.fromRotationMatrix(hole.frame.x,hole.frame.y,hole.frame.z,F.toRotationMatrix(hole.frame));
			var normal=F.transformVector(frame,0,0,1),axis=CastingBounds.axis(normal);
			if (axis==null) throw "Unsupported finishing extension for non-axial insert hole: "+hole.id;
			var direction=[normal.x,normal.y,normal.z],position=[frame.x,frame.y,frame.z];
			var low=[service.x,service.y,service.z],high=[service.x+service.width,service.y+service.depth,service.z+service.height];
			var positive=direction[axis]>0;
			var entry=positive ? (position[axis]-rawLow[axis])/direction[axis]
				: (rawHigh[axis]-position[axis])/-direction[axis];
			var exit=hole.extent==HoleExtent.Through ? (positive ? rawHigh[axis]-high[axis] : low[axis]-rawLow[axis])/Math.abs(direction[axis]) : 0.0;
			if (entry<0 && entry>-InsertFinishing.EPS_MM) entry=0;
			if (exit<0 && exit>-InsertFinishing.EPS_MM) exit=0;
			if (entry<0 || exit<0) throw "Finishing extension lies outside raw stock: "+hole.id;
			var throughThread=hole.threadDepth!=null && Math.abs(hole.threadDepth-hole.depth)<=InsertFinishing.EPS_MM;
			hole.frame=F.compose(frame,F.translation(0,0,-entry));
			hole.depth+=entry+exit;
			if (hole.threadDepth!=null) hole.threadDepth=hole.threadDepth+entry+(throughThread ? exit : 0);
			if (hole.counterbore!=null) hole.counterbore.depth+=entry;
			if (hole.relief!=null) hole.relief.start+=entry;
		}
		return InsertHole.InsertHoleCodec.snapshot(selected);
	}

	/** Check the service material, not the disposable stock, against the source
	 * engagement, back-wall and radial-wall rules. No rule is replaced by a default.
	 */
	public static function validateServiceHole(hole:InsertHole,box:Box3):Void {
		var axis=InsertHoleGeometry.axis(hole),normal=F.transformVector(hole.frame,0,0,1);
		var direction=[normal.x,normal.y,normal.z],position=[hole.frame.x,hole.frame.y,hole.frame.z];
		var low=[box.x,box.y,box.z],high=[box.x+box.width,box.y+box.depth,box.z+box.height];
		var boundary=direction[axis]>0 ? low[axis] : high[axis];
		if (Math.abs(position[axis]-boundary)>InsertFinishing.EPS_MM)
			throw "Insert hole entry must lie on its finished steel face: "+hole.id;
		var radius=(hole.thread==null ? hole.diameter : hole.thread.majorDiameter)/2;
		if (hole.counterbore!=null) radius=Math.max(radius,hole.counterbore.diameter/2);
		if (hole.relief!=null) radius=Math.max(radius,hole.relief.diameter/2);
		for (other in 0...3) if (other!=axis
			&& Math.min(position[other]-low[other],high[other]-position[other])-radius<hole.limits.minWall-InsertFinishing.EPS_MM)
			throw "Finishing violates the insert hole radial-wall rule: "+hole.id;
		var available=(high[axis]-low[axis])/Math.abs(direction[axis]);
		if (hole.extent==HoleExtent.Through && hole.depth<available-InsertFinishing.EPS_MM)
			throw "Finished through-hole depth does not reach its exit: "+hole.id;
		if (hole.extent==HoleExtent.Blind) {
			if (available-hole.depth<hole.limits.minBack-InsertFinishing.EPS_MM)
				throw "Finishing violates the insert hole back-wall rule: "+hole.id;
			if (hole.thread!=null && hole.threadDepth!=null
				&& hole.depth-hole.threadDepth<InsertHole.InsertHoleCodec.threadLimits(hole).minTipGap-InsertFinishing.EPS_MM)
				throw "Finishing violates the insert hole tip-gap rule: "+hole.id;
		}
	}
}
