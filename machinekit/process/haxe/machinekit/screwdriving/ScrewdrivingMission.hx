package machinekit.screwdriving;

import cadkit.modeling.Vector;
import machinekit.assembly.Diagnostics;
import machinekit.screwdriving.Fastenings.FasteningsResult;
import materia.project.SceneScrewDefinition.SceneScrew;
import materia.project.SceneArtifact.SceneArtifactMissionStep;

/** The cell owns reach and clearance; the generator asks about every named connection. */
interface ScrewAccess {
	/** Why the bit and arm cannot approach, drive and retract at this connection, or null. */
	function problem(fastening:Fastening):Null<String>;
}

/** Fresh connections become presenter trips; a proved bolt circle is driven in cross order. */
class ScrewdrivingMission {
	public final steps:Array<SceneArtifactMissionStep>;
	public final fastenings:Array<Fastening>;
	public final diagnostics:Diagnostics;

	function new(steps:Array<SceneArtifactMissionStep>, fastenings:Array<Fastening>, diagnostics:Diagnostics) {
		this.steps = steps; this.fastenings = fastenings; this.diagnostics = diagnostics;
	}
	public function require():Array<SceneArtifactMissionStep> {
		diagnostics.throwIfErrors();
		return steps;
	}

	/** The cell derives each qualified job and binds its actual supply; access failures prevent partial missions. */
	public static function generate(found:FasteningsResult, job:Fastening -> SceneScrew, ?access:ScrewAccess):ScrewdrivingMission {
		if (found == null || job == null) throw "Screw mission needs current CAD connections and a qualified job author";
		var diagnostics = new Diagnostics();
		for (item in found.diagnostics.items) diagnostics.add(item.severity,item.code,item.subject,item.message);
		var ordered = order(found.fastenings), steps:Array<SceneArtifactMissionStep> = [];
		var identities:Map<String,Bool> = [], screws:Map<String,Bool> = [];
		for (fastening in ordered) {
			var name = fastening.name();
			if (identities.exists(name)) diagnostics.error("screw.duplicate",name,'Fastening "$name" occurs more than once');
			identities.set(name,true);
			if (access != null) {
				try {
					var problem = access.problem(fastening);
					if (problem != null) diagnostics.error("screw.inaccessible",name,'Fastening "$name" cannot be driven: $problem');
				} catch (error:Dynamic) diagnostics.error("screw.access",name,'Fastening "$name" access check failed: $error');
			}
			try {
				var request = job(fastening);
				if (request == null || request.fastening != name) throw "Qualified job does not identify its source fastening";
				if (screws.exists(request.screw)) throw 'Screw occurrence "${request.screw}" supplies more than one fastening';
				screws.set(request.screw,true);
				steps.push({kind:"screw",screw:request});
			} catch (error:Dynamic) diagnostics.error("screw.qualification",name,'Fastening "$name" has no qualified job: $error');
		}
		if (ordered.length == 0 && !diagnostics.hasErrors()) diagnostics.error("screw.empty","assembly","The assembly has no driven fastenings");
		return new ScrewdrivingMission(diagnostics.hasErrors() ? [] : steps,ordered,diagnostics);
	}

	/** Stable source order outside circles; group only common members, direction and seat plane. */
	public static function order(found:Array<Fastening>):Array<Fastening> {
		var sorted = found.copy(); sorted.sort((a,b) -> Reflect.compare(a.screw.member,b.screw.member));
		var groups:Array<Array<Fastening>> = [];
		for (fastening in sorted) {
			var group:Array<Fastening> = null;
			for (candidate in groups) if (samePlane(candidate[0],fastening)) {group = candidate; break;}
			if (group == null) {group = []; groups.push(group);}
			group.push(fastening);
		}
		var result:Array<Fastening> = [];
		for (group in groups) for (fastening in crossOrder(group)) result.push(fastening);
		return result;
	}
	static function samePlane(a:Fastening, b:Fastening):Bool {
		return a.clamped.member == b.clamped.member && a.threaded.member == b.threaded.member &&
			a.frame.normal.dot(b.frame.normal) > 1-1e-9 &&
			Math.abs(b.frame.origin.subtract(a.frame.origin).dot(a.frame.normal)) <= Fastenings.TOLERANCE;
	}
	static function crossOrder(group:Array<Fastening>):Array<Fastening> {
		if (group.length < 4 || group.length % 2 != 0) return group;
		var center = new Vector();
		for (fastening in group) center = center.add(fastening.frame.origin);
		center = center.scale(1.0/group.length);
		var radius = group[0].frame.origin.subtract(center).length();
		if (radius <= Fastenings.TOLERANCE) return group;
		for (fastening in group) if (Math.abs(fastening.frame.origin.subtract(center).length()-radius) > Fastenings.TOLERANCE)
			return group;
		// Prove one opposite partner for every point. Equal radii alone do not
		// establish a cross pattern, and repeated points do not form a circle.
		var partners:Array<Int> = [];
		for (i in 0...group.length) {
			var a = group[i].frame.origin.subtract(center), partner = -1;
			for (j in 0...group.length) if (i != j) {
				var b = group[j].frame.origin.subtract(center);
				if (a.subtract(b).length() <= Fastenings.TOLERANCE) return group;
				if (a.add(b).length() <= Fastenings.TOLERANCE) {
					if (partner >= 0) return group;
					partner = j;
				}
			}
			if (partner < 0) return group;
			partners.push(partner);
		}
		// Choose the lowest remaining member, then its proved opposite. Face
		// orientation cannot reverse this stable convention between rebuilds.
		var visited:Array<Bool> = [for (_ in group) false], result:Array<Fastening> = [];
		for (i in 0...group.length) if (!visited[i]) {
			var partner = partners[i];
			result.push(group[i]); result.push(group[partner]);
			visited[i] = true; visited[partner] = true;
		}
		return result;
	}
}
