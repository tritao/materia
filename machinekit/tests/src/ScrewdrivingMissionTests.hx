import machinekit.screwdriving.ScrewdrivingMission;
import machinekit.screwdriving.ScrewdrivingMission.ScrewAccess;
import machinekit.screwdriving.Fastening;
import machinekit.screwdriving.Fastenings.FasteningsResult;
import machinekit.assembly.Diagnostics;
import machinekit.assembly.AssemblyPreview;
import machinekit.screwdriving.ScrewdrivingScene;
import materia.project.SceneScrewDefinition.SceneScrew;

private class BlockedScrewAccess implements ScrewAccess {
	public var visited:Array<String> = [];
	public function new() {}
	public function problem(fastening:Fastening):Null<String> {
		visited.push(fastening.screw.member);
		return fastening.screw.member == "flange/screw3" ? "Fixture blocks the bit approach" : null;
	}
}

/** CAD order and named diagnostics are proved before a whole-cell motion is attempted. */
class ScrewdrivingMissionTests {
	static var count = 0;
	static function check(value:Bool, message:String):Void {count++; if (!value) throw message;}
	static function jobs(mission:ScrewdrivingMission):Array<SceneScrew> {
		var result:Array<SceneScrew> = [];
		for (step in mission.steps) {
			var job = materia.project.SceneScrewInvocation.job(step.screw,mission.steps);
			if (job == null) throw "Generated step lost its screw job";
			result.push(job);
		}
		return result;
	}
	public static function run():Void {
		count = 0;
		var cell = new ScrewdrivingCell(null,0,machinekit.standard.DeepGrooveBearing.metric("6003"));
		var unqualified = cell.work.fastenings().require();
		for (fastening in unqualified) {
			var declared = cell.work.join(fastening.screw.member,null,{minimum:0.2,maximum:1.0});
		}
		var found = cell.work.fastenings();
		var scene = AssemblyPreview.scene(cell,"screw-mission-checks");
		function job(fastening:Fastening):SceneScrew return ScrewdrivingScene.fromFastening(cell.work,fastening,"work/",scene,
			"presenter/nest","presenter/screw","presenter/part-present",1,
			{runDownSpeed:20,finalSpeed:2,runDownTorqueLimit:1,finalTorqueLimit:19.5,maximumRunDownAngle:180,minimumStroke:2,maximumStroke:18},
			{leadInMm:0.5,maximumAxisAngle:0.1,prevailingTorque:0.2,stiffnessNmPerRad:40,stripTorque:22});
		var mission = ScrewdrivingMission.generate(found,job);
		check(mission.require().length == 4,"Every current flange fastening produces a presenter trip");
		check([for (fastening in mission.fastenings) fastening.screw.member].join(",") ==
			"flange/screw1,flange/screw3,flange/screw2,flange/screw4","Four CAD bolt-circle screws follow cross order");
		check([for (job in jobs(mission)) job.screw].join(",") ==
			"work/flange/screw1,work/flange/screw3,work/flange/screw2,work/flange/screw4","Jobs retain exact source body identities");
		check([for (job in jobs(mission)) job.threadFace].join(",") ==
			[for (fastening in mission.fastenings) fastening.threaded.faceName].join(","),"Jobs retain the exact named CAD faces");
		check([for (job in jobs(mission)) job.clamped].join(",") ==
			[for (fastening in mission.fastenings) "work/"+fastening.clamped.member].join(","),
			"Jobs retain the exact CAD clamped members after the seated screw mate is removed");
		var reversed = found.fastenings.copy(); reversed.reverse();
		var reordered = ScrewdrivingMission.generate(new FasteningsResult(reversed,new Diagnostics()),job);
		check([for (fastening in reordered.fastenings) fastening.name()].join(",") ==
			[for (fastening in mission.fastenings) fastening.name()].join(","),"Discovery order cannot change the CAD cross sequence");
		var access = new BlockedScrewAccess();
		var blocked = ScrewdrivingMission.generate(found,job,access);
		check(blocked.steps.length == 0 && access.visited.length == 4,"Access checks every connection and forbids partial missions");
		check(blocked.diagnostics.items.length == 1 && blocked.diagnostics.items[0].subject == mission.fastenings[1].name() &&
			blocked.diagnostics.items[0].message.indexOf("Fixture blocks") >= 0,"Clearance diagnostics identify the exact blocked fastening");
		var refused = false; try {var steps = blocked.require();} catch (_:Dynamic) refused = true;
		check(refused,"An inaccessible mission cannot be required");
		var empty = ScrewdrivingMission.generate(new FasteningsResult([],new Diagnostics()),job);
		check(empty.steps.length == 0 && empty.diagnostics.hasErrors(),"An empty assembly is diagnosed rather than reported complete");
		var duplicate = ScrewdrivingMission.generate(new FasteningsResult([found.fastenings[0],found.fastenings[0]],new Diagnostics()),job);
		check(duplicate.steps.length == 0 && duplicate.diagnostics.hasErrors(),"Duplicate connection and body identities cannot enter a mission");
		var wrong = ScrewdrivingMission.generate(found,(_) -> job(found.fastenings[0]));
		check(wrong.steps.length == 0 && wrong.diagnostics.hasErrors(),"A job author cannot substitute another source fastening");
		var reusedSpare = ScrewdrivingMission.generate(found,(fastening) -> {
			var selected = job(fastening);
			selected.recovery = {replacement:{screw:"shared/spare",presenter:"reserve/nest",presentSensor:"reserve/present",pickup:selected.pickup},
				rejectMember:"reject",reject:selected.pickup};
			return selected;
		});
		check(reusedSpare.steps.length == 0 && reusedSpare.diagnostics.hasErrors(),"A reserved replacement cannot supply two fastenings");
		var reusedPrimary = ScrewdrivingMission.generate(found,(fastening) -> {
			var selected = job(fastening);
			selected.recovery = {replacement:{screw:"work/flange/screw4",presenter:"reserve/nest",presentSensor:"reserve/present",pickup:selected.pickup},
				rejectMember:"reject",reject:selected.pickup};
			return selected;
		});
		check(reusedPrimary.steps.length == 0 && reusedPrimary.diagnostics.hasErrors(),"A replacement cannot consume another fastening's primary supply");
		Sys.println('Screwdriving mission: $count checks passed');
		count = 0;
		var staged = ScrewdrivingMission.generate(found,(fastening) -> {
			var request = job(fastening);
			request.recovery = {replacement:{screw:"reserve/"+fastening.screw.member,presenter:"reserve/nest",presentSensor:"reserve/present",pickup:request.pickup},
				rejectMember:"reject",reject:request.pickup};
			return request;
		},null,(_) -> 2);
		check(staged.require().length == 8,"A staged mission runs four snug passes then four final passes");
		materia.project.SceneScrewInvocation.validate(staged.steps);
		for (index in 0...4) {
			var first = staged.steps[index].screw, finalPass = staged.steps[index+4].screw;
			if (first == null || finalPass == null) throw "Staged generator lost an invocation";
			check(switch first {case Snug(job,slot):slot == 2 && job.fastening == staged.fastenings[index].name(); case _:false;},
				"Snug keeps the CAD cross order and its explicitly qualified slot");
			check(switch finalPass {case Final(name):name == staged.fastenings[index].name(); case _:false;},
				"Final references the corresponding snug job without duplicating supplies or qualification");
		}
		var missingPolicy = ScrewdrivingMission.generate(found,job,null,(_) -> 2);
		check(missingPolicy.steps.length == 0 && missingPolicy.diagnostics.hasErrors(),
			"Missing staged replacement policy produces named generation diagnostics before scene encoding");
		Sys.println('Screw staged generation: $count checks passed');
	}
	static function main():Void run();
}
