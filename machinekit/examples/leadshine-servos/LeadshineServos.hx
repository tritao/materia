import haxe.io.Bytes;
import machinekit.motion.LeadshineServo;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.AssemblyPreview;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;

/** Small and large frames, without and with brake, in separate rows. */
class LeadshineServos {
	public static function assembly():MachineAssembly {
		var result = new MachineAssembly();
		var names = ["0050MA40", "0100MA40", "0200MA60", "0400MA60", "0750MA80", "1000MA80"];
		for (brake in [false,true]) for (i in 0...names.length) {
			var motor = LeadshineServo.model('ELM1H-${names[i]}${brake ? "E" : "F"}');
			var id = 'motor${i}${brake ? "E" : "F"}';
			var pose = AssemblyFrames.identity(); pose.x = i*120; pose.y = brake ? 150 : 0;
			result.addComponent(id, motor, pose);
			result.addComponent(id+"-shaft", motor.shaft(), pose);
			result.addMate(id+"-shaft-seat", "fixed", id, "shaftAxis", id+"-shaft", "input");
		}
		var largeNames = ["ELM2L-1500LA100FT-H", "ELM2H-1800LD130F-HD", "ELM2M-2000LB130F-H",
			"ELM2M-2900LD180FT-H", "ELM2M-4400LD180FT-H", "ELM2M-5500LD180FT-H", "ELM2M-7500LD180FT-H"];
		for (brake in [false,true]) for (i in 0...largeNames.length) {
			var name = largeNames[i];
			if (brake) name = StringTools.replace(StringTools.replace(name, "FT-H", "ET-H"), "F-H", "E-H");
			var motor = LeadshineServo.model(name), id = 'large${i}${brake ? "E" : "F"}';
			var pose = AssemblyFrames.identity(); pose.x = i*250; pose.y = brake ? 780 : 480;
			result.addComponent(id, motor, pose);
			result.addComponent(id+"-shaft", motor.shaft(), pose);
			result.addMate(id+"-shaft-seat", "fixed", id, "shaftAxis", id+"-shaft", "input");
		}
		return result;
	}
	public static function preview():Bytes return SceneArtifact.encode(AssemblyPreview.scene(assembly(), "leadshine-servos"));
	public static function main():Void {
		var scene = SceneArtifact.decode(preview());
		if (scene.assemblyDefinition == null || scene.assemblyDefinition.occurrences.length != 52)
			throw "Leadshine gallery is missing motor or shaft occurrences";
		var directory = Sys.getEnv("LEADSHINE_STEP_DIR");
		if (directory != null) {
			if (!sys.FileSystem.exists(directory)) sys.FileSystem.createDirectory(directory);
			for (name in LeadshineServo.catalog().designations()) {
				var motor = LeadshineServo.model(name), housing = motor.geometry(), shaft = motor.shaft().geometry();
				housing.exportStep(directory+"/"+name+"-housing.step");
				shaft.exportStep(directory+"/"+name+"-shaft.step");
				housing.close(); shaft.close();
			}
		}
		Sys.println('Leadshine gallery: ${scene.parts.length} motor and shaft parts');
	}
}
