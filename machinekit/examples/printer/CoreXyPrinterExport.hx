import camkit.additive.Slicer;
import camkit.additive.SlicerSettings;
import cnckit.CncController;
import cnckit.printer.PrinterFlavour;
import cnckit.printer.PrinterWriter;
import machinekit.motion.NemaStepper;
import machinekit.printing.KlipperConfig;
import materia.project.SceneArtifact.SceneArtifactPrinting;
import sys.FileSystem;
import sys.io.File;

/**
 * What the printer example hands to a real Klipper machine: its `printer.cfg`, the list of pins the board must supply,
 * and the demo print (a bracket for a NEMA 17) sliced for it as Klipper G-code, written together in one directory.
 */
class CoreXyPrinterExport {
	public static inline var CONFIG:String = "printer.cfg";
	public static inline var PINS:String = "pins.txt";
	public static inline var BRACKET:String = "nema17-mount-bracket.gcode";

	/** The demo bracket sliced with the default settings, as Klipper G-code. */
	public static function bracketProgram():String {
		var bracket = new NemaMountBracket(NemaStepper.frame(17));
		var part = bracket.geometry();
		var sliced = Slicer.slice(part, new SlicerSettings(), null, "bracket");
		part.close();
		return PrinterWriter.write(sliced.program, new CncController(Printer), Klipper);
	}

	/** Writes the config, the pin report and the bracket's G-code into `directory`; returns the paths. */
	public static function write(directory:String, source:CoreXyPrinter, job:SceneArtifactPrinting, x:PrinterAxisLimit,
			y:PrinterAxisLimit, z:PrinterAxisLimit, cartesianSpeed:Float):Array<String> {
		if (!FileSystem.exists(directory)) FileSystem.createDirectory(directory);
		var model = CoreXyPrinterKlipper.printer(source, job, x, y, z, cartesianSpeed);
		var written:Array<String> = [];
		function save(name:String, text:String):Void {
			var path = directory + "/" + name;
			File.saveContent(path, text);
			written.push(path);
		}
		save(CONFIG, KlipperConfig.render(model));
		save(PINS, KlipperConfig.pinReport(model));
		save(BRACKET, bracketProgram());
		return written;
	}
}
