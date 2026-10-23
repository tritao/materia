import machinekit.tooling.CastingMoldCad;
import materia.tooling.ToolingCodec;

class MillColumnMoldPreview {
	static function main():Void {
		var frame=MillMoldSetup.frame(),document=MillColumnMoldSetup.document(frame);
		if (Sys.args().indexOf("--fabrication")>=0) {
			MillFabricationPreview.write(document,MillColumnMoldSetup.options(frame),frame.column,
				"column","mill-column-process-review");
			return;
		}
		var cwd=Sys.getCwd(),root=sys.FileSystem.exists(cwd+"/machinekit")
			? cwd+"/machinekit/examples/bench-mill" : cwd;
		var directory=root+"/build/tooling/column";
		machinekit.tooling.ToolingFiles.directory(directory);
		sys.io.File.saveContent(directory+"/source-document.json",ToolingCodec.encode(document));
		Sys.println("Column source: "+document.input.features.length+" features, "+document.input.inserts.length+" inserts");
		var plan=CastingMoldCad.plan(document,MillColumnMoldSetup.options(frame));
		var review=CastingMoldCad.review(plan);
		machinekit.tooling.CastingMoldExport.write(plan,directory,review);
		sys.io.File.saveContent(directory+"/review.json",haxe.Json.stringify(review,null,"\t")+"\n");
		Sys.println("Column mold review: "+directory);
	}
}
