import compiler.Source.SourceFile;
import compiler.documentation.HaxeXmlWriter;
import compiler.modules.ModuleState;
import compiler.syntax.ConditionalCompilation;
import compiler.syntax.Lexer;
import compiler.syntax.Parser;
import haxe.io.Path;
import sys.FileSystem;
import sys.io.File;

/**
 * Writes Haxe type-description XML for every module below the given source roots,
 * from Haxeon's parser alone: declarations and doc comments, no typing or code generation.
 *
 * Usage: ApiXml OUTPUT.xml ROOT...
 */
class ApiXml {
	static final DEFINES = ["target" => "hl", "hl" => "1", "sys" => "1", "haxeon" => "1"];

	static function main() {
		var arguments = Sys.args();
		if (arguments.length < 2) {
			Sys.stderr().writeString("usage: ApiXml OUTPUT.xml ROOT...\n");
			Sys.exit(2);
		}
		var modules = new Map<String, ModuleState>(), failures = 0;
		for (root in arguments.slice(1))
			for (path in sources(root)) {
				var file = new SourceFile(path, File.getContent(path));
				try {
					var conditional = ConditionalCompilation.process(file, DEFINES);
					var program = new Parser(new Lexer(file, conditional.text).tokenize()).parseProgram();
					var name = Path.withoutExtension(Path.withoutDirectory(path));
					if (program.packageName != null && program.packageName != "")
						name = program.packageName + "." + name;
					var state = new ModuleState(name, file);
					for (decl in program.classes) {
						var fields = decl.fields.filter(field -> isPublic(file.text, field.span.start));
						decl.fields.splice(0, decl.fields.length);
						for (field in fields)
							decl.fields.push(field);
						var methods = decl.methods.filter(method -> isPublic(file.text, method.span.start));
						decl.methods.splice(0, decl.methods.length);
						for (method in methods)
							decl.methods.push(method);
					}
					state.ast = program;
					modules.set(name, state);
				} catch (error:Dynamic) {
					failures++;
					Sys.stderr().writeString('ApiXml: skipped $path: $error\n');
				}
			}
		File.saveContent(arguments[0], HaxeXmlWriter.emit(modules));
		Sys.println('ApiXml: ${Lambda.count(modules)} modules, $failures skipped -> ${arguments[0]}');
	}

	static final MODIFIERS = [
		"public", "private", "static", "inline", "final", "var", "override", "dynamic", "extern", "overload", "macro"
	];

	/**
	 * Whether the member declared at `start` is public. Haxeon's AST drops member visibility
	 * (and its spans begin at the name or `function`), so this walks back over the modifiers,
	 * metadata and comments in front of the declaration. Class members are private by default.
	 */
	static function isPublic(text:String, start:Int):Bool {
		var position = start;
		while (true) {
			while (position > 0 && StringTools.isSpace(text, position - 1))
				position--;
			if (position >= 2 && text.substr(position - 2, 2) == "*/") {
				var open = text.lastIndexOf("/*", position - 2);
				if (open < 0)
					return false;
				position = open;
				continue;
			}
			if (position > 0 && text.charAt(position - 1) == ")") {
				var depth = 0;
				do {
					position--;
					var c = text.charAt(position);
					if (c == ")")
						depth++;
					else if (c == "(")
						depth--;
				} while (position > 0 && depth > 0);
			}
			var end = position;
			while (position > 0 && isWordCharacter(text.charCodeAt(position - 1)))
				position--;
			if (position == end)
				return false;
			var word = text.substring(position, end);
			if (position > 0 && text.charAt(position - 1) == ":" && position > 1 && text.charAt(position - 2) == "@") {
				position -= 2;
				continue;
			}
			if (position > 0 && text.charAt(position - 1) == "@") {
				position--;
				continue;
			}
			if (word == "public")
				return true;
			if (MODIFIERS.indexOf(word) < 0)
				return false;
		}
	}

	static function isWordCharacter(code:Int):Bool
		return (code >= "a".code && code <= "z".code) || (code >= "A".code && code <= "Z".code) || (code >= "0".code && code <= "9".code)
			|| code == "_".code || code == ".".code;

	static function sources(root:String):Array<String> {
		var found = [];
		if (!FileSystem.exists(root))
			return found;
		for (entry in FileSystem.readDirectory(root)) {
			var path = Path.join([root, entry]);
			if (FileSystem.isDirectory(path))
				found = found.concat(sources(path));
			else if (StringTools.endsWith(entry, ".hx"))
				found.push(path);
		}
		found.sort(Reflect.compare);
		return found;
	}
}
