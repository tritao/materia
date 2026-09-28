package cnckit.parse;

import toolpathkit.path.Provenance;
import cnckit.CncDiagnostic;
import cnckit.CncDiagnostic.CncSeverity;

/** Numeric words and both LinuxCNC comment forms, with line recovery. */
class CncLexer {
  public static function parse(source:String):CncParseResult {
    var blocks:Array<CncBlock> = [];
    var diagnostics:Array<CncDiagnostic> = [];
    var lines = source.split("\n");
    for (index in 0...lines.length) {
      var line = lines[index];
      if (StringTools.trim(line) == "%") continue;
      try {
        var block = lexLine(line, index + 1);
        var slash = line.indexOf("/");
        if (slash >= 0 && StringTools.trim(line.substring(0, slash)).length == 0)
          diagnostics.push(new CncDiagnostic(Warning, "CNC_BLOCK_DELETE_IGNORED",
            new Provenance(index + 1, slash + 1, 1),
            "block-delete marker ignored; block executed"));
        if (block.words.length > 0) blocks.push(block);
      } catch (error:CncDiagnostic) {
        diagnostics.push(error);
      }
    }
    return new CncParseResult(blocks, diagnostics);
  }

  public static function lexLine(line:String, lineNumber:Int):CncBlock {
    var words:Array<CncWord> = [];
    var index = 0;
    while (index < line.length && (line.charAt(index) == " " ||
        line.charAt(index) == "\t")) index++;
    if (index < line.length && line.charAt(index) == "/") index++;
    while (index < line.length) {
      var ch = line.charAt(index);
      if (ch == ";") break;
      if (ch == "(") {
        var close = line.indexOf(")", index + 1);
        if (close < 0) fail(lineNumber, index + 1, 1, "unclosed comment");
        index = close + 1; continue;
      }
      if (ch == " " || ch == "\t" || ch == "\r") { index++; continue; }
      var code = line.charCodeAt(index);
      if (!((code >= 65 && code <= 90) || (code >= 97 && code <= 122)))
        fail(lineNumber, index + 1, 1, 'unexpected character "$ch"');
      var letter = ch.toUpperCase(), column = index + 1;
      index++;
      var start = index, digits = 0, dots = 0;
      if (index < line.length && (line.charAt(index) == "+" ||
          line.charAt(index) == "-")) index++;
      while (index < line.length) {
        var current = line.charAt(index), currentCode = line.charCodeAt(index);
        if (currentCode >= 48 && currentCode <= 57) { digits++; index++; }
        else if (current == ".") { dots++; index++; }
        else break;
      }
      if (digits == 0 || dots > 1) fail(lineNumber, column, index - column + 1,
        'invalid $letter number');
      var number = Std.parseFloat(line.substring(start, index));
      if (!Math.isFinite(number)) fail(lineNumber, column, index - column + 1,
        'non-finite $letter number');
      words.push(new CncWord(letter, number,
        new Provenance(lineNumber, column, index - column + 1)));
    }
    return new CncBlock(words, new Provenance(lineNumber, 1, line.length));
  }

  static function fail(line:Int, column:Int, length:Int, message:String):Void
    throw new CncDiagnostic(Error, "CNC_LEX", new Provenance(line, column, length),
      message);
}
