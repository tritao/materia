package cnckit.parse;

import toolpathkit.path.Provenance;
import cnckit.CncDiagnostic;
import cnckit.CncDiagnostic.CncSeverity;
import cnckit.CncDialect;

/**
  Numeric words and both LinuxCNC comment forms, with line recovery. The printer dialect keeps
  comments as data, reads string words (`A"name"`), takes the rest of an M117/M118 line as a
  message, ignores line numbers and checksums, and reads Klipper extended commands.
**/
class CncLexer {
  public static function parse(source:String, ?dialect:CncDialect = LinuxCnc):CncParseResult {
    var printer = dialect == Printer;
    var blocks:Array<CncBlock> = [];
    var diagnostics:Array<CncDiagnostic> = [];
    var lines = source.split("\n");
    for (index in 0...lines.length) {
      var line = lines[index];
      if (StringTools.trim(line) == "%") continue;
      try {
        var block = printer ? lexPrinterLine(line, index + 1) : lexLine(line, index + 1);
        var slash = line.indexOf("/");
        if (slash >= 0 && StringTools.trim(line.substring(0, slash)).length == 0)
          diagnostics.push(new CncDiagnostic(Warning, "CNC_BLOCK_DELETE_IGNORED",
            new Provenance(index + 1, slash + 1, 1),
            "block-delete marker ignored; block executed"));
        if (block.words.length > 0 || block.command != null || block.comments.length > 0) blocks.push(block);
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

  /** A printer line: words, then comments and extended commands as data. */
  public static function lexPrinterLine(line:String, lineNumber:Int):CncBlock {
    var words:Array<CncWord> = [];
    var comments:Array<CncComment> = [];
    var index = 0;
    while (index < line.length && (line.charAt(index) == " " || line.charAt(index) == "\t")) index++;
    // A Klipper extended command: a name of two or more letters or underscores, then KEY=value arguments.
    if (index + 1 < line.length && isLetter(line.charCodeAt(index)) && isNameStart(line.charCodeAt(index + 1))) {
      var end = index;
      while (end < line.length && isNameChar(line.charCodeAt(end))) end++;
      var command = line.substring(index, end).toUpperCase();
      var semicolon = line.indexOf(";", end);
      var body = semicolon < 0 ? line.substring(end) : line.substring(end, semicolon);
      if (semicolon >= 0) comments.push(new CncComment(StringTools.trim(line.substring(semicolon + 1)),
        new Provenance(lineNumber, semicolon + 1, line.length - semicolon)));
      var arguments = new Map<String, String>();
      for (token in splitArguments(body)) {
        var equals = token.indexOf("=");
        if (equals <= 0) fail(lineNumber, index + 1, line.length - index, 'extended command $command argument "$token" is not KEY=value');
        arguments.set(token.substring(0, equals).toUpperCase(), unquote(token.substring(equals + 1)));
      }
      return new CncBlock([], new Provenance(lineNumber, 1, line.length), comments, command, arguments);
    }
    while (index < line.length) {
      var ch = line.charAt(index);
      if (ch == ";") {
        comments.push(new CncComment(StringTools.trim(line.substring(index + 1)),
          new Provenance(lineNumber, index + 1, line.length - index)));
        break;
      }
      if (ch == "*") break; // checksum
      if (ch == "(") {
        var close = line.indexOf(")", index + 1);
        if (close < 0) fail(lineNumber, index + 1, 1, "unclosed comment");
        comments.push(new CncComment(StringTools.trim(line.substring(index + 1, close)),
          new Provenance(lineNumber, index + 1, close - index + 1)));
        index = close + 1; continue;
      }
      if (ch == " " || ch == "\t" || ch == "\r") { index++; continue; }
      var code = line.charCodeAt(index);
      if (!isLetter(code)) fail(lineNumber, index + 1, 1, 'unexpected character "$ch"');
      var letter = ch.toUpperCase(), column = index + 1;
      index++;
      if (index < line.length && line.charAt(index) == '"') {
        var close = line.indexOf('"', index + 1);
        if (close < 0) fail(lineNumber, column, line.length - column + 1, 'unclosed $letter string');
        words.push(new CncWord(letter, 0.0, new Provenance(lineNumber, column, close - column + 1),
          line.substring(index + 1, close)));
        index = close + 1; continue;
      }
      var start = index, digits = 0, dots = 0;
      if (index < line.length && (line.charAt(index) == "+" || line.charAt(index) == "-")) index++;
      while (index < line.length) {
        var current = line.charAt(index), currentCode = line.charCodeAt(index);
        if (currentCode >= 48 && currentCode <= 57) { digits++; index++; }
        else if (current == ".") { dots++; index++; }
        else break;
      }
      if (digits == 0 || dots > 1) fail(lineNumber, column, index - column + 1, 'invalid $letter number');
      var number = Std.parseFloat(line.substring(start, index));
      if (!Math.isFinite(number)) fail(lineNumber, column, index - column + 1, 'non-finite $letter number');
      var word = new CncWord(letter, number, new Provenance(lineNumber, column, index - column + 1));
      words.push(word);
      // The rest of a display message is text, not words.
      if (letter == "M" && (number == 117 || number == 118)) {
        var message = StringTools.trim(line.substring(index));
        if (message.length > 0) comments.push(new CncComment(message, new Provenance(lineNumber, index + 1, line.length - index)));
        break;
      }
    }
    // Line numbers only order a host's stream.
    var kept = [for (word in words) if (word.letter != "N") word];
    return new CncBlock(kept, new Provenance(lineNumber, 1, line.length), comments);
  }

  static function isLetter(code:Int):Bool return (code >= 65 && code <= 90) || (code >= 97 && code <= 122);
  static function isNameStart(code:Int):Bool return isLetter(code) || code == 95;
  static function isNameChar(code:Int):Bool return isNameStart(code) || (code >= 48 && code <= 57);

  /** Whitespace-separated arguments, where a quoted value or a bracketed list may hold spaces. */
  static function splitArguments(body:String):Array<String> {
    var tokens:Array<String> = [], current = new StringBuf(), quote = "", depth = 0, any = false;
    for (index in 0...body.length) {
      var ch = body.charAt(index);
      if (quote != "") { current.add(ch); if (ch == quote) quote = ""; continue; }
      if (ch == '"' || ch == "'") { quote = ch; current.add(ch); any = true; continue; }
      if (ch == "[") depth++;
      if (ch == "]") depth--;
      if ((ch == " " || ch == "\t" || ch == "\r") && depth <= 0) {
        if (any) tokens.push(current.toString());
        current = new StringBuf(); any = false;
        continue;
      }
      current.add(ch); any = true;
    }
    if (any) tokens.push(current.toString());
    return tokens;
  }

  static function unquote(value:String):String {
    if (value.length >= 2 && (value.charAt(0) == '"' || value.charAt(0) == "'") &&
        value.charAt(value.length - 1) == value.charAt(0)) return value.substring(1, value.length - 1);
    return value;
  }

  static function fail(line:Int, column:Int, length:Int, message:String):Void
    throw new CncDiagnostic(Error, "CNC_LEX", new Provenance(line, column, length),
      message);
}
