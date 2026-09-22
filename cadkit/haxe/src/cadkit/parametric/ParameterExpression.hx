package cadkit.parametric;

import cadkit.parametric.ExpressionValue;
import cadkit.parametric.ParameterKind;

/** Small dimensional expression parser supporting identifiers, +, -, *, /, and parentheses. */
class ParameterExpression {
	public final source:String;
	public final dependencies:Array<String>;

	private var index:Int;
	private var lookup:String->ExpressionValue;

	public function new(source:String) {
		if (source == null || StringTools.trim(source) == "")
			throw new ParametricError("parameter expression must not be empty");
		this.source = source;
		dependencies = [];
		index = 0;
		lookup = function(name) return new ExpressionValue(0, ParameterKind.Scalar);
		parse(function(name) {
			if (dependencies.indexOf(name) < 0)
				dependencies.push(name);
			return new ExpressionValue(1, ParameterKind.Scalar);
		}, false);
	}

	public function evaluate(resolve:String->ExpressionValue):ExpressionValue {
		return parse(resolve, true);
	}

	private function parse(resolve:String->ExpressionValue, dimensional:Bool):ExpressionValue {
		index = 0;
		lookup = resolve;
		var result = addition(dimensional);
		skipSpace();
		if (index != source.length)
			throw new ParametricError("unexpected expression token at " + index);
		if (dimensional && !Math.isFinite(result.value))
			throw new ParametricError("parameter expression produced a non-finite value");
		return result;
	}

	private function addition(dimensional:Bool):ExpressionValue {
		var left = multiplication(dimensional);
		while (true) {
			skipSpace();
			if (!take("+") && !take("-"))
				return left;
			var operator = source.charAt(index - 1);
			var right = multiplication(dimensional);
			if (dimensional && left.kind != right.kind)
				throw new ParametricError("expression addition requires matching parameter types");
			left = dimensional
				? new ExpressionValue(operator == "+" ? left.value + right.value : left.value - right.value, left.kind)
				: new ExpressionValue(1, ParameterKind.Scalar);
		}
	}

	private function multiplication(dimensional:Bool):ExpressionValue {
		var left = unary(dimensional);
		while (true) {
			skipSpace();
			if (!take("*") && !take("/"))
				return left;
			var operator = source.charAt(index - 1);
			var right = unary(dimensional);
			var kind = dimensional ? combinedKind(left.kind, right.kind, operator) : ParameterKind.Scalar;
			left = dimensional
				? new ExpressionValue(operator == "*" ? left.value * right.value : left.value / right.value, kind)
				: new ExpressionValue(1, ParameterKind.Scalar);
		}
	}

	private function unary(dimensional:Bool):ExpressionValue {
		skipSpace();
		if (take("-")) {
			var value = unary(dimensional);
			return new ExpressionValue(-value.value, value.kind);
		}
		if (take("+"))
			return unary(dimensional);
		return primary(dimensional);
	}

	private function primary(dimensional:Bool):ExpressionValue {
		skipSpace();
		if (take("(")) {
			var value = addition(dimensional);
			skipSpace();
			if (!take(")"))
				throw new ParametricError("parameter expression is missing a closing parenthesis");
			return value;
		}
		var start = index;
		var first = source.charCodeAt(index);
		if (isDigit(first) || (first == 46 && isDigit(source.charCodeAt(index + 1)))) {
			while (isDigit(source.charCodeAt(index))) index++;
			if (source.charCodeAt(index) == 46) {
				index++;
				while (isDigit(source.charCodeAt(index))) index++;
			}
			var exponent = source.charCodeAt(index);
			if (exponent == 69 || exponent == 101) {
				index++;
				var sign = source.charCodeAt(index);
				if (sign == 43 || sign == 45) index++;
				var exponentStart = index;
				while (isDigit(source.charCodeAt(index))) index++;
				if (index == exponentStart)
					throw new ParametricError("invalid expression number");
			}
			if (source.charCodeAt(index) == 46)
				throw new ParametricError("invalid expression number");
			var number = Std.parseFloat(source.substring(start, index));
			if (!Math.isFinite(number))
				throw new ParametricError("invalid expression number");
			return new ExpressionValue(number, ParameterKind.Scalar);
		}
		var identifierStart = source.charCodeAt(index);
		if (!isLetter(identifierStart) && identifierStart != 95)
			throw new ParametricError("expected expression value at " + index);
		while (index < source.length) {
			var code = source.charCodeAt(index);
			if ((code >= 65 && code <= 90) || (code >= 97 && code <= 122) || (code >= 48 && code <= 57) || code == 95 || code == 46)
				index++;
			else
				break;
		}
		if (index == start)
			throw new ParametricError("expected expression value at " + index);
		return lookup(source.substring(start, index));
	}

	private static function isDigit(code:Null<Int>):Bool {
		return code != null && code >= 48 && code <= 57;
	}

	private static function isLetter(code:Null<Int>):Bool {
		return code != null && ((code >= 65 && code <= 90) || (code >= 97 && code <= 122));
	}

	private static function combinedKind(left:String, right:String, operator:String):String {
		if (operator == "*") {
			if (left == ParameterKind.Scalar) return right;
			if (right == ParameterKind.Scalar) return left;
			if (left == ParameterKind.Count) return right;
			if (right == ParameterKind.Count) return left;
			if (left == ParameterKind.Length && right == ParameterKind.Length) return ParameterKind.Area;
			if ((left == ParameterKind.Area && right == ParameterKind.Length)
				|| (left == ParameterKind.Length && right == ParameterKind.Area)) return ParameterKind.Volume;
		} else {
			if (right == ParameterKind.Scalar || right == ParameterKind.Count) return left;
			if (left == right) return ParameterKind.Scalar;
			if (left == ParameterKind.Area && right == ParameterKind.Length) return ParameterKind.Length;
			if (left == ParameterKind.Volume && right == ParameterKind.Length) return ParameterKind.Area;
			if (left == ParameterKind.Volume && right == ParameterKind.Area) return ParameterKind.Length;
		}
		throw new ParametricError("unsupported parameter types in expression: " + left + " " + operator + " " + right);
	}

	private function skipSpace():Void {
		while (index < source.length && StringTools.isSpace(source, index))
			index++;
	}

	private function take(token:String):Bool {
		if (source.charAt(index) != token)
			return false;
		index++;
		return true;
	}
}
