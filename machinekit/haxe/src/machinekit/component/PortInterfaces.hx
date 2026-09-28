package machinekit.component;

/** Service-port mating rules. Thread designations with -M/-F include sex;
 * legacy designations without a suffix retain exact-match behaviour. */
class PortInterfaces {
	public static function compatible(first:PortInterface, second:PortInterface):Bool {
		if (first == Unspecified || second == Unspecified) return true;
		return switch [first, second] {
			case [PushIn(a), PushIn(b)]: a == b;
			case [Thread(a), Thread(b)]:
				var aMale = StringTools.endsWith(a, "-M"), aFemale = StringTools.endsWith(a, "-F");
				var bMale = StringTools.endsWith(b, "-M"), bFemale = StringTools.endsWith(b, "-F");
				if ((aMale || aFemale) && (bMale || bFemale))
					a.substr(0, a.length - 2) == b.substr(0, b.length - 2) &&
					((aMale && bFemale) || (aFemale && bMale));
				else a == b;
			case [Plug(a, ap), Plug(b, bp)]: a == b && ap == bp;
			case _: false;
		};
	}
}
