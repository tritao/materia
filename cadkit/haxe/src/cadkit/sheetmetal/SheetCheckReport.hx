package cadkit.sheetmetal;

/** Findings and their developed geometry share one immutable evaluation snapshot. */
class SheetCheckReport {
	public final flat:FlatPattern;
	final findingData:Array<SheetFinding>;
	public function new(flat:FlatPattern, findings:Array<SheetFinding>) { this.flat = flat; findingData = findings.copy(); }
	public function findings():Array<SheetFinding> { return findingData.copy(); }
	public function has(kind:SheetCheckKind):Bool { for (finding in findingData) if (finding.kind == kind) return true; return false; }
}
