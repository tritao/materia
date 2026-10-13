package cadkit.sheetmetal;

import cadkit.sheetmetal.process.ProcessBendRule;

/** The explicit source of neutral length: an authored K table or a calibrated tooling setup. */
enum BendRule {
	KFactor(table:KFactorTable);
	Process(setup:ProcessBendRule);
}
