package cadkit.sheetmetal;

/** The source of neutral length; tooling-dependent rules are added with the process model. */
enum BendRule {
	KFactor(table:KFactorTable);
}
