/** Topological naming suites alone (`CADKIT_SMOKE_ENTRY=NamingFocused scripts/test-haxeon`). */
class NamingFocused {
	static function main():Int {
		NamingSmoke.run();
		NamingRobustnessSmoke.run();
		return 0;
	}
}
