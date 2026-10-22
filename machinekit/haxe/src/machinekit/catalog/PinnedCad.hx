package machinekit.catalog;

/** Reviewed supplier CAD files pinned by content: geometry loads only a file whose SHA-256 matches its review. */
class PinnedCad {
	/** Lower-case hex SHA-256 of the file at `path`. */
	public static function sha256(path:String):String {
		var digest = haxe.crypto.Sha256.make(sys.io.File.getBytes(path)), hex = new StringBuf();
		for (i in 0...digest.length) hex.add(StringTools.hex(digest.get(i), 2).toLowerCase());
		return hex.toString();
	}

	/** `path`, once it exists and matches `digest`; `prepare` says how to fetch it. */
	public static function require(path:String, digest:String, prepare:String):String {
		if (!sys.FileSystem.exists(path)) throw 'Missing pinned CAD $path; run $prepare';
		if (sha256(path) != digest) throw 'Unreviewed CAD $path: its digest differs from the pinned reference';
		return path;
	}
}
