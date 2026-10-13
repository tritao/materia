package cadkit.sheetmetal;

/** Boundary purpose derived from authoring operations, independent of opaque edge identities. */
enum BoundaryOrigin {
	Authored;
	Cut(owner:String);
	Relief(owner:String);
}
