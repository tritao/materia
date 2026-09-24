package app;

/** Callback that records one BIM mutation in the owning project history. */
typedef BimProjectEdit = (label:String, change:Void->Void) -> Void;
