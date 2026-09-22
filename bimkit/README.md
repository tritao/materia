# BimKit

BimKit adds BIM roles and relationships to CadKit without changing CadKit element records. It stores wall roles and authoritative host relationships by persistent CadKit element ID, and constructs ordinary CadKit features for opening tools, tool collections, and wall subtraction.

The first supported host adapter is deliberately narrow: straight vertical walls whose local baseline follows +X, with rectangular definition outputs placed by distance along the baseline and sill height. The hosted instance placement is derived from those coordinates and parented to the wall. Opening resolution always uses the wall's uncut feature.

`RepeatedHostedWindows` shows four instances of one window definition on two walls between levels. An opening may be unhosted or removed transactionally. A wall must have no hosted openings before removal. Host placement and wall-local cuts follow level changes after successful recompute.

Use `resizeWall` for thickness changes so hosted opening tools follow the new wall depth. Direct edits that leave a tool too shallow, move an opening outside its wall, or overlap two openings fail validation before recompute commits geometry. Failed host edits discard their new graph nodes; committed earlier nodes remain inactive for undo and redo. BimKit JSON version 2 persists the authored host coordinates and the placement to restore on unhosting.
