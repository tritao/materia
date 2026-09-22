# BimKit

BimKit adds BIM roles and relationships to CadKit without changing CadKit element records. It stores wall roles and authoritative host relationships by persistent CadKit element ID, and constructs ordinary CadKit features for opening tools, tool collections, and wall subtraction.

The first supported host adapter is deliberately narrow: straight vertical walls whose local baseline follows +X, with rectangular definition outputs placed by distance along the baseline and sill height. The hosted instance placement is derived from those coordinates and parented to the wall. Opening resolution always uses the wall's uncut feature.
