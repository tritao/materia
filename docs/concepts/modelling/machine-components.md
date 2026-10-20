---
title: Machine components
description: A part that knows its geometry, how it attaches, what it is made of, and how it appears on the bill of materials.
status: draft
---

# Machine components

## The problem

A CAD solid is just a shape. A machine needs more from each of its parts:

- where it attaches, so that other parts can be placed against it without
  hand-typed coordinates;
- what it is, so that it can be bought or made: a part number, a material, a
  line on the bill of materials;
- what it can do, for the parts that do something: a motor's torque curve, a
  driver's current, a rail's load rating;
- how sure we are of all that: a value from a datasheet is not the same as an
  assumption made to get a model running.

## The model

A **machine component** (`MachineComponent`) is one machine part with four
things attached:

1. **A geometry generator.** `geometry(detail)` builds the part's solid at one
   of two levels of detail: `Envelope`, the outer envelope for clearance and
   packaging, and `Preview`, recognizable exterior features with simplified
   internals. Threads are described (size, pitch, length), not modelled.
2. **Named connectors.** A connector is a frame on the part with a name and a
   role: a mounting face, a shaft axis, a bolt hole. Components attach **only**
   through connectors: a joint places one component's connector on another's.
   By convention a component's own axis (a rail's length, a motor's shaft) is
   its +Z, and its connectors' +Y points along it, so a default joint acts
   along the part's axis (see [Conventions](../foundations/conventions.md#joints-and-connectors)).
3. **A material and its mass.** Mass comes from a declared value when the
   datasheet gives one, otherwise from the geometry's volume and the material's
   density. Every mass records which of the two it is.
4. **A BOM line.** A part number (from the part's recipe, or its designation),
   a description, the material, and a quantity of one. The machine's bill of
   materials is the sum of its components' lines, so it can never drift from
   the model.

Components that do something also expose typed capabilities: a motor's drive
model, a driver's ratings, a cylinder, a valve, a pressure source. Components
that carry services have **ports** (compressed air, vacuum, electrical power,
signals) whose connections are checked: a consumer must be supplied, and the
supply must match its kind and interface.

### Catalog parts and how sure we are

Most components come from **catalogs** with published dimensions: NEMA
steppers, Leadshine servos, MGN and HGR linear rails, MISUMI extrusion, ISO
fasteners and bearings, Schmalz suction parts, roller chain. Every catalog row
records where its data came from and how far it was checked:

- the **source** (a datasheet URL) and the **standard** it follows;
- the **kind of dimension**: nominal, minimum, maximum, or *unverified*;
- **conformance**: an exact interface, a nominal envelope, or a generic
  approximation;
- which fields were **verified**.

Values that come from somewhere other than a datasheet say so. Each carries a
**basis**: derived, catalog, stated, assumed or measured. A part whose ratings
and envelope are assumptions is an **assumed catalog component**: its
designation ends in `-ASSUMED-…` and its description starts with "Assumed",
so it cannot be mistaken for vendor data on the bill of materials.

### Subassemblies

A machine assembly can **include** another as a subassembly: its members
appear under a prefix (`tool/cup`), and the connectors it exposes can be
joined from outside. Its ports are not exposed automatically; the including
assembly connects or re-exposes them.

## In Materia

| Piece | Where |
|---|---|
| Components, detail levels, connectors, BOM lines | `machinekit.component`: `MachineComponent`, `ComponentDetail`, `Connector`, `BomItem` |
| Catalogs and their provenance | `machinekit.catalog`: `Catalog`, `CatalogMetadata` |
| Value provenance | `machinekit.transmission.ValueBasis`, `EngineeringMetadata`, `AssumedCatalogComponent` |
| Bill of materials | `MachineAssembly.billOfMaterials()`, `AssemblyInventory`, `Bom` |
| Ports and services | `ComponentPort`, `ServiceNetwork` |
| Supplier STEP parts and sourcing | `SupplierCadComponent`, `SupplierOffer`, `SourcingPlan` (`machinekit/examples/supplier-fasteners`) |

`addTo(model, id)` registers a component and its connectors in a CadKit
assembly; from there components are joined with the assembly API (see
[Assemblies and mates](assemblies-and-mates.md)).

## What it isn't

- **Not a joint or a motion.** Joints, couplings and actuators belong to the
  assembly the component is placed in.
- **Not an occurrence.** A component is a part definition; placing it in an
  assembly creates an occurrence. One definition can be placed many times.
- **Not a purchase.** A component has no supplier or price; sourcing is a
  separate, optional overlay.
- **Not proof of vendor data.** A catalog part is only as verified as its
  metadata says.

## See also

- [The life of a machine](../start/life-of-a-machine.md), step 2
- [Assemblies and mates](assemblies-and-mates.md)
- [Transmissions](transmissions.md)
- [Manufacturing outputs](../process/manufacturing-outputs.md)

## Open questions

- **The bill of materials is not yet a product.** `machinekit/BOM_PLAN.md`
  (B1–B8, not started) adds stable part numbers with revision fingerprints, a
  required make-or-buy kind for every line, purchase units, and one JSON export.
  Today part numbers can contain raw floats, fasteners are missing, and
  workpieces and tools appear in the machine's BOM.
- **Two descriptions of `Envelope`.** The type says moving parts cover their
  full travel; the MachineKit README says it gives exact standard boundary
  dimensions.
