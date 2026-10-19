# Retro Golf Simulator — Detailed Specifications

Source document: [specifications.md](specifications.md) (product vision). This file turns that vision into
implementable requirements. It does **not** contain the implementation.

Repository (fork of OpenRCT2): <https://github.com/frutiemax92/open-golf-simulator>. All development, issues and
pull requests for the game happen there; upstream OpenRCT2 (<https://github.com/OpenRCT2/OpenRCT2>) is only a
source of engine fixes.

---

## 0. How to use this document

- Requirement IDs look like `TER-012`. Reference them in commit messages, code comments (sparingly) and tests.
- **MUST** = required for the feature to be accepted. **SHOULD** = expected unless there is a documented reason.
  **MAY** = optional / later.
- Each section ends with **Acceptance criteria** (AC). A feature is done only when every AC is met and covered by a
  test (unit test in `test/tests/` with GoogleTest, or a scripted in-game check when a unit test is impossible).
- Work in the phase order of §25. Do not start a phase before the previous phase's AC pass, unless the phase is
  marked independent.
- When a requirement is ambiguous, check §26 (Open questions). Default answers are given there; do not block on them.
- Follow the existing OpenRCT2 code style (`.clang-format`, `PascalCase` types, `camelCase` members as in
  recent code, `k`-prefixed constants, `namespace OpenRCT2`).
- Keep the simulation **deterministic** (see §3.4): every simulation change must be reproducible from the same
  save + same inputs.

---

## 1. Product summary

An isometric, retro-looking golf course management game built on the OpenRCT2 engine. The player shapes terrain,
paints land textures, lays out golf holes, driving ranges and mini-putt courses, builds facilities, streets and
parking lots, and manages a population of golfers whose needs, skills and happiness drive the course's
popularity and profits.

### 1.1 Goals
- G1: Reuse OpenRCT2's engine: map/tile storage, isometric viewport and paint sorting, windows/UI framework,
  object loading, finance, save/load, scripting.
- G2: Fully open-source assets. Zero dependency on RCT1/RCT2 data files (`g1.dat`, `g2.dat`, `csg*.dat`, `*.DAT`
  objects, `.sv4/.sv6/.td6`, original sounds and music) and **no reuse of OpenRCT2's own graphics or sounds**.
- G3: Every content item (terrain textures, streams, paths, streets, buildings, scenery, golfers, vehicles,
  animals, sounds) is a loadable object: JSON + assets packed in a compressed archive.

### 1.2 Non-goals (for the first release)
- Rides/roller coasters, ride ratings, track designs, RCT1/RCT2 scenario import.
- Network multiplayer (keep the code deterministic so it stays possible later).
- Realistic, simulation-grade golf physics. Physics must be believable and readable, not exact.

---

## 2. Units, coordinates and glossary

| Term | Definition |
|---|---|
| Tile | Map cell. **1 tile = 5 yards = 4.572 m** horizontally. 32 world units (`kCoordsXYStep`). |
| World unit (xy) | 1/32 tile = 0.142875 m. |
| Z unit | Same physical scale as the xy unit (0.142875 m) so that physics is isotropic. |
| Quarter step (Q) | 8 z units (`kCoordsZStep`) = 1.143 m. The new smallest land height increment. |
| Full step | 4 Q = 32 z units = 4.572 m. |
| Hole | A golf hole: up to 3 tee boxes + 1 green + 1 cup (pin). |
| Course | An ordered list of holes (1–18 for a regular course, 1–18 for a mini-putt course). |
| Golfer | The new peep type (replaces OpenRCT2 `Guest`). |
| Lie | The surface the ball is resting on (fairway, rough, sand…). |
| Tick | One simulation update. Keep OpenRCT2's 40 ticks/s game rate. |

Directions follow OpenRCT2: `0 = −x`, `1 = +y`, `2 = +x`, `3 = −y`. Corners: N, E, S, W as in
[Slope.h](../src/openrct2/world/tile_element/Slope.h).

Display units: yards by default, metres as an option (reuse the measurement-format setting in `Config`).
1 tile is displayed as 5 yd / 4.6 m.

---

## 3. Architecture strategy

### 3.1 Fork approach
- ARC-001 MUST: The game is developed in the fork
  [frutiemax92/open-golf-simulator](https://github.com/frutiemax92/open-golf-simulator). CMake produces a new
  target executable named `open-golf-simulator`. The rest of this document calls the game "the game".
- ARC-001a MUST: Branching: `develop` is the integration branch, as in OpenRCT2. Each phase (§25) or feature is
  developed on its own branch and merged through a pull request on the fork. Pull requests never target upstream
  OpenRCT2.
- ARC-001b SHOULD: Keep `upstream` as a git remote pointing to OpenRCT2. Upstream fixes to reused engine parts
  (§3.2) MAY be cherry-picked. After ARC-002, a full upstream merge is no longer expected.
- ARC-001c MUST: Rename the user-visible product identity: window title, About window, user data folder
  (e.g. `~/.config/open-golf-simulator`) and the game's version string. This keeps the game from sharing config
  or saves with an installed OpenRCT2. The GPLv3 licence and the OpenRCT2 copyright/contributors credits are kept.
- ARC-002 MUST: Remove or compile out subsystems that are not needed and that depend on RCT assets:
  rides (`src/openrct2/ride`), track paint (`paint/track`, `paint/track_pieces`, `paint/vehicle`), ride windows,
  RCT1/RCT2 importers (`rct1`, `rct12`, `rct2`, `TrackImporter`, `ParkImporter` for sv4/sv6), the title sequence
  that uses RCT2 parks. Removal may happen progressively; each removal MUST keep the build green.
- ARC-003 SHOULD: Prefer adding new modules under new folders rather than mutating ride/guest code:
  - `src/openrct2/golf/` — holes, courses, scheduling, tournaments, driving range, mini putt.
  - `src/openrct2/physics/` — ball physics, collision world.
  - `src/openrct2/entity/Golfer.*`, `Vehicle*` (cars, carts), `Animal.*`, `Ball.*`.
  - `src/openrct2/drawing/model/` — 3D model loading and software rasterisation (§11).
  - `src/openrct2/world/tile_element/StreamElement.*`, `StreetElement.*`, `BuildingElement.*`.
- ARC-004 MUST: Guests (`Guest.cpp`, 7.7k lines) are **not** extended into golfers. A new `Golfer` entity is
  written; reuse generic pieces only (`Peep` base movement, pathfinding helpers, thoughts/notification UI) where
  they fit.

### 3.2 Reused engine parts
Map storage (`TileElement`, `MapLimits`), viewport/paint sorting (`paint/Paint.cpp`, bounding boxes), window
system (`openrct2-ui`), object repository/manager/factory (`object/`), finance & loans (`management/Finance`),
date/time, weather (MAY be reused for wind/rain), save system (new format, §23), plugin/scripting API (`scripting/`).

### 3.3 Rendering stack
- ARC-010 MUST: Keep the 8-bit palette rendering pipeline (`X8DrawingEngine`, OpenGL engine) for terrain, UI and
  sprites so the look stays retro and close to OpenRCT2.
- ARC-011 MUST: 3D entities are rasterised in software into palettised sprites and inserted in the normal paint
  sort (§11). No separate GPU 3D pass is required.

### 3.4 Determinism
- ARC-020 MUST: Simulation code (ball physics, golfer AI, vehicles, animals, economy) uses deterministic
  arithmetic: fixed-point (recommended: 32-bit, 16.16 or a dedicated `Fixed` type in `core/`) or integer math.
  No `float`/`double` in simulation state. Rendering and audio MAY use floats.
- ARC-021 MUST: All randomness goes through the scenario RNG (`ScenarioRand`) so replays match.

---

## 4. Asset independence (Phase 0)

OpenRCT2 currently needs `g1.dat` for UI sprites, fonts, cursors, terrain and most images.

- AST-001 MUST: The game starts without any RCT1/RCT2 installation. The "locate RCT2" first-run flow is removed.
- AST-002 MUST: Provide a **base asset pack** (`data/base/`, packed as objects or as a single "base graphics"
  archive) that replaces every `g1` image index the remaining code uses: window frames, buttons, scrollbars, tab
  icons, toolbar icons, cursors, map tool icons, money/thought icons. The existing `SpriteIds.h` indices MAY be
  kept as stable logical IDs mapped to the new pack.
- AST-003 MUST: Text uses TTF fonts only (`drawing/TTF.cpp`) with an open-licence font (e.g. a pixel font under
  OFL) shipped in `data/`. Sprite fonts from `g1` are no longer used.
- AST-004 MUST: All placeholder art is generated or drawn for this project. Placeholders MUST be visually
  distinctive (flat colours, labels, checker patterns) so that a developer can identify each texture, quadrant,
  slope level and object at a glance. A small script (Python, under `scripts/placeholders/`) SHOULD generate them
  reproducibly.
- AST-005 MUST: No sound or music from RCT/OpenRCT2. Placeholder sounds are generated tones/noises or CC0 files
  with their licence recorded in the object JSON (`authors`, `licence`).
- AST-006 MUST: A CI check fails the build if any file under `data/` is byte-identical to a known RCT/OpenRCT2
  asset or if code references `g1.dat`, `g2.dat`, `csg1*.dat`.

**AC**: a clean machine with no RCT files launches the game to the main menu and into an empty map editor with
all UI drawn from the base pack.

---

## 5. Object system

### 5.1 Package format
- OBJ-001 MUST: An object is a ZIP archive (same container as OpenRCT2 `.parkobj`, loader in
  [ObjectFactory.cpp](../src/openrct2/object/ObjectFactory.cpp)) containing `object.json` at the root plus asset
  files (PNG images, `.glb` models, `.ogg`/`.wav` sounds). Extension: keep `.parkobj` (see §26).
- OBJ-002 MUST: Objects are discovered at startup in `data/object/` and the user object folder, indexed by the
  existing `ObjectRepository`, and loaded on demand by `ObjectManager`.
- OBJ-003 MUST: Common JSON fields (as in OpenRCT2): `id` (e.g. `golf.terrain.fairway`), `version`, `authors`,
  `licence` (new, required), `objectType`, `sourceGame: "custom"`, `strings` (`name`, `description`, localised:
  `{ "en-GB": "…" }`), `properties`.
- OBJ-004 MUST: Asset paths inside `properties` are relative to the archive root. Missing files fail the load
  with a clear error naming the object id and path.
- OBJ-005 MUST: New object types are added to `ObjectType` in
  [ObjectTypes.h](../src/openrct2/object/ObjectTypes.h); obsolete ride-related types are removed when ARC-002 is
  done. Object selection UI (`EditorObjectSelection`) lists the new types with tabs.

### 5.2 Object types to implement

| `objectType` | Purpose | Section |
|---|---|---|
| `golf_terrain_surface` | Land texture with physics | §6.5 |
| `golf_terrain_edge` | Cliff/side texture | §6.7 |
| `water` | Water palette/animation (existing type, new assets) | §7 |
| `stream` | Water stream overlay | §8 |
| `building` | Club house, cottage, food truck, kiosk… | §9 |
| `footpath_surface` / `footpath_railings` | Pathways (existing types, new assets) | §10 |
| `path_addition` | Benches, bins, lamps (existing type, new assets) | §10.3 |
| `street` | Roads (one-way / two-way) | §12 |
| `small_scenery` / `large_scenery` / `walls` | Scenery, fishnets (existing types + `collision`) | §19 |
| `golfer_model` | 3D golfer body + animations | §13 |
| `vehicle_model` | Cars and golf carts | §12, §14 |
| `animal` | Animals | §20 |
| `audio` | Sound banks (existing type, extended) | §21 |
| `club_set` | Golf club stats (MAY be hard-coded first) | §15.2 |

### 5.3 Common physics block
Any object a ball can hit (building, scenery, wall, vehicle) MUST accept an optional `collision` block:

```json
"collision": {
  "shape": "box",              // "box" | "cylinder" | "none"
  "height": 48,                 // z units, from base
  "footprint": [32, 32],        // xy units, box only; cylinder uses "radius"
  "restitution": 0.5,           // 0..1, energy kept on bounce
  "friction": 0.3,              // 0..1, tangential loss on bounce
  "passThroughChance": 0.0      // 0..1, e.g. tree foliage lets some balls through
}
```
If absent, defaults are derived from the object's clearance height and footprint, `restitution 0.4`,
`friction 0.3`, `passThroughChance 0`.

**AC**: an object with a missing image, bad JSON or unknown type is reported in the console and skipped without a
crash; the object selection window shows every new type.

---

## 6. Map and terrain

### 6.1 Map size
- TER-001 MUST: Default new map size 256×256 tiles (≈1280×1280 yd), maximum kept at OpenRCT2's technical limit
  (`kMaximumMapSizeTechnical`). Minimum supported for an 18-hole course: 256×256.
- TER-002 MUST: Performance target: a 256×256 map with an 18-hole course, 300 golfers, 60 vehicles and 50 animals
  runs at ≥ 30 fps at 1× zoom on a mid-range 2020 laptop, and the simulation tick stays < 10 ms.

### 6.2 Height model (4 slope levels)
OpenRCT2 stores a base height in land steps of 16 z units and a slope as "corners raised by one step" (+ a
diagonal flag for steep slopes). The new model uses quarter steps.

- TER-010 MUST: Each surface element stores a **base height** in Q (8 z units) and, for each of its 4 corners
  (N, E, S, W), a **corner offset** `0..4` Q above the base. At least one corner has offset 0 (normalised form).
- TER-011 MUST: Slope levels, defined as the height difference between two adjacent corners:
  - 1 Q = quarter step (new),
  - 2 Q = half step (equals current OpenRCT2 slope),
  - 3 Q = three-quarter step (new),
  - 4 Q = full step (equals current OpenRCT2 steep slope rise).
- TER-012 MUST: Any combination of corner offsets in `0..4` is a valid shape. That is `5⁴ − 4⁴ = 369` normalised
  shapes. The shape is stored compactly (4 × 3 bits) and a shape index table is generated at startup.
- TER-013 MUST: The surface of a tile is split into two triangles for both rendering and physics. The split
  diagonal is chosen deterministically: the diagonal joining the two corners whose sum of offsets is the larger
  (ties: N–S diagonal). Rendering (§6.4) and physics (§15.4) MUST use the same rule.
- TER-014 MUST: Helper API (replacing `GetSlopeCornerHeights` users):
  `TileCornerHeights GetCornerHeights(const SurfaceElement&)`, `int32_t GetSurfaceZAt(CoordsXY worldPos)` (exact
  height under any point, using TER-013), `Vector3Fixed GetSurfaceNormalAt(CoordsXY)`.
- TER-015 MUST: Land tools (raise/lower corner, raise/lower tile, smooth, level) work in Q increments. A modifier
  key (or tool option "step: ¼ / ½ / ¾ / 1") sets the increment. Smoothing propagates so neighbouring corners
  never differ by more than 4 Q across a shared edge unless a cliff edge is drawn (§6.7).
- TER-016 MUST: All code that reads `getSlope()` with the old bit flags (paths, scenery placement, water,
  construction clearance, map generator, tile inspector) is migrated to the new model. A compatibility function
  `ToLegacySlope()` MAY exist during migration but MUST be removed at the end of Phase 1.
- TER-017 MUST: The map generator (`world/map_generator`) produces heightmaps at Q resolution.

### 6.3 Quarter-texture tiles
- TER-020 MUST: A tile's surface is divided by its two diagonals into **4 triangular quadrants**, each adjacent to
  one tile edge: NE, SE, SW, NW (see [tile_texture.png](tile_texture.png): each colour is one quadrant).
- TER-021 MUST: Each quadrant references its own `golf_terrain_surface` object (4 object indices per tile). A tile
  painted uniformly stores the same index 4 times.
- TER-022 MUST: Paint tool: brush sizes 1×1 to 9×9 tiles, plus a **quadrant brush** that paints only the quadrant
  under the cursor (hit test on the triangle under the mouse) and a **half-tile brush** (2 quadrants on one
  side of a diagonal).
- TER-023 MUST: The physics lie (§15.5) of a ball is the quadrant containing the ball's xy position.
- TER-024 MUST: Storage: the `SurfaceElement` layout is redefined (it MUST stay 16 bytes, `kTileElementSize`):
  base height (in base header), 4 corner offsets (12 bits), 4 quadrant surface indices (4 × 8 bits, so at most
  255 surface objects loaded), edge object index, water height, ownership, flags. If 16 bytes is insufficient,
  document the trade-off and move rarely-used data (e.g. ownership) to a side table rather than growing the
  element.

### 6.4 Terrain rendering (procedural, masked)
Pre-drawing every texture × 369 shapes × 4 quadrant combinations is impossible, so terrain sprites are generated.

- TER-030 MUST: Each `golf_terrain_surface` provides **one flat, top-down square texture** (recommended 64×64 px,
  power of two, tileable) plus an optional icon. No pre-rendered slope sprites are required.
- TER-031 MUST: At load time or lazily, a **terrain sprite generator** produces, for a given
  `(shape, quadrant textures[4], view rotation, zoom level)`, an isometric palettised sprite by:
  1. projecting the two surface triangles (TER-013) to screen space (2:1 isometric, same as OpenRCT2),
  2. sampling each quadrant's square texture with a fixed affine mapping (texture space = tile space, so a
     texture tiles seamlessly across tiles),
  3. clipping each texture to its quadrant with a **triangle mask** (the "sprite masking" of the source spec;
     the 4 masks are the same for every shape and are themselves projected),
  4. applying a light-level shade from the triangle normal (at least 5 shade bands, using palette shade ramps),
  5. quantising to the game palette.
- TER-032 MUST: Generated sprites are stored in an LRU cache keyed by the tuple above (budget configurable, default
  64 MB). Cache misses MUST not drop a frame below the TER-002 target when scrolling at 1× zoom (pre-warm the
  visible area; generate at most N sprites per frame and draw a flat-shaded fallback for the rest).
- TER-033 MUST: Zoomed-out levels sample from mip levels of the square texture (generate 2× and 4× reduced
  versions at load).
- TER-034 MUST: Grid lines, selection highlights and construction-rights overlays work for all 369 shapes (render
  them procedurally with the same projector).
- TER-035 SHOULD: Debug overlay showing the shape index and quadrant indices of the hovered tile.

### 6.5 Terrain surface object (`golf_terrain_surface`)
Required textures (initial content): **Fairway, Green, Rough, Deep rough, Sand (bunker), Woods, Rocks**
(the source list contains "Sand" twice; see §26 Q1), plus a neutral **Grass** (unmanaged land) and **Tee** surface.

```json
{
  "id": "golf.terrain.fairway",
  "objectType": "golf_terrain_surface",
  "version": "1.0",
  "authors": ["…"],
  "licence": "CC0-1.0",
  "strings": {
    "name": { "en-GB": "Fairway" },
    "description": { "en-GB": "Short-mown grass, ideal lie." }
  },
  "properties": {
    "roughness": 0.15,
    "bounciness": 0.45,
    "buildCost": 20,
    "removeCost": 5,
    "image": "images/fairway.png",
    "icon": "images/fairway_icon.png",
    "mapColour": "#3fa34d",
    "smoothing": { "withSelf": true, "withOthers": true, "priority": 40 },
    "lie": { "distanceFactor": 1.0, "accuracyFactor": 1.0 },
    "isHazard": false,
    "isPuttable": false
  }
}
```

- TER-040 MUST: Fields from the source spec: `roughness` (0..1), `bounciness` (0..1), name, description,
  `buildCost`, `removeCost` (currency units, `money64`), `image` (path in the zip).
- TER-041 MUST: Additional fields: `icon`, `mapColour`, `smoothing`, `lie` (shot modifiers, §15.5), `isHazard`
  (sand/water-like penalties and golfer unhappiness), `isPuttable` (only `true` for Green; putter used there).
- TER-042 MUST: Default values for the initial set (tunable later, all within 0..1):

| Surface | roughness | bounciness | distanceFactor | accuracyFactor | puttable | hazard |
|---|---|---|---|---|---|---|
| Green | 0.05 | 0.30 | 1.00 | 1.00 | yes | no |
| Tee | 0.10 | 0.40 | 1.00 | 1.00 | no | no |
| Fairway | 0.15 | 0.45 | 1.00 | 1.00 | no | no |
| Grass | 0.35 | 0.35 | 0.90 | 0.90 | no | no |
| Rough | 0.45 | 0.30 | 0.85 | 0.85 | no | no |
| Deep rough | 0.75 | 0.20 | 0.60 | 0.65 | no | no |
| Sand | 0.90 | 0.05 | 0.55 | 0.75 | no | yes |
| Woods | 0.80 | 0.25 | 0.50 | 0.50 | no | no |
| Rocks | 0.30 | 0.75 | 0.70 | 0.60 | no | no |

- TER-043 SHOULD: Woods surface spawns tree scenery when painted, via an option on the paint tool (trees are
  obstacles, §19).
- TER-044 MAY: Grass on fairway/green grows over time and requires mowing staff (reuse `GrassLength`); if
  implemented, longer grass increases roughness.

### 6.6 Texture smoothing
- TER-050 MUST: Where two different surfaces meet (between quadrants of the same tile and across tile edges),
  draw a smoothing transition as OpenRCT2 does (see `ViewportSurfaceSmoothenEdge` in
  [Paint.Surface.cpp](../src/openrct2/paint/tile_element/Paint.Surface.cpp)). The surface with the higher
  `smoothing.priority` bleeds over the lower one along the boundary.
- TER-051 MUST: The transition is generated procedurally: a dithered/noisy blend band (default 3 px at 1× zoom)
  along the shared triangle edge, using a fixed per-tile noise seed so it does not flicker.
- TER-052 MUST: Honour the existing "landscape smoothing" config toggle.

### 6.7 Terrain edges (cliffs)
- TER-060 MUST: When adjacent tiles' shared edge heights differ, draw vertical edge faces with the tile's
  `golf_terrain_edge` object texture, procedurally for any Q difference (same generator approach as §6.4).
- TER-061 MUST: Edge object: `image` (square tileable texture), `buildCost`, `removeCost`, name, description.

**Phase 1 AC**: (1) every one of the 369 shapes renders in all 4 rotations and 4 zoom levels without gaps between
neighbouring tiles; (2) a tile with 4 different quadrant textures matches tile_texture.png's layout; (3) smoothing
appears between quadrants and across tiles; (4) unit test: `GetSurfaceZAt` matches the rendered triangle split
for random points on all 369 shapes; (5) old saves are not required to load.

---

## 7. Water

- WAT-001 MUST: Lakes/ponds behave like OpenRCT2 water: per-tile water height, raise/lower water tool, animated
  water, transparency option. Water height uses Q increments.
- WAT-002 MUST: New `water` object assets (palette animation or animated texture) drawn for this project.
- WAT-003 MUST: Water is a hazard: a ball whose rest position or landing point is under water is lost (§15.6).
- WAT-004 SHOULD: Water edges (shoreline) render correctly against every terrain shape, including partially
  submerged sloped tiles (clip the water plane against the triangles).

---

## 8. Water streams

- STR-001 MUST: A stream is a separate tile element type `StreamElement` placed on top of a surface (any
  texture). A tile has at most one stream element.
- STR-002 MUST: A stream has a **flow direction set**: any subset of the 4 edges it connects to (straight,
  corner, T, cross, end/source). Auto-connect with neighbouring streams like footpaths do.
- STR-003 MUST: Streams follow the terrain slope; they can be placed on any of the 369 shapes. Flow direction for
  animation goes downhill; on flat tiles it follows placement order.
- STR-004 MUST: `stream` object JSON: name, description, `buildCost`, `removeCost`, `width` (xy units, default 12),
  `image` (top-down tileable animated strip: `frames`, `frameDurationTicks`), `sound` (ambient loop, §21),
  `isHazard: true`.
- STR-005 MUST: Rendering reuses the procedural projector (§6.4): the stream texture is masked to its connection
  pattern and projected onto the surface triangles.
- STR-006 MUST: Physics: a ball coming to rest in the stream band is a water hazard (§15.6). A rolling ball
  crossing a stream loses 80 % of its speed.
- STR-007 MUST: Golfers walk across streams only on paths/bridges; vehicles only on streets/bridges.

**AC**: a stream can be drawn along a sloped valley, connects automatically, animates in the downhill direction and
a ball dropped in it is declared lost.

---

## 9. Buildings

### 9.1 General
- BLD-001 MUST: A `BuildingElement` (or reuse of large-scenery elements with a building flag) represents a
  building occupying a rectangular footprint of tiles. Buildings require flat land (any shape allowed only if the
  object sets `allowSloped: true`, in which case foundations are drawn).
- BLD-002 MUST: `building` object JSON:

```json
"properties": {
  "kind": "golf_club",            // golf_club | cottage | food_truck | driving_range_kiosk | cart_rental | restroom | generic
  "footprint": [3, 2],            // tiles
  "height": 64,                   // z units, used for collisions and clearance
  "images": { "view0": "…png", "view1": "…png", "view2": "…png", "view3": "…png" },
  "entrance": { "tile": [1, 0], "edge": 3 },
  "collision": { … },             // §5.3, defaults to footprint × height box
  "buildCost": 25000,
  "removeCost": 5000,
  "upkeepPerMonth": 300,
  "services": ["food", "drink", "toilet", "pro_shop"],
  "capacity": 20,
  "sounds": { "ambient": "sounds/club_ambient.ogg" }
}
```
- BLD-003 MUST: 4 view images (one per camera rotation), each covering the full footprint (multi-tile sprite split
  for paint sorting the way large scenery is split per tile).
- BLD-004 MUST: Buildings are obstacles for balls (collision box) and block golfer/vehicle movement except at the
  entrance, which must connect to a pathway.
- BLD-005 MUST: Golfers can enter a building, spend time inside (hidden), use services, pay, and exit.

### 9.2 Building kinds
- BLD-010 **Golf club** MUST: services `food`, `drink`, `toilet`, `pro_shop` (buy equipment, §13.6). Prices per
  service set in the building window. Also the place where golfers check in and pay green fees (§22.2).
  At least one golf club is required before a course can open.
- BLD-011 **Food truck** MUST: services `food`, `drink`. Placed on a path or street edge (1×1 or 1×2), cheap,
  limited menu. MAY move between designated spots (later).
- BLD-012 **Golf cottage** MUST: see §22.4. Not placed by the player directly; spawns on cottage lots.
- BLD-013 **Driving range kiosk** MUST: see §17.
- BLD-014 **Cart rental / cart parking** MUST: see §14.
- BLD-015 SHOULD: restrooms as a stand-alone small building.

### 9.3 Building window
Shows name, kind, services and prices, revenue/upkeep this month and last, visitors, open/closed toggle,
demolish button.

**AC**: a 3×2 golf club placed in all 4 rotations draws correctly, blocks a ball fired at it (bounce), and a golfer
enters, buys a drink (money transferred) and exits.

---

## 10. Pathways and path additions

### 10.1 Path geometry
- PTH-001 MUST: A pathway segment is **half a tile wide** (16 xy units ≈ 2.3 m, source spec says "2 meters").
  Paths are laid on a **half-tile sub-grid**: each tile has 2×2 path cells. See §26 Q3 for the fallback if this
  proves too costly.
- PTH-002 MUST: Paths connect automatically to neighbouring cells (including across tile boundaries) and to
  building entrances, parking lots, tee boxes, greens, the driving range and mini-putt holes.
- PTH-003 MUST: Paths can be flat or sloped in the 4 levels of §6.2 (rise of 1–4 Q per tile length, i.e. per 2
  cells). Queue paths are not needed; a "tee waiting area" path variant MAY be added.
- PTH-004 MUST: `footpath_surface` / `footpath_railings` objects (existing types) get new art; images required per
  slope level. Use procedural projection for sloped surfaces where possible to limit sprite counts.
- PTH-005 MUST: Golfers on foot and golf carts use paths. Carts MAY also drive on fairway/rough if the course
  setting "carts allowed off path" is on (default off). Cars never use paths.
- PTH-006 MUST: Paths are not obstacles for balls; the ball lie on a path is "hardpan" (fixed roughness 0.1,
  bounciness 0.8).

### 10.2 Bridges/tunnels
- PTH-010 SHOULD: Elevated path segments (bridges) with supports, so paths can cross streams, water and streets.

### 10.3 Benches and trash bins
- PTH-020 MUST: Benches and bins are `path_addition` objects placed on a path cell edge.
- PTH-021 MUST: Benches: golfers with low energy sit (§13.4 "sitting" action), restoring energy. Max 2 golfers per
  bench.
- PTH-022 MUST: Bins: golfers drop litter in bins within N cells; otherwise they litter. Litter lowers nearby
  golfers' happiness. Bins fill up and need emptying by staff (MAY reuse handyman logic).
- PTH-023 SHOULD: Lamps, signs and ball washers as additional path additions.

---

## 11. 3D entity rendering

Golfers, cars, golf carts and animals are 3D models drawn in the isometric view.

- R3D-001 MUST: Model format: **glTF 2.0 binary (`.glb`)** with skinned meshes and named animation clips.
  Loader: a vendored single-header library (e.g. `cgltf`, MIT) under `src/thirdparty/`.
- R3D-002 MUST: Models are rasterised by a **software rasteriser** into an off-screen sprite (RGBA + depth), using
  the game's fixed isometric camera for the current view rotation (orthographic, same 2:1 dimetric projection as
  terrain), then quantised to the 8-bit palette with palette shade ramps and a hard outline option, so the result
  looks like OpenRCT2 sprites ("rendered result similar to OpenRCT2").
- R3D-003 MUST: Each model is limited to a small triangle budget (golfer ≤ 800, cart ≤ 600, car ≤ 1000,
  animal ≤ 600) and textures ≤ 128×128. The loader rejects models above 2× the budget.
- R3D-004 MUST: The rendered sprite is inserted into the paint session as a normal entity sprite with a bounding
  box derived from the model's AABB, so paint sorting and occlusion with terrain/buildings/scenery stay correct.
- R3D-005 MUST: An **imposter cache** keyed by `(model, animation clip, frame, yaw bucket, pitch/roll bucket,
  zoom level, colour remap)` avoids re-rasterising identical poses. Yaw: 32 buckets; pitch/roll: 8 buckets of 5°;
  animation frames sampled at a fixed rate (default 12 fps). Budget configurable (default 64 MB).
- R3D-006 MUST: Terrain orientation: cars, carts and animals align to the terrain: pitch and roll computed from
  `GetSurfaceZAt` sampled at the model's wheel/foot contact points (front/back/left/right), smoothed over a few
  ticks. Golfers stay upright but their feet are placed at surface height.
- R3D-007 MUST: Recolouring: models declare up to 3 remappable material slots (like OpenRCT2 primary/secondary/
  tertiary colours) so golfers have varied clothes and cars varied paint.
- R3D-008 MUST: A debug window shows a model with all its clips in all rotations ("model viewer").
- R3D-009 SHOULD: Rasterisation runs on a worker thread pool; the main thread only composites cached sprites.

**AC**: 300 animated golfers + 60 vehicles on screen meet TER-002; a cart driving across a slope tilts visibly; a
golfer behind a building is correctly hidden.

---

## 12. Streets, cars and parking lots

### 12.1 Streets
- STE-001 MUST: `StreetElement` tile element, placed tile by tile with auto-connection (like paths, full-tile grid).
- STE-002 MUST: Two kinds: **two-way road** (one lane each direction) and **one-way road** (direction stored per
  element, shown with arrows). Junctions: T and cross; corners; dead ends allowed only at map-edge spawns.
- STE-003 MUST: Streets support the 4 slope levels (§6.2). Sprites per slope level and connection pattern; use
  procedural projection of a top-down road texture + masks to keep the sprite count low.
- STE-004 MUST: `street` object JSON: name, description, `buildCost`, `removeCost`, `oneWay` (bool),
  `speedLimit` (km/h), images (top-down road texture, markings mask), `sounds` (MAY).
- STE-005 MUST: Car spawn points ("road entrances") sit at the map edge on a street tile; the player places them in
  the scenario editor. At least one is needed to receive car traffic.

### 12.2 Parking lots
- STE-010 MUST: A parking lot is a set of **parking space tiles** (2 spaces per tile, oriented) connected to a
  street on one side and to a path on another.
- STE-011 MUST: Capacity = number of spaces. When all spaces are full (counting reserved spaces of cars already
  heading there), **no more cars spawn**.
- STE-012 MUST: A golfer leaves their car at a space, walks to the nearest path, and walks back to the same car
  when leaving.

### 12.3 Cars
- STE-020 MUST: `Car` entity driven by a simple lane-following model on the street graph: A* to target, keeps lane,
  yields at junctions (first-come priority), keeps a minimum gap to the car ahead, slows on slopes and turns.
- STE-021 MUST: Cars spawn at a road entrance carrying 1–4 golfers (a group, §13.1), drive to a parking lot with a
  free space, park, and the golfers get out.
- STE-022 MUST: Leaving: the group walks back, gets in, the car drives to the nearest road entrance and despawns.
- STE-023 MUST: Cars are `vehicle_model` objects (§11) with `kind: "car"`, seats, speed, acceleration, wheel
  contact points, engine/horn sounds.
- STE-024 MUST: Cars are obstacles for balls (§5.3); a ball hitting a car makes a sound and lowers the owner's
  happiness (MAY).

**AC**: with a parking lot of 10 spaces, exactly 10 cars park, the 11th never spawns until one leaves; one-way
streets are respected.

---

## 13. Golfers

### 13.1 Population and spawning
- GLF-001 MUST: `Golfer` entity (new). Groups: golfers arrive and play in **groups of 1–4** (a group shares a car,
  a tee time and plays holes together). See §26 Q6.
- GLF-002 MUST: Two spawn methods:
  1. **Car**: at a road entrance, inside a car (§12.3).
  2. **Cottage**: VIP golfers who own a cottage spawn on foot at its door (§22.4).
- GLF-003 MUST: On spawn each golfer gets: name (from a `peep_names` object), money (random within a range set by
  park reputation and golfer type; VIP richer), skills (§13.3), needs at start values, preferred course
  difficulty, equipment level.
- GLF-004 MUST: A configurable max golfer count (default 500) and arrival rate driven by course popularity (§22.1),
  green fee price, time of day and weather.
- GLF-005 MUST: Day cycle: the course has opening hours (default 06:00–20:00 in game time); golfers stop arriving
  before closing and leave by closing.

### 13.2 Needs
- GLF-010 MUST: Needs, each 0..255 (same scale as OpenRCT2 peeps):
  **Hunger**, **Thirst**, **Energy**, **Happiness**, plus **Bladder** (needed because the golf club offers toilets,
  source spec "bladder").
- GLF-011 MUST: Needs drain over time; walking and swinging drain energy; drain rate is scaled by the
  **Physique** skill (high physique = slower drain).
- GLF-012 MUST: When a need crosses a threshold, the golfer seeks the nearest building offering it (golf club, food
  truck, restroom) between holes, never in the middle of a hole unless critical.
- GLF-013 MUST: Happiness modifiers (each a documented constant in one table):
  - + good shot, birdie/par, distance sign seen at the range, nice scenery near the hole (reuse scenery scoring
    idea), clean paths, short wait;
  - − waiting at a tee (per minute waited), bogey or worse, lost ball, ball in hazard, hole too hard for skill
    (§16.5), hunger/thirst/bladder high, litter, being hit by a ball, crowded paths, high prices.
- GLF-014 MUST: Thoughts (reuse OpenRCT2 thought system UI) with new thought types: "this hole is too hard",
  "waited too long", "great drive", "lost my ball", "I'm hungry", "prices too high", "need a cart", etc.

### 13.3 Skills
- GLF-020 MUST: Skills, each 0..100:
  - **Long play**: maximum shot distance factor.
  - **Accuracy**: spread of direction and distance error.
  - **Foolproof**: resistance of accuracy/long play to fatigue (low energy).
  - **Physique**: need drain rate (§13.2).
- GLF-021 MUST: Skill gain: driving range → Long play; mini putt → Accuracy; regular course → all four. Gain per
  activity unit (per ball at the range, per hole played) with diminishing returns near 100.
- GLF-022 MUST: Skill decay: each in-game day a golfer does not play, every skill decays by a small amount
  (configurable, default 0.2 / day), never below the golfer's initial value minus 10.
- GLF-023 MUST: Because skills persist between visits, the game keeps a **golfer registry** of returning golfers
  (bounded, default 2000 entries, least recently seen evicted). Returning golfers are drawn from it with a
  probability tied to their happiness on their last visit. See §26 Q5.

### 13.4 Actions and animations
- GLF-030 MUST: Animation clips required in `golfer_model`: `idle`, `walk`, `sit_bench`, `talk`, `swing_full`
  (driver/woods/irons), `swing_chip`, `putt`, `miniputt_putt`, `celebrate`, `angry`, `rage_walk`, `drive_cart`,
  `enter_vehicle`, `exit_vehicle`, `carry_bag_walk` (MAY).
- GLF-031 MUST: Actions from the source spec: sitting on a bench, talking to another golfer (two golfers face each
  other, `talk` clip, small happiness gain), striking a ball, striking with a mini-putt club.
- GLF-032 MUST: A golfer state machine (documented in code) with at least: `Arriving`, `WalkingToCheckIn`,
  `CheckingIn`, `GoingToCart`, `GoingToTee`, `WaitingForTeeTime`, `PreparingShot`, `Swinging`, `WatchingBall`,
  `WalkingToBall`, `DrivingToBall`, `SearchingBall`, `Putting`, `HoleFinished`, `UsingFacility`, `Resting`,
  `Talking`, `AtDrivingRange`, `AtMiniPutt`, `ReturningCart`, `Leaving`, `RageQuitting`.

### 13.5 Rage quit
- GLF-040 MUST: A golfer rage quits when happiness < a threshold (default 30/255) **and** at least one cause is a
  course problem (hole too hard, long waits, lost balls, hit by a ball).
- GLF-041 MUST: Rage quitting: the golfer abandons their ball and group, walks (`rage_walk`) straight to the exit
  via paths, displays insult thought/speech bubbles (from a localised list of non-offensive, comic insults:
  "Who designed this hole, a goat?!") to every golfer within 3 tiles, who each lose a little happiness.
- GLF-042 MUST: News/notification to the player: "<name> rage quit at hole 7".

### 13.6 Equipment
- GLF-050 SHOULD: Golfers have an equipment level (0..3) that adds a small bonus to Long play/Accuracy. They can buy
  better equipment at the golf club pro shop if they have money, raising revenue.

### 13.7 Golfer window
Reuse the guest window pattern: tabs for overview (3D view of the golfer), needs, skills, scorecard (strokes per hole
vs par), thoughts, inventory/money, history.

**AC**: a group arrives by car, checks in, plays 3 holes, gets hungry, eats at the golf club, finishes, and leaves;
skills and scores are visible; forcing a golfer's happiness low on a too-hard hole triggers a rage quit.

---

## 14. Golf carts

- CRT-001 MUST: Carts are `vehicle_model` objects with `kind: "golf_cart"`, 2 seats, max speed, contact points.
- CRT-002 MUST: The player places **cart parking spaces** (tiles or path-side spots). Carts are always rendered,
  parked or moving. The player buys the number of carts (cost per cart); carts are placed at parking spaces.
- CRT-003 MUST: Golfers who want a cart (depends on energy, physique, course length, money) walk to a parking space
  with a free cart and pay a **cart fee** (set by the player). If no cart is free, they walk (happiness penalty).
- CRT-004 MUST: A cart carries 1–2 golfers of the same group. Drives on paths (and off path if PTH-005 setting is on),
  stops near the ball, golfers walk the last metres.
- CRT-005 MUST: After the last hole the cart is returned to **any** free cart parking space (nearest).
- CRT-006 MUST: Carts tilt with terrain (R3D-006), are obstacles for balls and other vehicles/golfers avoid them.
- CRT-007 SHOULD: Carts need recharging/maintenance (MAY be later).

**AC**: with 2 carts and a group of 4, both carts are used; a later group finds none and walks; carts come back
to parking spaces after the round.

---

## 15. Golf ball and physics

The source spec implies ball physics (textures affect it, objects are obstacles); this section defines it.

### 15.1 Ball entity
- BAL-001 MUST: `Ball` entity with fixed-point position `(x, y, z)`, velocity, spin (scalar side spin + back/top spin),
  state (`Teed`, `InFlight`, `Bouncing`, `Rolling`, `AtRest`, `Holed`, `Lost`, `OutOfBounds`), owner golfer.
- BAL-002 MUST: Rendered as a small sprite (2×2 px at 1× zoom) with a shadow projected onto the surface; a
  short trail while in flight (toggle in options).

### 15.2 Clubs
- BAL-010 MUST: Club set (hard-coded table first, later a `club_set` object): Driver, 3-wood, 5-wood, 3–9 irons,
  pitching wedge, sand wedge, putter, mini-putt club. Each: loft angle, base carry distance (yards, for skill 50),
  max speed, spin, accuracy modifier.
- BAL-011 MUST: Base carry distances (skill 50, flat, no wind): Driver 230 yd, 3W 210, 5W 195, 3i 180, 4i 170,
  5i 160, 6i 150, 7i 140, 8i 130, 9i 120, PW 105, SW 80, putter = ground roll only.

### 15.3 Shot model
- BAL-020 MUST: The AI (§16.4) chooses club, target point and power (0..1). The shot is then computed:
  - distance = base carry × power × f(Long play) × lie.distanceFactor × fatigue factor;
  - direction error and distance error are drawn from a distribution whose spread shrinks with Accuracy and with
    lie.accuracyFactor, and grows when Energy is low unless Foolproof is high;
  - initial velocity vector from loft and required distance (solve analytically for drag-free flight, then the
    simulation applies drag, so tune constants so that real carry ≈ requested carry).
- BAL-021 MUST: f(Long play) ranges from 0.6 (skill 0) to 1.3 (skill 100), linear; documented constants.

### 15.4 Simulation
- BAL-030 MUST: Integrated at fixed sub-steps (default 4 per tick), fixed-point, deterministic (ARC-020).
- BAL-031 MUST: Forces: gravity, linear drag, simple Magnus effect from side spin (curve) and back spin (lift),
  **wind** if the weather system provides it (SHOULD; wind direction/strength shown in the UI).
- BAL-032 MUST: Terrain collision against the triangle mesh of §6.2 (TER-013), using the surface normal. On impact:
  - normal velocity × `bounciness` of the quadrant's surface (restitution),
  - tangential velocity × (1 − `roughness` × k_impact),
  - below a normal-velocity threshold the ball switches to `Rolling`.
- BAL-033 MUST: Rolling: gravity along the slope (balls roll downhill, including on greens), rolling resistance
  proportional to `roughness`; `AtRest` when speed < ε and slope force < static friction.
- BAL-034 MUST: Obstacle collision: buildings, scenery, walls/fishnets, vehicles, golfers (golfers: ball stops,
  golfer is "hit" → happiness penalty and a thought). Shapes from §5.3. Reflect velocity on the contact normal with
  the obstacle's restitution/friction; honour `passThroughChance` (deterministic RNG).
- BAL-035 MUST: A broad-phase grid of obstacles per tile (rebuilt incrementally on placement/removal) keeps
  collision queries O(local).
- BAL-036 MUST: Cup: if the ball passes over the cup position with speed below a capture threshold → `Holed`;
  above it, the ball lips out (small deflection).

### 15.5 Lie
- BAL-040 MUST: Lie = surface object of the quadrant under the ball (TER-023), or path/street ("hardpan"), or
  water/stream (hazard). The lie's `distanceFactor`/`accuracyFactor` apply to the next shot.

### 15.6 Penalties and rules (simplified stroke play)
- BAL-050 MUST: Water/stream: ball lost, +1 stroke, drop at the last point where the ball crossed the hazard
  margin (or at the previous shot position if that point is invalid).
- BAL-051 MUST: Out of bounds (off the course boundary set by the player, off-map, onto streets/buildings' roofs):
  +1 stroke, replay from previous position.
- BAL-052 MUST: Lost ball in woods/deep rough: chance to not find the ball (`SearchingBall` up to 1 game minute)
  → +1 stroke, drop.
- BAL-053 MUST: Max strokes per hole = par + 4 (golfer picks up, unhappiness).

### 15.7 Debug tools
- BAL-060 MUST: A debug "shot" tool: click a position, choose club/power/direction, fire a ball and see its
  trajectory and bounces; overlay shows lie, speed, state.

**AC**: deterministic replay of 1000 random shots gives identical end positions on two runs; a ball rolls downhill
on a sloped green; a ball fired at a building bounces back; sand stops a ball much faster than fairway; unit tests
cover restitution, rolling, cup capture and penalties.

---

## 16. Golf holes and courses

### 16.1 Hole construction
- HOL-001 MUST: Construction tool flow: (1) place the **men's tee**, (2) optionally the **women's tee**,
  (3) optionally the **child's tee**, (4) place the **cup** on a Green surface. Up to 3 tees; a hole needs at least
  one tee and one cup.
- HOL-002 MUST: A tee box is a 1×1 (MAY be 2×1) area that forces/creates the Tee surface; it renders tee markers in
  a colour per category (configurable).
- HOL-003 MUST: The cup must be on a puttable surface; the flag pin is an entity/scenery drawn above the cup and is
  a (thin cylinder) obstacle.
- HOL-004 MUST: Holes are numbered in creation order and can be reordered in the course window.
- HOL-005 MUST: Length is measured along the **playing line**: the player MAY place 0–2 **aim points** (doglegs);
  length = sum of straight segments tee→aim points→cup, in yards (1 tile = 5 yd, so 400 yd = 80 tiles).
- HOL-006 MUST: Par is computed from the men's tee length (overridable): par 3 ≤ 250 yd, par 4 251–470 yd,
  par 5 > 470 yd (constants in one table, separate tables for women/children).
- HOL-007 MUST: The player draws the **course boundary** (out-of-bounds area) with a zone tool; defaults to all owned
  land.

### 16.2 Course
- HOL-010 MUST: A course is an ordered list of 1–18 holes with a name, green fee, open/closed status. A map can host
  several courses (e.g. a 9-hole and an 18-hole) plus driving ranges and mini-putt courses.
- HOL-011 MUST: Golfers always begin on hole 1 and play in order.
- HOL-012 MUST: Course window: hole list (number, par, length per tee, difficulty, average score, average wait),
  scheduling tab (§16.3), statistics tab (rounds played, revenue, satisfaction), and "test hole" button that fires
  AI shots with average skill to estimate difficulty.

### 16.3 Scheduling
- HOL-020 MUST: A **hole scheduling window** lists every hole with its time slot length (minutes of game time
  between two groups teeing off). Per hole, mode is **Auto** or **Manual**.
- HOL-021 MUST: Auto: estimated time = walk/drive time along the playing line + time per expected stroke ×
  (par + expected over-par for average skill) + green time; recomputed when the hole changes and periodically from
  measured averages.
- HOL-022 MUST: Tee times: groups get a tee time on hole 1 when checking in; their next holes are scheduled from the
  slot lengths. A group arriving early waits at the tee (waiting area = path cells near the tee).
- HOL-023 MUST: Waiting golfers lose happiness per minute waited (GLF-013), shown in hole statistics.
- HOL-024 SHOULD: If a group is too slow, the next group catches up and waits; the window highlights bottleneck
  holes in red.

### 16.4 Golfer play AI
- HOL-030 MUST: For each shot: compute candidate targets (along the playing line, fairway centres, green), evaluate
  expected outcome using the golfer's skills and the lie of the landing zone (prefer fairway/green, avoid hazards
  weighted by risk tolerance), choose club and power, then shoot (§15.3).
- HOL-031 MUST: Putting: on a Green, aim with slope compensation proportional to Accuracy (reads the break);
  power chosen to reach the cup plus a small margin.
- HOL-032 MUST: AI cost must stay < 0.5 ms per decision; decisions happen only when a golfer prepares a shot.

### 16.5 Difficulty
- HOL-040 MUST: Each hole has a difficulty rating (0..10) from length vs par, hazard density near the landing
  zones, green slope, obstacles on the playing line, elevation change. Shown in the course window.
- HOL-041 MUST: If difficulty − golfer's average skill-derived level > threshold, the golfer gets the "too hard"
  thought and loses happiness (source spec: "If the hole is too hard, the player's happiness should get lower").

**AC**: a 400-yard hole (80 tiles) shows par 4; the scheduling window in Auto mode yields a plausible slot (e.g.
10–15 game minutes); two groups do not tee off simultaneously; a too-hard hole lowers happiness.

---

## 17. Driving range

- DRV-001 MUST: A driving range is created by drawing its **area** (zone tool). Inside, the player designates
  **hitting tiles** (where golfers stand and shoot) and places one **driving range kiosk** building (§9.2).
- DRV-002 MUST: The game warns (non-blocking) when the kiosk is not behind the hitting line relative to the
  shooting direction ("kiosk may be hit by balls").
- DRV-003 MUST: **Fishnets** are wall-type objects (`walls` with `collision`, high `height`, low restitution) that
  can be placed on tile edges to stop stray balls.
- DRV-004 MUST: **Distance signs** placed inside the area automatically show the distance from the kiosk to the sign
  (yards or metres, integer, rounded to 5), updated when kiosk or sign moves.
- DRV-005 MUST: Golfers buy a bucket of balls at the kiosk (price set by player), walk to a free hitting tile and hit
  N balls (default 30). Each ball landing within 10 yd of a sign gives a small happiness gain ("I know my distance").
- DRV-006 MUST: Each ball hit improves Long play (GLF-021). Balls landing outside the area: if they hit a golfer or
  a car, penalties apply (§15.4).
- DRV-007 SHOULD: Ball picker staff/vehicle collects balls (cosmetic; MAY be skipped).

**AC**: golfers use a range with 5 hitting tiles; signs show correct distances; Long play increases.

---

## 18. Mini putt course

- MPT-001 MUST: Built like a regular course (§16) with holes of tee + cup, but the only club allowed is the
  **mini-putt club**; tees and cups placed on a dedicated **Mini-putt carpet** surface (add a terrain surface object
  with roughness 0.03, bounciness 0.6) or any puttable surface.
- MPT-002 MUST: Mini-putt obstacles: dedicated small scenery (bumpers, ramps, windmill MAY) with collision; low walls
  (`walls` objects) as borders.
- MPT-003 MUST: Playing a mini-putt hole improves Accuracy only. Regular courses improve all skills (GLF-021).
- MPT-004 MUST: Separate entry fee and scheduling (Auto by default).

**AC**: a 9-hole mini putt with walls is playable; balls bounce off walls; Accuracy increases.

---

## 19. Placeable scenery

- SCN-001 MUST: Reuse OpenRCT2 small/large scenery and walls placement (rotation, quadrants, height adjustment,
  scenery groups/tabs, clearance), with new art.
- SCN-002 MUST: Every scenery object has a `collision` block (§5.3) and a `height`. Balls hitting scenery bounce back
  (BAL-034). Trees use `cylinder` trunk + foliage with `passThroughChance`.
- SCN-003 MUST: Scenery near holes contributes to golfer happiness ("beautiful hole") and to course popularity.
- SCN-004 MUST: Scenery placement supports the new slope shapes (objects anchored at `GetSurfaceZAt` of their
  anchor point; large scenery requires its footprint to be within 1 Q of flat unless it allows slopes).

---

## 20. Animals

- ANM-001 MUST: `animal` objects: 3D model (§11) with clips `idle`, `walk`, `run`, `flee`, optional `fly`/`swim`;
  habitat (allowed surfaces: e.g. ducks → water and green banks; deer → woods/rough; squirrels → woods), group size,
  spawn rate, sounds.
- ANM-002 MUST: Animals spawn in suitable habitat, wander, flee golfers/carts within a radius, avoid streets (or cross
  carefully).
- ANM-003 MUST: Animals are obstacles for balls (small cylinder); being hit makes them flee and play a sound.
- ANM-004 SHOULD: A comic event: a bird/squirrel steals a ball at rest on rough (rare, +1 stroke, golfer thought).
- ANM-005 MUST: Animals raise happiness slightly for golfers who see them (within 5 tiles), capped per hole.

---

## 21. 3D spatial audio

- AUD-001 MUST: Every sound emitter (golfers, carts, cars, animals, streams, buildings, ball impacts, swings) plays
  sounds from its own object package (`sounds` block in JSON: `id → file`, plus `volume`, `loop`, `minDistance`,
  `maxDistance`, `rolloff`).
- AUD-002 MUST: Spatialisation: the listener is at the viewport centre projected onto the terrain, at a distance
  depending on zoom. For each emitter compute distance attenuation (inverse-distance clamped between min/max) and
  stereo pan from the screen-space horizontal offset; emitters behind the camera plane are attenuated.
- AUD-003 MUST: Extend the existing mixer ([Audio.h](../src/openrct2/audio/Audio.h): `pan`, `volume`) rather than
  adding a new audio library, unless profiling shows a need (then OpenAL Soft MAY be considered).
- AUD-004 MUST: Voice limit (default 32 simultaneous), priority by loudness after attenuation; looping sounds
  (streams, engines) fade in/out when entering/leaving range.
- AUD-005 MUST: Separate volume sliders: master, ambience (nature, streams, animals), vehicles, golfers, UI, music.
- AUD-006 MUST: Required placeholder sounds: driver swing, iron swing, putt, ball impact (grass/sand/hard/water
  splash/tree/building), cup drop, cart motor loop, car engine loop + door, crowd clap, golfer cheer/groan/insult
  grumble, bird chirp, duck, stream loop.

**AC**: scrolling the view past a stream makes its sound pan left→right and fade; zooming out lowers detail
sounds.

---

## 22. Economy and game goals

### 22.1 Popularity
- ECO-001 MUST: Course popularity (0..1000) from: average golfer happiness, hole quality/difficulty balance,
  scenery, facilities, price fairness, rage quits (strong negative), tournaments hosted (positive). Drives golfer
  arrival rate and cottage land sales.

### 22.2 Income and costs
- ECO-010 MUST: Income: green fees (per course, per round), cart fees, driving range buckets, mini putt fees,
  building services (food, drink, toilet, pro shop), cottage land sales (§22.4), tournament revenue.
- ECO-011 MUST: Costs: construction (terrain paint/shape per `buildCost`, removal per `removeCost`), building
  upkeep, staff wages (greenkeepers, handymen, ball pickers), cart purchase, loan interest, marketing (reuse).
- ECO-012 MUST: Reuse OpenRCT2 finance window and history graphs, renamed categories.

### 22.3 Loans
- ECO-020 MUST: Reuse OpenRCT2 loans (max loan, interest rate, increase/repay).

### 22.4 Golf cottages
- ECO-030 MUST: The player designates **cottage lots** (zone tool, each lot a rectangle sized for a cottage
  footprint, adjacent to a path or street).
- ECO-031 MUST: When popularity exceeds a threshold (scaled by lot attractiveness: view of a hole, water, scenery),
  a VIP golfer **buys the lot**: the player receives the land price (based on popularity and lot quality), a
  cottage building spawns (random cottage object of the loaded set), and ownership of the lot moves to the VIP.
- ECO-032 MUST: The VIP lives there: spawns on foot from the cottage (GLF-002), plays more often, pays reduced or no
  green fee (configurable), and is richer.
- ECO-033 MUST: Sold lots can no longer be modified by the player (terrain, scenery) except cosmetic paths.

### 22.5 Tournaments
- ECO-040 MUST: A tournament can be hosted when requirements are met (shown in a Tournament window with a
  checklist): e.g. an 18-hole course, total length ≥ 6000 yd, par 70–72, popularity ≥ threshold, at least one golf
  club, parking capacity ≥ N, and **reserved tiles** for tournament infrastructure.
- ECO-041 MUST: Reserved infrastructure zones the player must designate (zone tool): spectator area, media tent,
  extra parking, grandstand near the 18th green. Their minimum size is part of the requirements.
- ECO-042 MUST: Hosting a tournament: schedule a date; on that day the course is closed to regular golfers,
  professional golfers (high skills) play, spectators arrive (extra cars/peeps), revenue from tickets and
  sponsors, popularity bonus based on the result (players' happiness, no bottlenecks).
- ECO-043 SHOULD: Tournament tiers (local, regional, national) with increasing requirements and rewards.

### 22.6 Scenario objectives
- ECO-050 MUST: Scenario editor supports objectives: reach profit X by date, reach popularity Y, host a tournament
  of tier Z, have N golfers per day. Reuse OpenRCT2 scenario objective framework.

---

## 23. Save format and scenario editor

- SAV-001 MUST: New save file format (versioned, compressed) storing map with new elements, objects list, golfers
  (including registry), vehicles, animals, balls, courses/holes/schedules, zones (cottage lots, tournament zones,
  driving ranges, course boundaries), finances. Based on OpenRCT2's `.park` chunked format with a new magic/
  extension (`.golf`, see §26).
- SAV-002 MUST: Loading a save with a missing object reports it and replaces it with a placeholder object.
- SAV-003 MUST: Scenario editor: map size, terrain tools, objects selection, road entrances, starting money/loan,
  objectives.

---

## 24. User interface

New or adapted windows/tools (reuse `openrct2-ui` window framework):

| Window / tool | Notes |
|---|---|
| Land tool (adapted) | Q-step increments, slope level selector (§6.2) |
| Land paint tool | Texture picker, brush size, quadrant & half-tile brush (§6.3) |
| Water / stream tools | §7, §8 |
| Path tool (adapted) | Half-tile grid, slopes (§10) |
| Street tool | One/two-way, road entrance placement, parking lots (§12) |
| Building placement | By kind (§9) |
| Scenery (adapted) | Collision preview toggle (§19) |
| Hole construction | Tees, cup, aim points, preview playing line with length/par (§16.1) |
| Course window | Holes, stats, test hole (§16.2) |
| Hole scheduling window | Auto/manual per hole (§16.3) |
| Driving range tool | Area, hitting tiles, signs, fishnets (§17) |
| Mini-putt tool | §18 |
| Cart parking tool | §14 |
| Zone tool | Cottage lots, tournament zones, course boundary (§22) |
| Golfer window / golfer list | §13.7 |
| Building window | §9.3 |
| Tournament window | §22.5 |
| Finance window (adapted) | §22.2 |
| Debug: shot tool, model viewer, terrain shape overlay | §15.7, R3D-008, TER-035 |

- UI-001 MUST: Overlays: hole playing line, ball trajectories (last N shots), heat map of landing zones, lie map
  (colour per surface), wait-time map.
- UI-002 SHOULD: Plugin API (`scripting/`) exposes golfers, holes, balls and the new tile elements so tests and
  tools can be scripted.

---

## 25. Phasing and milestones

| Phase | Content | Exit criteria |
|---|---|---|
| P0 | Fork setup on open-golf-simulator (ARC-001 to ARC-001c), strip rides/importers (ARC-002), asset independence (§4), placeholder generator | §4 AC; build/CI green; app boots with no RCT files |
| P1 | Height model, 369 shapes, quarter textures, procedural terrain renderer, smoothing, edges, water adapted (§6, §7) | §6 Phase 1 AC |
| P2 | Object types: terrain surface/edge, stream, building, path (half-tile), scenery collision (§5, §8, §9.1, §10, §19) | §8, §9, §10 AC |
| P3 | 3D model loader, software rasteriser, imposter cache, terrain-aligned orientation (§11) | §11 AC with test models |
| P4 | Ball physics + debug shot tool (§15) | §15 AC |
| P5 | Holes, courses, scheduling, golfer entity basics: spawn on foot, play holes, needs, skills, AI (§13, §16) | §13, §16 AC |
| P6 | Streets, cars, parking, golf carts (§12, §14) | §12, §14 AC |
| P7 | Driving range, mini putt (§17, §18) | §17, §18 AC |
| P8 | Economy: popularity, fees, loans, cottages, tournaments, scenario objectives, save format (§22, §23) | §22 AC; save/load round-trip test |
| P9 | Animals, 3D spatial audio, polish (§20, §21) | §20, §21 AC; TER-002 perf target met |

P3 and P4 are independent of each other and MAY run in parallel after P2.

---

## 26. Open questions (with default answers)

| # | Question | Default until answered |
|---|---|---|
| Q1 | The source texture list has "Sand" twice. Was the second meant to be **Green** (putting surface)? | Yes: add Green. Tee and Grass are added too. |
| Q2 | "Quarter / half / ¾ / full step": is a full step the current OpenRCT2 steep slope rise (32 z units)? | Yes: 1 Q = 8 z units (§6.2). |
| Q3 | Pathways "half the size of a tile": half-width paths on a half-tile sub-grid, or full-tile paths drawn half-wide? | Half-tile sub-grid (PTH-001). Fallback if too costly: full-tile grid, path drawn half width on one side or centred. |
| Q4 | Which 3D model format? | glTF 2.0 `.glb` (R3D-001). |
| Q5 | Skills improve over visits: should golfers be persistent (returning members)? | Yes, via a bounded golfer registry (GLF-023). |
| Q6 | Do golfers play alone or in groups? | Groups of 1–4 (GLF-001). |
| Q7 | Game/executable name and file extensions | Name and executable `open-golf-simulator` (from the fork); objects keep `.parkobj`; saves use `.golf`. |
| Q8 | Wind? | SHOULD, driven by weather (BAL-031). |
| Q9 | Staff (greenkeepers mowing, handymen, ball pickers)? Not in the source spec. | Minimal: handymen for bins (PTH-022); mowing MAY (TER-044). |
| Q10 | Can carts drive on fairways? | Setting, default off (PTH-005). |
| Q11 | Licence of new assets | CC0 or CC-BY-SA, declared per object (OBJ-003). The game code stays GPLv3. |
