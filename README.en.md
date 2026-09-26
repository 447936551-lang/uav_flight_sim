# uav_flight_sim

A **pure algorithm layer** for drone flight simulation — flight control, steering-based
collision avoidance, rigid-body dynamics, path planning, and environmental perception,
with **zero proprietary dependencies** (no `@kit.AREngine` / `ArkGraphics3D` / `hilog` /
`MindSporeLiteKit`).

> This repository is the algorithm subset extracted from the author's own OpenHarmony
> drone-control app (project name **AR Drone**) and incubated into the
> **OpenHarmony UAV SIG**: the independently verifiable "decision / physics" logic is
> peeled away from the rendering layer so the same code can be reused across **CI / offline
> simulation / real hardware**.

**Source & trademark notice**: AR Drone is the author's own project name and is not
associated with any third-party vendor or its product trademarks. This repository does not
depend on or reference any third-party proprietary SDK or trademarked capability. For the full
provenance map, see [`docs/extraction-map.md`](./docs/extraction-map.md).

---

## Why this repository exists

The original app's flight logic was tightly coupled to AR rendering and depended on
OpenHarmony proprietary suites, making it impossible to compile or unit-test on a stock
developer machine or in CI. When incubating into the UAV SIG, the community cares about the
**algorithm itself** (flight feel, avoidance strategy, state machine) rather than AR rendering.

Therefore this repository keeps only the pure logic, and uses an abstract interface
`SpatialPerception` ("spatial perception input" contract) as the **single boundary** between
the algorithm and its data source:

```
   AR engine DepthSampler  ┐
   offline simulator       ├─ implement SpatialPerception ─▶ DroneController (reused unchanged)
   real sensors            ┘
```

## Directory structure

```
src/
  core/                 zero-dependency infrastructure
    Vec3.ts             self-implemented 3D vector (replaces ArkGraphics3D.Vec3)
    Logger.ts           self-implemented logger (replaces hilog)
    LoggerLevel.ts      log levels
  model/                flight dynamics state machine (kinematic kernel)
    DroneController.ts    flight-control state machine + velocity integration + attitude derivation
    WorldObject.ts        AR world-object base (anchor + offset pose)
  avoidance/            collision assessment + steering synthesis
    CollisionDetector.ts   collision / threat assessment (pure functions)
    SteeringBehavior.ts    Seek/Avoid steering + velocity-domain avoidance
  dynamics/             real rigid-body dynamics (added in v0.2.0)
    RotorMixer.ts          quadrotor mixing + actuator saturation + first-order motor lag
    FlightDynamics.ts      rigid-body integrator (semi-implicit Euler)
    CascadeController.ts   cascade control: position / velocity / attitude loops (pure functions)
  planning/             occupancy grid and path planning (added in v0.2.0)
    PathPlanner.ts         world plane → 2.5D grid → A* → visibility smoothing
    MeshGrid.ts            scene mesh → occupancy grid (alternative data source)
  environment/          atmosphere conversion and weather normalization (added in v0.2.0)
    Atmosphere.ts          wind vector / air density / low-pass smoothing / gusts
    WeatherCode.ts         wind-speed unit conversion / anomaly guards / WMO codes
  telemetry/            externally observable state derivation
    HudModel.ts          HUD threat level / colors / copy (pure functions)
  contract/             input contracts (pure interfaces)
    SpatialPerception.ts      spatial perception input (geometry: forward obstacle)
    EnvironmentPerception.ts  environment perception input (atmosphere: wind / density)
  index.ts              unified barrel export
tests/                 vitest unit tests + zero-dependency self-tests + golden simulation cases
sim/
  sim_flight.ts         headless flight-sim demo (with MockPerception)
docs/
  design.md             design notes and acceptance criteria
  extraction-map.md     source-to-module provenance map
CHANGELOG.md            version notes (added / changed / verified / not included)
```

Dependencies are strictly **unidirectional** bottom-up:

```
core ──▶ model / avoidance / dynamics / planning / environment ──▶ telemetry
                          ▲
                      contract (pure interfaces)
```

`core` has zero dependencies; `model` / `avoidance` / `dynamics` depend only on `core`;
`planning` depends on `core` + `avoidance`; `contract` is pure interfaces (only depends on
`core`'s `Vec3` type). **Upper layers must not reverse-depend on lower-layer implementations.**

## Quick start

```bash
npm install        # install typescript / vitest / tsx
npm run typecheck  # tsc type check (no output = pass)
npm test           # run vitest unit tests
npm run sim        # run headless flight-sim demo
npm run verify     # one-shot: typecheck + tests + sim
```

> Verify without a framework: `npm run selfcheck` runs the zero-dependency self-tests
> (calls each module's built-in `selfTest()` directly).

## Core contract: SpatialPerception

```ts
import { SpatialPerception, Vec3 } from 'uav_flight_sim';

class MyDepthSampler implements SpatialPerception {
  getObstacleDistance(): number {
    // return forward obstacle clearance in meters; Infinity = no obstacle / depth unavailable
    return this.lastDepthGap;
  }
  getObstacleNormal?(): Vec3 | null {
    // optional: obstacle-away unit vector, used as lateral reference
    return this.awayNormal;
  }
}

const drone = new DroneController();
drone.perception = new MyDepthSampler(); // takes effect immediately; flight code unchanged
```

- **Pure-algorithm mode (CI / offline)**: inject deterministic data with `MockPerception`;
  run avoidance and unit tests with no hardware.
- **Joint-simulation mode**: physics handled by the simulator; this repo owns only "decision / contract".
- **Real-hardware / AR mode**: the original app's `DepthSampler` implements this interface,
  feeding AREngine depth directly.

### Second contract: EnvironmentPerception

A second instance of the same idea — the former answers "how far is the obstacle ahead"
(**geometry**), the latter answers "what is the surrounding air doing" (**atmosphere**):

```ts
import { EnvironmentPerception, Vec3 } from 'uav_flight_sim';

class MyWindSource implements EnvironmentPerception {
  getWindVelocity(): Vec3 | null {
    // wind velocity vector (m/s, world frame); null = no wind / not connected
    return { x: this.windX, y: 0, z: this.windZ };
  }
  getAirDensity?(): number {
    // relative air-density factor (standard sea level = 1.0)
    return this.rho;
  }
}
```

⚠️ **Network fetching is not included.** Real-time weather comes from an HTTP interface
(latency, failures, vendor-specific units), which would break the "CI-offline, deterministic"
positioning if baked into the algorithm library. Therefore this repo keeps only the **pure
conversions** (`environment/Atmosphere.ts` / `WeatherCode.ts`); fetching and refresh stay on
the app side.

When not connected / perception is off, return `null` or a zero vector — the algorithm then
degrades to no-wind flight, **bit-for-bit identical** to the original behavior (zero regression,
pinned by unit tests).

## Acceptance criteria (single source of truth, aligned with hardware)

| Threshold | Value | Meaning |
| --- | --- | --- |
| `AVOID_MARGIN_M` | 0.4 m | hard-stop plane: forward component zeroed if closer (measured stop gap ≈ 0.38 m) |
| `AVOID_SLOW_BAND_M` | 1.2 m | slowdown band width: forward component converges linearly outward from here |
| `AVOID_SAFE_DIST_M` | 1.6 m | avoidance activation distance = hard-stop + slowdown band |
| `DRONE_RADIUS_M` | 0.19 m | body bounding-sphere radius |
| `AVOID_STEER_GAIN` | 1.25 | lateral bypass gain (closer ⇒ more lateral wall-following) |

Avoidance strength and forward scaling are **from the same source**: `fwdScale = (gap − margin) / band`,
`avoidStrength = 1 − fwdScale`. So the panel reading and the actual braking force always agree —
no "shows danger but doesn't brake" mismatch.

**Planned bypass** (`bypassMode`, default `false`): when the user pushes straight into an obstacle
with **no lateral input at all**, actively add a lateral velocity to go around instead of stopping
in place. It stacks with — not in conflict with — `steerGain`: the former only兜底 (fallback) when
there is zero lateral input, the latter only amplifies existing lateral input. Default off ⇒ zero regression.

**Blind is not safe**: `obstacleDistance === Infinity` has two opposite causes — truly no obstacle
ahead, or this frame's collision-avoidance is simply **blind**. `deriveHud()` therefore requires
`depthTrustworthy` (no default — a default would silently hide "blind"). When untrustworthy it reports
`Blind` (distance shown as `—`, deliberately not green). Otherwise "avoidance failed" would be rendered
as "∞ safe" and the user would never notice.

## Milestones

| Phase | Deliverable | Status |
| --- | --- | --- |
| M1 repo & conventions | standalone repo / Apache-2.0 / OWNERS / DCO / CI | ✅ done |
| M2 capability extraction & refactor | `core`/`model`/`avoidance`/`telemetry`/`contract` five modules + unit tests + golden sim cases in CI | ✅ done |
| M2+ second batch | `dynamics` / `planning` / `environment` three modules + `EnvironmentPerception` contract | ✅ done (v0.2.0) |
| M3 simulation plan & docs | scripted simulation plan, scenario examples, integration guide (incl. Simulator SIG onboarding) | ⏳ todo |
| M4 graduation prep | architecture SIG graduation review materials, QA SIG exit materials | ⏳ todo |

> v0.2.0 capabilities (rigid-body dynamics, path planning) exceed the original proposal scope
> (M1–M4 did not include them). They will be reported as additional deliverables when contributing
> upstream. See `CHANGELOG.md`.

## Support this project

If this repository helps your project, consider buying the author a coffee ☕:

- [GitHub Sponsors](https://github.com/sponsors/447936551-lang)
- [Afdian (爱发电)](https://afdian.com/a/uavflightsim) — recommended for users in China

Sponsorship only supports the open-source work itself; it does not influence the technical
direction or community governance of this project (Apache-2.0 / DCO apply as usual).

## License & compliance

- **Apache-2.0** (consistent with the OpenHarmony community); every source header carries copyright and license.
- **DCO** signed (Developer Certificate of Origin).
- This repository contains **no** private signatures / keys / assets from the original app and is safe to open-source.

## Relationship with the OpenHarmony UAV SIG

- Upstream portal repo: `openharmony-robot/uav`
- This repo is built first as a personal incubation repo, then contributed upstream per the SIG process.
- This repo owns "decision / physics / contract"; rendering, AR anchoring, and device adaptation are owned
  by the app / SIG main repo.
