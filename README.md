# LoRa mesh flooding in urban street canyons (ns-3)

> **Note:** this is the simulation code for a paper I was writing on LoRa mesh networks in dense cities. I've stopped working on the paper, so the code is here as a record of that work rather than a maintained project.

The simulation asks whether a flooding LoRa mesh holds together in a dense city once you model how radio actually travels down streets. Nodes sit on the real street network of Singapore's CBD (from OpenStreetMap, in `scratch/singapore_streets.h`), and each run floods messages through the mesh and measures delivery as node density goes from 50 to 1,000.

It compares two propagation models at 915 MHz (SX1262, SF10, 125 kHz):

- **Waveguide**: a dual-slope street-canyon model after ITU-R P.1411. Links along a street get the canyon's gentler first slope up to a breakpoint distance (366 m by default, calibrated for the Singapore CBD). Links that cut across blocks lose more for each building in the way.
- **Standard**: a single log-distance model with path-loss exponent n = 4.0, the usual simplification.

Results go to timestamped CSVs: raw per-run data and per-density summaries with 95% confidence intervals for delivery ratio, hop count, and collision and busy drops.

## Requirements

- [ns-3.41](https://www.nsnam.org/releases/ns-3-41/)
- The [`lorawan`](https://github.com/signetlabdei/lorawan) module in `contrib/lorawan`

## Running it

Replace the `scratch/` folder in your ns-3.41 tree with this repo's `scratch/`, then from the ns-3.41 root:

```sh
./ns3 configure && ./ns3 build
bash scratch/run_test.sh                 # validation sweep, about 20-25 minutes
caffeinate -i bash scratch/run_main.sh   # full sweep, about 6-7 hours on an M4 MacBook Air
```

`run_main.sh` starts with a short timing test at N = 1,000 and stops if it is too slow (see the comments in the script). Then it runs both models, a burst scenario with a 5-second wave interval, and a sweep of the breakpoint distance (27 m and 87 m). Output goes to `Results/`.

To run a single configuration:

```sh
./ns3 run "scratch/waveguide-mesh --waveguide=1 --grid=1 --runs=5 --waves=10 --outdir=waveguide"
```

| Flag | Meaning |
|---|---|
| `--waveguide` | `1` for the street-canyon model, `0` for standard n = 4.0 |
| `--grid` | `1` places nodes along streets, `0` uniformly at random |
| `--runs` | Monte Carlo iterations per density point |
| `--waves` | flooding waves per iteration |
| `--wave_ivl` | seconds between waves (default 900) |
| `--dbp` | breakpoint distance in metres (default 366) |
| `--preset` | `full` (20-point density sweep) or `burst` (12-point) |
| `--outdir` | subfolder of `Results/` to write to |

Two `lorawan` APIs vary between versions: `SetReceiveOkCallback` and the `ReceivedPacket` trace signature. The run scripts check for both before building and say what to change if either is missing.
