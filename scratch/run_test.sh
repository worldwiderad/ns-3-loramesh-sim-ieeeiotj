#!/bin/bash
# =============================================================
#  VALIDATION RUN v4 — M4 MacBook Air
#  Full density sweep: N = 50, 100, 150, 200, 300, 500, 750, 1000
#  runs=5, waves=10 — both models sequentially
#  Expected runtime: ~19-25 min
#
#  Purpose: verify the code compiles and produces physically
#  plausible results before committing to the 6-7h production run.
#  Check the output for:
#    - WG model: percolation at N=150-200 (meanHops 3-10)
#    - Standard model: no percolation until N=750-1000 (or 0% throughout)
#    - busyDrops: should be 0 or near-0 at low N
#    - No NS_FATAL_ERROR or NS_ABORT in log
#
#  Changes since v2 (code changes applied since last validation run):
#    - SimState::busyDrops field added (compilation fix)
#    - ExecuteRun: totalN/sinkId declared, dbpM parameter added
#    - PrintBanner: takes dbpM as 4th parameter
#    - NullRxOkCb: 5-arg no-op replaces MakeNullCallback (NS_FATAL guard)
#    - SetReceiveOkCallback loop: API-verified or delete-if-absent (see below)
#    - m_edPhy cached in Setup() — DynamicCast validated once, not per-send
#    - CalculatePathLoss: Manhattan→Euclidean building count
#    - Length-weighted street CDF sampling (lower_bound on segCumLen)
#    - d<1.0 returns PL_D0_DB (was 0.0 — 32dB discontinuity)
#    - --dbp CLI flag for sensitivity analysis (default 366m)
#    - DBP_m column added to both raw and summary CSVs
#    - T95[120] t-table at file scope (static constexpr)
#    - sH sentinel = -1.0 (NaN reverted for Excel/R compat)
#    - CI bounds clamped: max(mPDR-ci,0) / min(mPDR+ci,100)
#    - raw.fail() / sum.fail() checked after every flush
#    - PIPESTATUS check after each ./ns3 run
#
#  BEFORE FIRST BUILD — run these two greps:
#    grep -r 'SetReceiveOkCallback' <ns3_root>/contrib/lorawan/model/lora-phy.h
#    grep -A3 'ReceivedPacket' <ns3_root>/contrib/lorawan/model/simple-end-device-lora-phy.h
#  If SetReceiveOkCallback is absent: delete the for-loop in ExecuteRun.
#  If ReceivedPacket trace signature differs: update OnPhyReceive parameters.
# =============================================================

set -eo pipefail

RESULTS_DIR="Results"
mkdir -p "$RESULTS_DIR/waveguide"
mkdir -p "$RESULTS_DIR/standard"

echo "========================================================"
echo "  VALIDATION RUN v4 — $(date)"
echo "  Densities: 50 100 150 200 300 500 750 1000"
echo "  runs=5 | waves=10 | Both models"
echo "  Expected runtime: ~19-25 min"
echo "========================================================"
echo ""

# ---------------------------------------------------------------
# API verification (same checks as run_main.sh pre-flight)
# ---------------------------------------------------------------
LORAWAN_PHY=$(find . -path '*/lorawan/model/lora-phy.h' 2>/dev/null | head -1)
if [ -n "$LORAWAN_PHY" ]; then
    if grep -q 'SetReceiveOkCallback' "$LORAWAN_PHY"; then
        echo "[$(date +%H:%M:%S)] SetReceiveOkCallback found — OK"
    else
        echo "[$(date +%H:%M:%S)] WARNING: SetReceiveOkCallback NOT found in lorawan fork."
        echo "  Delete the SetReceiveOkCallback loop in ExecuteRun before building."
    fi
else
    echo "[$(date +%H:%M:%S)] WARNING: lora-phy.h not found — cannot verify API."
fi

START=$(date +%s)

# --- MODEL 1: Waveguide ITU-R P.1411 ---
echo "[$(date +%H:%M:%S)] === WAVEGUIDE MODEL START ==="
./ns3 run "scratch/waveguide-mesh --waveguide=1 --grid=1 --runs=5 --waves=10 --outdir=waveguide"
NS3_EXIT=$?
if [ "$NS3_EXIT" -ne 0 ]; then
    echo "!!! WAVEGUIDE RUN FAILED (exit $NS3_EXIT) !!!"
    exit 1
fi
echo "[$(date +%H:%M:%S)] === WAVEGUIDE MODEL DONE ==="

MID=$(date +%s)
ELAPSED=$(( MID - START ))
echo ""
echo "[$(date +%H:%M:%S)] Waveguide took $(( ELAPSED/60 ))m$(( ELAPSED%60 ))s"
echo ""

# --- MODEL 2: Standard n=4.0 ---
echo "[$(date +%H:%M:%S)] === STANDARD MODEL START ==="
./ns3 run "scratch/waveguide-mesh --waveguide=0 --grid=1 --runs=5 --waves=10 --outdir=standard"
NS3_EXIT=$?
if [ "$NS3_EXIT" -ne 0 ]; then
    echo "!!! STANDARD MODEL RUN FAILED (exit $NS3_EXIT) !!!"
    exit 1
fi
echo "[$(date +%H:%M:%S)] === STANDARD MODEL DONE ==="

# --- BURST SCENARIO: emergency alert cadence (wave_ivl=5s) ---
# Models emergency alert dissemination (wildfire/flood/earthquake).
# Short wave interval causes ALOHA collisions at high node density.
# Run waveguide model only — we expect collisionDrops > 0 here.
echo "[$(date +%H:%M:%S)] === BURST SCENARIO START ==="
mkdir -p "$RESULTS_DIR/burst"
    mkdir -p "$RESULTS_DIR/burst_std"
./ns3 run "scratch/waveguide-mesh --waveguide=1 --grid=1 --runs=5 --waves=50 --wave_ivl=5 --preset=burst --outdir=burst"
NS3_EXIT=$?
if [ "$NS3_EXIT" -ne 0 ]; then echo "!!! BURST WG FAILED !!!"; fi
./ns3 run "scratch/waveguide-mesh --waveguide=0 --grid=1 --runs=5 --waves=50 --wave_ivl=5 --preset=burst --outdir=burst_std"
NS3_EXIT=$?
if [ "$NS3_EXIT" -ne 0 ]; then echo "!!! BURST STD FAILED !!!"; fi
echo "[$(date +%H:%M:%S)] === BURST SCENARIO DONE ==="

# --- DBP SENSITIVITY QUICK CHECK (validation only) ---
# Runs DBP=27 and DBP=87 at reduced parameters to verify --dbp flag works.
echo "[$(date +%H:%M:%S)] === DBP SENSITIVITY CHECK START ==="
for DBP in 27 87; do
    echo "[$(date +%H:%M:%S)] --- DBP=${DBP}m (validation) ---"
    mkdir -p "$RESULTS_DIR/wg_dbp${DBP}"
    ./ns3 run "scratch/waveguide-mesh --waveguide=1 --grid=1 --runs=5 --waves=10 --dbp=${DBP} --outdir=wg_dbp${DBP}"
    NS3_EXIT=$?
    if [ "$NS3_EXIT" -ne 0 ]; then
        echo "!!! DBP=${DBP} VALIDATION FAILED !!!"
    fi
done
echo "[$(date +%H:%M:%S)] === DBP SENSITIVITY CHECK DONE ==="

END=$(date +%s)
TOTAL=$(( END - START ))
echo ""
echo "========================================================"
echo "  ALL DONE — $(date)"
echo "  Total: $(( TOTAL/60 ))m$(( TOTAL%60 ))s"
echo "  Results: $RESULTS_DIR/"
echo "========================================================"
