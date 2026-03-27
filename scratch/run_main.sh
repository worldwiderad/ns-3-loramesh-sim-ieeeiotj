#!/bin/bash
# =============================================================
#  PRODUCTION RUN — IEEE IoT-J Full Monte Carlo Sweep
#  Full density sweep: N = 50, 100, 150, 200, 300, 500, 750, 1000
#  runs=30, waves=30 — both models sequentially
#  Expected runtime: ~6-7 hours on M4 MacBook Air
#
#  IMPORTANT:
#    - Use caffeinate to prevent sleep: caffeinate -i ./run_main.sh
#    - Results write incrementally to CSV — safe to inspect mid-run
#    - Do NOT interrupt during N=750 or N=1000 (longest density points)
#    - Waveguide model runs first (~5.5 hrs), standard second (~40 min)
#
#  Output files (timestamped, in Results/):
#    Waveguide_ITU-R_P1411_Flooding_StreetGrid_<ts>_RawData.csv
#    Waveguide_ITU-R_P1411_Flooding_StreetGrid_<ts>_Summary.csv
#    Standard_n4.0_Flooding_StreetGrid_<ts>_RawData.csv
#    Standard_n4.0_Flooding_StreetGrid_<ts>_Summary.csv
#    production_<ts>.log
# =============================================================

RESULTS_DIR="Results"
mkdir -p "$RESULTS_DIR/waveguide"
mkdir -p "$RESULTS_DIR/standard"

echo "========================================================"
echo "  PRODUCTION RUN — $(date)"
echo "  Densities: 50 100 150 200 300 500 750 1000"
echo "  runs=50 | waves=50 | Both models"
echo "  Expected runtime: ~6-7 hours"
echo "========================================================"
echo ""

START=$(date +%s)

# =============================================================
# PRE-FLIGHT: Timing test at N=1000 to detect O(N²) PHY dispatch
# regression before committing to the full 6-7h campaign.
#
# LoraChannel::Send dispatches StartReceive to all N-1 receivers per TX.
# If SimpleEndDeviceLoraPhy::StartReceive does NOT early-return on
# sub-sensitivity signals, it will track them as interference — causing
# O(N²) events at N=1000 and making the production run infeasible.
#
# THRESHOLD: Full density sweep (r=1, w=5) should complete in < 10 min.
# N=1000 alone should take < 3 min (empirical from v1 validation: 3m41s total
# for all 8 density points at r=2,w=5, dominated by N=1000).
# RT-2 CALIBRATION: if N=1000 alone approaches 9 min, the 10-min threshold
# is too tight. To isolate N=1000 timing, run manually:
#   time ./ns3 run "scratch/waveguide-mesh --waveguide=1 --grid=1 --runs=1 --waves=5"
# and update the threshold below to 2× the N=1000-only time.
#   In simple-end-device-lora-phy.cc::StartReceive(), add before
#   m_interference->Add(packet, rxPowerDbm):
#     if (rxPowerDbm < m_sensitivity) { EndReceive(...); return; }
# =============================================================

# ---------------------------------------------------------------
# Build-time API verification for two version-sensitive calls:
#
# 1. SetReceiveOkCallback — severs MAC receive path after Install().
#    If missing: delete the SetReceiveOkCallback loop in ExecuteRun.
#
# 2. ReceivedPacket trace source — OnPhyReceive signature must match.
#    Signature used: (Ptr<const Packet>, uint32_t)
#    If mismatch: NS_ABORT fires at t=0.1s on node 0 during pre-flight
#    (the NS_ABORT_MSG_IF(!ok1,...) guard in StartApplication).
#    Fix: update OnPhyReceive parameters to match the trace source
#    typedef in simple-end-device-lora-phy.h at your installed commit.
#    Verify with:
#      grep -A3 'ReceivedPacket' \
#          <ns3_root>/contrib/lorawan/model/simple-end-device-lora-phy.h
# ---------------------------------------------------------------

LORAWAN_PHY=$(find . -path '*/lorawan/model/lora-phy.h' 2>/dev/null | head -1)
if [ -n "$LORAWAN_PHY" ]; then
    if grep -q 'SetReceiveOkCallback' "$LORAWAN_PHY"; then
        echo "[$(date +%H:%M:%S)] SetReceiveOkCallback found in $LORAWAN_PHY — OK"
    else
        echo "[$(date +%H:%M:%S)] WARNING: SetReceiveOkCallback NOT found in lorawan fork."
        echo "  Delete the SetReceiveOkCallback loop in ExecuteRun before building."
        echo "  See comment block in waveguide-mesh.cc for fallback instructions."
    fi

    # Also verify ReceivedPacket trace source exists
    LORAWAN_SEDPHY=$(find . -path '*/lorawan/model/simple-end-device-lora-phy.h' 2>/dev/null | head -1)
    if [ -n "$LORAWAN_SEDPHY" ]; then
        if grep -q 'ReceivedPacket' "$LORAWAN_SEDPHY"; then
            echo "[$(date +%H:%M:%S)] ReceivedPacket trace found in $LORAWAN_SEDPHY — OK"
        else
            echo "[$(date +%H:%M:%S)] WARNING: ReceivedPacket trace NOT found in $LORAWAN_SEDPHY."
            echo "  OnPhyReceive signature mismatch will abort at t=0.1s in pre-flight."
            echo "  Check simple-end-device-lora-phy.h for the correct trace typedef."
        fi
    fi
else
    echo "[$(date +%H:%M:%S)] WARNING: lora-phy.h not found — cannot verify SetReceiveOkCallback."
fi

echo "[$(date +%H:%M:%S)] === PRE-FLIGHT TIMING TEST ==="
echo "  Full density sweep (r=1, w=5). This tests all 8 density points."
echo "  Threshold: < 10 min for full sweep. N=1000 alone should be < 3 min."
# NOTE: target name must match CMakeLists.txt — verify before running.
# If build fails with "no rule to make target", check the executable
# name registered in scratch/CMakeLists.txt or wscript.
PREFLIGHT_START=$(date +%s)

./ns3 run "scratch/waveguide-mesh --waveguide=1 --grid=1 --runs=1 --waves=5"
NS3_EXIT=$?
if [ "$NS3_EXIT" -ne 0 ]; then
    echo "!!! PRE-FLIGHT: build/run failed (exit $NS3_EXIT). Check log above. !!!"
    echo "  Likely: SetReceiveOkCallback missing, compile error, or crash."
    exit 1
fi

PREFLIGHT_END=$(date +%s)
PREFLIGHT_ELAPSED=$(( PREFLIGHT_END - PREFLIGHT_START ))
echo "[$(date +%H:%M:%S)] Pre-flight took $(( PREFLIGHT_ELAPSED/60 ))m$(( PREFLIGHT_ELAPSED%60 ))s"

if (( PREFLIGHT_ELAPSED > 600 )); then
    echo ""
    echo "!!! PRE-FLIGHT FAILED: full sweep (r=1,w=5) took > 10 min !!!"
    echo "!!! O(N^2) PHY dispatch regression likely at N=1000. !!!"
    echo "!!! See comment in ExecuteRun for patch instructions. !!!"
    echo "!!! Aborting production run.                    !!!"
    exit 1
fi

echo "[$(date +%H:%M:%S)] Pre-flight PASSED. Proceeding with production run."
echo "  NOTE: pre-flight wrote CSV files to Results/ (r=1, w=5 calibration data)."
echo "  These are NOT production results. Filenames are timestamped for distinction."
echo ""

# --- MODEL 1: Waveguide ITU-R P.1411 ---
echo "[$(date +%H:%M:%S)] === WAVEGUIDE MODEL START ==="
./ns3 run "scratch/waveguide-mesh --waveguide=1 --grid=1 --runs=50 --waves=50 --outdir=waveguide"
NS3_EXIT=$?
if [ "$NS3_EXIT" -ne 0 ]; then
    echo "!!! WAVEGUIDE PRODUCTION RUN FAILED (exit $NS3_EXIT) — aborting. !!!"
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
./ns3 run "scratch/waveguide-mesh --waveguide=0 --grid=1 --runs=50 --waves=50 --outdir=standard"
NS3_EXIT=$?
if [ "$NS3_EXIT" -ne 0 ]; then
    echo "!!! STANDARD MODEL RUN FAILED (exit $NS3_EXIT). !!!"
    exit 1
fi
echo "[$(date +%H:%M:%S)] === STANDARD MODEL DONE ==="

# --- BURST SCENARIO: emergency alert cadence ---
echo "[$(date +%H:%M:%S)] === BURST SCENARIO START ==="
mkdir -p "$RESULTS_DIR/burst"
    mkdir -p "$RESULTS_DIR/burst_std"
./ns3 run "scratch/waveguide-mesh --waveguide=1 --grid=1 --runs=50 --waves=50 --wave_ivl=5 --preset=burst --outdir=burst"
NS3_EXIT=$?
if [ "$NS3_EXIT" -ne 0 ]; then echo "!!! BURST WG FAILED !!!"; fi
./ns3 run "scratch/waveguide-mesh --waveguide=0 --grid=1 --runs=50 --waves=50 --wave_ivl=5 --preset=burst --outdir=burst_std"
NS3_EXIT=$?
if [ "$NS3_EXIT" -ne 0 ]; then echo "!!! BURST STD FAILED !!!"; fi
echo "[$(date +%H:%M:%S)] === BURST SCENARIO DONE ==="

# --- DBP SENSITIVITY SWEEP ---
# Sweeps breakpoint distance at 3 values to prove waveguide advantage
# is robust to DBP calibration uncertainty.
#   DBP=27m  : theoretical (d_BP = 4*h_T*h_R/lambda, h=1.5m, f=915MHz)
#   DBP=87m  : typical field-measured urban value from literature
#   DBP=366m : Singapore CBD empirical calibration (default, already run above)
# Only waveguide model is swept — standard n=4.0 is DBP-independent by definition.
echo "[$(date +%H:%M:%S)] === DBP SENSITIVITY SWEEP START ==="
for DBP in 27 87; do
    echo "[$(date +%H:%M:%S)] --- DBP=${DBP}m ---"
    mkdir -p "$RESULTS_DIR/wg_dbp${DBP}"
    ./ns3 run "scratch/waveguide-mesh --waveguide=1 --grid=1 --runs=50 --waves=50 --dbp=${DBP} --outdir=wg_dbp${DBP}"
    NS3_EXIT=$?
    if [ "$NS3_EXIT" -ne 0 ]; then
        echo "!!! DBP=${DBP} RUN FAILED (exit $NS3_EXIT) !!!"
    fi
done
echo "[$(date +%H:%M:%S)] === DBP SENSITIVITY SWEEP DONE ==="

END=$(date +%s)
TOTAL=$(( END - START ))
echo ""
echo "========================================================"
echo "  ALL DONE — $(date)"
echo "  Total: $(( TOTAL/60 ))m$(( TOTAL%60 ))s"
echo "  Results: $RESULTS_DIR/"
echo "========================================================"
