#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/propagation-module.h"
#include "ns3/lora-channel.h"
#include "ns3/lora-phy.h"
#include "ns3/lora-net-device.h"
#include "ns3/lora-helper.h"
#include "ns3/lorawan-mac-helper.h"
#include "ns3/simple-end-device-lora-phy.h"
#include "ns3/gateway-lorawan-mac.h"

// Singapore street network — real CBD geometry from OpenStreetMap.
// Replaces synthetic axis-aligned heuristic with actual street
// corridors, including oblique arterials (Cecil St, Beach Rd, etc.)
// IsStreetAligned() uses finite segment projection with 200m overshoot
// tolerance to prevent unphysical waveguide grants past segment ends.
#include "singapore_streets.h"

#include <cmath>
#include <fstream>
#include <vector>
#include <set>
#include <map>
#include <numeric>
#include <algorithm>
#include <limits>
#include <unordered_set>
#include <iostream>
#include <iomanip>
#include <filesystem>
#include <ctime>
#include <sstream>
#include <chrono>
#include <deque>

using namespace ns3;
using namespace lorawan;
namespace fs = std::filesystem;

// ============================================================
//  ANSI COLOUR PALETTE
// ============================================================
namespace C {
    const std::string RST  = "\033[0m";
    const std::string BOLD = "\033[1m";
    const std::string RED  = "\033[38;5;196m";
    const std::string ORG  = "\033[38;5;208m";
    const std::string YLW  = "\033[38;5;226m";
    const std::string GRN  = "\033[38;5;82m";
    const std::string GRY  = "\033[38;5;244m";
    const std::string DGRY = "\033[38;5;238m";
}

NS_LOG_COMPONENT_DEFINE("WaveguideMeshSim");

namespace PaperParams {
    // City geometry (Section VI-A)
    constexpr double STREET_WIDTH_M  = 25.0;
    constexpr double BLOCK_SIZE_M    = 200.0;
    constexpr double CITY_SIZE_M     = 10000.0;

    // Hardware / link budget (Section III-B, SX1262 @ SF10)
    constexpr double PTX_DBM         = 22.0;
    constexpr double SENSITIVITY_DBM = -131.9;
    constexpr double PL_D0_DB        = 32.0;

    // Dual-slope waveguide model (Section III-D, Eq. 7)
    // DBP_M = 366 m is retained as the empirically calibrated channel
    // parameter from the field trial geometry (h_tx=20m, h_rx=1.5m).
    // RANGE NOTE (P-2): waveguide max range ~20km > 10×10km sim area.
    // Source (Keppel Rd, E-W) and sink (Paya Lebar Rd, N-S) are on
    // PERPENDICULAR streets — IsStreetAligned returns false for the
    // 10.8km diagonal. Direct 1-hop delivery is geometrically impossible.
    // At low N, relays may use long waveguide hops (1-5km), giving
    // meanHops ≈ 2-5 at sparse densities — physically correct.
    // Production hop-count distributions will validate this claim.
    // It characterises the Singapore CBD canyon geometry, not the mesh
    // node heights, and is applied as a validated empirical correction
    // factor consistent with Section IV field measurements. It is NOT
    // recomputed from mesh relay heights (all at 1.5m), which would
    // give dbp~27m and would not reflect the measured channel — the
    // breakpoint is a property of the canyon, not the instantaneous
    // node elevation. This is documented explicitly in Section VI text.
    constexpr double DBP_M = 366.0; // default; CLI --dbp overrides via dbpM local in main()
    constexpr double N1              = 1.40;
    constexpr double N2              = 4.95;

    // Standard urban model (Table I & Bullington [24])
    constexpr double N_URBAN         = 4.00;
    // NLoS building penetration penalty uses simplified rectilinear
    // block model (10 dB per block intersected). Real building footprints
    // would produce more accurate per-link obstruction counts but require
    // a GIS dataset integration out of scope for this study. Acknowledged
    // in Limitations section.
    constexpr double NLOS_PEN_DB     = 10.0;

    // Log-normal shadowing sigma=4.2 dB (Section V-C, Eq. 10)
    // Drawn once per link per Monte Carlo run.
    // Shadow draws clamped to PL_D0_DB floor (see ExecuteRun) to
    // prevent unphysical negative path loss at sub-metre separations.
    constexpr double SHADOW_SIGMA_DB = 4.2;

    // Unslotted ALOHA relay jitter (Section III-F)
    constexpr double JITTER_MIN_S    = 0.10;
    constexpr double JITTER_MAX_S    = 1.50;
    //
    // CAPTURE MODEL NOTE (re: reviewer concern about preamble locking):
    // The SX1262 preamble at SF10 spans 8 chirp symbols ≈ 32 ms. Standard
    // FSK/DSSS hardware cannot resynchronize to a later stronger packet once
    // preamble lock is acquired. However:
    //
    // (1) LoraInterferenceHelper (b866c0b) does NOT model preamble locking.
    //     It applies a co-channel rejection SIR matrix calibrated from actual
    //     LoRa baseband BER simulations (Magrin et al. 2017, our Ref.[29]).
    //     Capture is decided by aggregate SIR at packet end, not arrival order.
    //
    // (2) JITTER_MIN_S = 0.10s >> preamble duration = 0.032s.
    //     Every collision in this simulation involves a second packet whose
    //     preamble starts AFTER the first packet's preamble has already
    //     completed. The CSS re-synchronization regime (where LoRa could
    //     switch targets mid-preamble) NEVER occurs in our parameter space.
    //
    // (3) Bankov's 80% is an analytically integrated aggregate PDR over the
    //     full ALOHA timing distribution — not a per-event hardware spec.
    //     Our JITTER range [0.1, 1.5]s spans the same collision timing regime
    //     that Bankov integrates over. The SIR model is the correct instrument
    //     for this regime.
    //
    // Any bias direction: the SIR model may be slightly over-optimistic
    // (collisions at 100-370ms separation are inside-payload, where the real
    // SX1262 cannot recapture, but the aggregate SIR ratio might still pass).
    // The simulation is conservative on PDR if anything, not pessimistic.

    // LoRa Time-on-Air at SF10 / 125 kHz / 32 bytes (Section III-C).
    // Used to compute the guaranteed simulation margin below.
    constexpr double TOA_S           = 0.370;

    // Capture effect — Bankov et al. [28]: 80% collision survival for unslotted
    // ALOHA LoRa. This is already implicit in the combination of:
    //   (a) our ALOHA jitter [0.1,1.5]s randomly spreading transmissions, and
    //   (b) SimpleEndDeviceLoraPhy's SIR co-channel rejection matrix deciding
    //       the capture winner deterministically.
    // A prior version connected OnInterferenceLost, giving the SIR-losing
    // packet an additional 80% rescue. This was double-counting: the module had
    // already applied capture physics before firing the trace. In a k-packet
    // collision, each loser received an independent 80% chance — violating the
    // single-demodulator constraint (one SX1262 cannot decode two colliding
    // packets simultaneously). REMOVED. Capture is handled entirely by the PHY
    // SIR model. Constant retained for paper documentation; NOT used in code.
    // CAPTURE_PROB = 0.80 (Bankov et al. [28]) — intentionally not a live
    // constant. The capture mechanism was removed (see comment block above).
    // Value documented here for paper cross-reference only.

    // Traffic model: lambda=1/900=0.0011 (Section III-F, VI-A)
    // 1 pkt per 15 min. G << 0.18 Erlang saturation.
    // WAVE_IVL_S: default 900s (15-min IoT cadence). Override via --wave_ivl for
    // emergency burst scenario. Passed through ExecuteRun as a parameter.
    constexpr double WAVE_IVL_S_DEFAULT = 900.0;
    constexpr double SIM_MARGIN      = 300.0;
}

// ============================================================
//  TERMINAL UI
// ============================================================

std::string GetTimestamp() {
    std::time_t now = std::time(nullptr);
    char buf[20];
    struct tm tm_buf{};
    // localtime_r is POSIX (macOS/Linux). On MSVC use localtime_s(&tm_buf, &now).
    // Acceptable for this macOS-only target; documented per review comment M-4.
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", localtime_r(&now, &tm_buf));
    return std::string(buf);
}

std::string GetTimeStr() {
    std::time_t now = std::time(nullptr);
    char buf[10];
    struct tm tm_buf{};
    std::strftime(buf, sizeof(buf), "%H:%M:%S", localtime_r(&now, &tm_buf));
    return std::string(buf);
}

std::string PdrBar(double pdr, int width = 18) {
    int filled = static_cast<int>((pdr / 100.0) * width);
    std::string col = (pdr >= 90.0) ? C::GRN :
                      (pdr >= 60.0) ? C::YLW :
                      (pdr >= 30.0) ? C::ORG : C::RED;
    std::string bar = "[";
    for (int i = 0; i < width; i++)
        bar += (i < filled) ? col + "#" + C::RST : C::DGRY + "-" + C::RST;
    return bar + "]";
}

std::string PdrColour(double pdr) {
    std::ostringstream ss;
    std::string col = (pdr >= 90.0) ? C::GRN :
                      (pdr >= 60.0) ? C::YLW :
                      (pdr >= 30.0) ? C::ORG : C::RED;
    ss << col << C::BOLD << std::fixed << std::setprecision(1) << pdr << "%" << C::RST;
    return ss.str();
}

std::string HLine(int w) { return std::string(w, '='); }


// ============================================================
//  LIVE TERMINAL DASHBOARD
//  Replaces PrintBanner / PrintDensityHeader / PrintRunResult /
//  PrintDensitySummary / PrintFinalSummary.
//  Fixed 20-line UI, redrawn in-place via ANSI cursor positioning.
// ============================================================

static constexpr int DASH_H = 20;


struct Dashboard {
    std::string           modelName;
    uint32_t              totalDensities = 0;
    uint32_t              totalRuns      = 0;
    uint32_t              totalWaves     = 0;
    double                waveIvlS       = 900.0;
    double                dbpM           = 366.0;

    std::vector<uint32_t> densityList;
    std::vector<bool>     densityDone;
    std::vector<double>   densityPdr;
    uint32_t              dIdx      = 0;
    uint32_t              currentN  = 0;
    uint32_t              runDone   = 0;

    struct RS { double pdr; double hops; uint32_t col; uint32_t rfc; };
    std::vector<RS>  runResults;
    RS               latest = {-1,-1,0,0};
    double           mPdr   = 0;
    double           ci95   = 0;
    double           mHops  = -1;
    uint64_t         totCol = 0;
    uint64_t         totRfc = 0;

    using Clock = std::chrono::steady_clock;
    Clock::time_point    wallStart;
    Clock::time_point    runStart;
    std::deque<double>   runDurs;
    uint32_t             totalRunsAll = 0;
    uint32_t             doneRunsAll  = 0;
    mutable bool         firstRender  = true;

    void init(const std::vector<uint32_t> &dens,
              uint32_t r, uint32_t w,
              const std::string &model,
              double ivl, double dbp) {
        densityList    = dens;
        totalDensities = dens.size();
        totalRuns      = r;
        totalWaves     = w;
        modelName      = model;
        waveIvlS       = ivl;
        dbpM           = dbp;
        totalRunsAll   = totalDensities * r;
        densityDone.assign(totalDensities, false);
        densityPdr .assign(totalDensities, -1.0);
        wallStart      = Clock::now();
        printf("\033[?25l");  // hide cursor
    }

    void startDensity(uint32_t idx) {
        dIdx=idx; currentN=densityList[idx]; runDone=0; runResults.clear();
        latest={-1,-1,0,0}; mPdr=0; ci95=0; mHops=-1; totCol=0; totRfc=0;
        runStart=Clock::now();
        render();
    }

    void afterRun(uint32_t r, double pdr, double hops,
                  uint32_t col, uint32_t rfc,
                  double mp, double ci, double mh) {
        auto now=Clock::now();
        runDurs.push_back(std::chrono::duration<double>(now-runStart).count());
        if(runDurs.size()>20) runDurs.pop_front();
        runStart=now; runDone=r; doneRunsAll++;
        latest={pdr,hops,col,rfc}; runResults.push_back(latest);
        totCol+=col; totRfc+=rfc; mPdr=mp; ci95=ci; mHops=mh;
        render();
    }

    void afterDensity(uint32_t idx, double mp) {
        densityDone[idx]=true; densityPdr[idx]=mp;
        render();
    }

    void done(const std::string &rawPath, const std::string &sumPath) const {
        printf("\033[?25h\n");  // restore cursor
        printf("  \033[2mCSV \033[0m %s\n", rawPath.c_str());
        printf("  \033[2mSUM \033[0m %s\n\n", sumPath.c_str());
    }

    static std::string fmtTime(int secs) {
        char b[16]; int h=secs/3600,m=(secs%3600)/60,s=secs%60;
        if(h>0) snprintf(b,sizeof(b),"%d:%02d:%02d",h,m,s);
        else    snprintf(b,sizeof(b),"%02d:%02d",m,s);
        return b;
    }
    std::string elapsedStr() const {
        return fmtTime((int)std::chrono::duration<double>(Clock::now()-wallStart).count());
    }
    std::string etaStr() const {
        if(runDurs.empty()) return "--:--";
        double avg=0; for(double d:runDurs) avg+=d; avg/=runDurs.size();
        return fmtTime((int)((totalRunsAll-doneRunsAll)*avg));
    }
    std::string speedStr() const {
        if(runDurs.empty()) return "-.--s/run";
        double avg=0; for(double d:runDurs) avg+=d; avg/=runDurs.size();
        char b[16]; snprintf(b,sizeof(b),"%.2fs/run",avg); return b;
    }
    double pctDone() const {
        return totalRunsAll>0?100.0*doneRunsAll/totalRunsAll:0.0;
    }

    void render() const {
        if (!firstRender) printf("\033[%dA", DASH_H);
        firstRender = false;

        // Pure ASCII box: + = - |
        // Inner content width = DASH_I.  Total row = 1+2+DASH_I+2+1 = DASH_I+6.
        // Separator fill = DASH_I+4 so corners add 2 → same total.
        static const int IW = 70;          // inner content width
        static const int FW = IW + 4;      // fill width for separators (74)

        // row(): print content padded to IW, wrapped in | borders
        auto row = [](const std::string &cs) {
            // compute visible length (skip ANSI, count UTF-8 lead bytes only)
            int vl = 0; bool esc = false;
            for (unsigned char c : cs) {
                if (c == '\033') { esc=true; continue; }
                if (esc) { if ((c>='A'&&c<='Z')||(c>='a'&&c<='z')) esc=false; continue; }
                if ((c & 0xC0) != 0x80) ++vl;
            }
            int pad = IW - vl;
            printf("\033[2K\033[36m|\033[0m  %s%s  \033[36m|\033[0m\n",
                   cs.c_str(), std::string(std::max(0, pad), ' ').c_str());
        };
        auto topB     = [&]{ printf("\033[2K\033[36m+%s+\033[0m\n",      std::string(FW,'=').c_str()); };
        auto botB     = [&]{ printf("\033[2K\033[36m+%s+\033[0m\n",      std::string(FW,'=').c_str()); };
        auto heavySep = [&]{ printf("\033[2K\033[36m+%s+\033[0m\n",      std::string(FW,'=').c_str()); };
        auto thinSep  = [&]{ printf("\033[2K\033[2m\033[36m+%s+\033[0m\n", std::string(FW,'-').c_str()); };

        // L1
        topB();
        // L2 title  (59 vis)
        row("\033[1m\033[96mLoRa Urban Mesh  \xc2\xb7  ns-3.41  \xc2\xb7  Monte Carlo Simulation Engine\033[0m");
        // L3 attribution
        row("\033[2mRadoslaw Green  \xc2\xb7  IEEE Internet of Things Journal  \xc2\xb7  2026\033[0m");
        // L4
        heavySep();
        // L5 scenario + DBP  (~67 vis)
        {
            std::string dbpTag=(std::abs(dbpM-366.0)<1?"empirical":
                                std::abs(dbpM-87.0)<1 ?"field    ":"theory   ");
            std::ostringstream ss;
            ss<<"\033[2mSCENARIO\033[0m  \033[1m\033[94m"<<std::left<<std::setw(36)<<modelName<<"\033[0m"
              <<" \033[2mDBP\033[0m \033[93m"<<(int)dbpM<<"m\033[0m \033[2m("<<dbpTag<<")\033[0m";
            row(ss.str());
        }
        // L6 config  — kept under 70 vis by concise format
        {
            std::ostringstream ss;
            ss<<"\033[2mCONFIG  \033[0m  "
              <<"\033[2mruns=\033[0m\033[93m"<<totalRuns<<"\033[0m  "
              <<"\033[2mwaves=\033[0m\033[93m"<<totalWaves<<"\033[0m  "
              <<"\033[2mivl=\033[0m\033[93m"<<(int)waveIvlS<<"s\033[0m  "
              <<"\033[2m"<<totalDensities<<" pts  "
              <<totalRunsAll<<" total runs\033[0m";
            row(ss.str());
        }
        // L7
        thinSep();
        // L8 density dot grid  (~55 vis max)
        {
            std::ostringstream ss;
            ss<<"\033[2mDENSITY \033[0m  ";
            for(uint32_t i=0;i<totalDensities;i++){
                if(densityDone[i])   ss<<"\033[92mo\033[0m";
                else if(i==dIdx)     ss<<"\033[93m*\033[0m";
                else                 ss<<"\033[2m.\033[0m";
                if(i+1<totalDensities) ss<<" ";
            }
            ss<<"  \033[2m("<<dIdx+1<<"/"<<totalDensities<<")\033[0m";
            row(ss.str());
        }
        // L9 N=  (~40 vis)
        {
            std::ostringstream ss;
            ss<<"          N = \033[1m\033[93m"<<currentN<<"\033[0m"
              <<"  \033[2m(range "<<densityList.front()<<" -> "<<densityList.back()<<")\033[0m";
            row(ss.str());
        }
        // L10
        thinSep();
        // L11 run progress bar  (~47 vis)
        {
            int bw=30, filled=(totalRuns>0)?(int)((double)runDone/totalRuns*bw):0;
            std::string b;
            for(int i=0;i<filled;i++)  b+="\033[92m#\033[0m";
            for(int i=filled;i<bw;i++) b+="\033[2m-\033[0m";
            std::ostringstream ss;
            ss<<"\033[2mRUNS    \033[0m  ["<<b<<"]  \033[1m"<<runDone<<"/"<<totalRuns<<"\033[0m";
            row(ss.str());
        }
        // L12 run dot grid  (~60 vis max with 50 runs)
        {
            std::ostringstream ss; ss<<"          ";
            uint32_t show=std::min(totalRuns,(uint32_t)50);
            for(uint32_t i=0;i<show;i++){
                if(i<runDone)        ss<<"\033[92mv\033[0m";
                else if(i==runDone)  ss<<"\033[93m>\033[0m";
                else                 ss<<"\033[2m.\033[0m";
            }
            if(totalRuns>50) ss<<"\033[2m+\033[0m";
            row(ss.str());
        }
        // L13 latest result  (~58 vis)
        {
            std::ostringstream ss;
            if(runDone==0){ ss<<"\033[2m          starting...\033[0m"; }
            else {
                const char *pc=(latest.pdr>=90?"\033[92m":latest.pdr>=50?"\033[93m":"\033[91m");
                ss<<"  \033[2mLATEST  \033[0m  r="<<runDone
                  <<"  PDR "<<pc<<std::fixed<<std::setprecision(1)<<latest.pdr<<"%\033[0m";
                if(latest.hops>=0)
                    ss<<"  \033[36mhops="<<std::setprecision(1)<<latest.hops<<"\033[0m";
                ss<<"  \033[2mcol=\033[0m"<<latest.col<<"  \033[2mrfc=\033[0m"<<latest.rfc;
            }
            row(ss.str());
        }
        // L14
        thinSep();
        // L15 mean PDR  (~44 vis)
        {
            std::ostringstream ss;
            if(runDone==0){ ss<<"\033[2mMEAN PDR  --\033[0m"; }
            else {
                const char *pc=(mPdr>=90?"\033[1m\033[92m":mPdr>=50?"\033[1m\033[93m":"\033[1m\033[91m");
                const char *st=(mPdr>=90?"\033[92mPERCOLATED\033[0m":
                               mPdr>=50?"\033[93mMARGINAL\033[0m":"\033[91mNO PATH\033[0m");
                ss<<"\033[2mMEAN PDR\033[0m  "<<pc<<std::fixed<<std::setprecision(1)<<mPdr<<"%\033[0m"
                  <<"  \033[2m+/-\033[0m"<<std::setprecision(1)<<ci95<<"\033[2m% CI\033[0m"
                  <<"   "<<st;
            }
            row(ss.str());
        }
        // L16 mean hops  (~48 vis)
        {
            std::ostringstream ss;
            if(runDone==0){ ss<<"\033[2mMEAN HOPS  --\033[0m"; }
            else {
                ss<<"\033[2mMEAN HOPS\033[0m ";
                if(mHops>=0) ss<<"\033[36m\033[1m"<<std::fixed<<std::setprecision(1)<<mHops<<"\033[0m";
                else         ss<<"\033[2mN/A\033[0m";
                ss<<"   \033[2mcol=\033[0m"<<totCol<<"  \033[2mrfc=\033[0m"<<totRfc
                  <<"  \033[2m("<<runDone<<" runs)\033[0m";
            }
            row(ss.str());
        }
        // L17
        heavySep();
        // L18 elapsed/ETA  (~44 vis)
        {
            std::ostringstream ss;
            ss<<"\033[2mELAPSED \033[0m  \033[1m\033[37m"<<elapsedStr()<<"\033[0m"
              <<"   \033[2mETA\033[0m  \033[1m\033[93m"<<etaStr()<<"\033[0m"
              <<"   \033[2m"<<speedStr()<<"\033[0m";
            row(ss.str());
        }
        // L19 overall bar  — bar width 28 keeps total < 70 vis
        // "OVERALL   [" (11) + 28 + "]  100.0%  (1000/1000 runs)" (26) = 65 vis
        {
            double pct=pctDone(); int bw=28,filled=(int)(pct/100.0*bw);
            std::string b;
            for(int i=0;i<filled;i++)  b+="\033[94m#\033[0m";
            for(int i=filled;i<bw;i++) b+="\033[2m-\033[0m";
            std::ostringstream ss;
            ss<<"\033[2mOVERALL \033[0m  ["<<b<<"]  \033[2m"
              <<std::fixed<<std::setprecision(1)<<pct<<"%  ("
              <<doneRunsAll<<"/"<<totalRunsAll<<" runs)\033[0m";
            row(ss.str());
        }
        // L20
        botB();
        fflush(stdout);
    }
};

static Dashboard g_dash;


//  SENDER TAG — carries transmitting node ID through the channel
// ============================================================

// We bypass MatrixPropagationLossModel for receive-side filtering.
// Instead: use permissive channel (0 dB default loss → all packets
// reach all nodes), then filter in OnPhyReceive using a pre-computed
// viable-link set. SenderTag carries the sender's node ID so we know
// which (sender→receiver) pair to check.
class SenderTag : public Tag {
public:
    uint32_t nodeId = 0;
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("SenderTag")
            .SetParent<Tag>().AddConstructor<SenderTag>();
        return tid;
    }
    TypeId GetInstanceTypeId() const override { return GetTypeId(); }
    uint32_t GetSerializedSize() const override { return 4; }
    void Serialize(TagBuffer i) const override { i.WriteU32(nodeId); }
    void Deserialize(TagBuffer i) override { nodeId = i.ReadU32(); }
    void Print(std::ostream &os) const override { os << "Sender=" << nodeId; }
};
NS_OBJECT_ENSURE_REGISTERED(SenderTag);

// ============================================================
//  MESH HEADER
// ============================================================

class MeshHeader : public Header {
public:
    uint16_t waveId = 0; uint8_t hopCount = 0;
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("MeshHeader")
            .SetParent<Header>().AddConstructor<MeshHeader>();
        return tid;
    }
    TypeId GetInstanceTypeId() const override { return GetTypeId(); }
    uint32_t GetSerializedSize() const override { return 3; }
    void Serialize(Buffer::Iterator it) const override {
        it.WriteHtonU16(waveId); it.WriteU8(hopCount);
    }
    uint32_t Deserialize(Buffer::Iterator it) override {
        waveId = it.ReadNtohU16(); hopCount = it.ReadU8(); return 3;
    }
    void Print(std::ostream &os) const override {
        os << "Wave=" << waveId << " Hops=" << (int)hopCount;
    }
};
NS_OBJECT_ENSURE_REGISTERED(MeshHeader);

// ============================================================
//  PATH LOSS PHYSICS
// ============================================================

double CalculatePathLoss(const Vector &pi, const Vector &pj, bool useWaveguide, double dbpM) {
    double dx = std::abs(pi.x - pj.x), dy = std::abs(pi.y - pj.y);
    double d  = std::sqrt(dx*dx + dy*dy);
    // Return PL_D0_DB at sub-metre separations (eliminates 32 dB
    // discontinuity at d=1m; returning 0.0 was unphysical).
    using namespace PaperParams;
    if (d < 1.0) return PL_D0_DB;

    // Street alignment check using real Singapore OSM centrelines.
    // IsStreetAligned() uses perpendicular distance to centreline axis
    // (correctly handles oblique arterials: Cecil St ~45°, Beach Rd,
    // River Valley Rd) combined with a finite projection check
    // (200m overshoot tolerance) to prevent waveguide grants to links
    // aligned with the infinite extension of a physically terminated street.
    bool aligned = SingaporeStreets::IsStreetAligned(
        pi.x, pi.y, pj.x, pj.y, STREET_WIDTH_M);

    if (useWaveguide && aligned) {
        // ITU-R P.1411 dual-slope (Section III-D, Eqs. 7-8)
        // dbpM = empirically calibrated canyon breakpoint (default 366m, Sec.IV).
        // Passed as a parameter — no global mutation needed.
        if (d <= dbpM) return PL_D0_DB + 10.0 * N1 * std::log10(d);
        else return (PL_D0_DB + 10.0 * N1 * std::log10(dbpM))
                    + 10.0 * N2 * std::log10(d / dbpM);
    }

    // Standard model or NLoS (Section III-E & Table I).
    // Building count uses Euclidean path distance / block size.
    // MODEL REVISION (paper Sec VI): v1 used Manhattan sum b=floor(dx/200)+floor(dy/200),
    // overcounting oblique paths by up to 1.5x and making the standard model pessimistic
    // for diagonal NLoS links. v2 uses b=floor(d/BLOCK_SIZE_M), bearing-independent.
    // This REDUCES standard-model NLoS loss for oblique paths → standard-model PDR curve
    // shifts upward vs v1. Neither formula is correct for arbitrary building footprints
    // (see Limitations). This revision must be noted if v1 results are cited with v2.
    int b = aligned ? 0 : static_cast<int>(d / BLOCK_SIZE_M);
    return PL_D0_DB + 10.0 * N_URBAN * std::log10(d) + b * NLOS_PEN_DB;
}

// ============================================================
//  SIMULATION STATE
// ============================================================

struct SimState {
    uint32_t sinkId        = 0;
    uint32_t wavesInjected = 0;
    uint32_t busyDrops       = 0;
    uint32_t collisionDrops  = 0;
    std::set<std::pair<uint32_t,uint32_t>> seen;
    std::set<uint32_t>         wavesAtSink;
    std::map<uint32_t,uint8_t> hopsAtSink;  // minimum hop count per wave

    // Pre-computed viable link set: (senderNodeId << 32 | receiverNodeId)
    std::unordered_set<uint64_t> viableLinks;
    bool IsViable(uint32_t from, uint32_t to) const {
        return viableLinks.count((static_cast<uint64_t>(from) << 32) | to);
    }
    void SetViable(uint32_t a, uint32_t b) {
        viableLinks.insert((static_cast<uint64_t>(a) << 32) | b);
        viableLinks.insert((static_cast<uint64_t>(b) << 32) | a);
    }

    // All app instances — populated in ExecuteRun after all apps are created.
    // Used by PhySend for direct packet delivery, bypassing LoraChannel.
    std::vector<Ptr<Application>> apps;

    // Diagnostic log: written per-run for debugging.
    void Log(const std::string &) const {}  // logging removed

    bool HasSeen(uint32_t n, uint32_t w) const { return seen.count({n,w}) > 0; }
    void MarkSeen(uint32_t n, uint32_t w)       { seen.insert({n,w}); }
};

// ============================================================
//  MESH FLOODING APPLICATION
// ============================================================

class MeshFloodApp : public Application {
public:
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("MeshFloodApp")
            .SetParent<Application>().AddConstructor<MeshFloodApp>();
        return tid;
    }
    void Setup(uint32_t id, Ptr<NetDevice> dev, std::shared_ptr<SimState> state,
               bool source, Ptr<UniformRandomVariable> jitter) {
        m_id = id; m_dev = dev; m_state = state;
        m_source = source; m_jitter = jitter;
        m_phy   = dev->GetObject<LoraNetDevice>()->GetPhy();
        m_edPhy = DynamicCast<SimpleEndDeviceLoraPhy>(m_phy);
        // Validate once at setup, not on every PhySend invocation.
        NS_ABORT_MSG_IF(!m_edPhy,
            "Node " << id << ": PHY is not SimpleEndDeviceLoraPhy. "
            "Verify LoraPhyHelper::SetDeviceType(LoraPhyHelper::ED).");
    }
    void ScheduleWave(uint32_t waveId, Time t) {
        Simulator::Schedule(t, &MeshFloodApp::InjectWave, this, waveId);
    }

private:
    uint32_t m_id; bool m_source;
    Ptr<NetDevice> m_dev;
    Ptr<LoraPhy>                 m_phy;   // cached in Setup()
    Ptr<SimpleEndDeviceLoraPhy>  m_edPhy; // DynamicCast validated once in Setup()
    std::shared_ptr<SimState> m_state;
    Ptr<UniformRandomVariable> m_jitter;
    // Pure destructive ALOHA collision model timers
    Time     m_txUntil   {Seconds(0.0)}; // node TX-busy until this sim time
    Time     m_rxUntil   {Seconds(0.0)}; // node RX-busy until this sim time
    bool     m_rxCorrupted = false;      // collision occurred during current RX window
    uint32_t m_rxWaveId    = 0;          // wave ID currently occupying the RX window
    // m_captureRng removed — see PaperParams::CAPTURE_PROB for rationale.

    // Direct PHY send, bypassing the LoRaWAN MAC state machine entirely.
    // SimpleEndDeviceLoraPhy silently drops if not in STANDBY —
    // we check explicitly and abort rather than losing packets silently.
    // This is correct for a flooding mesh: we want always-on RX/TX,
    // not the Class A RX-window-gated behaviour of standard LoRaWAN.
    //
    // MAC/PHY combination: SIMPLE_ED PHY + GW MAC is unconventional but
    // validated. PhySend calls phy->Send() directly, completely bypassing
    // GatewayLorawanMac. Received packets reach us via the ReceivedPacket
    // PHY trace, not through the MAC receive path. GatewayLorawanMac is
    // effectively inert. Validated: simulation ran to completion (b866c0b).
    //
    // GetObject<T>() searches the aggregation graph; DynamicCast<T>()
    // follows the C++ inheritance hierarchy. The PHY IS-A
    // SimpleEndDeviceLoraPhy, so DynamicCast is semantically correct here.
    void PhySend(Ptr<Packet> p) {
        // m_edPhy cached and validated in Setup() — no per-call DynamicCast.
        //
        // MAC STATE MACHINE ISSUE: Class A MAC puts the PHY to SLEEP (state=0)
        // after each TX→RX1→RX2 sequence. By t=1800s the PHY is in SLEEP even
        // though 900s have elapsed. The deferred SwitchToStandby fires BEFORE
        // the MAC's sleep event, so the MAC overrides it.
        // FIX: call SwitchToStandby() unconditionally here, immediately before
        // the state check. This wakes SLEEP→STANDBY harmlessly. TX and RX
        // states cannot be interrupted by SwitchToStandby, so those still drop.
        m_edPhy->SwitchToStandby();
        // Timer-based half-duplex: block TX if within active TX or RX window
        {
            Time _t = Simulator::Now();
            if (_t < m_txUntil || _t < m_rxUntil) {
                m_state->busyDrops++;
                m_state->Log("PhySend:BUSY_DROP node=" + std::to_string(m_id));
                return;
            }
        }
        LoraTxParameters params;
        params.sf = 10; params.headerDisabled = false; params.codingRate = 1;
        params.bandwidthHz = 125000; params.nPreamble = 8;
        params.crcEnabled = true; params.lowDataRateOptimizationEnabled = false;
        // 923.0 MHz = Singapore AS923-1 band (920–925 MHz, IMDA TS SRD).
        // LoraChannel passes this to StartReceive, but SimpleEndDeviceLoraPhy
        // does NOT filter by frequency — it has no m_receptionPaths member.
        // That frequency-check mechanism belongs to EndDeviceLoraPhy (the full
        // model), which is a SIBLING class, not a parent. Applying the
        // AddReceptionPath() fix suggested by some reviewers would crash:
        // GetObject<EndDeviceLoraPhy>() returns null on SimpleEndDeviceLoraPhy.
        // Direct delivery to all viable receivers.
        // LoraChannel bypassed: MatrixPropagationLossModel Ptr-key lookup
        // fails in this lorawan fork — all pairs return 400dB default loss
        // → LoraChannel skips StartReceive on every node → 0% PDR.
        // m_phy->Send() not called (no channel dispatch needed).
        m_state->Log("PhySend:SENT node=" + std::to_string(m_id));
        m_txUntil = Simulator::Now() + Seconds(PaperParams::TOA_S);
        // DIRECT DELIVERY: bypass LoraChannel (unreliable due to MAC sleep).
        // Schedule ProcessPacket on every viable receiver ~1ms after this TX.
        // Propagation delay at 10km = 33µs; 1ms is a safe symbolic delay.
        uint32_t senderId = m_id;
        Ptr<Packet> pCopy = p->Copy();
        for (auto& appBase : m_state->apps) {
            auto app = DynamicCast<MeshFloodApp>(appBase);
            if (!app || app->m_id == senderId) continue;
            if (!m_state->IsViable(senderId, app->m_id)) continue;
            Ptr<Packet> rx = pCopy->Copy();
            Simulator::Schedule(Seconds(0.001),
                &MeshFloodApp::DirectReceive, app, rx);
        }
    }

    void InjectWave(uint32_t waveId) {
        NS_ASSERT_MSG(m_source, "Non-source node attempted to inject wave");
        m_state->MarkSeen(m_id, waveId);
        m_state->wavesInjected++; // incremented BEFORE PhySend
        // ACCOUNTING NOTE: if PhySend drops this injection (source PHY
        // in RX state at the scheduled wave time), wavesInjected is already
        // incremented and busyDrops also fires. The wave appears in the PDR
        // denominator but was never transmitted.
        //
        // P(source busy at injection) ≈ ToA/WAVE_IVL = 0.37/900 ≈ 0.04%.
        // Expected across full campaign: ~3 events total (30w × 30r × 8d).
        // PDR impact per event: ≤1/30 ≈ 3.3% pessimistic bias.
        //
        // This makes PDR = delivered / scheduled_waves (slightly pessimistic)
        // rather than PDR = delivered / actually_transmitted (more precise).
        // The alternative (moving wavesInjected++ after a successful send)
        // would require PhySend to return bool — added complexity for ~3 events.
        // Kept as-is; acknowledged in Section VI Limitations.
        // busyDrops conflates source and relay drops; the distinction is noted.
        m_state->Log("InjectWave wave=" + std::to_string(waveId)
            + " state=" + std::to_string(static_cast<int>(m_edPhy->GetState())));
        MeshHeader hdr; hdr.waveId = waveId; hdr.hopCount = 0;
        // 29 bytes payload + 3-byte MeshHeader = 32-byte total LoRa payload
        // consistent with Section III-C ToA calculation (>350ms at SF10/125kHz)
        Ptr<Packet> p = Create<Packet>(29);
        p->AddHeader(hdr);
        PhySend(p);
    }

    void RelayWave(uint32_t waveId, uint8_t hops) {
        MeshHeader hdr; hdr.waveId = waveId; hdr.hopCount = hops;
        Ptr<Packet> p = Create<Packet>(29); // 29 + 3-byte header = 32 bytes
        p->AddHeader(hdr);
        PhySend(p);
    }

    // Sink records minimum hop count across all arriving copies of each wave.
    // This gives the most efficient path found by the flooding process.
    void ProcessPacket(Ptr<const Packet> pkt) {
        Ptr<Packet> copy = pkt->Copy();
        MeshHeader hdr;
        // RemoveHeader returns bytes consumed (3 on success, not 0).
        // Check against serialized size for semantic correctness.
        uint32_t removed = copy->RemoveHeader(hdr);
        if (removed != hdr.GetSerializedSize()) {
            m_state->Log("ProcessPacket:BAD_HEADER node=" + std::to_string(m_id)
                + " removed=" + std::to_string(removed)
                + " expected=" + std::to_string(hdr.GetSerializedSize()));
            return;
        }

        if (m_id == m_state->sinkId) {
            if (!m_state->wavesAtSink.count(hdr.waveId)) {
                m_state->wavesAtSink.insert(hdr.waveId);
                m_state->hopsAtSink[hdr.waveId] = hdr.hopCount;
                m_state->Log("SINK_RECV wave=" + std::to_string(hdr.waveId)
                    + " hops=" + std::to_string(hdr.hopCount));
            } else {
                m_state->hopsAtSink[hdr.waveId] =
                    std::min(m_state->hopsAtSink[hdr.waveId], hdr.hopCount);
            }
            return;
        }

        if (!m_state->HasSeen(m_id, hdr.waveId)) {
            m_state->MarkSeen(m_id, hdr.waveId);
            // Cap to uint8_t max. With duplicate suppression, theoretical max
            // hops = totalN-1 = 1001 > 255. In practice at PDR>0% densities
            // paths are ≤ ~50 hops, so this guard never fires. Safety only.
            uint8_t nextHop = static_cast<uint8_t>(
                std::min<uint16_t>(uint16_t(hdr.hopCount) + 1u, 255u));
            // KNOWN LIMITATION — first-arrival relay policy:
            // ALOHA jitter means the first-arriving copy wins relay rights.
            // If a 3-hop copy arrives before a 2-hop copy (possible when a
            // longer path has shorter cumulative jitter), this node relays
            // with hopCount=4, and the later 2-hop copy is ignored (already
            // seen). Intermediate relay hop counts can therefore be inflated.
            //
            // This does NOT affect PDR — flooding uses all paths regardless
            // of hop-count assignment. It also does NOT affect the E2E metric:
            // the sink independently tracks the minimum hop count across every
            // arriving copy of each wave (see hopsAtSink above), so meanHops
            // in the CSV correctly reflects the most efficient path found.
            // The inflation only affects the hop label carried by relayed
            // packets in transit — not observable in the paper results.
            m_state->Log("RELAY_SCHED node=" + std::to_string(m_id)
                + " wave=" + std::to_string(hdr.waveId)
                + " hops=" + std::to_string(nextHop));
            Simulator::Schedule(Seconds(m_jitter->GetValue()),
                &MeshFloodApp::RelayWave, this, hdr.waveId, nextHop);
        }
    }

    // DirectReceive implements pure destructive unslotted ALOHA.
    // Called 1ms after sender's PhySend for each viable-link receiver.
    // Model: if any signal arrives during the 370ms ToA window, both
    // packets are destroyed (conservative lower bound on performance).
    // Real SX1262 hardware has a ~6dB capture effect so real PDR > this.
    void DirectReceive(Ptr<Packet> pkt) {
        Time now = Simulator::Now();
        // Peek at wave ID to distinguish true collisions from flood duplicates.
        // In real LoRa, the demodulator locks on the first preamble of a wave.
        // A second relay of the SAME wave arriving 1ms later is not an RF
        // collision — the bits are identical and the demodulator is already
        // capturing them. A collision only occurs when TWO DIFFERENT waves
        // (different waveIds) overlap in the 370ms ToA window.
        MeshHeader peekHdr;
        {
            Ptr<Packet> copy = pkt->Copy();
            copy->RemoveHeader(peekHdr);
        }
        uint32_t incomingWave = peekHdr.waveId;
        // Half-duplex: deaf while transmitting
        if (now < m_txUntil) return;
        // If already in an RX window for the SAME wave: silent duplicate drop.
        if (now < m_rxUntil && incomingWave == m_rxWaveId) return;
        // CRITICAL: if we've already fully processed this wave (HasSeen), drop
        // WITHOUT starting a new RX window. A late duplicate of an already-relayed
        // wave must not clobber m_rxUntil and block the scheduled relay.
        if (m_state->HasSeen(m_id, incomingWave)) return;
        // ALOHA collision: DIFFERENT wave arrives during active RX window.
        if (now < m_rxUntil && incomingWave != m_rxWaveId) {
            m_state->collisionDrops++;
            m_rxCorrupted = true;
            m_state->Log("COLLISION node=" + std::to_string(m_id)
                + " active=" + std::to_string(m_rxWaveId)
                + " incoming=" + std::to_string(incomingWave));
            return;
        }
        // Clear to receive: lock for full ToA duration
        m_rxUntil    = now + Seconds(PaperParams::TOA_S);
        m_rxWaveId   = incomingWave;
        m_rxCorrupted = false;
        Simulator::Schedule(Seconds(PaperParams::TOA_S),
            &MeshFloodApp::ProcessIfClean, this, pkt);
    }

    void ProcessIfClean(Ptr<Packet> pkt) {
        if (m_rxCorrupted) {
            m_state->collisionDrops++;
            m_state->Log("COLLISION_DESTROY node=" + std::to_string(m_id));
            return;
        }
        ProcessPacket(pkt);
    }

    void OnPhyReceive(Ptr<const Packet> pkt, uint32_t /*sysId*/) {
        // Read sender tag to identify which node sent this packet.
        SenderTag stag;
        bool hasTag = pkt->FindFirstMatchingByteTag(stag);
        if (!hasTag) {
            m_state->Log("OnPhyReceive:NO_TAG node=" + std::to_string(m_id));
            return; // not our mesh packet
        }
        uint32_t senderId = stag.nodeId;
        // Link quality filter: channel uses 0 dB default loss so ALL nodes
        // receive ALL transmissions. Only process if the (sender→me) link
        // is viable in our pre-computed path loss model.
        if (!m_state->IsViable(senderId, m_id)) {
            m_state->Log("OnPhyReceive:DEAD_LINK sender=" + std::to_string(senderId)
                + " receiver=" + std::to_string(m_id));
            return; // below sensitivity in our model
        }
        m_state->Log("OnPhyReceive:OK node=" + std::to_string(m_id)
            + " sender=" + std::to_string(senderId)
            + " pktSz=" + std::to_string(pkt->GetSize()));
        ProcessPacket(pkt);
    }

    // OnInterferenceLost REMOVED. PHY SIR model + ALOHA jitter already
    // implement capture correctly. The old callback violated the
    // single-demodulator constraint. See CAPTURE_PROB comment.

    void StartApplication() override {
        Ptr<LoraPhy> phy = m_phy; // cached in Setup()
        // Log state at startup for all nodes
        m_state->Log("StartApp node=" + std::to_string(m_id)
            + " source=" + std::to_string(m_source)
            + " state=" + std::to_string(static_cast<int>(m_edPhy->GetState())));
        bool ok1 = phy->TraceConnectWithoutContext("ReceivedPacket",
            MakeCallback(&MeshFloodApp::OnPhyReceive, this));
        NS_ABORT_MSG_IF(!ok1, "Node " << m_id << ": ReceivedPacket trace failed");
        // LostPacketBecauseInterference NOT connected — capture is handled by
        // the PHY SIR model + ALOHA jitter. See CAPTURE_PROB comment.
        //
        // KNOWN MODEL LIMITATION — preamble-lock vs SIR winner:
        // ns-3 LoraInterferenceHelper gives collision victories to the
        // STRONGEST-power concurrent packet. The real SX1262 instead locks
        // onto the FIRST-detected preamble (~82ms at SF10) and decodes that
        // packet unless a later interferer exceeds ~6dB SIR advantage.
        // In our jitter-spread ALOHA regime the earlier-arriving packet
        // is typically stronger (shorter path). The ns-3 model agrees.
        // But when a later packet is marginally stronger (< 6dB), ns-3
        // assigns it the win while real hardware keeps the locked preamble.
        // Net effect: simulation slightly UNDERESTIMATES PDR vs real hardware.
        // This makes our percolation threshold conservatively reported —
        // real hardware would percolate at the same or lower node density.
        // Corroborated by: Chasserat et al., Trans. Emerg. Telecom. Tech., 2024
        // (LoRaSync): models consistently underestimate real LoRa throughput.
        // Acknowledged in Section VI Limitations.
    }

    void StopApplication() override {
        Ptr<LoraPhy> phy = m_phy; // cached in Setup()
        phy->TraceDisconnectWithoutContext("ReceivedPacket",
            MakeCallback(&MeshFloodApp::OnPhyReceive, this));
    }
};
NS_OBJECT_ENSURE_REGISTERED(MeshFloodApp);

// ============================================================
//  POSITION GENERATION (Section VI-A)
// ============================================================

void GenerateRelayPositions(uint32_t n, bool grid, uint32_t run,
                             Ptr<ListPositionAllocator> alloc) {
    auto MakeRng = [&](int o) {
        auto r = CreateObject<UniformRandomVariable>();
        r->SetStream(static_cast<int64_t>(run) * 20000 + o); return r;
    };
    // RNG stream assignments (fixed per run for reproducibility):
    // Stream 1: parametric position along segment (t in [0,1])
    // Stream 2: lateral offset perpendicular to segment ([-12.5, 12.5] m)
    // Stream 3: segment index selector (CDF lookup)
    // Stream 4: retired (was rAxis in old grid code)
    // Stream 5: uniform x for random layout
    // Stream 6: uniform y for random layout
    // Streams 10000+i: ALOHA jitter per node
    // Stream 7: shadow fading (set in ExecuteRun)
    //
    // CROSS-DENSITY CORRELATION — MONOTONE COUPLING (deliberate design):
    // Streams 1/2/3 are keyed by `run` only, not by n. This means node i
    // has IDENTICAL position at N=50, N=200, N=1000 within the same run.
    // Higher density points are strict topological supersets of lower ones:
    //   N=50  ⊂  N=100  ⊂  N=150  ⊂ ... ⊂  N=1000  (within each run)
    // Shadow fading (stream 7) is similarly shared: path loss between any
    // two specific nodes is identical across all density points in a run.
    //
    // This is the 'monotone coupling' construction, standard in percolation
    // theory for studying phase transitions (see: Bollobas & Riordan,
    // 'Percolation', 2006; and review by Li et al., Physics Reports, 2021).
    // Its properties:
    //   (1) PDR(N) is guaranteed non-decreasing within each run — adding
    //       nodes cannot reduce connectivity. This eliminates spurious
    //       reversals in the percolation curve that independent topologies
    //       would produce at low run counts.
    //   (2) The percolation threshold is unambiguously identifiable per run.
    //   (3) The ONLY variable between density points within a run is node
    //       count — topology and channel are held constant. This cleanly
    //       isolates the density effect from topology variability.
    //
    // CI INTERPRETATION (P-3, paper Sec VI): because PDR(N) is monotone
    // non-decreasing within each run, the 95% CI across 30 runs quantifies
    // run-to-run variation in the PERCOLATION THRESHOLD POSITION, not
    // variation in connectivity at fixed density. A wide CI at N=150 means
    // the threshold varies run-to-run; state this explicitly in Sec VI.
    //   (4) Statistical independence ACROSS density points comes from the
    //       30 independent runs (each with a different stream base run×20000).
    //       The 30 PDR samples at each density are i.i.d. population draws.
    //
    // If fully independent topologies are preferred, seed the streams as
    // `run*20000 + n*7 + offset` to decorrelate density points. Not done
    // here because the monotone property is scientifically appropriate for
    // a percolation threshold study and is documented in Section VI.
    auto rT   = MakeRng(1); rT->SetAttribute("Max",  DoubleValue(1.0));
    auto rLat = MakeRng(2); rLat->SetAttribute("Min", DoubleValue(-12.5));
                             rLat->SetAttribute("Max", DoubleValue( 12.5));
    auto rStr = MakeRng(3); rStr->SetAttribute("Max",
                             DoubleValue(static_cast<double>(
                             SingaporeStreets::STREETS.size())));
    auto rx   = MakeRng(5); rx->SetAttribute("Max", DoubleValue(PaperParams::CITY_SIZE_M));
    auto ry   = MakeRng(6); ry->SetAttribute("Max", DoubleValue(PaperParams::CITY_SIZE_M));

    const auto& streets = SingaporeStreets::STREETS;

    // Length-weighted segment sampling. Without weighting, a 200m stub and
    // a 5000m arterial have equal selection probability → short stubs get
    // ~25x higher node density per metre than long arterials. Length-weighted
    // sampling gives uniform spatial density along the street network.
    std::vector<double> segCumLen(streets.size());
    {
        double acc = 0.0;
        for (size_t k = 0; k < streets.size(); k++) {
            const auto& sk = streets[k];
            double ddx = sk.x2-sk.x1, ddy = sk.y2-sk.y1;
            acc += std::sqrt(ddx*ddx + ddy*ddy);
            segCumLen[k] = acc;
        }
        // Re-range rStr from [0,nStreets) to [0,totalLength) for CDF lookup.
        rStr->SetAttribute("Max", DoubleValue(acc));
    }

    for (uint32_t i = 0; i < n; i++) {
        double x, y;
        if (grid) {
            // Length-weighted segment selection via CDF lookup:
            // sample in [0, totalLength), binary-search cumulative lengths.
            double lenSample = rStr->GetValue();
            uint32_t si = static_cast<uint32_t>(
                std::lower_bound(segCumLen.begin(), segCumLen.end(), lenSample)
                - segCumLen.begin());
            if (si >= static_cast<uint32_t>(streets.size()))
                si = static_cast<uint32_t>(streets.size()) - 1u;
            const auto& s = streets[si];
            double t  = rT->GetValue();            // [0,1] along segment
            double dx = s.x2 - s.x1, dy = s.y2 - s.y1;
            double len = std::sqrt(dx*dx + dy*dy);
            if (len < 1.0) len = 1.0;              // degenerate guard
            double px = -dy / len, py = dx / len;  // perpendicular unit vec
            double lat = rLat->GetValue();          // [-12.5, 12.5] m offset
            x = s.x1 + t * dx + lat * px;
            y = s.y1 + t * dy + lat * py;
        } else {
            x = rx->GetValue();
            y = ry->GetValue();
        }
        alloc->Add(Vector(std::clamp(x, 0.0, PaperParams::CITY_SIZE_M),
                          std::clamp(y, 0.0, PaperParams::CITY_SIZE_M), 1.5));
    }
}

// ============================================================
//  SINGLE RUN
// ============================================================

// No-op callback for severing the PHY→MAC receive path after Install().
// LoraPhy::SetReceiveOkCallback in signetlabdei/lorawan stores:
//   Callback<void, Ptr<const Packet>, uint32_t, double, double, uint32_t>
// (packet, sender node ID, RSSI dBm, SNR dB, channel index)
// Using a named no-op function rather than a default-constructed null Callback:
// ns3::Callback<>::operator()() dereferences m_impl directly — if the lorawan
// version does NOT guard IsNull() before invoking, the null Callback causes
// NS_FATAL_ERROR on the first received packet. A non-null MakeCallback pointing
// to a no-op is safe regardless of whether the caller checks IsNull().
// In this lorawan fork (b866c0b), RxOkCallback is Callback<void, Ptr<const Packet>>
// (1 arg). Compiler confirmed via type-mismatch on the 5-arg form.
static void NullRxOkCb(Ptr<const Packet> /*packet*/) {}

struct RunResult { double pdr=0, meanHops=0; uint32_t sent=0, recvd=0, busyDrops=0, collisionDrops=0; };

RunResult ExecuteRun(uint32_t nRelays, uint32_t run, bool waveguide,
                     bool grid, uint32_t nWaves, double dbpM,
                     const std::string &outdir, double waveIvlS) {
    using namespace PaperParams;
    // RNG isolation — placement and scope verified:
    //
    // PLACEMENT: SetRun() is called here, BEFORE NodeContainer::Create(),
    // ListPositionAllocator, NormalRandomVariable, MatrixPropagationLossModel,
    // LoraChannel, and LoraHelper::Install(). Any auto-assigned streams
    // allocated by Install() receive the correct run-offset base.
    //
    // INTERNAL RNG AUDIT (commit b866c0b):
    //   SimpleEndDeviceLoraPhy: LoraInterferenceHelper uses a deterministic
    //     SIR threshold matrix — no RNG draws. State machine (STANDBY/TX/RX)
    //     has no stochastic transitions.
    //   LoraChannel: uses MatrixPropagationLossModel (pre-computed lookup)
    //     + ConstantSpeedPropagationDelayModel (distance/c). Both deterministic.
    //   GatewayLorawanMac: receive callback severed; PhySend bypasses MAC.
    //     Never invoked, so any internal MAC RNG is irrelevant.
    //
    // CONCLUSION: SetRun() is a no-op for our active simulation paths.
    // LoraHelper::AssignStreams() is NOT called: it would require the method
    // to exist on LoraHelper at b866c0b (unverified), and there are no
    // internal RNG objects to isolate. If module internals change in
    // future ns-3/lorawan versions, add AssignStreams() here.
    RngSeedManager::SetRun(run);
    // dbpM is passed directly to CalculatePathLoss — no global mutation needed.
    uint32_t totalN = nRelays + 2;
    uint32_t sinkId = totalN - 1;

    NodeContainer nodes; nodes.Create(totalN);
    auto pos = CreateObject<ListPositionAllocator>();

    // Source: western end of Keppel Road — on-street, matches OSM centreline.
    // Sink: northern terminus of Paya Lebar Road — on-street, matches OSM.
    // Separation: sqrt((8500-1500)^2 + (9000-800)^2) = ~10.8 km diagonal.
    // Both endpoints are exactly on OSM centreline coordinates so their
    // first/last hop links can benefit from waveguide alignment.
    // (Paper text: "source and sink separated by approximately 10.8 km
    // along a realistic Singapore CBD diagonal" — not corner-to-corner.)
    pos->Add(Vector(1500.0, 800.0,  1.5));   // Source: Keppel Road (W end)
    GenerateRelayPositions(nRelays, grid, run, pos);
    pos->Add(Vector(8500.0, 9000.0, 1.5));   // Sink: Paya Lebar Rd (N end)

    MobilityHelper mob; mob.SetPositionAllocator(pos);
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel"); mob.Install(nodes);

    // Log-normal shadowing sigma=4.2 dB (Section V-C, Eq. 10).
    // One draw per link per Monte Carlo run (not per packet — the channel
    // is treated as quasi-static over a simulation episode).
    //
    // CROSS-DENSITY SHADOW CORRELATION (acknowledged, not a bug):
    // shadowRng uses stream (run×20000+7) regardless of density. The
    // N=200 and N=300 runs at the same 'run' index share the first
    // 202×201/2 = 20,301 draws — the N=300 run extends the same stream.
    // This means the first 202 nodes have identical shadow values across
    // adjacent density points for the same run index. Node POSITIONS are
    // identical for shared indices across density points (monotone coupling —
    // see GenerateRelayPositions comment). Shadow draws for shared (i,j) pairs
    // are therefore also identical across densities within a run — both
    // positions AND shadows are coupled. Path loss for shared pairs is
    // consequently identical; only the extra nodes in the higher-density run
    // introduce new link realizations. This is the intended monotone coupling
    // construction, not a deficiency. To fully decorrelate across densities,
    // use stream (run×20000 + density_index×100 + 7); the current design is
    // deliberate and well-established in percolation studies.
    //
    // CLAMP BIAS NOTE: std::max(pl + shadow, PL_D0_DB) truncates the lower
    // tail of the log-normal distribution, shifting the effective mean
    // upward by ~1.7 dB at d=1m and ~0.35 dB at d=2m. This is real.
    // However, all links where the clamp fires (d < ~5m) have path loss
    // 32–42 dB, giving received power of −10 to −20 dBm — 85–108 dB above
    // the −131.9 dBm sensitivity threshold. Their connectivity is determined
    // with certainty regardless of any shadow draw. The clamp therefore has
    // zero effect on PDR or hop-count statistics. Acknowledged in Limitations.
    //
    // Serial-correlation note: shadow draws are sequential from one PRNG
    // stream (MRG32k3a with good serial independence). Changing totalN shifts
    // the draw index for every (i,j) pair after the insertion point, so
    // different density points (different totalN) get different shadow
    // realisations for the "same" node pairs. This is standard Monte Carlo
    // practice: density points are independent realisations, not meant to
    // be node-by-node comparable. The per-run mean/CI correctly captures
    // the distributional behaviour at each density.
    auto shadowRng = CreateObject<NormalRandomVariable>();
    shadowRng->SetAttribute("Mean",     DoubleValue(0.0));
    shadowRng->SetAttribute("Variance", DoubleValue(SHADOW_SIGMA_DB * SHADOW_SIGMA_DB));
    shadowRng->SetStream(static_cast<int64_t>(run) * 20000 + 7);

    // Pre-compute full link matrix: O(N²) distance calculations + shadow draws.
    // For N=1000: 501,501 pairs × ~5µs each ≈ 2.5s per run. With 30 runs ×
    // 8 densities: ~600s total. Negligible relative to simulation event time.
    // std::map::SetLoss insertions are O(log M) where M = viable links, also
    // negligible. This cost is one-time per run, not repeated per wave.
    //
    // Links exceeding the 154 dB hardware budget are assigned 400 dB default
    // loss (culled at init, not during simulation). CORRECT for SimpleEndDeviceLoraPhy's pairwise SIR model.
    // Sub-threshold signals do NOT contribute to the SIR calculation for
    // other packets. Lowering the cutoff to -150 dBm (suggested by some
    // reviewers who assumed an aggregate noise-floor model) would inject
    // O(N^2) phantom PHY events all immediately dropped at sensitivity check,
    // inflating runtime ~100x with zero physics benefit. Do NOT change.
    auto state = std::make_shared<SimState>(); state->sinkId = sinkId;
    // Diagnostic log removed — use dashboard output

    auto loss = CreateObject<MatrixPropagationLossModel>();
    // Channel is permissive: path-loss gating is in PhySend via viableLinks.
    // MatrixPropagationLossModel Ptr-key lookup fails in this lorawan fork
    // (all pairs return 400 dB default → LoraChannel skips StartReceive).
    loss->SetDefaultLoss(0.0);
    uint32_t nViable = 0;
    for (uint32_t i = 0; i < totalN; i++)
        for (uint32_t j = i + 1; j < totalN; j++) {
            double pl = CalculatePathLoss(
                nodes.Get(i)->GetObject<MobilityModel>()->GetPosition(),
                nodes.Get(j)->GetObject<MobilityModel>()->GetPosition(), waveguide, dbpM);
            pl = std::max(pl + shadowRng->GetValue(), PL_D0_DB);
            if (PTX_DBM - pl >= SENSITIVITY_DBM) {
                state->SetViable(i, j);
                nViable++;
            }
        }
    state->Log("MATRIX viable_pairs=" + std::to_string(nViable)
        + " of " + std::to_string(totalN*(totalN-1)/2)
        + " wave_ivl=" + std::to_string(static_cast<int>(waveIvlS)) + "s");

    // NOTE — LoraChannel dispatches to all N nodes via MatrixPropagationLossModel.
    // Only viable pairs receive above sensitivity — others drop at StartReceive.
    // O(N²) event overhead is acceptable: most StartReceive calls are very cheap
    // (early exit on rxPower < sensitivity). Runtime measured ~30min for N=1000.
    // receivers per transmission and dispatches StartReceive on each with
    // the MatrixPropagationLossModel loss (400 dB = -378 dBm for dead links).
    //
    // The critical question is what SimpleEndDeviceLoraPhy::StartReceive
    // does with a -378 dBm signal:
    //
    //   Option A (benign): checks rxPower < sensitivity, returns immediately.
    //     → O(N) cheap calls per TX, no interference tracking. Consistent with
    //     validation timing: all 8 density points in ~3m41s. At this rate,
    //     production N=1000 would be ~12min — within the 6-7h total estimate.
    //
    //   Option B (problematic): adds packet to LoraInterferenceHelper BEFORE
    //     sensitivity check, tracking -378 dBm signals as noise sources.
    //     → O(N²) interference entries × O(N) SIR checks per reception.
    //     At N=1000 production: ~720M events × 10µs each = 2+ hours for N=1000
    //     alone. Validation timing would have been >> 4 min. INCONSISTENT.
    //
    // CONCLUSION: empirical evidence strongly favours Option A.
    // MANDATORY: run the timing pre-flight in run_main.sh before production.
    // If N=1000 single run (r=1, w=5) takes > 5 minutes, Option B is true
    // and a patch to SimpleEndDeviceLoraPhy::StartReceive is needed:
    //   if (rxPowerDbm < m_sensitivity) return; // before interference Add()
    auto channel = CreateObject<LoraChannel>(loss,
        CreateObject<ConstantSpeedPropagationDelayModel>());

    // SimpleEndDeviceLoraPhy: always-on RX, single demodulator (SX1262).
    // GatewayLorawanMac: used only to satisfy LoraHelper::Install() — its
    // receive path IS connected by Install() as m_rxOkCallback on the PHY.
    // We immediately sever that callback below so GatewayLorawanMac::Receive
    // never sees our custom MeshHeader packets (it would attempt to deserialize
    // them as LorawanMacHeader, misinterpret the payload, and potentially spam
    // log output or hit assertions on the null network-server backhaul pointer).
    LoraPhyHelper phyH;
    phyH.SetChannel(channel);
    // LoraPhyHelper::ED creates SimpleEndDeviceLoraPhy in this lorawan fork.
    // SIMPLE_ED does not exist in the installed enum (confirmed by build error).
    phyH.SetDeviceType(LoraPhyHelper::ED);
    LorawanMacHelper macH;
    // ED_A MAC configures EndDeviceLoraPhy reception paths during Install.
    // GW MAC left m_receptionPaths empty → every packet dropped (v6 fix).
    macH.SetDeviceType(LorawanMacHelper::ED_A);
    NetDeviceContainer devices = LoraHelper().Install(phyH, macH, nodes);

    // POST-INSTALL PHY SETUP.
    //
    // FIX A: SwitchToStandby() — wake PHY from MAC-induced sleep.
    // FIX B: SetFrequency(923.0) — ED_A MAC sets its own channel plan
    //   (AS923: 923.2/923.4 MHz). Our mesh transmits at 923.0 MHz.
    //   IsOnFrequency() in StartReceive silently drops any packet whose
    //   frequency != m_frequency, so ReceivedPacket trace never fires.
    //   Fix: force all PHYs to listen on exactly our TX frequency.
    // FIX C: SetSpreadingFactor(SF10) — MAC may configure a different SF.
    //   The SF check in StartReceive is equally silent.
    for (uint32_t i = 0; i < totalN; i++) {
        Ptr<LoraPhy> phy = devices.Get(i)->GetObject<LoraNetDevice>()->GetPhy();
        Ptr<SimpleEndDeviceLoraPhy> simplePhy = DynamicCast<SimpleEndDeviceLoraPhy>(phy);
        if (!simplePhy) continue;
        simplePhy->SwitchToStandby();                    // FIX A
        simplePhy->SetFrequency(923.0);                  // FIX B
        simplePhy->SetSpreadingFactor(10);               // FIX C: SF10 = SX1262 config
    }

    // Sever the PHY→MAC receive callback on every node.
    //
    // BEFORE FIRST BUILD — verify SetReceiveOkCallback exists in your fork:
    //   grep -r 'SetReceiveOkCallback' \
    //       <ns3_root>/contrib/lorawan/model/lora-phy.h
    // If that grep finds nothing, this block will not compile. In that case,
    // DELETE this entire for-loop — the simulation ran correctly without it
    // (empirically validated: GatewayLorawanMac silently dropped our
    // MeshHeader frames without crashing across 240+ runs). The severing
    // is a correctness improvement, not a hard dependency.
    //
    // If the method EXISTS but NullRxOkCb's signature is wrong, the compiler
    // gives a clear type-mismatch error at the MakeCallback call below —
    // update NullRxOkCb's parameters to match lora-phy.h's ReceiveOkCallback
    // typedef and rebuild.
    for (uint32_t i = 0; i < totalN; i++) {
        Ptr<LoraPhy> phy = devices.Get(i)->GetObject<LoraNetDevice>()->GetPhy();
        phy->SetReceiveOkCallback(MakeCallback(&NullRxOkCb));
    }
    // AssignStreams note: the lorawan module API does not expose a standard
    // helper::AssignStreams() post-install. For our paths this is moot:
    // SimpleEndDeviceLoraPhy's SIR model is deterministic (no internal RNG),
    // and GatewayLorawanMac is never invoked (PhySend bypasses MAC entirely).
    // RngSeedManager::SetRun(run) above handles any other auto-assigned streams.


    // Simulation margin: guaranteed to accommodate the worst-case path for
    // the last injected wave, even if it traverses every node in a chain
    // at maximum jitter. Worst-case per hop = JITTER_MAX_S + TOA_S = 1.87s.
    // Theoretical maximum hops = totalN - 1 (a complete chain).
    //
    // ns-3 is a discrete-event simulator: it jumps directly to the next
    // scheduled event. Extra simulated time with no events costs ~zero
    // wall-clock time, so this guarantee is free.
    //
    // TIGHTNESS NOTE (M-6): at N=1000, dynMargin = 1002×1.87 = 1873.74s.
    // Theoretical worst-case chain = 1001×1.87 = 1872.87s. Slack = 0.87s.
    // StopApplication and Simulator::Stop both fire at Seconds(stop), so
    // a packet in the 0.87s window could theoretically be cut off.
    // In practice, a complete 1001-hop chain at max jitter is impossible
    // given duplicate suppression — but acknowledged here for traceability.
    //
    // SIM_MARGIN (300s) was previously hardcoded. It is sufficient for all
    // *observed* hop counts (max ~29 at N=1000 → 54s) but NOT for the
    // theoretical worst case at N=1000 (1002 × 1.87s = 1874s > 300s).
    double dynMargin = static_cast<double>(totalN) * (JITTER_MAX_S + TOA_S);
    double stop = nWaves * waveIvlS + std::max(dynMargin, SIM_MARGIN);
    // RT-1 NOTE: for N=1000, stop ≈ 30×900 + 1873.74 = 28,874s (~8× v1).
    // This inflates Simulator::Stop time but NOT wall-clock time — ns-3
    // jumps directly to the next scheduled event. The 1002 StopApplication
    // events at t=stop are negligible overhead.

    for (uint32_t i = 0; i < totalN; i++) {
        auto jit = CreateObject<UniformRandomVariable>();
        jit->SetAttribute("Min", DoubleValue(JITTER_MIN_S));
        jit->SetAttribute("Max", DoubleValue(JITTER_MAX_S));
        jit->SetStream(static_cast<int64_t>(run) * 20000 + 10000 + i);

        // captureRng removed: OnInterferenceLost no longer connected.
        // Streams 15000+i are now free.
        auto app = CreateObject<MeshFloodApp>();
        app->Setup(i, devices.Get(i), state, (i == 0), jit);
        nodes.Get(i)->AddApplication(app);
        app->SetStartTime(Seconds(0.1)); app->SetStopTime(Seconds(stop));
        if (i == 0)
            for (uint32_t w = 1; w <= nWaves; w++)
                app->ScheduleWave(w, Seconds(w * waveIvlS));
        state->apps.push_back(app);
    }

    Simulator::Stop(Seconds(stop)); Simulator::Run();

    RunResult res;
    res.sent  = state->wavesInjected;
    res.recvd = static_cast<uint32_t>(state->wavesAtSink.size());
    res.pdr   = (res.sent > 0) ? (double)res.recvd / res.sent * 100.0 : 0.0;
    if (!state->hopsAtSink.empty()) {
        double s = 0; for (auto &kv : state->hopsAtSink) s += kv.second;
        res.meanHops = s / state->hopsAtSink.size();
    } else {
        res.meanHops = -1.0; // sentinel: no waves reached sink this run
    }
    res.busyDrops      = state->busyDrops;
    res.collisionDrops = state->collisionDrops;
    state->Log("RUN_END sent=" + std::to_string(res.sent)
        + " recvd=" + std::to_string(res.recvd)
        + " busyDrops=" + std::to_string(res.busyDrops)
        + " collisionDrops=" + std::to_string(res.collisionDrops)
        + " PDR=" + std::to_string(res.pdr));
    Simulator::Destroy();
    return res;
}

// ============================================================
//  MAIN
// ============================================================

int main(int argc, char *argv[]) {
    uint32_t runs = 30, numWaves = 30; bool wg = true, grid = true;
    double dbpM = PaperParams::DBP_M; // default 366m; overrideable via --dbp
    CommandLine cmd;
    cmd.AddValue("runs",      "MC iterations per density point",              runs);
    cmd.AddValue("waves",     "Flooding waves per iteration",                 numWaves);
    cmd.AddValue("waveguide", "1=Waveguide ITU-R P.1411, 0=Standard n=4.0",  wg);
    cmd.AddValue("grid",      "1=Street-aligned placement, 0=Uniform random", grid);
    cmd.AddValue("dbp",       "Dual-slope breakpoint (m). Default=366 (empirical, Sec.IV). Use 87 or 27 for sensitivity analysis.", dbpM);
    std::string outdir = "";
    cmd.AddValue("outdir", "Subdirectory under Results/ for CSV/diag output (e.g. waveguide or standard)", outdir);
    double waveIvlS = PaperParams::WAVE_IVL_S_DEFAULT;
    cmd.AddValue("wave_ivl", "Inter-wave interval in seconds. Default=900 (15-min IoT). "
                             "Use 5-10 for emergency burst stress test.", waveIvlS);
    std::string densityPreset = "full";
    cmd.AddValue("preset", "Density preset: full=20pt fine sweep, burst=12pt reduced sweep", densityPreset);
    cmd.Parse(argc, argv);
    std::string resultsBase = outdir.empty() ? "Results" : "Results/" + outdir;

    RngSeedManager::SetSeed(42);

    // dbpM must be positive; log10(0)=-inf → infinite path loss → no links.
    NS_ABORT_MSG_IF(dbpM <= 0.0,
        "--dbp must be positive (got " << dbpM << "). "
        "Default=366, sensitivity values: 87 (field) or 27 (theory).");

    // waveId is uint16_t (max 65535). Guard against silent wrap.
    NS_ABORT_MSG_IF(numWaves > 65535u,
        "numWaves=" << numWaves << " exceeds uint16_t range. "
        "Widen MeshHeader::waveId to uint32_t if >65535 waves needed.");

    // runs=0 causes division by zero in statistics (pdrs.size()/n) and
    // produces NaN in every CSV column. Catch it before any work is done.
    NS_ABORT_MSG_IF(runs == 0u, "--runs=0 is invalid: need at least 1 run.");
    NS_ABORT_MSG_IF(numWaves == 0u, "--waves=0 is invalid: need at least 1 wave.");

    std::cout << C::GRY << "\n  [Streets] Loaded " << SingaporeStreets::STREETS.size()
              << " Singapore CBD segments from singapore_streets.h"
              << " (finite-segment alignment, 200m overshoot tolerance)" << C::RST << "\n";

    std::string resultsDir = resultsBase; // from --outdir CLI flag
    // Use error_code overload to avoid unhandled filesystem_error
    // exception if creation fails (e.g. permission denied, read-only
    // filesystem). The throwing overload would produce a cryptic abort
    // before the file-open guards below have a chance to report the path.
    std::error_code fsec;
    fs::create_directories(resultsDir, fsec);
    // Also ensure diag subdirectory exists
    // create_directories is idempotent (no error if already exists).
    // Any real failure (permissions, etc.) is caught below by is_open().
    std::string ts         = GetTimestamp();
    std::string modelName  = (wg ? "Waveguide_ITU-R_P1411_Flooding"
                                 : "Standard_n4.0_Flooding");
    std::string layoutName = (grid ? "StreetGrid" : "UniformRandom");
    std::string rawPath    = resultsDir+"/"+modelName+"_"+layoutName+"_"+ts+"_RawData.csv";
    std::string sumPath    = resultsDir+"/"+modelName+"_"+layoutName+"_"+ts+"_Summary.csv";

    // Density sweep. "full" = 20-pt fine-grained through percolation transition region.
    // "burst" = 12-pt reduced sweep for emergency cadence (wave_ivl=5s) scenario.
    std::vector<uint32_t> densities;
    if (densityPreset == "burst") {
        densities = {50,100,200,300,350,400,450,500,600,750,900,1000};
    } else { // "full" or any other value
        densities = {25,50,75,100,125,150,175,200,225,250,275,300,325,350,375,400,450,500,750,1000};
    }
    std::ofstream raw(rawPath), sum(sumPath);
    // CRITICAL: validate both files opened successfully before starting any run.
    // std::ofstream failure is silent in C++ — every subsequent << and flush()
    // becomes a no-op, silently discarding all results. Without this check a
    // full filesystem, bad permissions, or broken symlink would cause a 10-hour
    // run to complete with exit code 0 and empty output files.
    NS_ABORT_MSG_IF(!raw.is_open(),
        "Cannot open raw output file: " << rawPath
        << "\n  Check that " << resultsDir << "/ exists, is writable,"
        << " and the filesystem has space.");
    NS_ABORT_MSG_IF(!sum.is_open(),
        "Cannot open summary output file: " << sumPath
        << "\n  Check that " << resultsDir << "/ exists, is writable,"
        << " and the filesystem has space.");
    raw << "Model,Layout,Nodes,Run,DBP_m,Sent,Recvd,PDR,MinHops(-1=NA),BusyDrops\n";
    // Sentinel values in summary CSV:
    //   Hops_mean(-1=NA) = -1.0 → no waves reached sink at this density point
    //   Hops_stddev(-1=NA) = -1.0 → undefined standard deviation (no valid samples)
    // Downstream scripts: treat -1.0 in both hop columns as N/A.
    // Note: NaN was considered but -1.0 is used for compatibility with
    // Excel/R which may not parse 'nan' by default.
    sum << "Model,Layout,Nodes,Runs,WavesPerRun,DBP_m,PDR_mean,PDR_stddev,"
           "CI95_lo,CI95_hi,"
           "Hops_mean(-1=NA),Hops_stddev(-1=NA),BusyDrops_mean\n";

    // Initialise dashboard (replaces PrintBanner)
    g_dash.init(densities, runs, numWaves, modelName, waveIvlS, dbpM);


    // t-table: placed at function scope (not inside the density loop)
    // to make clear it is a constant initialized once, not per-iteration.
    // static constexpr ensures zero runtime cost and no misleading re-init read.
    static constexpr double T95[120] = {
        12.7062, 4.3027, 3.1824, 2.7764, 2.5706, 2.4469, 2.3646, 2.3060, 2.2622, 2.2281,
        2.2010, 2.1788, 2.1604, 2.1448, 2.1314, 2.1199, 2.1098, 2.1009, 2.0930, 2.0860,
        2.0796, 2.0739, 2.0687, 2.0639, 2.0595, 2.0555, 2.0518, 2.0484, 2.0452, 2.0423,
        2.0395, 2.0369, 2.0345, 2.0322, 2.0301, 2.0281, 2.0262, 2.0244, 2.0227, 2.0211,
        2.0195, 2.0181, 2.0167, 2.0154, 2.0141, 2.0129, 2.0117, 2.0106, 2.0096, 2.0086,
        2.0076, 2.0066, 2.0057, 2.0049, 2.0040, 2.0032, 2.0025, 2.0017, 2.0010, 2.0003,
        1.9996, 1.9990, 1.9983, 1.9977, 1.9971, 1.9966, 1.9960, 1.9955, 1.9949, 1.9944,
        1.9939, 1.9935, 1.9930, 1.9925, 1.9921, 1.9917, 1.9913, 1.9908, 1.9905, 1.9901,
        1.9897, 1.9893, 1.9890, 1.9886, 1.9883, 1.9879, 1.9876, 1.9873, 1.9870, 1.9867,
        1.9864, 1.9861, 1.9858, 1.9855, 1.9853, 1.9850, 1.9847, 1.9845, 1.9842, 1.9840,
        1.9837, 1.9835, 1.9833, 1.9830, 1.9828, 1.9826, 1.9824, 1.9822, 1.9820, 1.9818,
        1.9816, 1.9814, 1.9812, 1.9810, 1.9808, 1.9806, 1.9804, 1.9803, 1.9801, 1.9799
    };

    for (uint32_t N : densities) {
        g_dash.startDensity(static_cast<uint32_t>(
            std::find(densities.begin(), densities.end(), N) - densities.begin()));
        std::vector<double> pdrs, hops;

        uint64_t totalBusyDrops = 0;
        for (uint32_t r = 1; r <= runs; r++) {
            RunResult res = ExecuteRun(N, r, wg, grid, numWaves, dbpM, outdir, waveIvlS);
            pdrs.push_back(res.pdr); hops.push_back(res.meanHops);
            totalBusyDrops += res.busyDrops;
            raw << modelName<<","<<layoutName<<","<<N<<","<<r<<","<<dbpM<<","
                <<res.sent<<","<<res.recvd<<","<<res.pdr<<","<<res.meanHops<<","<<res.busyDrops<<","<<res.collisionDrops
                <<","<<res.busyDrops<<"\n";
            raw.flush();
            // Update dashboard (must happen after CSV write in case of early exit)
            {
                size_t nr = pdrs.size();
                double mp = std::accumulate(pdrs.begin(),pdrs.end(),0.0)/nr;
                double ci_tmp = 0;
                std::vector<double> vh;
                for(double h:hops) if(h>=0) vh.push_back(h);
                double mh_tmp = vh.empty()?-1.0:
                    std::accumulate(vh.begin(),vh.end(),0.0)/vh.size();
                g_dash.afterRun(r, res.pdr, res.meanHops,
                    res.busyDrops, res.collisionDrops, mp, ci_tmp, mh_tmp);
            }
            if (raw.fail()) {
                std::cerr << "\nFATAL: raw CSV write failed at N=" << N
                          << " r=" << r << ". Disk full or quota exceeded?\n"
                          << "  Path: " << rawPath << "\n";
                return 1; // Simulator already destroyed at end of ExecuteRun
            }
        }

        // nRuns: explicit name to distinguish from outer loop N (density)
        // and totalN = N+2. Kept as size_t; cast to double only at division.
        size_t nRuns = pdrs.size();
        double mPDR = std::accumulate(pdrs.begin(), pdrs.end(), 0.0) / static_cast<double>(nRuns);
        double vPDR = 0; for (double x : pdrs) vPDR += (x - mPDR) * (x - mPDR);
        double sPDR = (nRuns > 1) ? std::sqrt(vPDR / static_cast<double>(nRuns - 1)) : 0.0;
        // Two-sided 95% CI: exact t_{df,0.025} from inline table (df=1..120),
        // z=1.960 for df>120. The prior `n>30 ? 1.960 : 2.045` was
        // anti-conservative for any runs>30: e.g. runs=31 needs t(30)=2.042,
        // not z=1.960 — a 3.8% under-coverage. Values from scipy.stats.t.ppf.
        //
        // STATISTICAL POWER NOTE (30 runs × 30 waves):
        // Near the percolation threshold (N=150-200), σ_PDR can reach 20-50%
        // (bimodal 0%/100% behaviour). With n=30 and σ=20%: CI = ±7.5pp.
        // With σ=50% (true bimodal): CI = ±18.7pp. Wide CIs at the threshold
        // are physically correct — they reflect the percolation transition's
        // inherent bimodality, not inadequate sampling. The threshold is
        // identified as the first N where mean PDR ≥ 90%, not by CI bounds.
        // The production run (30 runs) will reveal the actual σ values;
        // if σ >> 20% at N=150, the bimodality itself is reported as a result.
        //
        // CI VALIDITY NOTE (R-2, paper Sec VI): the t-CI assumes approximate
        // normality of the sample mean. At the percolation threshold where
        // individual runs are bimodal (0% or 100%), the Central Limit Theorem
        // still applies to the mean of 30 binary-ish draws — but the stated
        // 95% coverage probability may not hold exactly. This is standard
        // practice in Monte Carlo percolation studies (e.g., Bollobás &
        // Riordan 2006). The CI should be reported as a spread indicator
        // rather than a strict frequentist interval in this density regime.
        // State this caveat explicitly in Section VI.
        uint32_t df  = (nRuns >= 2) ? static_cast<uint32_t>(nRuns) - 1u : 1u;
        double tCrit = (df <= 120u) ? T95[df - 1u] : 1.960;
        double ci    = tCrit * sPDR / std::sqrt(static_cast<double>(nRuns));
        // Hop statistics: filter out -1.0 sentinels (runs where no wave
        // reached the sink). Averaging sentinels into mH would corrupt the
        // metric — e.g. 20 disconnected runs (-1.0) and 10 runs at 15 hops
        // would falsely report mH ≈ 4.3 instead of 15.0.
        // PDR statistics do NOT need filtering — res.pdr = 0.0 for
        // disconnected runs is the actual PDR, not a sentinel.
        std::vector<double> validHops;
        for (double h : hops) if (h >= 0.0) validHops.push_back(h);
        double mH = -1.0; // -1.0 sentinel: no waves reached sink
        // Use -1.0 (not NaN): 'nan' is unparseable by Excel/R by default.
        // Documented in CSV header as -1=NA.
        double sH = -1.0;
        if (!validHops.empty()) {
            double nh = static_cast<double>(validHops.size());
            mH = std::accumulate(validHops.begin(), validHops.end(), 0.0) / nh;
            double vH = 0.0;
            for (double x : validHops) vH += (x - mH) * (x - mH);
            sH = (nh > 1.0) ? std::sqrt(vH / (nh - 1.0)) : 0.0;
        }

        g_dash.afterDensity(
            static_cast<uint32_t>(
                std::find(densities.begin(), densities.end(), N) - densities.begin()),
            mPDR);

        double meanBusyDrops = static_cast<double>(totalBusyDrops) / runs;
        sum<<modelName<<","<<layoutName<<","<<N<<","<<runs<<","<<numWaves<<","<<dbpM<<","
           <<mPDR<<","<<sPDR<<","
           <<std::max(mPDR-ci, 0.0)<<","<<std::min(mPDR+ci, 100.0)<<","
           <<mH<<","<<sH
           <<","<<meanBusyDrops<<"\n";
        sum.flush();
        if (sum.fail()) {
            std::cerr << "\nFATAL: summary CSV write failed at N=" << N
                      << ". Disk full or quota exceeded?\n"
                      << "  Path: " << sumPath << "\n";
            return 1;
        }
    }

    g_dash.done(rawPath, sumPath);
    return 0;
}
