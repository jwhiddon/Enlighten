// ENLIGHTEN SHOW VISUALIZER
//
// Animated terminal simulation of a .show file: each poofer is a column
// with a flame that whooshes up while its solenoid is open and decays
// after it closes — so you can watch and iterate on a show with nothing
// connected.  Before animating, the show is replayed through the REAL
// SafetySupervisor (same validation as tools/showplay).
//
// Usage:
//   showviz <file.show> [--speed X] [--turbo]
//     --speed X   playback speed multiplier (default 1.0)
//     --turbo     no frame delay (for scripted/CI runs)
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <vector>

#include "core/config.h"
#include "core/safety.h"
#include "core/timebase.h"
#include "sim.h"

// windows.h last: it #defines ALTERNATE (wingdi), which would otherwise
// collide with ModeId::ALTERNATE in the core headers.
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOGDI
#include <windows.h>
#endif

namespace {

struct Change {
  uint32_t t;
  uint16_t mask;
};

constexpr int MAXH = 6;         // flame height in rows
constexpr uint32_t FRAME_MS = 33;  // ~30 fps

std::vector<Change> load(const char* path, uint32_t* poofers) {
  FILE* f = std::fopen(path, "r");
  if (!f) {
    std::fprintf(stderr, "cannot open %s\n", path);
    std::exit(1);
  }
  std::vector<Change> tl;
  char line[128];
  *poofers = 16;
  while (std::fgets(line, sizeof(line), f)) {
    if (line[0] == '#') {
      const char* p = std::strstr(line, "poofers=");
      if (p) *poofers = (uint32_t)std::atoi(p + 8);
      continue;
    }
    unsigned t, m;
    if (std::sscanf(line, "%u %x", &t, &m) == 2)
      tl.push_back({(uint32_t)t, (uint16_t)m});
  }
  std::fclose(f);
  if (tl.empty()) {
    std::fprintf(stderr, "%s: no events\n", path);
    std::exit(1);
  }
  return tl;
}

// Replay through the real safety core; returns true if it plays verbatim.
bool validate(const std::vector<Change>& tl, bool* invariants_ok) {
  SafetySupervisor safety;
  InvariantChecker checker;
  ShowInput in;
  in.link_ok = true;
  HwInputs hw{true, true};
  TimeMs now = 0;
  safety.begin(FaultCode::NONE, now);
  for (int i = 0; i < 5; ++i) safety.filter(0, in, hw, ++now);
  in.arm_a = in.arm_b = true;
  for (uint32_t i = 0; i <= cfg::ARM_HOLD_MS; ++i)
    safety.filter(0, in, hw, ++now);

  size_t idx = 0;
  uint16_t mask = 0;
  uint32_t clipped = 0;
  for (uint32_t t = 0; t <= tl.back().t; ++t) {
    while (idx < tl.size() && tl[idx].t <= t) mask = tl[idx++].mask;
    uint16_t out = safety.filter(mask, in, hw, ++now);
    checker.observe(safety.state(), hw.estop_ok, out, now);
    if (out != mask) ++clipped;
  }
  *invariants_ok = checker.ok();
  return clipped == 0;
}

void enableVt() {
#ifdef _WIN32
  HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
  DWORD m = 0;
  if (h != INVALID_HANDLE_VALUE && GetConsoleMode(h, &m))
    SetConsoleMode(h, m | 0x0004 /*ENABLE_VIRTUAL_TERMINAL_PROCESSING*/);
#endif
}

// Flame cell by depth below the tip: tip is wispy red, base bright yellow.
const char* cellFor(int depth, int flicker) {
  switch (depth) {
    case 1:  return flicker ? "\x1b[31m &  " : "\x1b[31m .  ";
    case 2:  return "\x1b[91m *  ";
    case 3:  return "\x1b[33m ## ";
    default: return "\x1b[93m @@ ";
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <file.show> [--speed X] [--turbo]\n",
                 argv[0]);
    return 1;
  }
  double speed = 1.0;
  bool turbo = false;
  for (int i = 2; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--speed") && i + 1 < argc)
      speed = std::atof(argv[++i]);
    else if (!std::strcmp(argv[i], "--turbo"))
      turbo = true;
  }
  if (speed <= 0) speed = 1.0;

  uint32_t poofers = 16;
  std::vector<Change> tl = load(argv[1], &poofers);
  uint32_t total = tl.back().t;

  bool invariants_ok = false;
  bool verbatim = validate(tl, &invariants_ok);

  // Whole-show totals for the stats line.
  uint32_t total_poofs = 0;
  {
    uint16_t prev = 0;
    for (const auto& c : tl) {
      total_poofs += (uint32_t)__builtin_popcount((unsigned)(c.mask & ~prev));
      prev = c.mask;
    }
  }

  enableVt();
  std::printf("\x1b[2J\x1b[?25l");  // clear screen, hide cursor

  int heights[16] = {};
  uint32_t rng = 0xBADC0DE5;
  size_t idx = 0;
  uint16_t mask = 0;
  uint16_t prev_mask = 0;
  double show_t = 0;

  // Operator stats accumulated as the show plays.
  uint32_t fires[16] = {};      // rising edges per poofer
  double flame_ms = 0;          // cumulative open channel-time
  // Rolling 10 s duty tracking: 100 x 100 ms buckets per channel.
  constexpr int DBUCKETS = 100;
  double duty[16][DBUCKETS] = {};
  int dbucket = 0;
  double dbucket_t = 0;

  auto anyFlame = [&] {
    for (uint32_t i = 0; i < poofers; ++i)
      if (heights[i]) return true;
    return false;
  };

  while (show_t <= total || anyFlame()) {
    uint32_t t = (uint32_t)show_t;
    while (idx < tl.size() && tl[idx].t <= t) mask = tl[idx++].mask;

    // Stats: rising edges, flame time, rolling duty buckets.
    double dt = FRAME_MS * speed;
    dbucket_t += dt;
    while (dbucket_t >= 100.0) {
      dbucket_t -= 100.0;
      dbucket = (dbucket + 1) % DBUCKETS;
      for (uint32_t i = 0; i < poofers; ++i) duty[i][dbucket] = 0;
    }
    for (uint32_t i = 0; i < poofers; ++i) {
      uint16_t b = (uint16_t)(1u << i);
      if ((mask & b) && !(prev_mask & b)) ++fires[i];
      if (mask & b) {
        flame_ms += dt;
        duty[i][dbucket] += dt;
      }
    }
    prev_mask = mask;

    for (uint32_t i = 0; i < poofers; ++i) {
      bool open = (mask >> i) & 1u;
      if (open) heights[i] = heights[i] + 2 > MAXH ? MAXH : heights[i] + 2;
      else if (heights[i]) --heights[i];
    }
    rng ^= rng << 13;
    rng ^= rng >> 17;
    rng ^= rng << 5;

    // Compose the frame.
    char frame[6144];
    int n = std::snprintf(frame, sizeof(frame),
                          "\x1b[H\x1b[0m ENLIGHTEN showviz  %-24s\n\n",
                          argv[1]);
    for (int r = MAXH - 1; r >= 0; --r) {
      n += std::snprintf(frame + n, sizeof(frame) - n, "  ");
      for (uint32_t i = 0; i < poofers; ++i) {
        int depth = heights[i] - r;
        if (depth <= 0) {
          n += std::snprintf(frame + n, sizeof(frame) - n, "    ");
        } else {
          int flick = (int)((rng >> (i & 15)) & 1u);
          n += std::snprintf(frame + n, sizeof(frame) - n, "%s",
                             cellFor(depth, flick));
        }
      }
      n += std::snprintf(frame + n, sizeof(frame) - n, "\x1b[0m\n");
    }
    // Burners and numbers.
    n += std::snprintf(frame + n, sizeof(frame) - n, "  \x1b[90m");
    for (uint32_t i = 0; i < poofers; ++i)
      n += std::snprintf(frame + n, sizeof(frame) - n, "====");
    n += std::snprintf(frame + n, sizeof(frame) - n, "\x1b[0m\n  ");
    for (uint32_t i = 0; i < poofers; ++i)
      n += std::snprintf(frame + n, sizeof(frame) - n, "%3u ", i + 1);
    // Per-poofer fire counters (dim), aligned under the numbers.
    n += std::snprintf(frame + n, sizeof(frame) - n, "\n  \x1b[90m");
    for (uint32_t i = 0; i < poofers; ++i)
      n += std::snprintf(frame + n, sizeof(frame) - n, "%3u ",
                         fires[i] > 999 ? 999 : fires[i]);
    n += std::snprintf(frame + n, sizeof(frame) - n, "\x1b[0m fires\n");

    // Timecode + progress.
    auto tc = [](double ms, char* buf, unsigned cap) {
      uint32_t v = (uint32_t)ms;
      std::snprintf(buf, cap, "%02u:%02u.%03u", v / 60000, (v / 1000) % 60,
                    v % 1000);
    };
    char tc_now[16], tc_total[16];
    tc(show_t > total ? (double)total : show_t, tc_now, sizeof(tc_now));
    tc((double)total, tc_total, sizeof(tc_total));
    double frac = total ? (show_t > total ? 1.0 : show_t / total) : 1.0;
    int fill = (int)(frac * 24);
    n += std::snprintf(frame + n, sizeof(frame) - n, "\n  TC %s / %s  [",
                       tc_now, tc_total);
    for (int i = 0; i < 24; ++i)
      n += std::snprintf(frame + n, sizeof(frame) - n, "%c",
                         i < fill ? '=' : ' ');
    n += std::snprintf(frame + n, sizeof(frame) - n, "]  x%.1f\n", speed);

    // Operator stats: open count, poof progress, flame time, worst rolling
    // duty (predicts limiter throttling), countdown to the next cue.
    uint32_t open_now = (uint32_t)__builtin_popcount((unsigned)mask);
    uint32_t fired_so_far = 0;
    for (uint32_t i = 0; i < poofers; ++i) fired_so_far += fires[i];
    double max_duty = 0;
    for (uint32_t i = 0; i < poofers; ++i) {
      double s = 0;
      for (int k = 0; k < DBUCKETS; ++k) s += duty[i][k];
      if (s > max_duty) max_duty = s;
    }
    char next[16];
    if (idx < tl.size())
      std::snprintf(next, sizeof(next), "%5.2fs", (tl[idx].t - show_t) / 1000.0);
    else
      std::snprintf(next, sizeof(next), "--");
    n += std::snprintf(frame + n, sizeof(frame) - n,
                       "  open %2u/%-2u  poofs %4u/%-4u  flame %6.1fs  "
                       "duty(10s max) %3.0f%%  next %s   \n",
                       open_now, poofers, fired_so_far, total_poofs,
                       flame_ms / 1000.0, 100.0 * max_duty / 10000.0, next);
    std::fputs(frame, stdout);
    std::fflush(stdout);

    if (!turbo)
      std::this_thread::sleep_for(std::chrono::milliseconds(FRAME_MS));
    show_t += FRAME_MS * speed;
  }

  std::printf("\x1b[?25h\x1b[0m\n");  // cursor back
  std::printf("summary: %.1f s, %zu state changes\n", total / 1000.0,
              tl.size());
  std::printf("  safety-filter replay: %s\n",
              verbatim ? "VERBATIM (nothing clipped by duty limits)"
                       : "CLIPPED - the rig would modify this show!");
  std::printf("  invariants: %s\n", invariants_ok ? "OK" : "VIOLATED");
  return (verbatim && invariants_ok) ? 0 : 1;
}
