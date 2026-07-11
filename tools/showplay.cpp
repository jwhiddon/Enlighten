// ENLIGHTEN SHOW PLAYER / VALIDATOR
//
// Plays a .show file (from tools/seqgen) as a terminal visualization and
// replays every millisecond through the REAL SafetySupervisor to prove the
// rig would play it verbatim — if the safety filter would clip anything,
// this reports it.
//
// Usage:
//   showplay <file.show>            fast printout + validation
//   showplay <file.show> --animate  real-time terminal animation
//   showplay <file.show> --speed 4  animation speed multiplier
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "core/config.h"
#include "core/safety.h"
#include "core/timebase.h"
#include "sim.h"

namespace {

struct Change {
  uint32_t t;
  uint16_t mask;
};

void maskBar(uint16_t m, char* out) {
  int pos = 0;
  out[pos++] = '[';
  for (int i = 0; i < 16; ++i) {
    if (i == 8) out[pos++] = '|';
    out[pos++] = ((m >> i) & 1) ? '#' : '.';
  }
  out[pos++] = ']';
  out[pos] = 0;
}

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
    unsigned t;
    unsigned m;
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

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <file.show> [--animate] [--speed X]\n",
                 argv[0]);
    return 1;
  }
  bool animate = false;
  double speed = 1.0;
  for (int i = 2; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--animate")) animate = true;
    else if (!std::strcmp(argv[i], "--speed") && i + 1 < argc)
      speed = std::atof(argv[++i]);
  }
  if (speed <= 0) speed = 1.0;

  uint32_t poofers = 16;
  std::vector<Change> tl = load(argv[1], &poofers);
  uint32_t total = tl.back().t;

  // ---- validation replay through the real safety core -------------------
  SafetySupervisor safety;
  InvariantChecker checker;
  ShowInput in;
  in.link_ok = true;
  HwInputs hw{true, true};
  TimeMs now = 0;
  safety.begin(FaultCode::NONE, now);
  // Arm exactly like an operator: edge, handshake, hold.
  for (int i = 0; i < 5; ++i) safety.filter(0, in, hw, ++now);
  in.arm_a = in.arm_b = true;
  for (uint32_t i = 0; i <= cfg::ARM_HOLD_MS; ++i) safety.filter(0, in, hw, ++now);

  size_t idx = 0;
  uint16_t mask = 0;
  uint32_t clipped_ms = 0;
  uint32_t open_ms = 0;
  uint32_t poof_edges[16] = {};
  uint16_t prev = 0;
  uint32_t t0 = now;
  for (uint32_t t = 0; t <= total; ++t) {
    while (idx < tl.size() && tl[idx].t <= t) mask = tl[idx++].mask;
    uint16_t out = safety.filter(mask, in, hw, ++now);
    checker.observe(safety.state(), hw.estop_ok, out, now);
    if (out != mask) ++clipped_ms;
    for (int c = 0; c < 16; ++c) {
      if ((out >> c) & 1) {
        ++open_ms;
        if (!((prev >> c) & 1)) ++poof_edges[c];
      }
    }
    prev = out;
  }
  (void)t0;

  // ---- playback ---------------------------------------------------------
  std::printf("ENLIGHTEN showplay: %s  (%u poofers, %.1f s)\n", argv[1],
              poofers, total / 1000.0);
  char bar[20];
  if (animate) {
    uint32_t t = 0;
    for (size_t i = 0; i < tl.size(); ++i) {
      uint32_t wait = tl[i].t - t;
      if (wait)
        std::this_thread::sleep_for(
            std::chrono::microseconds((int64_t)(wait * 1000 / speed)));
      t = tl[i].t;
      maskBar(tl[i].mask, bar);
      std::printf("\r  t=%7.2fs  %s   ", t / 1000.0, bar);
      std::fflush(stdout);
    }
    std::printf("\n");
  } else {
    for (const auto& c : tl) {
      maskBar(c.mask, bar);
      std::printf("  t=%7u ms  %s\n", c.t, bar);
    }
  }

  // ---- report -------------------------------------------------------------
  std::printf("\nsummary\n");
  std::printf("  duration: %.1f s, %zu state changes, flame time %.1f s\n",
              total / 1000.0, tl.size(), open_ms / 1000.0);
  std::printf("  poofs per poofer:");
  for (uint32_t c = 0; c < poofers; ++c) std::printf(" %u", poof_edges[c]);
  std::printf("\n");
  bool verbatim = clipped_ms == 0;
  std::printf("  safety-filter replay: %s\n",
              verbatim ? "VERBATIM (nothing clipped by duty limits)"
                       : "CLIPPED - the rig would modify this show!");
  if (!verbatim) std::printf("  clipped on %u ms\n", clipped_ms);
  std::printf("  invariants: %s\n",
              checker.ok() ? "OK" : checker.message());
  return (verbatim && checker.ok()) ? 0 : 1;
}
