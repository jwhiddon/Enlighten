// ENLIGHTEN SEQUENCE GENERATOR
//
// Composes an aesthetically structured poofer show of a given duration and
// poofer count, validates it against the firmware's physical duty limits
// (using the same constants and invariant checker as the firmware/tests),
// and writes:
//
//   <name>.show  - text timeline for tools/showplay (preview + validation)
//   <name>.mid   - Standard MIDI File playable into the REAL rig through
//                  the firmware's MIDI input (notes 36..51 = poofers 1..16,
//                  CC23 sets flame duration, CC22 selects RAW mode).
//                  The file NEVER contains the arm CCs: arming is a human act.
//
// Usage:
//   seqgen --seconds 60 --poofers 16 [--seed 42] [--bpm 120] [--out myshow]
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <map>
#include <random>
#include <string>
#include <vector>

#include "core/config.h"
#include "core/timebase.h"
#include "sim.h"  // InvariantChecker — the same guard rails as the tests

namespace {

struct Event {
  uint32_t t;    // ms
  uint32_t dur;  // ms
  uint8_t ch;    // 0-based poofer
};

struct Options {
  uint32_t seconds = 60;
  uint32_t poofers = 16;
  uint32_t bpm = 120;
  uint32_t seed = 0;
  bool seed_set = false;
  std::string out = "show";
};

// ---------------------------------------------------------------- composer
class Composer {
 public:
  Composer(const Options& o)
      : N(o.poofers),
        total(o.seconds * 1000),
        beat(60000 / o.bpm),
        rng(o.seed) {}

  std::vector<Event> compose() {
    events.clear();
    const uint32_t finale_len = total >= 15000 ? 4200 : 1600;
    const uint32_t finale_start = total - finale_len - 500;
    const uint32_t intro_end = 400 + (finale_start - 400) * 22 / 100;
    const uint32_t dev_end = 400 + (finale_start - 400) * 62 / 100;

    uint32_t t = 400;  // a beat of darkness before the first flame
    while (t < intro_end) t += introFigure(t);
    while (t < dev_end) t += devFigure(t);
    while (t < finale_start) t += climaxFigure(t);
    finale(finale_start, finale_len);
    return events;
  }

 private:
  uint32_t N, total, beat;
  std::mt19937 rng;
  std::vector<Event> events;
  uint8_t recent[4] = {255, 255, 255, 255};
  int recent_i = 0;

  uint32_t rnd(uint32_t n) { return (uint32_t)(rng() % n); }
  static uint32_t clampDur(uint32_t d) {
    return d < 80 ? 80 : (d > cfg::MAX_POOF_MS ? cfg::MAX_POOF_MS : d);
  }
  void add(uint8_t ch, uint32_t t, uint32_t dur) {
    if (ch >= N) return;
    dur = clampDur(dur);
    if (t + dur + 400 > total) return;  // keep the ending dark
    events.push_back({t, dur, ch});
  }

  // ---- figure library ------------------------------------------------
  // Each figure writes events starting at t and returns the ms consumed
  // (including its trailing breath).

  uint32_t sweep(uint32_t t, bool up, uint32_t step) {
    uint32_t dur = clampDur(step * 3 / 4);
    for (uint32_t i = 0; i < N; ++i)
      add((uint8_t)(up ? i : N - 1 - i), t + i * step, dur);
    return N * step + beat / 2;
  }

  uint32_t bounce(uint32_t t, uint32_t step) {
    uint32_t dur = clampDur(step * 3 / 4);
    uint32_t steps = N > 1 ? 2 * N - 2 : 1;
    for (uint32_t i = 0; i <= steps; ++i) {
      uint32_t phase = i % (2 * N - 2 == 0 ? 1 : 2 * N - 2);
      uint32_t ch = phase < N ? phase : (2 * N - 2 - phase);
      add((uint8_t)ch, t + i * step, dur);
    }
    return (steps + 1) * step + beat / 2;
  }

  uint32_t mirror(uint32_t t, bool inward, uint32_t step) {
    uint32_t pairs = (N + 1) / 2;
    uint32_t dur = clampDur(step * 3 / 4);
    for (uint32_t i = 0; i < pairs; ++i) {
      uint32_t a = inward ? i : pairs - 1 - i;
      add((uint8_t)a, t + i * step, dur);
      if (N - 1 - a != a) add((uint8_t)(N - 1 - a), t + i * step, dur);
    }
    return pairs * step + beat / 2;
  }

  uint32_t alternate(uint32_t t, uint32_t reps, uint32_t period) {
    uint32_t dur = clampDur(period * 2 / 5);
    for (uint32_t r = 0; r < reps; ++r) {
      for (uint32_t c = 0; c < N; c += 2) add((uint8_t)c, t + r * period, dur);
      for (uint32_t c = 1; c < N; c += 2)
        add((uint8_t)c, t + r * period + period / 2, dur);
    }
    return reps * period + beat / 2;
  }

  uint32_t callResponse(uint32_t t, uint32_t reps, uint32_t period) {
    uint32_t half = N / 2 ? N / 2 : 1;
    uint32_t dur = clampDur(period * 2 / 5);
    for (uint32_t r = 0; r < reps; ++r) {
      for (uint32_t c = 0; c < half; ++c) add((uint8_t)c, t + r * period, dur);
      for (uint32_t c = half; c < N; ++c)
        add((uint8_t)c, t + r * period + period / 2, dur);
    }
    return reps * period + beat / 2;
  }

  uint32_t sparkle(uint32_t t, uint32_t beats) {
    uint32_t grid = beat / 2;
    uint32_t dur = clampDur(beat / 3);
    for (uint32_t g = 0; g < beats * 2; ++g) {
      if (rnd(100) < 45) {
        uint8_t ch;
        int guard = 0;
        do {
          ch = (uint8_t)rnd(N);
        } while (++guard < 8 &&
                 (ch == recent[0] || ch == recent[1] || ch == recent[2] ||
                  ch == recent[3]));
        recent[recent_i++ & 3] = ch;
        add(ch, t + g * grid, dur);
      }
    }
    return beats * beat + beat / 2;
  }

  uint32_t wave(uint32_t t, bool up, uint32_t step) {
    // Overlapping chase: each flame overlaps the next, reads as motion.
    uint32_t dur = clampDur(step * 9 / 5);
    for (uint32_t i = 0; i < N; ++i)
      add((uint8_t)(up ? i : N - 1 - i), t + i * step, dur);
    return N * step + beat / 2;
  }

  uint32_t buildup(uint32_t t) {
    // Accelerating sweep — a crescendo into whatever follows.
    uint32_t at = t;
    for (uint32_t i = 0; i < N; ++i) {
      uint32_t step = beat * (N - i) / N;
      if (step < 90) step = 90;
      add((uint8_t)i, at, clampDur(step * 3 / 4));
      at += step;
    }
    return (at - t) + beat / 2;
  }

  uint32_t allHit(uint32_t t, uint32_t dur) {
    for (uint32_t c = 0; c < N; ++c) add((uint8_t)c, t, dur);
    return dur + beat;
  }

  uint32_t rest(uint32_t beats) { return beats * beat; }

  // ---- sections --------------------------------------------------------
  uint32_t introFigure(uint32_t t) {
    switch (rnd(5)) {
      case 0: return sparkle(t, 2 + rnd(3));
      case 1: return sweep(t, rnd(2) == 0, beat);
      case 2: return mirror(t, true, beat);
      case 3: return rest(1 + rnd(2));
      default: return sparkle(t, 2);
    }
  }

  uint32_t devFigure(uint32_t t) {
    switch (rnd(8)) {
      case 0: return sweep(t, true, beat / 2);
      case 1: return sweep(t, false, beat / 2);
      case 2: return bounce(t, beat / 2);
      case 3: return mirror(t, true, beat / 2);
      case 4: return mirror(t, false, beat / 2);
      case 5: return callResponse(t, 2 + rnd(3), beat);
      case 6: return alternate(t, 2 + rnd(2), beat);
      default: return sparkle(t, 2) ;
    }
  }

  uint32_t climaxFigure(uint32_t t) {
    uint32_t third = beat / 3 < 90 ? 90 : beat / 3;
    switch (rnd(6)) {
      case 0: return wave(t, true, beat / 2);
      case 1: return wave(t, false, beat / 2);
      case 2: return sweep(t, rnd(2) == 0, third);
      case 3: return alternate(t, 3, beat / 2 < 180 ? 180 : beat / 2);
      case 4: return buildup(t);
      default: return mirror(t, rnd(2) == 0, third);
    }
  }

  void finale(uint32_t t, uint32_t len) {
    if (len >= 4200) {
      allHit(t, 350);
      allHit(t + 800, 350);
      allHit(t + 1600, 350);
      // the big one
      allHit(t + 2700, cfg::MAX_POOF_MS);
    } else {
      allHit(t, cfg::MAX_POOF_MS);
    }
  }
};

// ---------------------------------------------------------------- validator
// Enforce the firmware's physical limits so the generated show plays
// VERBATIM: nothing for the duty limiter to clip.
std::vector<Event> enforceLimits(std::vector<Event> ev, uint32_t* dropped) {
  std::stable_sort(ev.begin(), ev.end(),
                   [](const Event& a, const Event& b) { return a.t < b.t; });
  *dropped = 0;
  std::vector<Event> keep;
  uint32_t last_end[16];
  std::vector<Event> accepted[16];
  for (auto& le : last_end) le = 0;

  const uint32_t GAP = cfg::MIN_CLOSE_MS + 10;         // margin over 50 ms
  const uint32_t BUDGET = cfg::MAX_OPEN_PER_WINDOW_MS - 200;  // margin

  for (auto& e : ev) {
    if (e.dur > cfg::MAX_POOF_MS) e.dur = cfg::MAX_POOF_MS;
    // Minimum gap between opens on the same channel.
    if (!accepted[e.ch].empty() && e.t < last_end[e.ch] + GAP) {
      ++*dropped;
      continue;
    }
    // Sliding duty budget: open ms in the 10 s ending at this event's end.
    uint32_t window_start = e.t + e.dur > cfg::DUTY_WINDOW_MS
                                ? e.t + e.dur - cfg::DUTY_WINDOW_MS
                                : 0;
    uint32_t used = e.dur;
    for (auto it = accepted[e.ch].rbegin(); it != accepted[e.ch].rend(); ++it) {
      uint32_t end = it->t + it->dur;
      if (end <= window_start) break;
      uint32_t begin = it->t > window_start ? it->t : window_start;
      used += end - begin;
    }
    if (used > BUDGET) {
      ++*dropped;
      continue;
    }
    accepted[e.ch].push_back(e);
    last_end[e.ch] = e.t + e.dur;
    keep.push_back(e);
  }
  return keep;
}

// Build the (time, mask) state-change timeline from events.
std::vector<std::pair<uint32_t, uint16_t>> toTimeline(
    const std::vector<Event>& ev) {
  std::map<uint32_t, std::pair<uint16_t, uint16_t>> edges;  // t -> (set, clear)
  for (const auto& e : ev) {
    edges[e.t].first |= (uint16_t)(1u << e.ch);
    edges[e.t + e.dur].second |= (uint16_t)(1u << e.ch);
  }
  std::vector<std::pair<uint32_t, uint16_t>> tl;
  uint16_t mask = 0;
  for (const auto& kv : edges) {
    uint16_t next = (uint16_t)((mask & ~kv.second.second) | kv.second.first);
    if (next != mask || tl.empty()) tl.push_back({kv.first, next});
    mask = next;
  }
  return tl;
}

// Final proof: replay every millisecond through the tests' invariant
// checker.  If this fails, the generator has a bug — refuse to emit.
bool verify(const std::vector<std::pair<uint32_t, uint16_t>>& tl,
            uint32_t total_ms, const char** msg) {
  InvariantChecker chk;
  size_t i = 0;
  uint16_t mask = 0;
  for (uint32_t t = 1; t <= total_ms; ++t) {
    while (i < tl.size() && tl[i].first <= t) mask = tl[i++].second;
    chk.observe(SafetyState::ARMED, true, mask, t);
    if (!chk.ok()) {
      *msg = chk.message();
      return false;
    }
  }
  return true;
}

// ---------------------------------------------------------------- writers
void writeShow(const std::string& path,
               const std::vector<std::pair<uint32_t, uint16_t>>& tl,
               const Options& o) {
  FILE* f = std::fopen(path.c_str(), "w");
  if (!f) {
    std::fprintf(stderr, "cannot write %s\n", path.c_str());
    std::exit(2);
  }
  std::fprintf(f, "# ENLIGHTEN SHOW v1\n");
  std::fprintf(f, "# poofers=%u duration_ms=%u bpm=%u seed=%u\n", o.poofers,
               o.seconds * 1000, o.bpm, o.seed);
  std::fprintf(f, "# lines: <time_ms> <mask_hex>  (bit i = poofer i+1 open)\n");
  for (const auto& p : tl) std::fprintf(f, "%u %04X\n", p.first, p.second);
  std::fprintf(f, "%u 0000\n", o.seconds * 1000);
  std::fclose(f);
}

struct MidiWriter {
  std::vector<uint8_t> trk;
  uint32_t last_tick = 0;
  uint32_t bpm;

  explicit MidiWriter(uint32_t b) : bpm(b) {}
  uint32_t ticks(uint32_t ms) const {
    return (uint32_t)((uint64_t)ms * bpm / 125u);  // 480 tpqn
  }
  void vlq(uint32_t v) {
    uint8_t buf[4];
    int n = 0;
    buf[n++] = (uint8_t)(v & 0x7F);
    while (v >>= 7) buf[n++] = (uint8_t)((v & 0x7F) | 0x80);
    while (n) trk.push_back(buf[--n]);
  }
  void delta(uint32_t tick) {
    vlq(tick - last_tick);
    last_tick = tick;
  }
  void event(uint32_t tick, uint8_t a, uint8_t b, uint8_t c) {
    delta(tick);
    trk.push_back(a);
    trk.push_back(b);
    trk.push_back(c);
  }
  void write(const std::string& path) {
    // tempo meta at t=0 was pushed by caller; append end-of-track
    vlq(0);
    trk.push_back(0xFF);
    trk.push_back(0x2F);
    trk.push_back(0x00);
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) {
      std::fprintf(stderr, "cannot write %s\n", path.c_str());
      std::exit(2);
    }
    auto be16 = [&](uint16_t v) {
      std::fputc(v >> 8, f);
      std::fputc(v & 0xFF, f);
    };
    auto be32 = [&](uint32_t v) {
      for (int s = 24; s >= 0; s -= 8) std::fputc((v >> s) & 0xFF, f);
    };
    std::fwrite("MThd", 1, 4, f);
    be32(6);
    be16(0);    // format 0
    be16(1);    // one track
    be16(480);  // ticks per quarter note
    std::fwrite("MTrk", 1, 4, f);
    be32((uint32_t)trk.size());
    std::fwrite(trk.data(), 1, trk.size(), f);
    std::fclose(f);
  }
};

void writeMidi(const std::string& path, std::vector<Event> ev,
               const Options& o) {
  // Firmware RAW mode: flame duration = CC23, not note length.  Emit CC23
  // before any note whose duration differs from the current value, and a
  // 1 Hz CC23 refresh as a keepalive (the firmware disarms after 2 s of
  // MIDI silence; DAWs don't send Active Sensing).
  struct MEv {
    uint32_t t;
    int order;  // 0=CC, 1=note-off, 2=note-on
    uint8_t s, d1, d2;
  };
  std::vector<MEv> mev;
  auto durToCc = [](uint32_t dur) {
    if (dur < cfg::MIN_POOF_MS) dur = cfg::MIN_POOF_MS;
    if (dur > cfg::MAX_POOF_MS) dur = cfg::MAX_POOF_MS;
    return (uint8_t)((dur - cfg::MIN_POOF_MS) * 127u /
                     (cfg::MAX_POOF_MS - cfg::MIN_POOF_MS));
  };

  std::stable_sort(ev.begin(), ev.end(),
                   [](const Event& a, const Event& b) { return a.t < b.t; });
  int current_cc = -1;
  for (const auto& e : ev) {
    uint8_t cc = durToCc(e.dur);
    if ((int)cc != current_cc) {
      mev.push_back({e.t, 0, 0xB0, 23, cc});
      current_cc = cc;
    }
    uint8_t note = (uint8_t)(cfg::MIDI_NOTE_FIRST + e.ch);
    mev.push_back({e.t, 2, 0x90, note, 127});
    mev.push_back({e.t + e.dur, 1, 0x80, note, 0});
  }
  // Keepalive refresh every second.
  uint32_t total = o.seconds * 1000;
  for (uint32_t t = 1000; t < total; t += 1000)
    mev.push_back({t, 0, 0xB0, 25, 64});  // CC25 (rate) = 64: harmless in RAW

  std::stable_sort(mev.begin(), mev.end(), [](const MEv& a, const MEv& b) {
    return a.t != b.t ? a.t < b.t : a.order < b.order;
  });

  MidiWriter w(o.bpm);
  // Tempo meta.
  uint32_t uspq = 60000000u / o.bpm;
  w.vlq(0);
  w.trk.push_back(0xFF);
  w.trk.push_back(0x51);
  w.trk.push_back(0x03);
  w.trk.push_back((uint8_t)(uspq >> 16));
  w.trk.push_back((uint8_t)(uspq >> 8));
  w.trk.push_back((uint8_t)uspq);
  // RAW mode + repeat off.  NO arm CCs — the operator arms by hand.
  w.event(0, 0xB0, 22, 125);
  w.event(0, 0xB0, 26, 0);
  for (const auto& m : mev) w.event(w.ticks(m.t), m.s, m.d1, m.d2);
  w.event(w.ticks(total), 0xB0, 123, 0);  // parting all-notes-off
  w.write(path);
}

void usage(const char* argv0) {
  std::fprintf(stderr,
               "usage: %s --seconds N --poofers M [--seed S] [--bpm B] "
               "[--out NAME]\n"
               "  --seconds  show length, 10..3600\n"
               "  --poofers  poofers used, 2..16\n"
               "  --seed     RNG seed (default: random — print for reuse)\n"
               "  --bpm      groove tempo, 60..180 (default 120)\n"
               "  --out      output basename (default 'show')\n",
               argv0);
  std::exit(1);
}

}  // namespace

int main(int argc, char** argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    auto need = [&](const char* flag) -> const char* {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "%s needs a value\n", flag);
        usage(argv[0]);
      }
      return argv[++i];
    };
    if (!std::strcmp(argv[i], "--seconds")) o.seconds = (uint32_t)std::atoi(need("--seconds"));
    else if (!std::strcmp(argv[i], "--poofers")) o.poofers = (uint32_t)std::atoi(need("--poofers"));
    else if (!std::strcmp(argv[i], "--seed")) { o.seed = (uint32_t)std::strtoul(need("--seed"), nullptr, 0); o.seed_set = true; }
    else if (!std::strcmp(argv[i], "--bpm")) o.bpm = (uint32_t)std::atoi(need("--bpm"));
    else if (!std::strcmp(argv[i], "--out")) o.out = need("--out");
    else usage(argv[0]);
  }
  if (o.seconds < 10 || o.seconds > 3600 || o.poofers < 2 || o.poofers > 16 ||
      o.bpm < 60 || o.bpm > 180)
    usage(argv[0]);
  if (!o.seed_set) o.seed = (uint32_t)std::time(nullptr) ^ 0x9E3779B9u;

  Composer c(o);
  std::vector<Event> ev = c.compose();
  uint32_t dropped = 0;
  ev = enforceLimits(ev, &dropped);

  auto tl = toTimeline(ev);
  const char* msg = "";
  if (!verify(tl, o.seconds * 1000, &msg)) {
    std::fprintf(stderr, "INTERNAL: generated show violates '%s' — not written\n", msg);
    return 2;
  }

  std::string show_path = o.out + ".show";
  std::string midi_path = o.out + ".mid";
  writeShow(show_path, tl, o);
  writeMidi(midi_path, ev, o);

  uint32_t per_ch[16] = {};
  for (const auto& e : ev) ++per_ch[e.ch];
  std::printf("ENLIGHTEN seqgen: %us show for %u poofers  (bpm %u, seed %u)\n",
              o.seconds, o.poofers, o.bpm, o.seed);
  std::printf("  %zu flame events (%u trimmed by the duty pre-check)\n",
              ev.size(), dropped);
  std::printf("  per poofer:");
  for (uint32_t i = 0; i < o.poofers; ++i) std::printf(" %u", per_ch[i]);
  std::printf("\n  verified against firmware limits: OK (plays verbatim)\n");
  std::printf("  wrote %s (preview: showplay %s)\n", show_path.c_str(),
              show_path.c_str());
  std::printf("  wrote %s (play into the rig over MIDI - arm by hand first)\n",
              midi_path.c_str());
  return 0;
}
