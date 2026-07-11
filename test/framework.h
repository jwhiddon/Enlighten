// Minimal assert-based test framework — no dependencies.
#pragma once
#include <cstdio>

namespace testfw {

struct Case {
  const char* name;
  void (*fn)();
};

inline Case* cases() {
  static Case c[512];
  return c;
}
inline int& caseCount() {
  static int n = 0;
  return n;
}
inline int& failures() {
  static int f = 0;
  return f;
}
inline const char*& currentTest() {
  static const char* t = "";
  return t;
}

inline void expect(bool ok, const char* expr, const char* file, int line) {
  if (!ok) {
    ++failures();
    std::printf("FAIL [%s] %s:%d: %s\n", currentTest(), file, line, expr);
  }
}

inline int runAll() {
  for (int i = 0; i < caseCount(); ++i) {
    currentTest() = cases()[i].name;
    cases()[i].fn();
  }
  if (failures()) {
    std::printf("\n%d FAILURE(S) across %d tests\n", failures(), caseCount());
    return 1;
  }
  std::printf("ALL %d TESTS PASSED\n", caseCount());
  return 0;
}

struct Registrar {
  Registrar(const char* n, void (*f)()) {
    cases()[caseCount()++] = {n, f};
  }
};

}  // namespace testfw

#define TEST(name)                                        \
  static void testbody_##name();                          \
  static testfw::Registrar testreg_##name(#name, testbody_##name); \
  static void testbody_##name()

#define EXPECT_TRUE(x) testfw::expect(!!(x), #x, __FILE__, __LINE__)
#define EXPECT_FALSE(x) testfw::expect(!(x), "!(" #x ")", __FILE__, __LINE__)
#define EXPECT_EQ(a, b) \
  testfw::expect((a) == (b), #a " == " #b, __FILE__, __LINE__)
