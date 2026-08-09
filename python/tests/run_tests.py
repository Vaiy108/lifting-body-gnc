"""Minimal test runner (no pytest required): python tests/run_tests.py"""

import sys
import traceback

import test_phase1 as m1
import test_phase2 as m2


def main() -> int:
    tests = [(n, f) for mod in (m1, m2) for n, f in vars(mod).items()
             if n.startswith("test_") and callable(f)]
    failed = []
    for name, fn in tests:
        try:
            fn()
            print(f"PASS  {name}")
        except Exception:
            failed.append(name)
            print(f"FAIL  {name}")
            traceback.print_exc()
    print(f"\n{len(tests) - len(failed)}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
