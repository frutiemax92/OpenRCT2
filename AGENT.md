# Agent instructions

## Models and cost

Use Sonnet 5.5 (`claude-sonnet-5-5`) by default for all work, to keep token costs low.
When you hit a problem or an open question you can't resolve with reasonable effort (a failing test with an unclear cause, a tricky bug, an architecture or design decision, a repeated failed attempt), launch a subagent with the Agent tool using `model: "opus"` (Opus 5.5, `claude-opus-5-5`) to address it.

- Give the subagent a self-contained prompt: the problem, what you already tried, relevant file paths and error output.
- Use it only for the hard part; take its answer back and continue the routine work (edits, builds, commits) yourself on Sonnet.
- Don't spawn Opus for routine or well-understood tasks.

## Git workflow

This repository is the Open Golf Simulator, built on top of OpenRCT2.
Work happens on the local branch `golf-develop`, which tracks `develop` on the `golf` remote:

- Remote `golf`: https://github.com/frutiemax92/open-golf-simulator (branch `develop`)
- Do **not** push to `origin` (OpenRCT2/OpenRCT2) or `fork` (frutiemax92/OpenRCT2).

After every important feature or fix:

1. Write unit tests for it (see "Unit tests" below).
2. Build and run the **whole** test suite, not only the new tests. Do not commit if anything fails to build or any test fails; fix it first.
3. Stage only the files that belong to that change (no build output, logs or unrelated files).
4. Commit with a clear message describing what changed and why. Include the tests in the same commit as the feature or fix.
5. Push to develop: `git push golf golf-develop:develop`

Keep one commit per feature or fix; don't bundle unrelated changes together.
If the push is rejected (remote has new commits), run `git pull --rebase golf develop`, resolve any conflicts, and push again. Never force-push.

## Unit tests

Every new feature needs unit tests, and every bug fix needs a regression test that fails without the fix.

- Tests use GoogleTest and live in `test/tests/`. Add new test files to the `test_files` list in `test/tests/CMakeLists.txt`.
- Cover normal behaviour, edge cases and error handling, not just the happy path.
- Tests are off by default. Configure the build with them enabled:
  `cmake -B build -DWITH_TESTS=ON`
- Build and run all tests before every commit:
  `cmake --build build && ctest --test-dir build --output-on-failure`
