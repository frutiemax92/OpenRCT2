# Gemini Agent Instructions

## Models and Cost

Use **Gemini Flash** (e.g. Gemini 3.8 Flash / Gemini 2.5 Flash) by default for all work to keep token costs minimal and execution fast.

When you hit a problem or an open question you cannot resolve with reasonable effort (e.g. a failing test with an unclear cause, a tricky memory/concurrency bug, an architectural or design decision, or repeated failed attempts), launch a subagent with the `invoke_subagent` tool using `Model: "pro"` (Gemini Pro) to resolve it:

- Give the subagent a self-contained prompt: the problem statement, what you already tried, relevant file paths, and exact error output.
- Use Gemini Pro only for the hard reasoning or design work. Bring its recommendation back to the primary Gemini Flash session to perform the routine implementation (code edits, builds, tests, and commits).
- Do not spawn Gemini Pro for routine, well-understood tasks.
- For long-running autonomous execution across multiple features or milestones, recommend the `/goal` command.

## Scope of Work: Implement Everything

The goal is to implement **all** features detailed in `specifications/detailed-specifications.md`, not just one.
Do not stop after a single feature or commit. Keep iterating, feature by feature, until every requirement is implemented or you are genuinely blocked.

1. Read the specification and follow the phase order in §25. Each phase's acceptance criteria must pass before starting the next, unless the phase is marked independent.
2. Pick the next unimplemented requirement (by ID, e.g. `TER-012`). Implement it, test it, commit it, and push it (see "Git Workflow"). Then immediately move on to the next requirement.
3. Check `git log` and existing code/tests to see what is already completed; requirement IDs in commit messages indicate progress. Do not redo completed work.
4. Treat a short status summary after each commit as a progress milestone, not as a stopping point. Do not ask "should I continue?"; the answer is always yes.
5. Only stop when:
   - Every requirement in the specification is implemented and its acceptance criteria pass, or
   - You are blocked on something only the user can resolve (e.g., missing asset files, credentials, or an architectural choice with no default in §26). Clearly document what is blocking and what was attempted, then continue with any other unblocked work.
6. When finished or reporting status, list any requirements that were skipped or only partially implemented, along with the rationale.

## Git Workflow

This repository is the **Open Golf Simulator**, built on top of OpenRCT2.
Work happens on the local branch `golf-develop`, which tracks `develop` on the `golf` remote:

- Remote `golf`: `https://github.com/frutiemax92/open-golf-simulator` (branch `develop`)
- Do **not** push to `origin` (`OpenRCT2/OpenRCT2`) or `fork` (`frutiemax92/OpenRCT2`).

After every feature or fix:

1. Write unit tests for it (see "Unit Tests" below).
2. Build and run the **entire** test suite, not only the new tests. Do not commit if anything fails to build or any test fails; fix all failures first.
3. Stage only the files that belong to that change (exclude build artifacts, logs, or untracked scratch files).
4. Commit with a clear message describing what changed and why. Include requirement IDs (e.g., `(ANM-001 to ANM-005)`) and ensure tests are committed alongside the feature code.
   Always include Gemini as a co-author at the end of the commit message:
   ```
   Co-Authored-By: Gemini <noreply@google.com>
   ```
   so that Gemini appears in the contributors on GitHub.
5. Push to the develop branch: `git push golf golf-develop:develop`
6. If the push is rejected (remote has new commits), rebase with `git pull --rebase golf develop`, resolve any conflicts, and push again. Never force-push (`--force` or `+`).

Keep one commit per feature or fix; do not bundle unrelated changes together. After each commit and push, proceed directly to the next feature (see "Scope of Work").

## Unit Tests

Every new feature requires unit tests, and every bug fix requires a regression test that demonstrates the failure without the fix.

- Tests use **GoogleTest** and reside in `test/tests/`. Add new test files to the `test_files` list in `test/tests/CMakeLists.txt`.
- Cover normal behavior, edge cases, and error handling, not just the happy path.
- Tests are off by default. Configure the build with them enabled:
  ```bash
  cmake -B build -DWITH_TESTS=ON
  ```
- Build and run all tests before every commit:
  ```bash
  cmake --build build && ctest --test-dir build --output-on-failure
  ```

## Code Style and Conventions

- Follow the existing OpenRCT2 code conventions (`.clang-format`, `PascalCase` for types/classes, `camelCase` for member variables and functions, `k`-prefixed constants, and `namespace OpenRCT2`).
- Maintain simulation **determinism** (§3.4): all simulation logic must produce identical results given the same save state and inputs.
- Keep terminal commands clean and standard to allow persistent execution inside the sandbox.

