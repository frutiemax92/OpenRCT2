# Agent instructions

## Models and cost

Use Sonnet 5.5 (`claude-sonnet-5-5`) by default for all work, to keep token costs low.
When you hit a problem or an open question you can't resolve with reasonable effort (a failing test with an unclear cause, a tricky bug, an architecture or design decision, a repeated failed attempt), launch a subagent with the Agent tool using `model: "opus"` (Opus 5.5, `claude-opus-5-5`) to address it.

- Give the subagent a self-contained prompt: the problem, what you already tried, relevant file paths and error output.
- Use it only for the hard part; take its answer back and continue the routine work (edits, builds, commits) yourself on Sonnet.
- Don't spawn Opus for routine or well-understood tasks.

## Scope of work: implement everything

The goal is to implement **all** features of `specifications/gameplay-testing-plan.md`, not one. Do not stop after a feature, a commit or a phase.

1. Follow the phase order of section 3 (Phase 1 to Phase 6). Pick the next unimplemented feature by ID (`NAV-001`, `UI-001`, `TOOL-001`, `VIEW-001`, `MON-001`, `TEST-001`, ...), implement it, test it, commit it, push it (see "Git workflow"), then immediately take the next one.
2. Check `git log`, the code and the tests to see what is done; feature IDs in commit messages show progress. Don't redo finished work.
3. **One commit per feature** (one ID, or the smallest unit the plan lists as a feature), each with its tests, after the whole test suite passes. Never bundle features.
4. A short status line after a commit is a milestone, not a stopping point. Don't ask "should I continue?": the answer is always yes.
5. Stop only when every feature in the plan is implemented and its acceptance criteria pass, or when you are blocked on something only the user can decide (missing assets, credentials, a choice with no default in the plan). In that case document the blocker and what was tried, then continue with other unblocked work.
6. When you do stop, list the features that were skipped or only partly done, with the reason.

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

## In-game verification of every gameplay mechanic

Unit tests are not proof. A passing test only shows that the headless logic does what the test asserts; it says nothing about whether the player can reach the feature, see it, or whether it behaves in the running game. **Never assume a passing test means the in-game feature is correctly implemented.** Confirm each mechanic by playing it in the real game and looking at a capture.

### What to cover

Walk through **every gameplay mechanic defined in `specifications/detailed-specifications.md`** (sections 6 to 24: terrain, water, streams, buildings, paths, 3D entities, streets/cars/parking, golfers, carts, ball and shot, holes/courses/scheduling, driving range, mini putt, scenery, animals, audio, economy/tournaments/objectives, save/editor, UI). Use `specifications/gameplay-testing-plan.md` for the known navigation blockers and the sandbox panel.

1. Make a checklist of the requirement IDs (`TER-`, `BLD-`, `HOL-`, `BAL-`, ...) and tick them off one by one. Keep it in a scratch file (not committed) and update it as you go.
2. For each mechanic, reach it **only through the UI as a player would** (title menu, toolbar buttons, windows, viewport clicks). A console command or a unit-test fixture does not count as reachable. If the only way in is a console command, that is an unreachable feature: fix it.
3. Exercise the mechanic (place it, run time, trigger it with the sandbox panel) and capture the result.

### Capturing

Build and run the real game (not the test binary), with a scratch `--user-data-path`.

- **Full frame (UI included):** `touch /tmp/openrct2-frame-dump.request`. Within a few frames the game removes the file and writes a PNG of the whole frame (toolbar and windows included) to the screenshots folder of the user-data path; the path is logged (`Frame dump: ...`). This is the `DumpFrameIfRequested` hook in `HardwareDisplayDrawingEngine.cpp`. It is currently uncommitted; commit it as its own change if it is still missing.
- **Map viewport only:** the in-game screenshot shortcut (`CTRL+S`, `gScreenshotCountdown`).
- Desktop grabs (`xwd`) are striped on this machine; do not use them.

Open every capture with the Read tool and **look at it**. Check what the spec asks for (shape, colours, text, counters, animation state), not only that something rendered. For anything that moves or changes (golfers, balls, carts, weather, income), record a video as described below instead of relying on stills. Cite the capture path and what you saw when you report a mechanic as verified.

### Capturing sequences (video)

Anything that moves or changes over time (golfers walking and playing, ball flight and rolling, carts, cars parking, animals, weather and turf transitions, tournaments, income counters, window animations) must be verified on a **video**, not on a single still. A still can look right while the motion is wrong (teleporting, sliding, flicker, wrong facing, a ball passing through terrain).

1. **Record.** Record the real game for the duration of the scenario (a shot, a full hole, a golfer's arrival to departure, a day cycle). Use the frame-dump hook as a recorder: extend it (and commit it as its own change, with a test if the logic is testable) so that a request file such as `/tmp/openrct2-record.request` containing a frame count and a step (every N game ticks) writes numbered PNGs `frame_00001.png`, ... into a scratch folder. Stepping by game ticks, not wall-clock time, keeps the sequence deterministic and independent of the machine's speed. Use the sandbox panel to speed up or trigger the scenario, but keep the recorded frames at a step small enough that the fastest motion changes by a few pixels between frames.
2. **Assemble.** `ffmpeg` is installed. Build the video from the frames:
   `ffmpeg -y -framerate 30 -i frame_%05d.png -c:v libx264 -pix_fmt yuv420p scenario.mp4`
   Keep the video and frames in the scratchpad, not in the repository.
3. **Analyse frame by frame.** You cannot watch a video, so extract and read the frames yourself with the Read tool:
   - Read the frames in order, at the step where the action happens (start, each key moment, end), then the frames in between wherever something looks off. For long clips build contact sheets: `ffmpeg -i scenario.mp4 -vf "select='not(mod(n,10))',scale=480:-1,tile=4x3" sheet_%02d.png`, then read the individual frames around anything suspicious.
   - Compare consecutive frames for continuity: position changes smoothly, no jump or flicker, correct facing and animation frame, correct draw order and depth (nothing drawn through a building or under terrain), the ball follows the expected arc and lands, rolls and stops where the physics says, counters and text update when they should.
   - Write down, for each scenario, the frame numbers where you saw an error and what the error was.
4. **Report.** For each verified mechanic give the video path, the frame range reviewed and what you saw. A video you did not review frame by frame (or at least at every key frame) is not verification.

Any error found on a video goes through "When something is wrong" below: fix, regression test, record the **same scenario again** and review the new video to confirm the fix before committing.

### When something is wrong

If a capture shows a bug, a missing or wrong visual, or a feature the player cannot reach (no button, no window, no click target, simulation gated off):

1. Fix it in the code (not by working around it in the test or the capture setup).
2. Add a regression test that fails without the fix, when the problem can be tested headlessly.
3. Rebuild, rerun the game and capture again to **confirm the fix in-game**. If the capture still looks wrong, the fix is not done, whatever the tests say.
4. Build and run the **whole** test suite; fix any failure first.
5. Commit (one commit per fix or feature, tests included) and push: `git push golf golf-develop:develop`. Follow "Git workflow".
6. Continue with the next mechanic. Don't stop to ask.

Mechanics that cannot be verified (needs assets or input you don't have) are documented with the reason and listed in the final report, never silently marked as done.
