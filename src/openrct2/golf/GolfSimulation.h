/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "Carts.h"
#include "CourseManager.h"
#include "GameClock.h"
#include "DrivingRange.h"
#include "Golfer.h"
#include "HoleDifficulty.h"
#include "HoleSchedule.h"

#include <deque>
#include <map>
#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

namespace OpenRCT2::Golf
{
    // The behaviour of the golfers (GLF-001 to GLF-014, GLF-030 to GLF-032, GLF-040 to GLF-042, section 16): groups arrive,
    // check in, wait for their tee time, play the holes of a course with the play AI and the ball physics, use the
    // buildings that serve their needs, and leave or rage quit. It needs nothing of the game but the ball world, so it runs
    // headless in tests; the game draws the golfers and balls from its state.

    // A building that serves needs (GLF-012): the golf club, a food truck, a restroom.
    struct Service
    {
        uint32_t id = 0;
        FixedVec3 position;
        bool food = false;
        bool drink = false;
        bool toilet = false;
        bool rest = false;
        bool proShop = false;
        int32_t minutes = 2;   // game minutes a visit takes
        int64_t price = 0;
    };

    // A path bench where golfers sit to restore energy (PTH-021, GLF-031). Max 2 occupants per bench.
    struct Bench
    {
        uint32_t id = 0;
        FixedVec3 position;
        uint8_t occupants = 0;
        static constexpr uint8_t kMaxOccupants = 2;
    };

    // A litter bin where golfers drop food and drink containers (PTH-022).
    struct LitterBin
    {
        uint32_t id = 0;
        FixedVec3 position;
        uint32_t trashCount = 0;
        static constexpr uint32_t kCapacity = 20;
    };

    // What the simulation did, for the interface (thoughts, news, sounds) and the economy.
    struct SimEvent
    {
        enum class Type : uint8_t
        {
            teedOff,
            shotHit,        // a golfer hit a ball (sound, animation)
            ballSettled,
            holeFinished,   // value = strokes against par
            thought,
            usedFacility,
            paidFee,
            rageQuit,       // value = hole number; insult = index of the insult spoken
            roundFinished,
            tooHard,
            lostBall,
            restedOnBench,
            talkedWithGolfer,
            littered,
            paidCartFee,    // value = the fee (CRT-003)
            paidRangeFee,   // value = the price of the bucket (DRV-005)
        };
        Type type = Type::thought;
        uint32_t golferId = 0;
        int32_t value = 0;
        int32_t insult = -1;
        ThoughtType thought = ThoughtType::enjoyingTheRound;
    };

    // Timings in ticks.
    constexpr int32_t kCheckInTicks = kTicksPerGameMinute;
    constexpr int32_t kPrepareShotTicks = 60;
    constexpr int32_t kSwingTicks = 50;
    constexpr int32_t kRestTicksPerMinute = kTicksPerGameMinute;
    constexpr int32_t kRestEnergyPerMinute = 30;
    constexpr int32_t kTalkMinutes = 1;
    // Walking speed in world units per tick: 90 yards per game minute (HoleSchedule).
    constexpr int32_t kWalkSpeedHundredths = 288;
    constexpr int32_t kArriveDistanceUnits = 2;
    // A group waiting at a tee longer than this many minutes counts it as a course problem (GLF-013, GLF-040).
    constexpr int32_t kLongWaitMinutes = 8;
    constexpr int32_t kFreeWaitMinutes = 1;
    // A shot that comes to rest within this share of the distance meant counts as good.
    constexpr int32_t kGoodShotPercent = 12;

    class GolfSimulation
    {
    public:
        struct Environment
        {
            const IBallWorld* world = nullptr;
            CourseManager* courses = nullptr;
            IBallRandom* rng = nullptr;
            FixedVec3 clubEntrance;   // the check-in desk
            FixedVec3 exit;           // where golfers leave
            std::vector<Service> services;
            std::vector<Bench> benches;
            std::vector<LitterBin> bins;
            // CRT-002 to CRT-005: the golf carts. When there is no fleet everyone walks.
            CartFleet* carts = nullptr;
            // Positions along the paths, where carts may stop (CRT-004).
            std::function<std::vector<FixedVec3>()> pathPoints;
        };

        explicit GolfSimulation(Environment environment);

        Environment& environment()
        {
            return _env;
        }
        const Environment& environment() const
        {
            return _env;
        }

        struct Spawn
        {
            std::string name;
            uint32_t groupId = 0;
            GolferSkills skills;
            uint8_t equipmentLevel = 0;
            int64_t money = 10000;
            bool vip = false;
            Fixed riskTolerance = Fixed::ratio(5, 10);
            uint32_t courseId = 0;
            TeeCategory tee = TeeCategory::men;
            FixedVec3 position;
        };
        // GLF-001: a golfer arrives. Returns the new golfer.
        Golfer& spawn(const Spawn& spawn);

        // STE-021: a golfer who is still in the car that is driving to a parking space. They do not move, grow tired or hungry
        // until they get out (release); a car that never gets to park takes its golfers back (remove).
        bool putInCar(uint32_t id);
        // STE-012: the golfer gets out of the car at a position and walks to the club; the car is where they go back to when
        // they leave (STE-022).
        bool release(uint32_t id, const FixedVec3& position, const FixedVec3& carPosition);
        // Takes a golfer off the course without a trace (the car could not come).
        bool remove(uint32_t id);

        // Advances every golfer by one tick. `nowTicks` is the game tick, which gives the game clock.
        void tick(uint32_t nowTicks);

        // The balls hit at the driving ranges, for drawing.
        std::vector<RangeSimulation::BallView> rangeBalls() const;

        std::vector<Golfer>& golfers()
        {
            return _golfers;
        }
        const std::vector<Golfer>& golfers() const
        {
            return _golfers;
        }
        Golfer* find(uint32_t id);
        // Events since the last call, oldest first.
        std::vector<SimEvent> takeEvents();
        // Golfers that have left (or rage quit) are removed after their last tick; their registry entries are the caller's.
        std::vector<Golfer> takeLeft();

        // Income from green fees and services since the start.
        int64_t income() const
        {
            return _income;
        }
        uint32_t nextGolferId() const
        {
            return _nextId;
        }
        TeeSheet& teeSheet(uint32_t courseId);
        // The difficulty of a hole, computed on first use and kept.
        const HoleDifficulty& difficultyOf(uint32_t courseId, uint32_t holeId);
        void forgetDifficulty()
        {
            _difficulty.clear();
        }
        uint32_t minuteNow() const
        {
            return gameMinuteAt(_now);
        }

    private:
        friend struct GolfSaveAccess;
        void updateGolfer(Golfer& g);
        void onMinute(Golfer& g);
        bool walk(Golfer& g, const FixedVec3& to);
        // The driving ranges (section 17).
        void syncRanges();
        bool wantsRange(const Golfer& g);
        void visitRange(Golfer& g, const DrivingRange& range);
        void beginRound(Golfer& g, const Course& course);
        void endRangeVisit(Golfer& g);
        RangeSimulation* rangeOf(const Golfer& g);
        // Carts (section 14).
        void decideCarts(Golfer& g, const Course& course);
        bool ride(Golfer& g, const FixedVec3& to);
        void board(Golfer& g);
        void dismount(Golfer& g);
        void returnCartAndLeave(Golfer& g);
        void abandonCart(Golfer& g);
        Cart* cartOf(const Golfer& g) const;
        void setState(Golfer& g, GolferState state);
        const Course* courseOf(const Golfer& g) const;
        const Hole* holeOf(const Golfer& g) const;
        void startHole(Golfer& g);
        void teeOff(Golfer& g);
        void judgeBall(Golfer& g);
        void finishHole(Golfer& g);
        void goToNextStep(Golfer& g);
        bool seekFacility(Golfer& g, NeedKind need);
        void startRageQuit(Golfer& g);
        void leave(Golfer& g);
        void emit(SimEvent::Type type, const Golfer& g, int32_t value = 0);
        void think(Golfer& g, ThoughtType thought);
        bool groupBusy(const Golfer& g) const;
        Fixed groundZ(Fixed x, Fixed y) const;

        Environment _env;
        const IBallWorld* _rawWorld = nullptr;       // the world as given
        std::shared_ptr<IBallWorld> _cupWorld;       // the same with the cups of the holes (what the golfers play on)
        std::vector<Golfer> _golfers;
        std::vector<Golfer> _left;
        std::vector<SimEvent> _events;
        std::unordered_map<uint32_t, TeeSheet> _sheets;
        std::unordered_map<uint64_t, HoleDifficulty> _difficulty;
        // The tee time of a group on a hole, so all members play from the same booking: key = group << 32 | holeIndex.
        std::unordered_map<uint64_t, uint32_t> _groupTeeTime;
        std::map<uint32_t, std::unique_ptr<RangeSimulation>> _ranges; // by range id (DRV-005)
        uint32_t _now = 0;
        uint32_t _nextId = 1;
        int64_t _income = 0;
    };
} // namespace OpenRCT2::Golf
