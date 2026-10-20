/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Tournament.h"

#include <algorithm>
#include <array>

namespace OpenRCT2::Golf
{
    namespace
    {
        constexpr std::array<TierRequirements, static_cast<size_t>(TournamentTier::count)> kTiers = { {
            // holes, yards, par range, popularity, clubs, parking, spectators, media, extra parking, grandstand, distance
            { 18, 5000, 68, 74, 400, 1, 20, 20, 4, 20, 6, 12, 8000, 6, 40 },
            { 18, 6000, 70, 72, 600, 1, 40, 40, 8, 40, 12, 12, 30000, 10, 80 },
            { 18, 6600, 71, 72, 800, 2, 80, 80, 16, 80, 24, 10, 120000, 14, 160 },
        } };
    } // namespace

    const char* tierName(TournamentTier tier)
    {
        switch (tier)
        {
            case TournamentTier::local:
                return "Local";
            case TournamentTier::regional:
                return "Regional";
            case TournamentTier::national:
                return "National";
            default:
                return "";
        }
    }

    const TierRequirements& requirementsFor(TournamentTier tier)
    {
        const auto index = static_cast<size_t>(tier);
        return kTiers[index < kTiers.size() ? index : 0];
    }

    std::vector<Requirement> checklist(TournamentTier tier, const TournamentInputs& in)
    {
        const auto& r = requirementsFor(tier);
        std::vector<Requirement> list;
        list.push_back({ RequirementId::holes, "Holes on the course", in.holes, r.holes, in.holes >= r.holes });
        list.push_back({ RequirementId::length, "Total length in yards", in.totalYards, r.minYards, in.totalYards >= r.minYards });
        // The par must lie in a window; "required" shows the minimum and the window is in the label.
        list.push_back({ RequirementId::par, "Course par between the limits", in.totalPar, r.minPar,
                         in.totalPar >= r.minPar && in.totalPar <= r.maxPar });
        list.push_back({ RequirementId::popularity, "Popularity", in.popularity, r.minPopularity, in.popularity >= r.minPopularity });
        list.push_back({ RequirementId::golfClub, "Golf clubs", in.golfClubs, r.minClubs, in.golfClubs >= r.minClubs });
        list.push_back({ RequirementId::parking, "Parking spaces", in.parkingCapacity, r.minParking, in.parkingCapacity >= r.minParking });
        list.push_back({ RequirementId::spectatorZone, "Spectator area tiles", in.spectatorTiles, r.minSpectatorTiles,
                         in.spectatorTiles >= r.minSpectatorTiles });
        list.push_back({ RequirementId::mediaZone, "Media tent tiles", in.mediaTiles, r.minMediaTiles, in.mediaTiles >= r.minMediaTiles });
        list.push_back({ RequirementId::extraParkingZone, "Extra parking tiles", in.extraParkingTiles, r.minExtraParkingTiles,
                         in.extraParkingTiles >= r.minExtraParkingTiles });
        list.push_back({ RequirementId::grandstandZone, "Grandstand tiles", in.grandstandTiles, r.minGrandstandTiles,
                         in.grandstandTiles >= r.minGrandstandTiles });
        list.push_back({ RequirementId::grandstandNearGreen, "Grandstand within tiles of the last green",
                         in.grandstandDistanceToLastGreen, r.maxGrandstandDistanceToLastGreen,
                         in.grandstandTiles > 0 && in.grandstandDistanceToLastGreen <= r.maxGrandstandDistanceToLastGreen });
        return list;
    }

    bool canHost(TournamentTier tier, const TournamentInputs& inputs)
    {
        const auto list = checklist(tier, inputs);
        return std::all_of(list.begin(), list.end(), [](const Requirement& r) { return r.met; });
    }

    BookResult book(Tournament& t, TournamentTier tier, const TournamentInputs& inputs, uint32_t today, uint32_t day, int64_t ticketPrice)
    {
        if (t.phase == TournamentPhase::scheduled || t.phase == TournamentPhase::running)
            return BookResult::alreadyBooked;
        if (ticketPrice < 0)
            return BookResult::negativePrice;
        if (day <= today)
            return BookResult::dayInThePast;
        if (!canHost(tier, inputs))
            return BookResult::requirementsNotMet;
        t = Tournament{};
        t.phase = TournamentPhase::scheduled;
        t.tier = tier;
        t.day = day;
        t.ticketPrice = ticketPrice;
        t.professionals = kProfessionalFieldSize;
        t.spectators = spectatorCount(tier, inputs.popularity);
        return BookResult::ok;
    }

    void cancel(Tournament& t)
    {
        if (t.phase == TournamentPhase::scheduled)
            t = Tournament{};
    }

    bool advanceDay(Tournament& t, uint32_t today)
    {
        if (t.phase == TournamentPhase::scheduled && today >= t.day)
        {
            t.phase = TournamentPhase::running;
            return true;
        }
        return false;
    }

    bool courseClosedToPublic(const Tournament& t)
    {
        return t.phase == TournamentPhase::running;
    }

    GolferSkills professionalSkills(TournamentTier tier, IBallRandom& rng)
    {
        // 85 to 100, higher at the top tier.
        const int32_t floorValue = 85 + static_cast<int32_t>(tier) * 4;
        auto roll = [&]() { return static_cast<uint8_t>(std::min<int32_t>(100, floorValue + static_cast<int32_t>(rng.next() % (101 - floorValue)))); };
        return { roll(), roll(), roll(), roll() };
    }

    int32_t spectatorCount(TournamentTier tier, int32_t popularity)
    {
        const auto& r = requirementsFor(tier);
        return std::max(0, popularity) * r.spectatorsPerPopularityPoint / 10;
    }

    int32_t tournamentResult(int32_t averagePlayerHappiness, int32_t bottleneckHoles, int32_t rageQuits, int32_t holes)
    {
        int32_t score = std::clamp(averagePlayerHappiness, 0, 255) * 100 / 255;
        // Every bottleneck hole and every rage quit takes from the score; a bottleneck matters more on a short course.
        const int32_t h = std::max(1, holes);
        score -= std::max(0, bottleneckHoles) * 40 / h;
        score -= std::max(0, rageQuits) * 15;
        return std::clamp(score, 0, 100);
    }

    int64_t tournamentRevenue(const Tournament& t)
    {
        return static_cast<int64_t>(t.spectators) * t.ticketPrice + requirementsFor(t.tier).sponsorMoney;
    }

    void finish(Tournament& t, int32_t averagePlayerHappiness, int32_t bottleneckHoles, int32_t rageQuits, int32_t holes, int32_t popularity)
    {
        if (t.phase != TournamentPhase::running)
            return;
        t.result = tournamentResult(averagePlayerHappiness, bottleneckHoles, rageQuits, holes);
        t.revenue = tournamentRevenue(t);
        // The popularity bonus scales with how well it went.
        t.popularityGained = requirementsFor(t.tier).popularityBonus * t.result / 100;
        (void)popularity;
        t.phase = TournamentPhase::finished;
    }
} // namespace OpenRCT2::Golf
