/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <gtest/gtest.h>
#include <openrct2/golf/Cottages.h>
#include <openrct2/golf/Popularity.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Golf;

TEST(Popularity, ASharesAddUpToOneThousand)
{
    EXPECT_EQ(kHappinessShare + kBalanceShare + kSceneryShare + kFacilityShare + kPriceShare, 1000);
}

TEST(Popularity, AnEmptyParkIsUnpopularAndAPerfectOneIsFull)
{
    PopularityInputs none;
    none.averageHappiness = 0;
    none.difficultyBalance = 0;
    none.priceFairness = 0;
    EXPECT_EQ(computePopularity(none), 0);
    PopularityInputs perfect;
    perfect.averageHappiness = 255;
    perfect.difficultyBalance = 100;
    perfect.sceneryScore = 100;
    perfect.facilityScore = 100;
    perfect.priceFairness = 100;
    EXPECT_EQ(computePopularity(perfect), 1000);
}

TEST(Popularity, EveryPartHelpsInProportionToItsShare)
{
    PopularityInputs base;
    base.averageHappiness = 0;
    base.difficultyBalance = 0;
    base.priceFairness = 0;
    auto with = [&](auto change) {
        auto in = base;
        change(in);
        return computePopularity(in);
    };
    EXPECT_NEAR(with([](auto& i) { i.averageHappiness = 255; }), kHappinessShare, 2);
    EXPECT_NEAR(with([](auto& i) { i.difficultyBalance = 100; }), kBalanceShare, 2);
    EXPECT_NEAR(with([](auto& i) { i.sceneryScore = 100; }), kSceneryShare, 2);
    EXPECT_NEAR(with([](auto& i) { i.facilityScore = 100; }), kFacilityShare, 2);
    EXPECT_NEAR(with([](auto& i) { i.priceFairness = 100; }), kPriceShare, 2);
    // Out of range values are clamped.
    EXPECT_EQ(with([](auto& i) { i.sceneryScore = 5000; }), with([](auto& i) { i.sceneryScore = 100; }));
    EXPECT_EQ(with([](auto& i) { i.priceFairness = 150; }), with([](auto& i) { i.priceFairness = 100; }));
    EXPECT_EQ(with([](auto& i) { i.averageHappiness = -4; }), 0);
}

TEST(Popularity, RageQuitsAreAStrongNegative)
{
    PopularityInputs happy;
    happy.averageHappiness = 220;
    happy.difficultyBalance = 80;
    happy.sceneryScore = 60;
    happy.facilityScore = 60;
    happy.priceFairness = 100;
    happy.recentRounds = 100;
    const int32_t before = computePopularity(happy);
    auto one = happy;
    one.recentRageQuits = 1;
    EXPECT_LE(computePopularity(one), before - kRageQuitPenalty);
    auto many = happy;
    many.recentRageQuits = 30;
    EXPECT_LT(computePopularity(many), computePopularity(one));
    EXPECT_GE(computePopularity(many), before - kRageQuitMaxPenalty);
    auto everyone = happy;
    everyone.recentRageQuits = 100;
    everyone.recentRounds = 0;
    EXPECT_LT(computePopularity(everyone), before - 250);
}

TEST(Popularity, TournamentsLiftItUpToACap)
{
    PopularityInputs in;
    in.averageHappiness = 100;
    const int32_t base = computePopularity(in);
    in.tournamentsHosted = 1;
    EXPECT_EQ(computePopularity(in), base + kTournamentBonus);
    in.tournamentsHosted = 50;
    EXPECT_EQ(computePopularity(in), base + kTournamentMaxBonus);
    in.tournamentsHosted = -3;
    EXPECT_EQ(computePopularity(in), base);
    PopularityInputs top;
    top.averageHappiness = 255;
    top.difficultyBalance = 100;
    top.sceneryScore = 100;
    top.facilityScore = 100;
    top.priceFairness = 100;
    top.tournamentsHosted = 10;
    EXPECT_EQ(computePopularity(top), 1000); // never above the maximum
}

TEST(Popularity, MovesTowardsTheTargetOneStepADay)
{
    EXPECT_EQ(movePopularity(100, 500), 100 + kPopularityDailyStep);
    EXPECT_EQ(movePopularity(500, 100), 500 - kPopularityDailyStep);
    EXPECT_EQ(movePopularity(490, 500), 500);
    EXPECT_EQ(movePopularity(510, 500), 500);
    EXPECT_EQ(movePopularity(500, 500), 500);
    EXPECT_EQ(movePopularity(-50, 3000), kPopularityDailyStep);
    int32_t p = 0;
    int days = 0;
    while (p < 600 && days < 1000)
    {
        p = movePopularity(p, 600);
        days++;
    }
    EXPECT_EQ(days, 24);
}

TEST(Ledger, IncomeCostsAndProfit)
{
    Ledger l;
    l.greenFees = 5000;
    l.cartFees = 300;
    l.rangeBuckets = 200;
    l.miniPuttFees = 100;
    l.buildingServices = 700;
    l.cottageSales = 20000;
    l.tournamentRevenue = 1000;
    l.construction = 4000;
    l.buildingUpkeep = 250;
    l.staffWages = 600;
    l.cartPurchases = 1500;
    l.loanInterest = 80;
    l.marketing = 70;
    EXPECT_EQ(l.income(), 27300);
    EXPECT_EQ(l.costs(), 6500);
    EXPECT_EQ(l.profit(), 20800);
    Ledger total;
    total.add(l);
    total.add(l);
    EXPECT_EQ(total.profit(), 41600);
    EXPECT_EQ(total.greenFees, 10000);
    EXPECT_EQ(Ledger{}.profit(), 0);
}

TEST(Ledger, LoanInterestIsChargedPerDay)
{
    EXPECT_EQ(dailyLoanInterest(0, 50), 0);
    EXPECT_EQ(dailyLoanInterest(100000, 0), 0);
    EXPECT_EQ(dailyLoanInterest(-5, 50), 0);
    // 5% a year on 360000 is 18000, or 50 a day.
    EXPECT_EQ(dailyLoanInterest(360000, 50), 50);
}

namespace
{
    CottageLot lot(uint32_t id, int x0, int y0, int x1, int y1, bool sold = false)
    {
        CottageLot l;
        l.id = id;
        l.min = { x0, y0 };
        l.max = { x1, y1 };
        l.sold = sold;
        return l;
    }
} // namespace

TEST(Cottages, LotsAreRectanglesOfASensibleSize)
{
    EXPECT_EQ(lot(1, 0, 0, 2, 3).width(), 3);
    EXPECT_EQ(lot(1, 0, 0, 2, 3).depth(), 4);
    EXPECT_EQ(checkLot(lot(1, 0, 0, 3, 3), true, {}, false), LotCheck::ok);
    EXPECT_EQ(checkLot(lot(1, 0, 0, 0, 3), true, {}, false), LotCheck::tooSmall);
    EXPECT_EQ(checkLot(lot(1, 0, 0, 8, 3), true, {}, false), LotCheck::tooLarge);
    EXPECT_EQ(checkLot(lot(1, 0, 0, 3, 3), false, {}, false), LotCheck::notNextToRoad);
    EXPECT_EQ(checkLot(lot(1, 0, 0, 3, 3), true, {}, true), LotCheck::overlapsHole);
}

TEST(Cottages, LotsMayNotOverlapEachOther)
{
    const std::vector<CottageLot> existing = { lot(1, 0, 0, 3, 3) };
    EXPECT_EQ(checkLot(lot(2, 3, 3, 6, 6), true, existing, false), LotCheck::overlapsLot);
    EXPECT_EQ(checkLot(lot(2, 4, 0, 7, 3), true, existing, false), LotCheck::ok);
    // A lot does not overlap itself when it is edited.
    EXPECT_EQ(checkLot(lot(1, 0, 0, 4, 4), true, existing, false), LotCheck::ok);
}

TEST(Cottages, AttractivenessComesFromTheView)
{
    LotFeatures bare;
    bare.distanceToClubTiles = 40;
    const int32_t plain = lotAttractiveness(bare);
    LotFeatures nice = bare;
    nice.holesInView = 3;
    nice.waterTiles = 20;
    nice.sceneryItems = 10;
    nice.distanceToClubTiles = 8;
    const int32_t good = lotAttractiveness(nice);
    EXPECT_GT(good, plain + 40);
    EXPECT_LE(good, 100);
    LotFeatures huge = nice;
    huge.holesInView = 100;
    huge.waterTiles = 1000;
    huge.sceneryItems = 1000;
    EXPECT_EQ(lotAttractiveness(huge), 100);
    EXPECT_GE(lotAttractiveness(LotFeatures{ 0, 0, 0, 500 }), 0);
}

TEST(Cottages, AttractiveLotsSellAtLowerPopularity)
{
    EXPECT_EQ(salePopularityThreshold(0), 650);
    EXPECT_EQ(salePopularityThreshold(100), 350);
    EXPECT_LT(salePopularityThreshold(80), salePopularityThreshold(20));
    EXPECT_EQ(salePerMillionPerDay(649, 0), 0);
    EXPECT_GT(salePerMillionPerDay(650, 0), 0);
    EXPECT_GT(salePerMillionPerDay(800, 0), salePerMillionPerDay(650, 0));
    EXPECT_LE(salePerMillionPerDay(1000, 0), 200000);
    // The same popularity sells a good lot and not a bad one.
    EXPECT_EQ(salePerMillionPerDay(500, 10), 0);
    EXPECT_GT(salePerMillionPerDay(500, 80), 0);
}

TEST(Cottages, ThePriceGrowsWithPopularityQualityAndSize)
{
    const int64_t base = lotPrice(300, 20, 12);
    EXPECT_GT(lotPrice(800, 20, 12), base);
    EXPECT_GT(lotPrice(300, 90, 12), base);
    EXPECT_EQ(lotPrice(300, 20, 24), base * 2);
    EXPECT_EQ(lotPrice(300, 20, 0), 0);
    EXPECT_EQ(lotPrice(-5, -5, 4), lotPrice(0, 0, 4));
}

TEST(Cottages, VipsPayAReducedFeeOrNone)
{
    EXPECT_EQ(kDefaultVipFeePercent, 0);
    EXPECT_EQ(vipGreenFee(2500, kDefaultVipFeePercent), 0);
    EXPECT_EQ(vipGreenFee(2500, 50), 1250);
    EXPECT_EQ(vipGreenFee(2500, 100), 2500);
    EXPECT_EQ(vipGreenFee(2500, 400), 2500);
    EXPECT_GT(kVipPlayChancePerMille, 100);
}

TEST(Cottages, SoldLotsAreLockedExceptForCosmeticPaths)
{
    const std::vector<CottageLot> lots = { lot(1, 10, 10, 13, 13, true), lot(2, 20, 10, 23, 13, false) };
    const TileCoordsXY inSold{ 11, 11 };
    const TileCoordsXY inFree{ 21, 11 };
    const TileCoordsXY outside{ 40, 40 };
    for (auto edit : { LotEdit::terrain, LotEdit::scenery, LotEdit::building, LotEdit::water })
    {
        EXPECT_FALSE(isEditAllowed(lots, inSold, edit)) << static_cast<int>(edit);
        EXPECT_TRUE(isEditAllowed(lots, inFree, edit));
        EXPECT_TRUE(isEditAllowed(lots, outside, edit));
    }
    EXPECT_TRUE(isEditAllowed(lots, inSold, LotEdit::path));
    ASSERT_NE(lotAt(lots, inSold), nullptr);
    EXPECT_EQ(lotAt(lots, inSold)->id, 1u);
    EXPECT_EQ(lotAt(lots, outside), nullptr);
    EXPECT_TRUE(lots[0].contains({ 13, 13 }));
    EXPECT_FALSE(lots[0].contains({ 14, 13 }));
}
