/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <gtest/gtest.h>
#include <openrct2/GameState.h>
#include <openrct2/actions/GameActionRunner.h>
#include <openrct2/actions/park/ParkSetLoanAction.h>
#include <openrct2/golf/Cottages.h>
#include <openrct2/golf/Popularity.h>
#include <openrct2/management/Finance.h>
#include <openrct2/scripting/ScriptEngine.h>

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

TEST(Finances, RenamedCategoriesAndPaymentTracking)
{
    // ECO-012: OpenRCT2 finance categories renamed to golf operations and track cash flow.
    auto& park = getGameState().park;
    park.cash = 50000.00_GBP;
    park.expenditureTable[0][EnumValue(ExpenditureType::parkEntranceTickets)] = 0;
    park.expenditureTable[0][EnumValue(ExpenditureType::parkRideTickets)] = 0;
    park.expenditureTable[0][EnumValue(ExpenditureType::shopSales)] = 0;
    park.expenditureTable[0][EnumValue(ExpenditureType::rideConstruction)] = 0;

    // Green fees income (negative cost passed to FinancePayment increases cash)
    FinancePayment(-150.00_GBP, ExpenditureType::parkEntranceTickets);
    EXPECT_EQ(park.cash, 50150.00_GBP);
    EXPECT_EQ(park.expenditureTable[0][EnumValue(ExpenditureType::parkEntranceTickets)], 150.00_GBP);

    // Cart / range fees income
    FinancePayment(-30.00_GBP, ExpenditureType::parkRideTickets);
    EXPECT_EQ(park.cash, 50180.00_GBP);
    EXPECT_EQ(park.expenditureTable[0][EnumValue(ExpenditureType::parkRideTickets)], 30.00_GBP);

    // Course construction expenditure (positive cost reduces cash)
    FinancePayment(500.00_GBP, ExpenditureType::rideConstruction);
    EXPECT_EQ(park.cash, 49680.00_GBP);
    EXPECT_EQ(park.expenditureTable[0][EnumValue(ExpenditureType::rideConstruction)], -500.00_GBP);

    // ScriptEngine expenditure string names mapping (ECO-012)
    using OpenRCT2::Scripting::ScriptEngine;
    EXPECT_EQ(ScriptEngine::ExpenditureTypeToString(ExpenditureType::parkEntranceTickets), "green_fees");
    EXPECT_EQ(ScriptEngine::ExpenditureTypeToString(ExpenditureType::parkRideTickets), "cart_range_fees");
    EXPECT_EQ(ScriptEngine::ExpenditureTypeToString(ExpenditureType::rideConstruction), "course_construction");
    EXPECT_EQ(ScriptEngine::ExpenditureTypeToString(ExpenditureType::rideRunningCosts), "facility_upkeep");
    EXPECT_EQ(ScriptEngine::ExpenditureTypeToString(ExpenditureType::research), "tournaments");

    // Both golf names and legacy aliases resolve
    EXPECT_EQ(ScriptEngine::StringToExpenditureType("green_fees"), ExpenditureType::parkEntranceTickets);
    EXPECT_EQ(ScriptEngine::StringToExpenditureType("park_entrance_tickets"), ExpenditureType::parkEntranceTickets);
    EXPECT_EQ(ScriptEngine::StringToExpenditureType("course_construction"), ExpenditureType::rideConstruction);
    EXPECT_EQ(ScriptEngine::StringToExpenditureType("ride_construction"), ExpenditureType::rideConstruction);
}

TEST(Loans, ParkSetLoanActionValidatesLimitsAndUpdatesBalances)
{
    // ECO-020: Reuse OpenRCT2 loans (max loan, interest rate, increase/repay)
    auto& gameState = getGameState();
    auto& park = gameState.park;
    park.cash = 10000.00_GBP;
    park.bankLoan = 5000.00_GBP;
    park.maxBankLoan = 25000.00_GBP;

    // Normal increase within maxBankLoan
    auto increase = GameActions::ParkSetLoanAction(10000.00_GBP);
    EXPECT_EQ(increase.Query(gameState, park).error, GameActions::Status::ok);
    EXPECT_EQ(increase.Execute(gameState, park).error, GameActions::Status::ok);
    EXPECT_EQ(park.bankLoan, 10000.00_GBP);
    EXPECT_EQ(park.cash, 15000.00_GBP);

    // Exceeding maxBankLoan is disallowed
    auto tooBig = GameActions::ParkSetLoanAction(30000.00_GBP);
    EXPECT_EQ(tooBig.Query(gameState, park).error, GameActions::Status::disallowed);

    // Negative loan is disallowed
    auto negative = GameActions::ParkSetLoanAction(-1000.00_GBP);
    EXPECT_EQ(negative.Query(gameState, park).error, GameActions::Status::invalidParameters);

    // Repaying loan decreases loan and subtracts cash
    auto repay = GameActions::ParkSetLoanAction(4000.00_GBP);
    EXPECT_EQ(repay.Query(gameState, park).error, GameActions::Status::ok);
    EXPECT_EQ(repay.Execute(gameState, park).error, GameActions::Status::ok);
    EXPECT_EQ(park.bankLoan, 4000.00_GBP);
    EXPECT_EQ(park.cash, 9000.00_GBP);

    // Repaying more than cash on hand is rejected with insufficientFunds
    park.cash = 500.00_GBP;
    auto cantAfford = GameActions::ParkSetLoanAction(2000.00_GBP); // tries to pay 2000 with 500 cash
    EXPECT_EQ(cantAfford.Query(gameState, park).error, GameActions::Status::insufficientFunds);

    // Increasing loan when in debt (negative cash) is allowed
    park.cash = -200.00_GBP;
    auto bailOut = GameActions::ParkSetLoanAction(6000.00_GBP);
    EXPECT_EQ(bailOut.Query(gameState, park).error, GameActions::Status::ok);
    EXPECT_EQ(bailOut.Execute(gameState, park).error, GameActions::Status::ok);
    EXPECT_EQ(park.cash, 1800.00_GBP);
}
