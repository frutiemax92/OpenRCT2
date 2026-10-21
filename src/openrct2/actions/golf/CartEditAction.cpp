/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "CartEditAction.h"

#include "../../GameState.h"
#include "../../golf/GolfRuntime.h"
#include "../../localisation/StringIds.h"
#include "../../management/Finance.h"

#include <algorithm>

namespace OpenRCT2::GameActions
{
    namespace
    {
        constexpr int64_t kMaxCartsAtOnce = 50;
        constexpr int64_t kMaxCartFee = 100000;
    } // namespace

    CartEditAction::CartEditAction(CartEditType type, int64_t value)
        : _type(type)
        , _value(value)
    {
    }

    void CartEditAction::AcceptParameters(GameActionParameterVisitor& visitor)
    {
        visitor.Visit("type", _type);
        visitor.Visit("value", _value);
    }

    uint16_t CartEditAction::GetActionFlags() const
    {
        return GameAction::GetActionFlags();
    }

    void CartEditAction::Serialise(DataSerialiser& stream)
    {
        GameAction::Serialise(stream);

        stream << DS_TAG(_type) << DS_TAG(_value);
    }

    Result CartEditAction::Query(GameState_t& gameState, Park::ParkData& park) const
    {
        auto& fleet = Golf::runtimeOf(gameState).director().fleet;
        auto res = Result();
        switch (_type)
        {
            case CartEditType::buy:
            {
                if (_value < 1 || _value > kMaxCartsAtOnce)
                    return Result(Status::invalidParameters, STR_CART_CANT_CHANGE, kStringIdNone);
                // Carts are bought into free bays: as many as there are, and no more.
                int32_t freeBays = 0;
                for (const auto& space : fleet.spaces())
                    freeBays += space.cartId == 0 && space.reservedBy == 0 ? 1 : 0;
                if (freeBays == 0)
                    return Result(Status::disallowed, STR_CART_CANT_CHANGE, STR_CART_NO_FREE_BAY);
                res.cost = std::min<int64_t>(_value, freeBays) * fleet.cartPrice;
                res.expenditure = ExpenditureType::rideConstruction;
                break;
            }
            case CartEditType::sell:
                if (_value < 1 || _value > kMaxCartsAtOnce || fleet.freeCarts() == 0)
                    return Result(Status::invalidParameters, STR_CART_CANT_CHANGE, kStringIdNone);
                break;
            case CartEditType::setFee:
                if (_value < 0 || _value > kMaxCartFee)
                    return Result(Status::invalidParameters, STR_CART_CANT_CHANGE, kStringIdNone);
                break;
            default:
                return Result(Status::invalidParameters, STR_CART_CANT_CHANGE, kStringIdNone);
        }
        return res;
    }

    Result CartEditAction::Execute(GameState_t& gameState, Park::ParkData& park) const
    {
        auto res = Query(gameState, park);
        if (res.error != Status::ok)
            return res;

        auto& fleet = Golf::runtimeOf(gameState).director().fleet;
        switch (_type)
        {
            case CartEditType::buy:
                fleet.buyCarts(static_cast<int32_t>(_value));
                break;
            case CartEditType::sell:
                fleet.sellFreeCarts(static_cast<int32_t>(_value));
                break;
            case CartEditType::setFee:
                fleet.cartFee = _value;
                break;
        }
        return res;
    }
} // namespace OpenRCT2::GameActions
