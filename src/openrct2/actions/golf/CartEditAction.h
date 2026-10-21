/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../GameAction.hpp"

#include <cstdint>

namespace OpenRCT2::GameActions
{
    enum class CartEditType : uint8_t
    {
        buy,     // value = how many carts to buy; each goes to a free cart bay (CRT-002)
        sell,    // value = how many free carts to sell back
        setFee,  // value = the cart fee a rider pays (CRT-003)
    };

    // Buys and sells golf carts and sets the cart fee (CRT-002, CRT-003, ECO-011).
    class CartEditAction final : public GameActionBase<GameCommand::editCarts>
    {
    private:
        CartEditType _type{};
        int64_t _value{};

    public:
        CartEditAction() = default;
        CartEditAction(CartEditType type, int64_t value);

        void AcceptParameters(GameActionParameterVisitor&) final;

        uint16_t GetActionFlags() const override;

        void Serialise(DataSerialiser& stream) override;
        Result Query(GameState_t& gameState, Park::ParkData& park) const override;
        Result Execute(GameState_t& gameState, Park::ParkData& park) const override;
    };
} // namespace OpenRCT2::GameActions
