/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "ProvisionalElements.h"

#include <openrct2-ui/windows/Windows.h>
#include <openrct2/profiling/Profiling.h>

using namespace OpenRCT2::Ui::Windows;

namespace OpenRCT2::Ui
{
    void ProvisionalElementsRemove()
    {
        PROFILED_FUNCTION();

        FootpathRemoveProvisionalTemporarily();
    }

    void ProvisionalElementsRestore()
    {
        PROFILED_FUNCTION();

        FootpathRestoreProvisional();
    }
} // namespace OpenRCT2::Ui
