/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include <functional>
#include <openrct2/Identifiers.h>
#include <openrct2/core/StringTypes.h>
#include <openrct2/interface/WindowTypes.h>
#include <openrct2/localisation/StringIdType.h>
#include <openrct2/world/Location.hpp>
#include <openrct2/world/ScenerySelection.h>
#include <optional>
#include <string_view>

struct ScreenCoordsXY;
struct StringWithArgs;
struct RideSelection;

enum class ScatterToolDensity : uint8_t;

namespace OpenRCT2
{
    enum class GuestListFilterType : int32_t;

    class Formatter;
    struct ObjectEntryDescriptor;
    struct Peep;
    struct Ride;
    struct TileElement;
    struct Vehicle;

    using LoadSaveCallback = void (*)(ModalResult result, const utf8* path);
    using ScenarioSelectCallback = void (*)(const utf8* path);
} // namespace OpenRCT2

namespace OpenRCT2::Drawing
{
    enum class Colour : uint8_t;

    struct RenderTarget;
} // namespace OpenRCT2::Drawing

namespace OpenRCT2::Ui::Windows
{
    constexpr int32_t kTabBarHeight = 43;

    extern bool gWindowSceneryScatterEnabled;
    extern uint16_t gWindowSceneryScatterSize;
    extern ScatterToolDensity gWindowSceneryScatterDensity;
    extern uint8_t gWindowSceneryRotation;
    extern bool gWindowSceneryEyedropperEnabled;
    extern bool gDisableErrorWindowSound;

    // About
    WindowBase* AboutOpen();

    // AssetPacks
    WindowBase* AssetPacksOpen();

    // Banner
    WindowBase* BannerOpen(WindowNumber number);

    // Changelog
    WindowBase* ChangelogOpen(WindowView personality);

    // Cheats
    WindowBase* CheatsOpen();

    // ClearScenery
    WindowBase* ClearSceneryOpen();
    void ToggleClearSceneryWindow();

    // CustomCurrency
    WindowBase* CustomCurrencyOpen();

    // DateInfoPanel
    WindowBase* dateInfoPanelOpen();

    // DebugPaint
    WindowBase* DebugPaintOpen();

    // ModelViewer
    WindowBase* ModelViewerOpen();

    // ShotTool
    WindowBase* ShotToolOpen();

    // Golfer window (section 13.7), opened for the entity that shows the golfer
    WindowBase* GolferOpen(EntityId golferEntity);

    // The building window (section 9.3), for the building on a position
    WindowBase* BuildingOpen(const CoordsXY& position);

    // Golf carts (section 14)
    WindowBase* CartsOpen();

    // Street tool (section 12.1)
    WindowBase* StreetToolOpen();

    // Hole construction tool (section 16.1)
    WindowBase* HoleToolOpen();

    // Course (HOL-012)
    WindowBase* CourseOpen(uint32_t courseId = 0);

    // EditorInventionsList
    WindowBase* EditorInventionsListOpen();

    // EditorObjectSelection
    WindowBase* EditorObjectSelectionOpen();
    bool EditorObjectSelectionWindowCheck();
    void EditorObjectSelectionClose();

    // EditorParkEntrance
    WindowBase* EditorParkEntranceOpen();

    // EditorScenarioOptions
    WindowBase* EditorScenarioOptionsOpen();

    // EditorStatusLine
    WindowBase* editorStatusLineOpen();

    // EditorStepController
    WindowBase* editorStepControllerOpen();

    // Error
    WindowBase* ErrorOpen(StringId title, StringId message, const class Formatter& formatter, bool autoClose = false);
    WindowBase* ErrorOpen(std::string_view title, std::string_view message, bool autoClose = false);

    // Finances
    WindowBase* FinancesOpen();
    WindowBase* FinancesResearchOpen();
    WindowBase* FinancesMarketingOpen();

    // Footpath
    WindowBase* FootpathOpen();
    void WindowFootpathResetSelectedPath();
    void ToggleFootpathWindow();
    void WindowFootpathKeyboardShortcutTurnLeft();
    void WindowFootpathKeyboardShortcutTurnRight();
    void WindowFootpathKeyboardShortcutSlopeDown();
    void WindowFootpathKeyboardShortcutSlopeUp();
    void WindowFootpathKeyboardShortcutBuildCurrent();
    void WindowFootpathKeyboardShortcutDemolishCurrent();
    bool WindowFootpathSelectDefault();

    // GameStatusBar
    WindowBase* gameStatusBarOpen();

    // Guest
    WindowBase* GuestOpen(Peep* peep);

    // GuestList
    WindowBase* GuestListOpen();
    WindowBase* GuestListOpenWithFilter(GuestListFilterType type, int32_t index);
    void WindowGuestListRefreshList();

    // Land
    WindowBase* LandOpen();
    void ToggleLandWindow();

    // LandRights
    WindowBase* LandRightsOpen();

    // LoadSave
    WindowBase* LoadsaveOpen(
        LoadSaveAction action, LoadSaveType type, std::string_view defaultPath,
        std::function<void(ModalResult result, std::string_view)> callback, bool isJsCallback);
    void WindowLoadSaveInputKey(WindowBase* w, uint32_t keycode);

    // Main
    WindowBase* MainOpen();

    // Map
    WindowBase* MapOpen();
    void WindowMapReset();

    // MapGen
    WindowBase* MapgenOpen();

    // MapTooltip
    void SetMapTooltip(Formatter& ft);
    const Formatter& GetMapTooltip();
    void WindowMapTooltipUpdateVisibility();

    // Multiplayer
    WindowBase* MultiplayerOpen();
    void MultiplayerRefreshList();

    // NewCampaign
    WindowBase* NewCampaignOpen(int16_t campaignType);
    void WindowCampaignRefreshRides();

    // News
    WindowBase* NewsOpen();

    // NewsTicker
    WindowBase* newsTickerOpen();
    void newsTickerInvalidateNewsItem();

    // NetworkStatus
    WindowBase* NetworkStatusOpen(const std::string& text, CloseCallback onClose);
    WindowBase* NetworkStatusOpenPassword();
    void WindowNetworkStatusClose();

    // ObjectLoadError
    WindowBase* ObjectLoadErrorOpen(utf8* path, size_t numMissingObjects, const ObjectEntryDescriptor* missingObjects);

    // Options
    WindowBase* OptionsOpen();

    // OverwritePrompt
    WindowBase* WindowOverwritePromptOpen(
        std::string_view name, std::string_view path, LoadSaveAction action, LoadSaveType type);
    void WindowLoadSaveOverwritePromptInputKey(WindowBase* w, uint32_t keycode);

    // Park
    WindowBase* ParkAwardsOpen();
    WindowBase* ParkEntranceOpen();
    WindowBase* ParkGuestsOpen();
    WindowBase* ParkObjectiveOpen();
    WindowBase* ParkRatingOpen();

    // ParkInfoPanel
    WindowBase* parkInfoPanelOpen();

    // Player
    WindowBase* PlayerOpen(uint8_t id);

    // ProgressWindow
    WindowBase* ProgressWindowOpen(const std::string& text, CloseCallback onClose = nullptr);
    void ProgressWindowSet(uint32_t currentProgress, uint32_t totalCount, StringId format = kStringIdNone);
    void ProgressWindowClose();

    // PatrolArea
    WindowBase* PatrolAreaOpen(EntityId staffId);
    EntityId WindowPatrolAreaGetCurrentStaffId();

    // Research
    WindowBase* ResearchOpen();
    void WindowResearchDevelopmentMouseUp(WidgetIndex widgetIndex, WidgetIndex baseWidgetIndex);
    void WindowResearchDevelopmentPrepareDraw(WindowBase* w, WidgetIndex baseWidgetIndex);
    void WindowResearchDevelopmentDraw(WindowBase* w, Drawing::RenderTarget& rt, WidgetIndex baseWidgetIndex);
    void WindowResearchFundingMouseDown(WindowBase* w, WidgetIndex widgetIndex, WidgetIndex baseWidgetIndex);
    void WindowResearchFundingMouseUp(WidgetIndex widgetIndex, WidgetIndex baseWidgetIndex);
    void WindowResearchFundingDropdown(WidgetIndex widgetIndex, int32_t selectedIndex, WidgetIndex baseWidgetIndex);
    void WindowResearchFundingPrepareDraw(WindowBase* w, WidgetIndex baseWidgetIndex);
    void WindowResearchFundingDraw(WindowBase* w, Drawing::RenderTarget& rt);

    // SavePrompt
    WindowBase* SavePromptOpen();

    // ScenarioSelect
    WindowBase* ScenarioselectOpen(ScenarioSelectCallback callback);
    WindowBase* ScenarioselectOpen(std::function<void(std::string_view)> callback);

    // Scenery
    WindowBase* SceneryOpen();
    void WindowScenerySetSelectedItem(
        const ScenerySelection& sceneryconst, std::optional<Drawing::Colour> primary, std::optional<Drawing::Colour> secondary,
        std::optional<Drawing::Colour> tertiary, std::optional<uint8_t> rotation);
    void WindowScenerySetSelectedTab(ObjectEntryIndex sceneryGroupIndex);
    void WindowScenerySetDefaultPlacementConfiguration();
    void WindowSceneryInit();
    void WindowSceneryResetSelectedSceneryItems();
    const ScenerySelection WindowSceneryGetTabSelection();
    void ToggleSceneryWindow();

    // SceneryScatter
    WindowBase* SceneryScatterOpen();

#ifndef DISABLE_NETWORK
    // ServerList
    WindowBase* ServerListOpen();

    // ServerStart
    WindowBase* ServerStartOpen();
#endif

    // ShortcutKeys
    WindowBase* ShortcutKeysOpen();

    // Sign
    WindowBase* SignOpen(WindowNumber number);
    WindowBase* SignSmallOpen(WindowNumber number);

    // Staff
    WindowBase* StaffOpen(Peep* peep);

    // StaffFirePrompt
    WindowBase* StaffFirePromptOpen(Peep* peep);

    // StaffList
    WindowBase* StaffListOpen();
    void WindowStaffListRefresh();

    // TextInput
    void WindowTextInputKey(WindowBase* w, uint32_t keycode);
    void WindowTextInputOpen(
        WindowBase* call_w, WidgetIndex call_widget, StringId title, StringId description, const Formatter& descriptionArgs,
        StringId existing_text, uintptr_t existing_args, int32_t maxLength);
    void WindowTextInputRawOpen(
        WindowBase* call_w, WidgetIndex call_widget, StringId title, StringId description, const Formatter& descriptionArgs,
        const_utf8string existing_text, int32_t maxLength);
    void WindowTextInputOpen(
        std::string_view title, std::string_view description, std::string_view initialValue, size_t maxLength,
        std::function<void(std::string_view)> okCallback, std::function<void()> cancelCallback);

    // Themes
    WindowBase* ThemesOpen();

    // TileInspector
    WindowBase* TileInspectorOpen();
    void WindowTileInspectorClearClipboard();
    void WindowTileInspectorKeyboardShortcutToggleInvisibility();

    // TitleExit
    WindowBase* TitleExitOpen();

    // TitleLogo
    WindowBase* TitleLogoOpen();

    // TitleMenu
    WindowBase* TitleMenuOpen();

    // TitleOptions
    WindowBase* TitleOptionsOpen();

    // TitleVersion
    WindowBase* TitleVersionOpen();

    // Tooltip
    void WindowTooltipReset(const ScreenCoordsXY& screenCoords);
    void WindowTooltipShow(const StringWithArgs& message, ScreenCoordsXY screenCoords);
    void WindowTooltipOpen(WindowBase* widgetWindow, WidgetIndex widgetIndex, const ScreenCoordsXY& screenCoords);
    void WindowTooltipClose();

    // TopToolbar
    WindowBase* TopToolbarOpen();

    // Transparency
    WindowBase* TransparencyOpen();

    // ViewClipping
    WindowBase* ViewClippingOpen();

    // Viewport
    WindowBase* ViewportOpen();

    // Water
    WindowBase* WaterOpen();
    void ToggleWaterWindow();
} // namespace OpenRCT2::Ui::Windows
