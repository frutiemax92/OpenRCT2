/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#ifdef ENABLE_SCRIPTING

    #include "../../../Context.h"
    #include "../../../object/FootpathSurfaceObject.h"
    #include "../../../object/LargeSceneryObject.h"
    #include "../../../object/ObjectManager.h"
    #include "../../../object/RideObject.h"
    #include "../../../object/SceneryGroupObject.h"
    #include "../../../object/SmallSceneryObject.h"
    #include "../../ScriptEngine.h"
    #include "ScInstalledObject.hpp"

    #include <memory>
    #include <optional>

namespace OpenRCT2::Scripting
{
    class ScObject;
    extern ScObject gScObject;

    class ScObject : public ScBase
    {
    protected:
        struct ObjectData
        {
            ObjectType _type{};
            int32_t _index{};
        };

    public:
        void Register(JSContext* ctx)
        {
            static constexpr JSCFunctionListEntry kFuncs[] = {
                JS_CGETSET_DEF("installedObject", ScObject::installedObject_get, nullptr),
                JS_CGETSET_DEF("type", ScObject::type_get, nullptr),
                JS_CGETSET_DEF("index", ScObject::index_get, nullptr),
                JS_CGETSET_DEF("identifier", ScObject::identifier_get, nullptr),
                JS_CGETSET_DEF("legacyIdentifier", ScObject::legacyIdentifier_get, nullptr),
                JS_CGETSET_DEF("name", ScObject::name_get, nullptr),
                JS_CGETSET_DEF("baseImageId", ScObject::baseImageId_get, nullptr),
                JS_CGETSET_DEF("numImages", ScObject::numImages_get, nullptr),
            };
            RegisterBase(ctx, "Object", Finalise, kFuncs);
        }

        JSValue NewInstance(JSContext* ctx, ObjectType type, int32_t index)
        {
            return MakeWithOpaque(ctx, new ObjectData{ type, index });
        }

        JSValue NewDerivedInstance(JSContext* ctx, ObjectType type, int32_t index, JSValue derivedProto)
        {
            return MakeWithOpaqueAndProto(ctx, new ObjectData{ type, index }, derivedProto);
        }

        static JSValue New(JSContext* ctx, ObjectType type, int32_t index)
        {
            return gScObject.NewInstance(ctx, type, index);
        }

    private:
        static void Finalise(JSRuntime* rt, JSValue thisVal)
        {
            ObjectData* data = GetObjectData(thisVal);
            if (data)
                delete data;
        }

        static JSValue installedObject_get(JSContext* ctx, JSValue thisVal)
        {
            auto obj = GetObject(thisVal);
            if (obj != nullptr)
            {
                auto& objectRepository = GetContext()->GetObjectRepository();
                auto installedObject = objectRepository.FindObject(obj->GetDescriptor());
                if (installedObject != nullptr)
                {
                    return gScInstalledObject.New(ctx, installedObject->Id);
                }
            }
            return JS_NULL;
        }

        static JSValue type_get(JSContext* ctx, JSValue thisVal)
        {
            auto* data = GetObjectData(thisVal);
            return JSFromStdString(ctx, std::string(objectTypeToString(data->_type)));
        }

        static JSValue index_get(JSContext* ctx, JSValue thisVal)
        {
            auto* data = GetObjectData(thisVal);
            return JS_NewInt32(ctx, data->_index);
        }

        static JSValue identifier_get(JSContext* ctx, JSValue thisVal)
        {
            auto obj = GetObject(thisVal);
            std::string str;
            if (obj != nullptr)
            {
                if (obj->GetGeneration() == ObjectGeneration::dat)
                {
                    str = obj->GetDescriptor().ToString();
                }
                else
                {
                    str = obj->GetIdentifier();
                }
            }
            else
            {
                str = "";
            }
            return JSFromStdString(ctx, str);
        }

        static JSValue legacyIdentifier_get(JSContext* ctx, JSValue thisVal)
        {
            auto obj = GetObject(thisVal);
            if (obj != nullptr && obj->GetLegacyIdentifier().find('\0') == std::string::npos)
            {
                return JSFromStdString(ctx, obj->GetLegacyIdentifier());
            }
            return JSFromStdString(ctx, {});
        }

        static JSValue name_get(JSContext* ctx, JSValue thisVal)
        {
            auto obj = GetObject(thisVal);
            return JSFromStdString(ctx, obj != nullptr ? obj->GetName() : "");
        }

        static JSValue baseImageId_get(JSContext* ctx, JSValue thisVal)
        {
            auto obj = GetObject(thisVal);
            return JS_NewUint32(ctx, obj != nullptr ? obj->GetBaseImageId() : 0);
        }

        static JSValue numImages_get(JSContext* ctx, JSValue thisVal)
        {
            auto obj = GetObject(thisVal);
            return JS_NewUint32(ctx, obj != nullptr ? obj->GetNumImages() : 0);
        }

    protected:
        static ObjectData* GetObjectData(JSValue thisVal)
        {
            return gScObject.GetOpaque<ObjectData*>(thisVal);
        }

        static Object* GetObject(JSValue thisVal)
        {
            ObjectData* data = GetObjectData(thisVal);
            auto& objManager = GetContext()->GetObjectManager();
            return objManager.GetLoadedObject(data->_type, data->_index);
        }
    };

    class ScSceneryObject;
    extern ScSceneryObject gScSceneryObject;

    class ScSceneryObject : public ScObject
    {
    public:
        void Register(JSContext* ctx)
        {
            static constexpr JSCFunctionListEntry kFuncs[] = {
                JS_CGETSET_DEF("sceneryGroups", ScSceneryObject::sceneryGroups_get, nullptr),
            };
            RegisterDerived(ctx, gScObject, kFuncs);
        }

        static JSValue New(JSContext* ctx, ObjectType type, int32_t index)
        {
            return gScObject.NewDerivedInstance(ctx, type, index, gScSceneryObject.GetProto());
        }

    private:
        static JSValue sceneryGroups_get(JSContext* ctx, JSValue thisVal)
        {
            JSValue result = JS_NewArray(ctx);
            auto obj = GetSceneryObject(thisVal);
            if (obj != nullptr)
            {
                auto& scgDescriptor = obj->GetPrimarySceneryGroup();
                if (scgDescriptor.HasValue())
                {
                    JS_SetPropertyInt64(ctx, result, 0, JSFromStdString(ctx, scgDescriptor.ToString()));
                }
            }
            return result;
        }

    protected:
        static SceneryObject* GetSceneryObject(JSValue thisVal)
        {
            return static_cast<SceneryObject*>(GetObject(thisVal));
        }
    };

    class ScSmallSceneryObject;
    extern ScSmallSceneryObject gScSmallSceneryObject;

    class ScSmallSceneryObject final : public ScSceneryObject
    {
    public:
        void Register(JSContext* ctx)
        {
            static constexpr JSCFunctionListEntry kFuncs[] = {
                JS_CGETSET_DEF("flags", ScSmallSceneryObject::flags_get, nullptr),
                JS_CGETSET_DEF("height", ScSmallSceneryObject::height_get, nullptr),
                JS_CGETSET_DEF("price", ScSmallSceneryObject::price_get, nullptr),
                JS_CGETSET_DEF("removalPrice", ScSmallSceneryObject::removalPrice_get, nullptr),
            };
            RegisterDerived(ctx, gScSceneryObject, kFuncs);
        }

        static JSValue New(JSContext* ctx, ObjectType type, int32_t index)
        {
            return gScObject.NewDerivedInstance(ctx, type, index, gScSmallSceneryObject.GetProto());
        }

    private:
        static JSValue flags_get(JSContext* ctx, JSValue thisVal)
        {
            auto sceneryEntry = GetLegacyData(thisVal);
            return JS_NewUint32(ctx, sceneryEntry != nullptr ? sceneryEntry->flags.holder : 0);
        }

        static JSValue height_get(JSContext* ctx, JSValue thisVal)
        {
            auto sceneryEntry = GetLegacyData(thisVal);
            return JS_NewUint32(ctx, sceneryEntry != nullptr ? sceneryEntry->height : 0);
        }

        static JSValue price_get(JSContext* ctx, JSValue thisVal)
        {
            auto sceneryEntry = GetLegacyData(thisVal);
            return JS_NewInt64(ctx, sceneryEntry != nullptr ? sceneryEntry->price : 0);
        }

        static JSValue removalPrice_get(JSContext* ctx, JSValue thisVal)
        {
            auto sceneryEntry = GetLegacyData(thisVal);
            return JS_NewInt64(ctx, sceneryEntry != nullptr ? sceneryEntry->removal_price : 0);
        }

    protected:
        static SmallSceneryEntry* GetLegacyData(JSValue thisVal)
        {
            auto obj = GetSmallSceneryObject(thisVal);
            if (obj != nullptr)
            {
                return static_cast<SmallSceneryEntry*>(obj->GetLegacyData());
            }
            return nullptr;
        }

        static SmallSceneryObject* GetSmallSceneryObject(JSValue thisVal)
        {
            return static_cast<SmallSceneryObject*>(GetObject(thisVal));
        }
    };

    class ScLargeSceneryObjectTile;
    extern ScLargeSceneryObjectTile gScLargeSceneryObjectTile;

    class ScLargeSceneryObjectTile final : public ScBase
    {
    private:
        struct TileData
        {
            LargeSceneryTile _tile{};
        };

    public:
        void Register(JSContext* ctx)
        {
            static constexpr JSCFunctionListEntry kFuncs[] = {
                JS_CGETSET_DEF("offset", ScLargeSceneryObjectTile::offset_get, nullptr),
                JS_CGETSET_DEF("zClearance", ScLargeSceneryObjectTile::zClearance_get, nullptr),
                JS_CGETSET_DEF("hasSupports", ScLargeSceneryObjectTile::hasSupports_get, nullptr),
                JS_CGETSET_DEF("allowSupportsAbove", ScLargeSceneryObjectTile::allowSupportsAbove_get, nullptr),
                JS_CGETSET_DEF("corners", ScLargeSceneryObjectTile::corners_get, nullptr),
                JS_CGETSET_DEF("walls", ScLargeSceneryObjectTile::walls_get, nullptr),
            };
            RegisterBase(ctx, "LargeSceneryObjectTile", Finalise, kFuncs);
        }

        JSValue New(JSContext* ctx, const LargeSceneryTile& tile)
        {
            return MakeWithOpaque(ctx, new TileData{ tile });
        }

    private:
        static void Finalise(JSRuntime* rt, JSValue thisVal)
        {
            TileData* data = GetObjectData(thisVal);
            if (data)
                delete data;
        }

        static JSValue offset_get(JSContext* ctx, JSValue thisVal)
        {
            TileData* data = GetObjectData(thisVal);
            return ToJSValue(ctx, data->_tile.offset);
        }

        static JSValue zClearance_get(JSContext* ctx, JSValue thisVal)
        {
            TileData* data = GetObjectData(thisVal);
            return JS_NewInt32(ctx, data->_tile.zClearance);
        }

        static JSValue hasSupports_get(JSContext* ctx, JSValue thisVal)
        {
            TileData* data = GetObjectData(thisVal);
            return JS_NewBool(ctx, data->_tile.hasSupports);
        }

        static JSValue allowSupportsAbove_get(JSContext* ctx, JSValue thisVal)
        {
            TileData* data = GetObjectData(thisVal);
            return JS_NewBool(ctx, data->_tile.allowSupportsAbove);
        }

        static JSValue corners_get(JSContext* ctx, JSValue thisVal)
        {
            TileData* data = GetObjectData(thisVal);
            return JS_NewUint32(ctx, data->_tile.corners);
        }

        static JSValue walls_get(JSContext* ctx, JSValue thisVal)
        {
            TileData* data = GetObjectData(thisVal);
            return JS_NewUint32(ctx, data->_tile.walls);
        }

        static TileData* GetObjectData(JSValue thisVal)
        {
            return gScLargeSceneryObjectTile.GetOpaque<TileData*>(thisVal);
        }
    };

    class ScLargeSceneryObject;
    extern ScLargeSceneryObject gScLargeSceneryObject;

    class ScLargeSceneryObject final : public ScSceneryObject
    {
    public:
        void Register(JSContext* ctx)
        {
            static constexpr JSCFunctionListEntry kFuncs[] = {
                JS_CGETSET_DEF("tiles", ScLargeSceneryObject::tiles_get, nullptr),
            };
            RegisterDerived(ctx, gScSceneryObject, kFuncs);
        }

        static JSValue New(JSContext* ctx, ObjectType type, int32_t index)
        {
            return gScObject.NewDerivedInstance(ctx, type, index, gScLargeSceneryObject.GetProto());
        }

    private:
        static JSValue tiles_get(JSContext* ctx, JSValue thisVal)
        {
            JSValue result = JS_NewArray(ctx);
            auto entry = GetEntry(thisVal);
            if (entry != nullptr)
            {
                int64_t index = 0;
                for (auto& tile : entry->tiles)
                {
                    JS_SetPropertyInt64(ctx, result, index++, gScLargeSceneryObjectTile.New(ctx, tile));
                }
            }
            return result;
        }

        static LargeSceneryObject* GetLargeSceneryObject(JSValue thisVal)
        {
            return static_cast<LargeSceneryObject*>(GetObject(thisVal));
        }

        static const LargeSceneryEntry* GetEntry(JSValue thisVal)
        {
            auto obj = GetLargeSceneryObject(thisVal);
            if (obj != nullptr)
            {
                return static_cast<LargeSceneryEntry*>(obj->GetLegacyData());
            }
            return nullptr;
        }
    };

    class ScWallObject final : public ScSceneryObject
    {
    public:
        static JSValue New(JSContext* ctx, ObjectType type, int32_t index)
        {
            return ScSceneryObject::New(ctx, type, index);
        }
    };

    class ScFootpathAdditionObject final : public ScSceneryObject
    {
    public:
        static JSValue New(JSContext* ctx, ObjectType type, int32_t index)
        {
            return ScSceneryObject::New(ctx, type, index);
        }
    };

    class ScBannerObject final : public ScSceneryObject
    {
    public:
        static JSValue New(JSContext* ctx, ObjectType type, int32_t index)
        {
            return ScSceneryObject::New(ctx, type, index);
        }
    };

    class ScFootpathSurfaceObject;
    extern ScFootpathSurfaceObject gScFootpathSurfaceObject;

    class ScFootpathSurfaceObject final : public ScObject
    {
    public:
        void Register(JSContext* ctx)
        {
            static constexpr JSCFunctionListEntry kFuncs[] = {
                JS_CGETSET_DEF("flags", ScFootpathSurfaceObject::flags_get, nullptr),
            };
            RegisterDerived(ctx, gScObject, kFuncs);
        }

        static JSValue New(JSContext* ctx, ObjectType type, int32_t index)
        {
            return gScObject.NewDerivedInstance(ctx, type, index, gScFootpathSurfaceObject.GetProto());
        }

    private:
        static JSValue flags_get(JSContext* ctx, JSValue thisVal)
        {
            auto footpathObject = GetFootpathSurfaceObject(thisVal);
            return JS_NewUint32(ctx, footpathObject != nullptr ? footpathObject->Flags : 0);
        }

    protected:
        static FootpathSurfaceObject* GetFootpathSurfaceObject(JSValue thisVal)
        {
            return static_cast<FootpathSurfaceObject*>(GetObject(thisVal));
        }
    };

    class ScSceneryGroupObject;
    extern ScSceneryGroupObject gScSceneryGroupObject;

    class ScSceneryGroupObject final : public ScObject
    {
    public:
        void Register(JSContext* ctx)
        {
            static constexpr JSCFunctionListEntry kFuncs[] = {
                JS_CGETSET_DEF("items", ScSceneryGroupObject::items_get, nullptr),
            };
            RegisterDerived(ctx, gScObject, kFuncs);
        }

        static JSValue New(JSContext* ctx, ObjectType type, int32_t index)
        {
            return gScObject.NewDerivedInstance(ctx, type, index, gScSceneryGroupObject.GetProto());
        }

    private:
        static JSValue items_get(JSContext* ctx, JSValue thisVal)
        {
            JSValue result = JS_NewArray(ctx);
            auto obj = GetSceneryGroupObject(thisVal);
            if (obj != nullptr)
            {
                auto& items = obj->GetItems();
                int64_t index = 0;
                for (const auto& item : items)
                {
                    JS_SetPropertyInt64(ctx, result, index++, JSFromStdString(ctx, item.ToString()));
                }
            }
            return result;
        }

        static SceneryGroupObject* GetSceneryGroupObject(JSValue thisVal)
        {
            return static_cast<SceneryGroupObject*>(GetObject(thisVal));
        }
    };
} // namespace OpenRCT2::Scripting

#endif
