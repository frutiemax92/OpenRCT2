/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "CourseEditAction.h"

#include "../../GameState.h"
#include "../../localisation/StringIds.h"

namespace OpenRCT2::GameActions
{
    namespace
    {
        constexpr size_t kMaxNameLength = 64;
    }

    CourseEditAction::CourseEditAction(CourseEditType type, uint32_t courseId, const std::string& name, int64_t value, int32_t index)
        : _type(type)
        , _courseId(courseId)
        , _name(name)
        , _value(value)
        , _index(index)
    {
    }

    void CourseEditAction::AcceptParameters(GameActionParameterVisitor& visitor)
    {
        visitor.Visit("type", _type);
        visitor.Visit("course", _courseId);
        visitor.Visit("name", _name);
        visitor.Visit("value", _value);
        visitor.Visit("index", _index);
    }

    uint16_t CourseEditAction::GetActionFlags() const
    {
        return GameAction::GetActionFlags();
    }

    void CourseEditAction::Serialise(DataSerialiser& stream)
    {
        GameAction::Serialise(stream);
        stream << DS_TAG(_type) << DS_TAG(_courseId) << DS_TAG(_name) << DS_TAG(_value) << DS_TAG(_index);
    }

    Result CourseEditAction::Query(GameState_t& gameState, Park::ParkData& park) const
    {
        const auto& golf = gameState.golf;
        if (_name.size() > kMaxNameLength)
            return Result(Status::invalidParameters, STR_COURSE_CANT_CHANGE, kStringIdNone);

        if (_type == CourseEditType::create)
        {
            if (golf.courses().size() >= Golf::CourseManager::kMaxCourses)
                return Result(Status::disallowed, STR_COURSE_CANT_CHANGE, STR_COURSE_TOO_MANY);
            return Result();
        }

        const auto* course = golf.findCourse(_courseId);
        if (course == nullptr)
            return Result(Status::invalidParameters, STR_COURSE_CANT_CHANGE, STR_COURSE_NOT_FOUND);

        switch (_type)
        {
            case CourseEditType::setFee:
                if (_value < 0)
                    return Result(Status::invalidParameters, STR_COURSE_CANT_CHANGE, kStringIdNone);
                break;
            case CourseEditType::setOpen:
                if (_value != 0 && _value != 1)
                    return Result(Status::invalidParameters, STR_COURSE_CANT_CHANGE, kStringIdNone);
                break;
            case CourseEditType::moveHole:
                if (_value < 0 || static_cast<size_t>(_value) >= course->holes.size() || _index < 0
                    || static_cast<size_t>(_index) >= course->holes.size())
                    return Result(Status::invalidParameters, STR_COURSE_CANT_CHANGE, STR_HOLE_NOT_FOUND);
                break;
            case CourseEditType::removeHole:
                if (course->findHole(static_cast<uint32_t>(_value)) == nullptr)
                    return Result(Status::invalidParameters, STR_COURSE_CANT_CHANGE, STR_HOLE_NOT_FOUND);
                break;
            case CourseEditType::setKind:
                if (_value != 0 && _value != 1)
                    return Result(Status::invalidParameters, STR_COURSE_CANT_CHANGE, kStringIdNone);
                break;
            case CourseEditType::setCartsAllowedOffPath:
                if (_value != 0 && _value != 1)
                    return Result(Status::invalidParameters, STR_COURSE_CANT_CHANGE, kStringIdNone);
                break;
            default:
                break;
        }
        return Result();
    }

    Result CourseEditAction::Execute(GameState_t& gameState, Park::ParkData& park) const
    {
        auto res = Query(gameState, park);
        if (res.error != Status::ok)
            return res;

        auto& golf = gameState.golf;
        CourseEditResult data;
        data.courseId = _courseId;

        if (_type == CourseEditType::create)
        {
            auto* course = golf.createCourse(_name.empty() ? "Course" : _name);
            if (course == nullptr)
                return Result(Status::disallowed, STR_COURSE_CANT_CHANGE, STR_COURSE_TOO_MANY);
            data.courseId = course->id;
            res.setData(std::move(data));
            return res;
        }
        if (_type == CourseEditType::remove)
        {
            golf.removeCourse(_courseId);
            res.setData(std::move(data));
            return res;
        }

        auto* course = golf.findCourse(_courseId);
        switch (_type)
        {
            case CourseEditType::rename:
                course->name = _name;
                break;
            case CourseEditType::setFee:
                course->greenFee = _value;
                break;
            case CourseEditType::setOpen:
                course->open = _value != 0;
                break;
            case CourseEditType::moveHole:
                course->moveHole(static_cast<size_t>(_value), static_cast<size_t>(_index));
                break;
            case CourseEditType::removeHole:
                data.holeId = static_cast<uint32_t>(_value);
                golf.removeHole(_courseId, data.holeId);
                break;
            case CourseEditType::setKind:
                course->kind = _value == 1 ? Golf::CourseKind::miniPutt : Golf::CourseKind::regular;
                break;
            case CourseEditType::setCartsAllowedOffPath:
                course->cartsAllowedOffPath = (_value != 0);
                break;
            default:
                break;
        }
        res.setData(std::move(data));
        return res;
    }
} // namespace OpenRCT2::GameActions
