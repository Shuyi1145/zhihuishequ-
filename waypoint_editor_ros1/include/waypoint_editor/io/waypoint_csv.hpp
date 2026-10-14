#ifndef WAYPOINT_EDITOR__IO__WAYPOINT_CSV_HPP_
#define WAYPOINT_EDITOR__IO__WAYPOINT_CSV_HPP_

#include <string>
#include <vector>

#include "waypoint_editor/core/waypoint.hpp"

namespace waypoint_editor
{
namespace io
{

class WaypointCsv
{
public:
    static bool Save(const std::vector<Waypoint> &waypoints, const std::string &path, std::string &error);
    static bool Load(const std::string &path, std::vector<Waypoint> &waypoints, std::string &error);
};

}  // namespace io
}  // namespace waypoint_editor

#endif  // WAYPOINT_EDITOR__IO__WAYPOINT_CSV_HPP_
