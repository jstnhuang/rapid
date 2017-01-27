#include "rapid_db/name_db.h"

#include <string>

#include "ros/node_handle.h"
#include "ros/service_client.h"

namespace rapid {
namespace db {
NameDb::NameDb(const ros::NodeHandle& nh, const std::string& database,
               const std::string& collection)
    : nh_(nh),
      database_(database),
      collection_(collection),
      proxy_(nh, collection, database) {}

}  // namespace db
}  // namespace rapid
