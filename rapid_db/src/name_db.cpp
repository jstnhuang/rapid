#include "rapid_db/name_db.hpp"

#include <string>

#include "mongodb_store/message_store.h"
#include "ros/node_handle.h"

namespace rapid {
namespace db {
NameDb::NameDb(const ros::NodeHandle& nh, const std::string& database,
               const std::string& collection)
    : nh_(nh),
      database_(database),
      collection_(collection),
      proxy_(nh, collection, database) {}

mongodb_store::MessageStoreProxy* NameDb::proxy() { return &proxy_; }
}  // namespace db
}  // namespace rapid
