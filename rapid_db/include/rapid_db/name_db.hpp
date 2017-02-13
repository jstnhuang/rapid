#ifndef _RAPID_NAME_DB_H_
#define _RAPID_NAME_DB_H_

#include <string>
#include <vector>

#include "boost/shared_ptr.hpp"
#include "mongodb_store/message_store.h"
#include "ros/node_handle.h"

namespace rapid {
namespace db {
// NameDb is a simple database for named ROS messages based on mongodb_store.
//
// In NameDb, messages are stored in flat collections. Collections are
// themselves stored in databases. As a guideline, each project should use its
// own database, and messages are organized within that database as needed. We
// also recommend that point clouds be stored separately from other data
// associated with the point cloud.
//
// All messages have a name. You insert, query, and delete messages by name.
// Names should be unique. If they are not, then Get, Update, and Delete will
// operate on an arbitrary message of that name.
//
// Because we are using names, operations may be slow if the collection grows
// large. In that case, you should create an index manually through the MongoDB
// shell. For example, if your database is "mydb" and your collection is
// "mycollection":
//   use mydb;
//   db.mycollection.createIndex({"_meta.name": 1})
//
// ROS must be running to use this class.
class NameDb {
 public:
  NameDb(const ros::NodeHandle& nh, const std::string& database,
         const std::string& collection);

  template <typename MsgType>
  void Insert(const std::string& name, const MsgType& msg);

  template <typename MsgType>
  void List(std::vector<std::string>* names);

  template <typename MsgType>
  bool Get(const std::string& name, MsgType* msg);

  template <typename MsgType>
  bool Update(const std::string& name, const MsgType& msg);

  template <typename MsgType>
  bool Delete(const std::string& name);

  mongodb_store::MessageStoreProxy* proxy();

 private:
  ros::NodeHandle nh_;
  std::string database_;
  std::string collection_;
  mongodb_store::MessageStoreProxy proxy_;
};

template <typename MsgType>
void NameDb::Insert(const std::string& name, const MsgType& msg) {
  std::string id = proxy_.insertNamed(name, msg);
  // Duplicate the ID in the _meta document, since there's no other way to
  // retrieve the _id just by knowing the name.
  mongo::BSONObj meta = BSON("_id" << id);
  proxy_.updateID(id, msg, meta);
}

template <typename MsgType>
void NameDb::List(std::vector<std::string>* names) {
  std::vector<std::pair<boost::shared_ptr<MsgType>, mongo::BSONObj> > messages;
  mongo::BSONObj message_query;
  mongo::BSONObj meta_query;
  mongo::BSONObj sort_query;
  mongo::BSONObj projection_query = BSON("_meta" << 1);
  const bool kFindOne = false;
  const bool kDecodeMetas = true;
  const int kLimit = 0;
  bool success = proxy_.queryWithProjection(messages, message_query, meta_query,
                                            sort_query, projection_query,
                                            kFindOne, kDecodeMetas, kLimit);
  if (!success) {
    return;
  }

  names->clear();
  for (size_t i = 0; i < messages.size(); ++i) {
    const mongo::BSONObj& meta = messages[i].second;
    const std::string& name = meta.getStringField("name");
    names->push_back(name);
  }
}

template <typename MsgType>
bool NameDb::Get(const std::string& name, MsgType* msg) {
  const bool kFindOne = true;
  std::vector<boost::shared_ptr<MsgType> > results;
  bool success = proxy_.queryNamed(name, results, kFindOne);
  if (!success || results.size() == 0) {
    return false;
  }
  *msg = *(results[0]);
  return true;
}

template <typename MsgType>
bool NameDb::Update(const std::string& name, const MsgType& msg) {
  return proxy_.updateNamed(name, msg);
}

template <typename MsgType>
bool NameDb::Delete(const std::string& name) {
  std::vector<std::pair<boost::shared_ptr<MsgType>, mongo::BSONObj> > messages;
  mongo::BSONObj message_query;
  mongo::BSONObj meta_query = BSON("name" << name);
  mongo::BSONObj sort_query;
  mongo::BSONObj projection_query = BSON("_id" << 1 << "_meta" << 1);
  const bool kFindOne = false;
  const bool kDecodeMetas = true;
  const int kLimit = 0;
  bool success = proxy_.queryWithProjection(messages, message_query, meta_query,
                                            sort_query, projection_query,
                                            kFindOne, kDecodeMetas, kLimit);
  if (!success || messages.size() == 0) {
    return false;
  }

  const mongo::BSONObj& meta = messages[0].second;
  const std::string& id = meta.getStringField("_id");
  return proxy_.deleteID(id);
}
}  // namespace db
}  // namespace rapid

#endif  // _RAPID_DB_NAME_DB_H_
