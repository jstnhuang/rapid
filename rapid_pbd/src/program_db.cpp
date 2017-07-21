#include "rapid_pbd/program_db.h"

#include <vector>

#include "mongodb_store/message_store.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "rapid_pbd_msgs/Program.h"
#include "rapid_pbd_msgs/ProgramInfo.h"
#include "rapid_pbd_msgs/ProgramInfoList.h"

using boost::shared_ptr;
using rapid_pbd_msgs::Program;
using rapid_pbd_msgs::ProgramInfo;
using rapid_pbd_msgs::ProgramInfoList;
using sensor_msgs::PointCloud2;
using std::pair;
using std::vector;

namespace rapid {
namespace pbd {

SceneDb::SceneDb(mongodb_store::MessageStoreProxy* db) : db_(db) {}

std::string SceneDb::Insert(const PointCloud2& cloud) {
  std::string id = db_->insert(cloud);
  return id;
}

bool SceneDb::Get(const std::string& db_id, PointCloud2* cloud) const {
  std::vector<shared_ptr<PointCloud2> > results;
  bool success = db_->queryID(db_id, results);
  if (!success || results.size() < 1) {
    ROS_ERROR("Can't get scene with ID: \"%s\"", db_id.c_str());
    return false;
  }
  *cloud = *results[0];
  return true;
}

bool SceneDb::Delete(const std::string& db_id) {
  bool success = db_->deleteID(db_id);
  if (!success) {
    ROS_ERROR("Could not delete scene with ID \"%s\"", db_id.c_str());
  }
  return success;
}

ProgramDb::ProgramDb(const ros::NodeHandle& nh,
                     mongodb_store::MessageStoreProxy* db,
                     ros::Publisher* list_pub)
    : nh_(nh), db_(db), list_pub_(list_pub), program_pubs_() {}

void ProgramDb::Start() { PublishList(); }

std::string ProgramDb::Insert(const rapid_pbd_msgs::Program& program) {
  std::string id = db_->insert(program);
  PublishList();
  return id;
}

void ProgramDb::Update(const std::string& db_id,
                       const rapid_pbd_msgs::Program& program) {
  bool success = db_->updateID(db_id, program);
  if (!success) {
    ROS_ERROR("Failed to update program with ID: \"%s\"", db_id.c_str());
    return;
  }
  PublishList();
  PublishProgram(db_id);
}

void ProgramDb::StartPublishingProgramById(const std::string& db_id) {
  if (program_pubs_.find(db_id) != program_pubs_.end()) {
    return;
  }
  vector<shared_ptr<Program> > results;
  bool success = db_->queryID(db_id, results);
  if (!success || results.size() < 1) {
    ROS_ERROR("Can't start publishing program with ID: \"%s\"", db_id.c_str());
    return;
  }
  ros::Publisher pub = nh_.advertise<Program>("program/" + db_id, 1, true);
  program_pubs_[db_id] = pub;
  program_pubs_[db_id].publish(results[0]);
}

bool ProgramDb::Get(const std::string& db_id,
                    rapid_pbd_msgs::Program* program) const {
  vector<shared_ptr<Program> > results;
  bool success = db_->queryID(db_id, results);
  if (!success || results.size() < 1) {
    ROS_ERROR("Can't get program with ID: \"%s\"", db_id.c_str());
    return false;
  }
  *program = *results[0];
  return true;
}

bool ProgramDb::GetByName(const std::string& name,
                          rapid_pbd_msgs::Program* program) const {
  vector<shared_ptr<Program> > results;
  mongo::BSONObj query = BSON("name" << name);
  mongo::BSONObj meta_query;
  mongo::BSONObj sort_query;
  bool find_one = true;
  bool decode_metas = false;
  int limit = 1;

  vector<std::pair<shared_ptr<Program>, mongo::BSONObj> > msg_and_metas;
  bool success = db_->query(msg_and_metas, query, meta_query, sort_query,
                            find_one, decode_metas, limit);
  if (!success || msg_and_metas.size() < 1) {
    ROS_ERROR("Can't get program with name: \"%s\"", name.c_str());
    return false;
  }
  shared_ptr<Program> program_p = msg_and_metas[0].first;
  if (!program_p) {
    ROS_ERROR("Database returned null message for name: \"%s\"", name.c_str());
    return false;
  }
  *program = *program_p;
  return true;
}

void ProgramDb::Delete(const std::string& db_id) {
  bool success = db_->deleteID(db_id);

  if (success) {
    PublishList();
    if (program_pubs_.find(db_id) != program_pubs_.end()) {
      program_pubs_[db_id].shutdown();
      program_pubs_.erase(db_id);
    }
  } else {
    ROS_ERROR("Could not delete program with ID \"%s\"", db_id.c_str());
  }
}

void ProgramDb::PublishList() {
  if (list_pub_ == NULL) {
    return;
  }
  vector<pair<shared_ptr<Program>, mongo::BSONObj> > results;
  db_->query<Program>(results);
  ProgramInfoList msg;
  for (size_t i = 0; i < results.size(); ++i) {
    ProgramInfo info;
    info.name = results[i].first->name;
    info.db_id = results[i].second.getField("_id").OID().str();
    msg.programs.push_back(info);
  }
  list_pub_->publish(msg);
}

void ProgramDb::PublishProgram(const std::string& db_id) {
  if (program_pubs_.find(db_id) == program_pubs_.end()) {
    ROS_ERROR("No publisher for program ID: \"%s\"", db_id.c_str());
    return;
  }
  vector<shared_ptr<Program> > results;
  bool success = db_->queryID(db_id, results);
  if (!success || results.size() < 1) {
    ROS_ERROR("Could not republish program with ID: \"%s\"", db_id.c_str());
    return;
  }
  program_pubs_[db_id].publish(results[0]);
}
}  // namespace pbd
}  // namespace rapid
