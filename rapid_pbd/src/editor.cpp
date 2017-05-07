#include "rapid_pbd/editor.h"

#include "rapid_pbd/program_db.h"
#include "rapid_pbd_msgs/Program.h"

using rapid_pbd_msgs::EditorEvent;
using rapid_pbd_msgs::Program;

namespace rapid {
namespace pbd {
Editor::Editor(const ProgramDb& db) : db_(db) {}

void Editor::Start() { db_.Start(); }

void Editor::HandleEvent(const EditorEvent& event) {
  if (event.action == EditorEvent::CREATE) {
    Program program;
    program.name = event.program_info.name;
    db_.Insert(program);
  } else if (event.action == EditorEvent::UPDATE) {
    db_.Update(event.program_info.db_id, event.program);
  } else if (event.action == EditorEvent::DELETE) {
    db_.Delete(event.program_info.db_id);
  } else if (event.action == EditorEvent::OPEN) {
    db_.StartPublishingProgramById(event.program_info.db_id);
  } else {
    ROS_ERROR("Unknown event type \"%s\"", event.action.c_str());
  }
}
}  // namespace pbd
}  // namespace rapid
