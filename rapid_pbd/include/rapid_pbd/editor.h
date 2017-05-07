#ifndef _RAPID_PBD_EDITOR_H_
#define _RAPID_PBD_EDITOR_H_

#include "rapid_pbd/program_db.h"
#include "rapid_pbd_msgs/EditorEvent.h"

namespace rapid {
namespace pbd {

static const char kEditorEventsTopic[] = "rapid_pbd/editor_events";

class Editor {
 public:
  Editor(const ProgramDb& db);
  void Start();
  void HandleEvent(const rapid_pbd_msgs::EditorEvent& event);

 private:
  ProgramDb db_;
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_EDITOR_H_
