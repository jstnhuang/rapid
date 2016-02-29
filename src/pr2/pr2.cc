#include "rapid/pr2/pr2.h"

#include "visualization_msgs/Marker.h"

#include "rapid/display/display.h"
#include "rapid/pr2/head.h"
#include "rapid/sound/sound.h"

using visualization_msgs::Marker;

namespace rapid {
namespace pr2 {
Pr2::Pr2(rapid::display::DisplayInterface& display,
         rapid::pr2::HeadInterface& head, rapid::sound::SoundInterface& sound)
    : display(display), head(head), sound(sound) {}

void GetManipulationWorkspace(Marker* ws) {
  // All measurements are in meters relative to base_link.
  // Since this is used for perception, we err on the side of a larger
  // workspace.
  double min_x = 0.2;  // PR2 can't reach closer than 20 cm in front.
  double max_x = 1.2;  // PR2 can't reach farther than 120 cm in front.
  double min_y = -1;   // PR2 can't reach farther than 100 cm right.
  double max_y = 1;    // PR2 can't reach farther than 100 cm left.
  double min_z = 0.3;  // PR2 can't reach lower than 30 cm above ground.
  double max_z = 1.7;  // PR2 can't reach higher than 170 cm above ground.
  ws->header.frame_id = "base_link";
  ws->type = Marker::CUBE;
  ws->pose.position.x = (max_x + min_x) / 2;
  ws->pose.position.y = (max_y + min_y) / 2;
  ws->pose.position.z = (max_z + min_z) / 2;
  ws->pose.orientation.w = 1;  // Assume axis-aligned box.
  ws->scale.x = max_x - min_x;
  ws->scale.y = max_y - min_y;
  ws->scale.z = max_z - min_z;
}

Pr2 BuildReal() {
  rapid::display::DisplayInterface* display = new rapid::display::Blinky();
  HeadInterface* head = new Head();
  rapid::sound::SoundInterface* sound = new rapid::sound::SoundPlay();
  Pr2 pr2(*display, *head, *sound);
  return pr2;
}
}  // namespace pr2
}  // namespace rapid
