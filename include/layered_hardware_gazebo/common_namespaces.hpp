#ifndef LAYERED_HARDWARE_GAZEBO_COMMON_NAMESPACES_HPP
#define LAYERED_HARDWARE_GAZEBO_COMMON_NAMESPACES_HPP

/////////////////////
// common namespaces

namespace gazebo {
namespace common {}
namespace event {}
namespace physics {}
} // namespace gazebo

namespace hardware_interface {}

namespace layered_hardware {}

namespace layered_hardware_gazebo {}

namespace transmission_interface {}

/////////////////////////
// ailias under 'gazebo'

namespace gazebo {
namespace hi = hardware_interface;
namespace lh = layered_hardware;
namespace lhg = layered_hardware_gazebo;
} // namespace gazebo

//////////////////////////////////////////
// ailias under 'layered_hardware_gazebo'

namespace layered_hardware_gazebo {
namespace gz = gazebo;
namespace gzc = gz::common;
namespace gze = gz::event;
namespace gzp = gz::physics;
namespace hi = hardware_interface;
namespace lh = layered_hardware;
namespace ti = transmission_interface;
} // namespace layered_hardware_gazebo

#endif