#ifndef LAYERED_HARDWARE_GAZEBO_COMMON_NAMESPACES_HPP
#define LAYERED_HARDWARE_GAZEBO_COMMON_NAMESPACES_HPP

/////////////////////
// common namespaces

namespace gazebo {
namespace physics {}
} // namespace gazebo

namespace hardware_interface {}

namespace layered_hardware {}

namespace layered_hardware_gazebo {}

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
namespace gzp = gz::physics;
namespace hi = hardware_interface;
namespace lh = layered_hardware;
} // namespace layered_hardware_gazebo

#endif