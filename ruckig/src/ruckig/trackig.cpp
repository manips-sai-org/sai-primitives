#include <ruckig/trackig.hpp>


namespace ruckig {

template class TargetState<1>;
template class TargetState<2>;
template class TargetState<3>;
template class TargetState<6>;
template class TargetState<DynamicDOFs>;

template class Trackig<1>;
template class Trackig<2>;
template class Trackig<3>;
template class Trackig<6>;
template class Trackig<DynamicDOFs>;

} // namespace ruckig
