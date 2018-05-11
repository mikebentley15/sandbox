#define B_CPP

#include "B.h"

// explicitly instantiate from the template instead of specializing
template class B<int>;
template class B<float>;
