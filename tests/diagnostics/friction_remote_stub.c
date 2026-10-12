/* Read-only remote observer fixture; no input transport or actuator control. */
#include "friction_remote_bench.h"
FrictionRemoteBenchStatus_s remote_observer_fixture;
void FrictionRemoteBenchGetStatus(FrictionRemoteBenchStatus_s *status)
{
    *status = remote_observer_fixture;
}
