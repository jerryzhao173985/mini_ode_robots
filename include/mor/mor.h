/* mini_ode_robots — umbrella header.
 *
 * The single front door: `#include "mor/mor.h"` pulls in the whole engine (math,
 * physics core, joints, sensors/motors, scene objects, simulation, robot base).
 * Prefer this in applications; the individual headers remain available if you want
 * a minimal include set. Everything lives in namespace `mor`.
 */
#ifndef MOR_MOR_H
#define MOR_MOR_H

// math + types
#include "mor/osg_compat.h"
#include "mor/pose_types.h"
// physics core
#include "mor/globaldata.h"
#include "mor/odehandle.h"
#include "mor/substance.h"
#include "mor/primitive.h"
#include "mor/joint.h"
// control & sensing (attachable Sensor/Motor + concretes)
#include "mor/pid.h"
#include "mor/sensor.h"
#include "mor/motor.h"
#include "mor/servo.h"
#include "mor/jointsensor.h"
#include "mor/bodysensors.h"
#include "mor/contactsensor.h"
#include "mor/raysensor.h"
#include "mor/angularmotor.h"
// scene + observability
#include "mor/obstacles.h"
#include "mor/heightfield.h"
#include "mor/logger.h"
// simulation + robot base
#include "mor/simulation.h"
#include "mor/oderobot.h"

#endif
