#pragma once

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/math/angles.h>

using Meters = argos::Real;

static const Meters MASS                = 500.0f;

static const Meters BODY_RADIUS         = 0.05f; //= 0.035f;
static const Meters BODY_HEIGHT         = 0.2f; //= 0.086f;
static const Meters BODY_ELEVATION      = 0;

static const argos::CRadians LED_RING_START_ANGLE
                                        = argos::CRadians((ARGOS_PI / 8.0f) * 0.5f);
static const Meters LED_HEIGHT          = 0.01f;
static const Meters LED_RING_RADIUS     = BODY_RADIUS + 0.007;
static const Meters LED_RING_INNER_RADIUS
                                        = BODY_RADIUS - LED_HEIGHT;
static const Meters LED_RING_ELEVATION  = BODY_HEIGHT + BODY_ELEVATION;
static const Meters RAB_ELEVATION       = 0.1f;
