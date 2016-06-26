#include "colored_blob_perspective_camera_default_sensor.h"
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/perspective_camera_equipped_entity.h>

namespace argos {

void CustomPerspectiveCameraLeft::SetRobot(CComposableEntity& c_entity) {
   /* Get omndirectional camera equipped entity */
   m_pcCamEntity = &(c_entity.GetComponent<CPerspectiveCameraEquippedEntity>("perspective_camera_left"));
   /* Get controllable entity */
   m_pcControllableEntity = &(c_entity.GetComponent<CControllableEntity>("controller"));
   /* Get embodied entity */
   m_pcEmbodiedEntity = &(c_entity.GetComponent<CEmbodiedEntity>("body"));
}

void CustomPerspectiveCameraRight::SetRobot(CComposableEntity& c_entity) {
   /* Get omndirectional camera equipped entity */
   m_pcCamEntity = &(c_entity.GetComponent<CPerspectiveCameraEquippedEntity>("perspective_camera_right"));
   /* Get controllable entity */
   m_pcControllableEntity = &(c_entity.GetComponent<CControllableEntity>("controller"));
   /* Get embodied entity */
   m_pcEmbodiedEntity = &(c_entity.GetComponent<CEmbodiedEntity>("body"));
}

void CustomPerspectiveCameraFront::SetRobot(CComposableEntity& c_entity) {
   /* Get omndirectional camera equipped entity */
   m_pcCamEntity = &(c_entity.GetComponent<CPerspectiveCameraEquippedEntity>("perspective_camera_front"));
   /* Get controllable entity */
   m_pcControllableEntity = &(c_entity.GetComponent<CControllableEntity>("controller"));
   /* Get embodied entity */
   m_pcEmbodiedEntity = &(c_entity.GetComponent<CEmbodiedEntity>("body"));
}

void CustomPerspectiveCameraBack::SetRobot(CComposableEntity& c_entity) {
   /* Get omndirectional camera equipped entity */
   m_pcCamEntity = &(c_entity.GetComponent<CPerspectiveCameraEquippedEntity>("perspective_camera_back"));
   /* Get controllable entity */
   m_pcControllableEntity = &(c_entity.GetComponent<CControllableEntity>("controller"));
   /* Get embodied entity */
   m_pcEmbodiedEntity = &(c_entity.GetComponent<CEmbodiedEntity>("body"));
}

REGISTER_SENSOR(CustomPerspectiveCameraLeft,
                "perspective_camera_left", "default",
                "Paweł Jakubowski", "1.0",
                "A custom perspective camera sensor to detect colored blobs.",
                "", "Usable"
);

REGISTER_SENSOR(CustomPerspectiveCameraRight,
                "perspective_camera_right", "default",
                "Paweł Jakubowski", "1.0",
                "A custom perspective camera sensor to detect colored blobs.",
                "", "Usable"
);

REGISTER_SENSOR(CustomPerspectiveCameraFront,
                "perspective_camera_front", "default",
                "Paweł Jakubowski", "1.0",
                "A custom perspective camera sensor to detect colored blobs.",
                "", "Usable"
);

REGISTER_SENSOR(CustomPerspectiveCameraBack,
                "perspective_camera_back", "default",
                "Paweł Jakubowski", "1.0",
                "A custom perspective camera sensor to detect colored blobs.",
                "", "Usable"
);

}
