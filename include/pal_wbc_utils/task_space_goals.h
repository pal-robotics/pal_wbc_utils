/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#include <ariles/ariles_all.h>
#include <pal_physics_utils/rbcomposite/urdf_model.h>

namespace pal_wbc
{
class TagTaskSpaceGoal : public pal::rbcomposite::TagPoint
{
#define ARILES_SECTION_ID "TaskSpaceGoalTags"
#define ARILES_AUTO_DEFAULTS
#define ARILES_ENTRIES                                                                   \
  ARILES_PARENT(pal::rbcomposite::TagPoint)                                              \
  ARILES_TYPED_ENTRY_(reference_type, std::string)
// reference_type: interactive_marker, ref_pose_minjerk_topic,
// rigid_body_trajectory_action
#include ARILES_INITIALIZE
};


typedef pal::rbcomposite::TagSet<TagTaskSpaceGoal> TaskSpaceGoalTags;
}
