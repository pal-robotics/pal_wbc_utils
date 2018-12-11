#include <ariles/ariles_all.h>
#include <pal_physics_utils/rbcomposite/urdf_model.h>

namespace pal_wbc
{
/// @todo AS: add interactive goals
class TaskSpaceGoalTags : public ariles::ConfigurableBase
{
#define ARILES_SECTION_ID "TaskSpaceGoalTags"
#define ARILES_ENTRIES                                                                   \
  ARILES_TYPED_ENTRY_(topic_goals, pal::rbcomposite::TagPointSet)                        \
  ARILES_TYPED_ENTRY_(interactive_goals, pal::rbcomposite::TagPointSet)
#include ARILES_INITIALIZE

public:
  TaskSpaceGoalTags()
  {
    setDefaults();
  }

  void setDefaults()
  {
    topic_goals_.setDefaults();
    interactive_goals_.setDefaults();
  }
};
}
