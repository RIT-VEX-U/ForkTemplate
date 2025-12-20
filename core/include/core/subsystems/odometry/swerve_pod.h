#include "core/utils/controls/feedback_base.h"
#include "core/utils/math/geometry/rotation2d.h"
#include "core/utils/controls/pid.h"
#include "core/subsystems/custom_encoder.h"
#include "core/utils/math/geometry/translation2d.h"
#include "vex.h"

class SwervePod{
  public:
    SwervePod(vex::motor &Top, vex::motor &Bottom, Translation2d offset, Feedback &rotation_feedback, double rotation_gr, double translation_gr, double wheel_diam);

    SwervePod(vex::motor &Top, vex::motor &Bottom, vex::encoder &vex_enc, Translation2d offset, Feedback &rotation_feedback, double rotation_gr, double translation_gr, double wheel_diam);

    SwervePod(vex::motor &Top, vex::motor &Bottom, CustomEncoder &custom_enc, Translation2d offset, Feedback &rotation_feedback, double rotation_gr, double translation_gr, double wheel_diam);

    Rotation2d get_rotation();

    void set_rotation(Rotation2d new_rot);

    void move_swerve(Rotation2d new_rot, double translation_power, bool do_flip);

    double get_angular_vel(Translation2d center_of_rot);

    Rotation2d optimized_angle(Rotation2d target_angle);

    Translation2d get_translation();

    Translation2d get_location();

    double get_tangential_vel();

    Feedback *rotation_feedback;
    private:
    vex::motor *Top;
    vex::motor *Bottom;
    Translation2d offset;
    vex::encoder *vex_enc;
    CustomEncoder *custom_enc;
    double rotation_gr;
    double translation_gr;
    double wheel_diam;
    Rotation2d rotation_offset = Rotation2d(0);
};
