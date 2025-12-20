#include "core/subsystems/odometry/swerve_pod.h"
#include <vex_rotation.h>
#include "core/utils/controls/feedback_base.h"
#include "core/utils/math/geometry/rotation2d.h"
#include "core/utils/math/geometry/translation2d.h"

SwervePod::SwervePod(vex::motor &Top, vex::motor &Bottom, Translation2d offset, Feedback &rotation_feedback, double rotation_gr, double translation_gr, double wheel_diam) :
rotation_feedback(&rotation_feedback), Top(&Top), Bottom(&Bottom), offset(offset), vex_enc(NULL), custom_enc(NULL),  rotation_gr(rotation_gr), translation_gr(translation_gr), wheel_diam(wheel_diam){
  rotation_feedback.set_limits(-100, 100);
};

SwervePod::SwervePod(vex::motor &Top, vex::motor &Bottom, vex::encoder &vex_enc, Translation2d offset, Feedback &rotation_feedback, double rotation_gr, double translation_gr, double wheel_diam) :
rotation_feedback(&rotation_feedback), Top(&Top), Bottom(&Bottom), offset(offset), vex_enc(&vex_enc), custom_enc(NULL),  rotation_gr(rotation_gr), translation_gr(translation_gr), wheel_diam(wheel_diam){
  rotation_feedback.set_limits(-100, 100);
};

SwervePod::SwervePod(vex::motor &Top, vex::motor &Bottom, CustomEncoder &custom_enc, Translation2d offset, Feedback &rotation_feedback, double rotation_gr, double translation_gr, double wheel_diam) :
rotation_feedback(&rotation_feedback), Top(&Top), Bottom(&Bottom), offset(offset), vex_enc(NULL), custom_enc(&custom_enc),  rotation_gr(rotation_gr), translation_gr(translation_gr), wheel_diam(wheel_diam){
  rotation_feedback.set_limits(-100, 100);
};

Rotation2d SwervePod::get_rotation(){
    Rotation2d Top_Rot;
    Rotation2d Bottom_Rot;
    if(custom_enc != NULL){
        return from_revolutions(custom_enc->position(vex::rotationUnits::rev));
    }
    else if(vex_enc != NULL){
        return from_revolutions(vex_enc->position(vex::rotationUnits::rev));
    }
    else{
        Top_Rot = from_revolutions(Top->position(vex::rotationUnits::rev));
        Bottom_Rot = from_revolutions(Bottom->position(vex::rotationUnits::rev));
        return from_revolutions(((Top_Rot.revolutions() + Bottom_Rot.revolutions()) / 2) * rotation_gr) + rotation_offset;
    }
}

void SwervePod::set_rotation(Rotation2d new_rot){
  if(custom_enc != NULL){
    custom_enc->setPosition(new_rot.revolutions(), vex::rotationUnits::rev);
  }
  else if(vex_enc != NULL){
    vex_enc->setPosition(new_rot.revolutions(), vex::rotationUnits::rev);
  }
  else{
    Top->resetPosition();
    Bottom->resetPosition();
    rotation_offset = new_rot;
  }
}

Translation2d SwervePod::get_location(){
  return this->offset;
}

Translation2d SwervePod::get_translation(){
  Rotation2d Top_Rot = from_revolutions(Top->position(vex::rotationUnits::rev));
  Rotation2d Bottom_Rot = from_revolutions(Bottom->position(vex::rotationUnits::rev));
  Rotation2d Translation_Rot = from_revolutions(((Top_Rot.revolutions() - Bottom_Rot.revolutions()) / 2) * translation_gr);
  double translation_magnitude =  Translation_Rot.radians() * (wheel_diam / 2);
  return Translation2d(translation_magnitude, this->get_rotation());
}

double SwervePod::get_tangential_vel(){
    double Top_vel = (Top->velocity(vex::velocityUnits::rpm));
    double Bottom_vel = (Bottom->velocity(vex::velocityUnits::rpm));
    double velocity_radpersec = ((Top_vel - Bottom_vel) / 2) * (2 * M_PI / 60);

    return  velocity_radpersec * (wheel_diam / 2);
}

double SwervePod::get_angular_vel(Translation2d center_of_rotation){
  Translation2d to_center = this->get_location() - center_of_rotation;
  return this->get_tangential_vel() * to_center.theta().f_tan() / to_center.norm();
}

void SwervePod::move_swerve(Rotation2d target_rotation, double translation_power, bool do_flip){
  Rotation2d delta_rot = from_degrees(std::abs(target_rotation.degrees() - this->get_rotation().degrees()));
  bool flip = delta_rot.degrees() > 90 && delta_rot.degrees() < 270 && do_flip;
  if(flip){
    if(target_rotation.degrees() > 0 || (target_rotation.degrees() == 0 && this->get_rotation().degrees() < 0)){
      target_rotation = target_rotation - from_degrees(180);
      translation_power *= -1;
    }
    else if(target_rotation.degrees() < 0 || (target_rotation.degrees() == 0 && this->get_rotation().degrees() > 0)){
      target_rotation = target_rotation + from_degrees(180);
      translation_power *= -1;
    }
  }
  this->rotation_feedback->init(this->get_rotation().degrees(), target_rotation.degrees());
  double rotation_out = this->rotation_feedback->update(this->get_rotation().degrees());
  Top->spin(vex::forward, rotation_out - translation_power, vex::velocityUnits::pct);
  Bottom->spin(vex::forward, rotation_out + translation_power, vex::velocityUnits::pct);
}
