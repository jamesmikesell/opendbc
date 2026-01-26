import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.nissan import nissancan
from opendbc.car.nissan.values import CAR, CarControllerParams
from opendbc.sunnypilot.car.nissan.icbm import IntelligentCruiseButtonManagementInterface

VisualAlert = structs.CarControl.HUDControl.VisualAlert

RATE_LIMIT_TOLERANCE = 0.1  # deg
LKAS_DISABLE_DEBOUNCE_FRAMES = 10
STEER_BLEND_MAX_DELTA = 20.0  # deg


class CarController(CarControllerBase, IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    self.car_fingerprint = CP.carFingerprint
    self.apply_angle_last = 0
    self.lkas_temporarily_disabled = False
    self.lkas_disable_debounce = 0
    self.packer = CANPacker(dbc_names[Bus.pt])

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel

    can_sends = []

    ### STEER ###
    steer_hud_alert = 1 if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw) else 0

    self.lkas_disable_debounce = max(0, self.lkas_disable_debounce - 1)

    current_steering_wheel_angle = CS.out.steeringAngleDeg
    self_drive_desired_angle = actuators.steeringAngleDeg
    desired_lkas_temporarily_disabled = False

    desired_angle = 0
    if CS.out.steeringPressed:
      # this is angle based steering.  we can't specify how much torque the driver feels as the
      # self driving nudges the wheel towards the desired angle, we can only specify the max
      # torque LKAS provides to the wheel before giving up and cutting out abruptly.
      #
      # to simulate a light "nudging" torque, we're instructing the LKAS to never attempt to
      # steer to an angle more than STEER_BLEND_MAX_DELTA degrees from the angle the driver
      # currently has the wheel at
      blended_target_angle = np.clip(self_drive_desired_angle,
                              current_steering_wheel_angle - STEER_BLEND_MAX_DELTA,
                              current_steering_wheel_angle + STEER_BLEND_MAX_DELTA)

      desired_angle = apply_std_steer_angle_limits(blended_target_angle, self.apply_angle_last, CS.out.vEgoRaw,
                                                          current_steering_wheel_angle, True, CarControllerParams.ANGLE_LIMITS)

      # If the desired angle is rate-limited, drop LKAS until the desired angle is reachable without rate
      # limiting to avoid fighting the driver.
      rate_limited = abs(desired_angle - blended_target_angle) > RATE_LIMIT_TOLERANCE
      desired_lkas_temporarily_disabled = rate_limited
    else:
      desired_angle = self_drive_desired_angle

    if self.lkas_disable_debounce == 0 and desired_lkas_temporarily_disabled != self.lkas_temporarily_disabled:
      self.lkas_temporarily_disabled = desired_lkas_temporarily_disabled
      self.lkas_disable_debounce = LKAS_DISABLE_DEBOUNCE_FRAMES

    lat_active_cmd = CC.latActive and not self.lkas_temporarily_disabled
    self.apply_angle_last = apply_std_steer_angle_limits(desired_angle, self.apply_angle_last, CS.out.vEgoRaw,
                                                      current_steering_wheel_angle, lat_active_cmd, CarControllerParams.ANGLE_LIMITS)

    lkas_max_torque = 0
    if lat_active_cmd:
      # Max torque from driver before EPS will give up and not apply torque
      if not bool(CS.out.steeringPressed):
        lkas_max_torque = CarControllerParams.LKAS_MAX_TORQUE
      else:
        # Scale max torque based on how much torque the driver is applying to the wheel
        # TODO: This scaling likely doesn't add value and can probably be simplified.
        # lkas_max_torque = CarControllerParams.LKAS_MAX_TORQUE * 0.5
        lkas_max_torque = max(
          # Scale max torque down to half LKAX_MAX_TORQUE as a minimum
          CarControllerParams.LKAS_MAX_TORQUE * 0.5,
          # Start scaling torque at STEER_THRESHOLD
          CarControllerParams.LKAS_MAX_TORQUE - 0.6 * max(0, abs(CS.out.steeringTorque) - CarControllerParams.STEER_THRESHOLD)
        )

    if self.CP.carFingerprint == CAR.NISSAN_ALTIMA and pcm_cancel_cmd:
      can_sends.append(nissancan.create_acc_cancel_cmd(self.packer, self.car_fingerprint, CS.cruise_throttle_msg))

    can_sends.append(nissancan.create_steering_control(
      self.packer, self.apply_angle_last, self.frame, lat_active_cmd, lkas_max_torque))

    if self.CP.carFingerprint != CAR.NISSAN_ALTIMA and self.frame % 2 == 0:
      icbm_msg = IntelligentCruiseButtonManagementInterface.update(self, CS, CC_SP, self.packer, self.frame, self.last_button_frame)
      if pcm_cancel_cmd:
        can_sends.append(nissancan.create_cruise_throttle_msg(self.packer, self.car_fingerprint, CS.cruise_throttle_msg, self.frame, "CANCEL_BUTTON"))
      else:
        if icbm_msg:
          can_sends.extend(icbm_msg)
        else:
          can_sends.append(nissancan.create_cruise_throttle_msg(self.packer, self.car_fingerprint, CS.cruise_throttle_msg, self.frame))

    # Below are the HUD messages. We copy the stock message and modify
    if self.CP.carFingerprint != CAR.NISSAN_ALTIMA:
      if self.frame % 2 == 0:
        can_sends.append(nissancan.create_lkas_hud_msg(self.packer, CS.lkas_hud_msg, CC_SP.mads.enabled, hud_control.leftLaneVisible,
                                                       hud_control.rightLaneVisible, hud_control.leftLaneDepart, hud_control.rightLaneDepart))

      if self.frame % 50 == 0:
        can_sends.append(nissancan.create_lkas_hud_info_msg(
          self.packer, CS.lkas_hud_info_msg, steer_hud_alert
        ))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
