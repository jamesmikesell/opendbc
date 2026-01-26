please analyze opendbc_repo/opendbc/car/nissan/carcontroller.py

this is angle based steering, meaning that we specify an angle to set the steering wheel, and a max torque allowed to be applied to reach that angle. if the steering wheel can't reach the specified angle with max torque, LKAS will just give up completely. remember that max torque only serves to apply that much torque before giving up, it will not result in the wheel actually achieving an angle faster.

it's a bad driver experience when the driver is the one applying a lot of torque to the wheel, and then the lkas max torque is reached, which then results in an abbupt cut out. though remember whenver lkas max torque is reached, it's always an abrupt cutout. it's just more unpleasuerable if a high torque is reached before cutout.

when the driver attempts to turn the wheel, we want them to feel some torque to let them know what the CS.out.steeringAngleDeg is attempting to do.

one of the problems i believe we have is that in some situations apply_std_steer_angle_limits has to limit how fast the wheel can turn.  however in many of these situations because the wheel turn rate is limited, the driver has to inetervien to try and turn the wheel faster manually. however in these situations, LKAS acts to aggressively fight the driver because LKAS is trying to keep the wheel at the specified angle (even though CS.out.steeringAngleDeg is likely the correct angle).  In this situation would it make sense to have the code realize that rate limit has been reached, and the driver is attempting to steer in the same direction as CS.out.steeringAngleDeg, thus possibly max torque should be temporarily set 0 so the driver can set the wheel position manually.  in this situation torque likely shouldn't be set to >0 until apply_std_steer_angle_limits() isn't rate limiting.

additionally, let's analyze this section:
```
  lkas_max_torque = max(
    # Scale max torque down to half LKAX_MAX_TORQUE as a minimum
    CarControllerParams.LKAS_MAX_TORQUE * 0.5,
    # Start scaling torque at STEER_THRESHOLD
    CarControllerParams.LKAS_MAX_TORQUE - 0.6 * max(0, abs(CS.out.steeringTorque) - CarControllerParams.STEER_THRESHOLD)
  )
```

given that all we can do is provide a max torque until cutout, i don't think this is doing anything useful. it's not providing a value on how much push back the wheel should have against the driver, it's just defining when LKAS will cutout, which will always converge at the same point CS.out.steeringTorque exceeds the max torque...