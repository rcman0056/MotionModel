@file:JvmName("Ins")
package modules.stateblocks.ins

import golem.*


/**
 * Returns the Honeywell HG1700 IMU model.
 */
fun getImuModelHG1700() = ImuModel(accelRandomWalkSigma = .0095, // m/s^(3/2)   (0.0143 m/s/hr^(1/2))
                                   gyroRandomWalkSigma = .0000873, // rad/s^(1/2)   (0.002 deg/hr^(1/2))
                                   accelBiasSigma = .0098, // m/s^2
                                   accelBiasTau = 3600.0, // sec
                                   gyroBiasSigma = 4.8481e-6, // rad/s  (0.0015 deg/hr))
                                   gyroBiasTau = 3600.0) // sec

/**
 * Returns the Honeywell HG9900 IMU model.
 */
fun getImuModelHG9900() = ImuModel(accelRandomWalkSigma = 0.0143 / 60.0, // m/s^(3/2)
                                   gyroRandomWalkSigma = 0.002 * PI / 180 / 60, // rad/s^(1/2)
                                   accelBiasSigma = 2.45e-4, // m/s^2
                                   accelBiasTau = 3600.0, // sec
                                   gyroBiasSigma = 0.0015 * PI / 180 / 3600, // rad/s
                                   gyroBiasTau = 3600.0) // sec

/**
 * Returns an IMU model attempting to represent a "typical" commercial grade INS, such as a cell-phone.
 */
fun getImuModelCommercial() = ImuModel(accelRandomWalkSigma = .0043, // m/s^(3/2)
                                       gyroRandomWalkSigma = 6.5e-4, // rad/s^(1/2)
                                       accelBiasSigma = .1960, // m/s^2
                                       accelBiasTau = 3600.0, // sec
                                       gyroBiasSigma = .0087, // rad/s
                                       gyroBiasTau = 3600.0) // sec
