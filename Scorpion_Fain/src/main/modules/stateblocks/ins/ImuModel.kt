package modules.stateblocks.ins

/**
 * Represents the errors associated with an IMU. Represents the IMU errors as a first-order Gauss
 * Markov bias + white noise (random walk).
 *
 * @param accelBiasSigma The steady-state sigma of the bias state (NOT the input noise)
 * @param gyroBiasSigma The steady-state sigma of the bias state (NOT the input noise)
 * @param accelBiasTau The time constant for the accelerometer's FOGM process
 * @param gyroBiasTau The time constant for the gyro's FOGM process
 * @param accelRandomWalkSigma The sigma for the accelerometer random walk process
 * @param gyroRandomWalkSigma The sigma for the gyro random walk process
 *
 */
data class ImuModel(
        @JvmField var accelRandomWalkSigma: Double,
        @JvmField var gyroRandomWalkSigma: Double,
        @JvmField var accelBiasSigma: Double,
        @JvmField var accelBiasTau: Double,
        @JvmField var gyroBiasSigma: Double,
        @JvmField var gyroBiasTau: Double)