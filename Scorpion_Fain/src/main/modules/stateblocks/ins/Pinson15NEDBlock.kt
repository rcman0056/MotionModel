package modules.stateblocks.ins

import golem.*
import golem.containers.*
import golem.matrix.*
import scorpion.filters.sensor.containers.Dynamics
import scorpion.filters.sensor.containers.StateBlock

/**
 * A 15-state representation of the error model of an inertial navigation system in NED frame.
 * States: 0..2 - Position Error (m).
 *         3..5 - Velocity Error (m/s).
 *         6..8 - Tilt Error (rad).
 *         9..11 - Accel Bias Error (m/s^2).
 *         12..14 - Gyro Bias Error (rad/s).
 *
 * @param label The label uniquely identifying this particular set of states.
 * @param imuModel An instance of ImuModel specifying the grade of the INS being used
 * @param curTime The time to initialize this block to.
 */
class Pinson15NEDBlock(override var label: String, var imuModel: ImuModel) : StateBlock {
    override var numStates: Int = 15

    private var auxData: Pinson15AuxData? = null

    /**
     * This block receives INS data via calls to this function. The parameter must be of type
     * [Pinson15AuxData], which is a container passing in an INS navigation solution and a
     * specific force vector.
     */
    override fun receiveAuxData(auxData: Any) {
        if (auxData is Pinson15AuxData) {
            this.auxData = auxData
        } else {
            throw UnsupportedOperationException("This class only accepts Pinson15AuxData instances")
        }
    }

    override fun generateDynamics(xhat: Matrix<Double>, timeFrom: Time, timeTo: Time): Dynamics {
        // Earth sidereal rate in the e frame
        var w_ie_e = mat[0, 0, 7.292115e-5]

        var aux = this.auxData ?: throw UnsupportedOperationException("Pinson15 Cannot propagate unless it first receives auxData with a Pose object")
        var pose = aux.navSolution.pose
        var vel = aux.navSolution.vel
        var force = aux.force

        val dt = timeTo.time - timeFrom.time

        val F = generateFPinson15(pos = pose.pos.matrix,
                                  vel = vel,
                                  force = force,
                                  imuModel = imuModel,
                                  Cns = pose.rotMat)
        val Q = generateQPinson15(imuModel)
        var Phi = expm(F * dt)
        var f = { x: Matrix<Double> -> Phi * x }
        // Second order approximation
        var Qd = (Phi * Q * Phi.T + Q) * dt / 2

        return Dynamics(f, Phi, Qd)
    }

    /**
     * Takes an instance of the [IMUModel] class and generates the continuous time covariance matrix Q(t) = E[w(t)w(t)^T]
     * describing the states.
     */
    fun generateQPinson15(imuModel: ImuModel): Matrix<Double> {
        var q = zeros(1, 15)
        q[0, 0..2] = zeros(1, 3)
        q[0, 3..5] = fill(rows = 1, cols = 3, value = imuModel.accelRandomWalkSigma)
        q[0, 6..8] = fill(rows = 1, cols = 3, value = imuModel.gyroRandomWalkSigma)
        q[0, 9..11] = fill(rows = 1, cols = 3, value = imuModel.accelBiasSigma * sqrt(2 / imuModel.accelBiasTau))
        q[0, 12..14] = fill(rows = 1, cols = 3, value = imuModel.gyroBiasSigma * sqrt(2 / imuModel.gyroBiasTau))

        return fill(rows = 15, cols = 15) { row, col -> if (row == col) pow(q[row], 2) else 0.0 }
    }

    //@formatter:off

    /**
     * Takes in the vehicles current position, velocity, and specific force and calculates the continuous time dynamics matrix F
     * such that x_dot = Fx + w
     *
     * @param pos The current ellipsoidal position (rad, rad, m).
     * @param vel The current NED velocity (m/s, m/s, m/s).
     * @param force The current specific force measured in the sensor frame (m/s^2, m/s^2, m/s^2).
     * @param imuModel An instance of the [IMUModel] class.
     * @param Cns INS sensor-to-nav frame DCM.
     */
    fun generateFPinson15(pos: Matrix<Double>,
                          vel: Matrix<Double>,
                          force: Matrix<Double>,
                          imuModel: ImuModel,
                          Cns: Matrix<Double>): Matrix<Double> {
        // Update the earth model
        var earth = EarthModel(pos = pos, vel = vel)

        val height = pos[2]

        val r_0 = earth.r_zero
        val g = earth.g_n[2]
        val omega_en_n = earth.omega_en_n                    // nav frame (NED) craft/transport rate, (rad/s)
        val omega_ie_n = earth.omega_ie_n                    // nav frame (NED) earth rate, (rad/s)

        // Convert body forces into the navigation frame
        var f_n = Cns * force.asColVector()

        val F = zeros(15, 15)

        // Pinson 9
        F[0..2, 0..2] = skew(-omega_en_n)                    // nav frame (NED) craft/transport rate, (rad/s)
        F[0..2, 3..5] = eye(3)                               //

        F[3..5, 0..2] = mat[-g / r_0, 0, 0 end               // nav frame (NED) gravity vector, (m/s*s)
                            0, -g / r_0, 0 end
                            0, 0, 2 * g / (r_0 + height)]
        F[3..5, 3..5] = skew(-(2 * omega_ie_n + omega_en_n)) // nav frame (NED) coriolis effect, (rad/s)
        F[3..5, 6..8] = skew(f_n)                            // nav frame (NED) specific force vector, (m/s*s)

        F[6..8, 6..8] = skew(-(omega_ie_n + omega_en_n))     // nav frame (NED) spatial rate, (rad/s)

        // Pinson 15
        F[3..5, 9..11] = Cns                                 // Add in accel bias to vdot
        F[6..8, 12..14] = -Cns                               // Add in gyro bias to tiltdot

        F[9..11, 9..11] = -eye(3) / imuModel.accelBiasTau    // Accelerometer FOGM bias
        F[12..14, 12..14] = -eye(3) / imuModel.gyroBiasTau   // Gyro FOGM bias

        return F
    }
    //@formatter:on

}