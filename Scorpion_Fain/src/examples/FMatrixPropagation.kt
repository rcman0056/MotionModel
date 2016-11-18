import golem.*
import golem.containers.*
import golem.matrix.*
import modules.stateblocks.ins.EarthModel
import modules.stateblocks.ins.Pinson15NEDBlock
import modules.stateblocks.ins.getImuModelHG1700
import navutils.containers.NavSolution
import navutils.containers.Pose
import navutils.containers.Vector3
import org.knowm.xchart.BitmapEncoder

/**
 *  An example which plots the pinson-block propagated error from different initial INS errors.
 */
object FMatrixPropagation {
    private var block: Pinson15NEDBlock = Pinson15NEDBlock(label = "pinson15", imuModel = getImuModelHG1700())
    private var solution = NavSolution(pose = Pose(rotMat = eye(3),
                                                   pos = Vector3(45.0 * PI / 180.0, 0.0, 0.0),
                                                   time = Time(0.0)), vel = zeros(3, 1))
    private var earth = EarthModel(pos = solution.pose.pos.matrix, vel = solution.vel)
    private var att_err: Matrix<Double> = zeros(3, 1)
    private val dt: Int = 10

    val ARCSEC_2_RAD = (1.0 / 3600.0) * (PI / 180.0)      // convert from arc-seconds to radians
    val RAD_2_ARCSEC = 1 / ARCSEC_2_RAD               // convert from radians to arc-seconds
    val DPH_2_RPS = (PI / 180.0) / 3600.0               // convert from deg/hr to rad/sec

    @JvmStatic
    fun main(args: Array<String>) {
        testAccelXBiasError()
        testAccelYBiasError()
        testAccelZBiasError()
        testGyroXBiasError()
        testGyroYBiasError()
        testGyroZBiasError()
        testMultiStateError()
        testNorthTiltError()
        testNorthVelError()
    }
    //@formatter:off

    fun testAccelXBiasError() {
        val u = mat[100e-6 * 9.81, // x-axis accelerometer bias error
                    0, // y-axis accelerometer bias error
                    0, // z-axis accelerometer bias error
                    0, // x-axis gyroscope bias error
                    0, // y-axis gyroscope bias error
                    0]                    // z-axis gyroscope bias error
        val x = mat[0, 0, 0, // nav frame (NED) position error states
                    0, 0, 0, // nav frame (NED) velocity error states
                    0, 0, 0, // nav frame (NED) attitude (roll, pitch, yaw) error states
                    0, 0, 0, // body frame accelerometer bias error states
                    0, 0, 0]                        // body frame gyroscope bias error states

        genericPinson15ModelTest(x0 = x.T, u = u.T, title = "100 uG X-Axis Accel Bias Error", fig = 0)
    }

    fun testAccelYBiasError() {
        val u = mat[0, // x-axis accelerometer bias error
                    100e-6 * 9.81, // y-axis accelerometer bias error
                    0, // z-axis accelerometer bias error
                    0, // x-axis gyroscope bias error
                    0, // y-axis gyroscope bias error
                    0]                    // z-axis gyroscope bias error
        val x = mat[0, 0, 0, // nav frame (NED) position error states
                    0, 0, 0, // nav frame (NED) velocity error states
                    0, 0, 0, // nav frame (NED) attitude (roll, pitch, yaw) error states
                    0, 0, 0, // body frame accelerometer bias error states
                    0, 0, 0]                        // body frame gyroscope bias error states

        genericPinson15ModelTest(x0 = x.T, u = u.T, title = "100 uG Y-Axis Accel Bias Error", fig = 1)
    }

    fun testAccelZBiasError() {
        val u = mat[0, // x-axis accelerometer bias error
                    0, // y-axis accelerometer bias error
                    100e-6 * 9.81, // z-axis accelerometer bias error
                    0, // x-axis gyroscope bias error
                    0, // y-axis gyroscope bias error
                    0]                    // z-axis gyroscope bias error
        val x = mat[0, 0, 0, // nav frame (NED) position error states
                    0, 0, 0, // nav frame (NED) velocity error states
                    0, 0, 0, // nav frame (NED) attitude (roll, pitch, yaw) error states
                    0, 0, 0, // body frame accelerometer bias error states
                    0, 0, 0]                        // body frame gyroscope bias error states

        genericPinson15ModelTest(x0 = x.T, u = u.T, title = "100 uG Z-Axis Accel Bias Error", fig = 2)
    }

    fun testGyroXBiasError() {
        val u = mat[0, // x-axis accelerometer bias error
                    0, // y-axis accelerometer bias error
                    0, // z-axis accelerometer bias error
                    0.001 * DPH_2_RPS, // x-axis gyroscope bias error
                    0, // y-axis gyroscope bias error
                    0]                // z-axis gyroscope bias error
        val x = mat[0, 0, 0, // nav frame (NED) position error states
                    0, 0, 0, // nav frame (NED) velocity error states
                    0, 0, 0, // nav frame (NED) attitude (roll, pitch, yaw) error states
                    0, 0, 0, // body frame accelerometer bias error states
                    0, 0, 0]                        // body frame gyroscope bias error states

        genericPinson15ModelTest(x0 = x.T, u = u.T, title = "0.001 deg/hr X-Axis Gyro Bias Error", fig = 3)
    }

    fun testGyroYBiasError() {
        val u = mat[0, // x-axis accelerometer bias error
                    0, // y-axis accelerometer bias error
                    0, // z-axis accelerometer bias error
                    0, // x-axis gyroscope bias error
                    0.001 * DPH_2_RPS, // y-axis gyroscope bias error
                    0]                // z-axis gyroscope bias error
        val x = mat[0, 0, 0, // nav frame (NED) position error states
                    0, 0, 0, // nav frame (NED) velocity error states
                    0, 0, 0, // nav frame (NED) attitude (roll, pitch, yaw) error states
                    0, 0, 0, // body frame accelerometer bias error states
                    0, 0, 0]                        // body frame gyroscope bias error states

        genericPinson15ModelTest(x0 = x.T, u = u.T, title = "0.001 deg/hr Y-Axis Gyro Bias Error", fig = 4)
    }

    fun testGyroZBiasError() {
        val u = mat[0, // x-axis accelerometer bias error
                    0, // y-axis accelerometer bias error
                    0, // z-axis accelerometer bias error
                    0, // x-axis gyroscope bias error
                    0, // y-axis gyroscope bias error
                    0.001 * DPH_2_RPS]                  // z-axis gyroscope bias error
        val x = mat[0, 0, 0, // nav frame (NED) position error states
                    0, 0, 0, // nav frame (NED) velocity error states
                    0, 0, 0, // nav frame (NED) attitude (roll, pitch, yaw) error states
                    0, 0, 0, // body frame accelerometer bias error states
                    0, 0, 0]                        // body frame gyroscope bias error states

        genericPinson15ModelTest(x0 = x.T, u = u.T, title = "0.001 deg/hr Z-Axis Gyro Bias Error", fig = 5)
    }

    fun testNorthVelError() {
        val u = mat[0, // x-axis accelerometer bias error
                    0, // y-axis accelerometer bias error
                    0, // z-axis accelerometer bias error
                    0, // x-axis gyroscope bias error
                    0, // y-axis gyroscope bias error
                    0]                              // z-axis gyroscope bias error
        val x = mat[0, 0, 0, // nav frame (NED) position error states
                    0.1, 0, 0, // nav frame (NED) velocity error states
                    0, 0, 0, // nav frame (NED) attitude (roll, pitch, yaw) error states
                    0, 0, 0, // body frame accelerometer bias error states
                    0, 0, 0]                      // body frame gyroscope bias error states

        genericPinson15ModelTest(x0 = x.T, u = u.T, title = "0.1 m/s North Velocity Error", fig = 6)
    }

    fun testNorthTiltError() {
        val u = mat[0, // x-axis accelerometer bias error
                    0, // y-axis accelerometer bias error
                    0, // z-axis accelerometer bias error
                    0, // x-axis gyroscope bias error
                    0, // y-axis gyroscope bias error
                    0]                              // z-axis gyroscope bias error
        val x = mat[0, 0, 0, // nav frame (NED) position error states
                    0, 0, 0, // nav frame (NED) velocity error states
                    -10 * ARCSEC_2_RAD, 0, 0, // nav frame (NED) attitude (roll, pitch, yaw) error states
                    0, 0, 0, // body frame accelerometer bias error states
                    0, 0, 0]         // body frame gyroscope bias error states

        genericPinson15ModelTest(x0 = x.T, u = u.T, title = "10 arc-sec North Tilt Error", fig = 7)
    }

    fun testMultiStateError() {
        val u = mat[100e-6 * 9.81, // x-axis accelerometer bias error
                    0, // y-axis accelerometer bias error
                    0, // z-axis accelerometer bias error
                    0, // x-axis gyroscope bias error
                    0.01 * DPH_2_RPS, // y-axis gyroscope bias error
                    0]                 // z-axis gyroscope bias error
        val x = mat[0, 0, 0, // nav frame (NED) position error states
                    0, 0, 0, // nav frame (NED) velocity error states
                    0, 0, 0, // nav frame (NED) attitude (roll, pitch, yaw) error states
                    0, 0, 0, // body frame accelerometer bias error states
                    0, 0, 0]                        // body frame gyroscope bias error states

        genericPinson15ModelTest(x0 = x.T,
                                 u = u.T,
                                 title = "100 uG X-Axis Accel Bias and 0.01 deg/hr Y-Axis Gyro Bias",
                                 fig = 8)
    }
    //@formatter:on

    private fun genericPinson15ModelTest(x0: Matrix<Double>, u: Matrix<Double>, title: String, fig: Int) {
        val A = block.generateFPinson15(pos = solution.pose.pos.matrix,
                                        vel = solution.vel,
                                        force = mat[0.0, 0.0, -9.81],
                                        imuModel = block.imuModel,
                                        Cns = solution.pose.rotMat)
        val B = zeros(15, 6)
        val Cnb = solution.pose.rotMat
        B[3..5, 0..2] = Cnb
        B[6..8, 3..5] = -Cnb
        //        println("A=\n$A\nB=\n$B\n")

        val (m, n) = Pair(A.numRows(), A.numCols())
        val (mb, nb) = Pair(B.numRows(), B.numCols())
        val s = expm(vstack(hstack(A, B) * dt, zeros(nb, n + nb)))
        val Ad = s[0..n - 1, 0..n - 1]
        //        println("Ad.shape=${Ad.shape()}\n")
        val Bd = s[0..n - 1, n..n + nb - 1]
        //        println("Bd.shape=${Bd.shape()}\n")

        var x = x0
        val time = Array(4 * 3600 / dt, { i -> i * dt / 3600.0 }).toDoubleArray()
        //        println("time.size=${time.size}\n")
        //        println("time[0..3]=${time[0]}, ${time[1]}, ${time[2]}, ${time[3]}\n")
        val lat_err = zeros(time.size)
        val lon_err = zeros(time.size)
        val roll_err = zeros(time.size)
        val pitch_err = zeros(time.size)
        val yaw_err = zeros(time.size)
        val vn_err = zeros(time.size)
        val ve_err = zeros(time.size)
        val vd_err = zeros(time.size)
        for ((idx, t) in time.withIndex()) {
            //            println("(Ad*x).shape=${(Ad*x).shape()}\n")
            //            println("(Bd*u).shape=${(Bd*u).shape()}\n")
            x = Ad * x + Bd * u                         // update system state
            x[2] = solution.pose.pos.matrix[2]      // assume perfect aiding of vertical axis error

            val err_state = errStateCorrection(x, solution.pose.pos.matrix)

            lat_err[idx] = err_state.pos_err[0] * RAD_2_ARCSEC
            lon_err[idx] = err_state.pos_err[1] * RAD_2_ARCSEC

            vn_err[idx] = err_state.vel_err[0]
            ve_err[idx] = err_state.vel_err[1]
            vd_err[idx] = err_state.vel_err[2]

            roll_err[idx] = err_state.att_err[0] * RAD_2_ARCSEC
            pitch_err[idx] = err_state.att_err[1] * RAD_2_ARCSEC
            yaw_err[idx] = err_state.att_err[2] * RAD_2_ARCSEC
        }

        figure(fig * 3)
        plot(time, lon_err, "g", "Lon")
        plot(time, lat_err, "b", "Lat")
        title("Pinson15 Position Error: " + title)
        xlabel("Time (hr)")
        ylabel("Position Error (arc-sec)")

        figure(fig * 3 + 1)
        plot(time, vn_err, "g", "North")
        plot(time, ve_err, "b", "East")
        plot(time, vd_err, "r", "Down")
        title("Pinson15 Velocity Error: " + title)
        xlabel("Time (hr)")
        ylabel("Velocity Error (m/s)")

        figure(fig * 3 + 2)
        plot(time, roll_err, "b", "Roll")
        plot(time, pitch_err, "g", "Pitch")
        plot(time, yaw_err, "r", "Yaw")
        title("Pinson15 Orientation Error: " + title)
        xlabel("Time (hr)")
        ylabel("Orientation Error (arc-sec)")

        Thread.sleep(500)
        var c = figures[fig * 3]!!.first
        BitmapEncoder.saveBitmapWithDPI(c, "./${fig}_PosErr", BitmapEncoder.BitmapFormat.PNG, 300);

        Thread.sleep(500)
        c = figures[fig * 3 + 1]!!.first
        BitmapEncoder.saveBitmapWithDPI(c, "./${fig}_VelErr", BitmapEncoder.BitmapFormat.PNG, 300);

        Thread.sleep(500)
        c = figures[fig * 3 + 2]!!.first
        BitmapEncoder.saveBitmapWithDPI(c, "./${fig}_TiltErr", BitmapEncoder.BitmapFormat.PNG, 300);

    }

    data class Pinson15ErrState(var pos_err: Matrix<Double>, var vel_err: Matrix<Double>, var att_err: Matrix<Double>, var f_err: Matrix<Double>)

    private fun errStateCorrection(xhat: Matrix<Double>, pos: Matrix<Double>): Pinson15ErrState {
        //        println("xhat.shape=${xhat.shape()}\n")

        val dr = xhat[0..2, 0]                      // nav frame (NED) position error
        val vel = xhat[3..5, 0]                     // nav frame (NED) velocity error
        val psi = xhat[6..8, 0]                     // nav frame (NED) tilt error
        //        println("dr.shape=${dr.shape()}\n")
        //        println("vel.shape=${vel.shape()}\n")
        //        println("psi.shape=${psi.shape()}\n")

        // Determine error corrections
        val dtheta = zeros(3, 1)                    // nav frame (NED) angular position error, (rad)
        dtheta[0] = dr[1] / (earth.r_e + earth.alt_msl)
        dtheta[1] = -dr[0] / (earth.r_n + earth.alt_msl)
        dtheta[2] = -tan(earth.lat) * dtheta[0]
        //        println("dtheta.shape=${dtheta.shape()}\n")

        val lat_err = -dtheta[1]                    // latitude position error, (m)
        val lon_err = dtheta[0] * earth.sec_l         // longitude position error, (m)

        val phi = dtheta + psi                      // nav frame (NED) angular orientation error, (rad)
        val roll_err = -phi[0]
        val pitch_err = -phi[1]
        val yaw_err = -phi[2]

        val accel_bias = xhat[9..11, 0]         // nav frame (NED) accelerometer bias error
        val gyro_bias = xhat[12..14, 0]         // nav frame (NED) gyroscope bias error

        // integrate to get (rad)
        att_err += gyro_bias * dt

        val f_err = accel_bias                  // (m/s/s)

        return Pinson15ErrState(mat[lat_err, lon_err, dr[2]], vel, mat[roll_err, pitch_err, yaw_err], f_err)
    }

}
