import golem.*
import golem.containers.*
import golem.matrix.*
import insect.InertialNED
import modules.master_navs.MasterNavNED
import modules.measurements.DirectPositionMeasurementProcessor
import modules.stateblocks.ins.Pinson15AuxData
import modules.stateblocks.ins.Pinson15NEDBlock
import modules.stateblocks.ins.getImuModelHG9900
import navutils.containers.Pose
import navutils.containers.Vector3
import navutils.eastToDeltaLon
import navutils.northToDeltaLat
import scorpion.buffers.Buffer
import scorpion.filters.sensor.StandardSensorEKF
import scorpion.filters.sensor.containers.Measurement
import java.util.ArrayList
import java.util.Random

/**
 * Complete GPS/INS example using simulated data.
 */

const val PINSON_LABEL = "pinson15"  // Name of Pinson15NED block in filter
const val GPS_LABEL = "gps"  // Name of MeasurementProcessor in filter
const val E_RATE = 7.292115e-5  // Earth rate used for IMU data simulation
const val GPS_NOISE_SIGMA_METERS = 3.0  // Meters of uncertainty added to simulated GPS measurements (each axis)

val truth = simulatedTruthForGpsInsExample  // Simulated truth container

object GpsInsExample {
    /**
     * Main function. Sets up the lateinit variables, generates faked data,
     * runs the filter, and plots results
     */
    @JvmStatic
    fun main(args: Array<String>) {
        var filter = setupFilter()
        var ins = setupIns()
        var master = setupMasterNav(filter)

        // This call will not be necessary if using real data.
        val data = generateData()
        val results = runFilter(data, filter, ins, master)
        plotResults(results)
    }
}

/**
 * Initializes an EKF with a Pinson15NED state block and
 * DirectPositionMeasurementProcessor MeasurementProcessor for position
 * updates.
 */
fun setupFilter(): StandardSensorEKF {
    // Set up a filter that has a Pinson15NED state block and accepts GPS measurements.
    var filter = StandardSensorEKF(curTime = Time(truth.truthTime[0]), buffer = Buffer(standard = TimeStandard.WALL))

    // Create the desired state block- requires an IMU model for propagation noise parameters.
    var model = getImuModelHG9900()
    var block = Pinson15NEDBlock(label = PINSON_LABEL, imuModel = model)
    filter.addStateBlock(block)

    // Set the initial covariance on this state block
    var initCov = zeros(15, 15)
    initCov[0..2, 0..2] = eye(3) * pow(3.0, 2)
    initCov[3..5, 3..5] = eye(3) * 0.1
    initCov[6..8, 6..8] = eye(3) * .001
    initCov[9..11, 9..11] = eye(3) * pow(model.accelBiasSigma, 2)
    initCov[12..14, 12..14] = eye(3) * pow(model.gyroBiasSigma, 2)
    filter.setStateBlockCovariance(label = PINSON_LABEL, covariance = initCov)

    // Add a DirectPositionMeasurementProcessor for the GPS position data
    // that accepts whole-valued position measurements of lat, lon, altitude
    filter.addMeasurementProcessor(DirectPositionMeasurementProcessor(GPS_LABEL, PINSON_LABEL))
    return filter
}

/**
 * Initializes an INS class from truth for mechanizing IMU measurements.
 * using the first time and PVA found in [truth].
 */
fun setupIns(): InertialNED {
    // Initialize the inertial that will be used to mechanize the IMU
    // measurements
    val pose = Pose(rotMat = truth.truthRot0, pos = Vector3(truth.truthPos0),
                    time = Time(truth.truthTime[0]))
    return InertialNED(pose, truth.truthVel0)
}

/**
 * Creates a MasterNavNED, with the initial estimate and covariance
 * matrices of the Pinson15 block within the [filter].
 */
fun setupMasterNav(filter: StandardSensorEKF): MasterNavNED {
    var master = MasterNavNED(10.0)
    master.giveErrorState(filter.curTime, filter.getStateBlockEstimate(PINSON_LABEL),
                          sqrt(filter.getStateBlockCovariance(PINSON_LABEL).diag()))
    return master
}

/**
 * Simulates imu and GPS measurement vectors based upon truth data.
 * IMU measurements assume a constant velocity and no rotation. Random
 * noise is added to the GPS measurements based upon the GPS_NOISE_SIGMA_METERS
 * constant.
 * @return: An array list of 1x7 (imu) or 1x4 (gps) Matrix<Double>. Imu
 * is in the format of [time(s), dv_x, dv_y, dv_z, dth_x, dth_y, dth_z].
 * Gps is in [time(s), latitude(rad), longitude(rad), altitude(m)].
 *
 */
fun generateData(): ArrayList<Matrix<Double>> {
    // Timestamps for GPS measurements and IMU data
    val imuDeltaTime = 0.01
    val gpsTime = truth.truthTime.copy()
    val imuTime = arange(0.0, 60.0, imuDeltaTime)

    // Constant velocity and no rotation; imu data is (relatively) unchanging.
    // Note that in reality gravity and transport rate vary with position.
    val staticDv = mat[0.0, -3.13224089e-5, -9.78912010509615] * imuDeltaTime
    val staticDth = mat[cos(truth.truthPos[0, 0]) * E_RATE, 0.0, -sin(truth.truthPos[0, 0]) * E_RATE] * imuDeltaTime

    val ran = Random()

    // Loop over the IMU time and fill array with IMU/GPS measurements at appropriate times.
    var measurements: ArrayList<Matrix<Double>> = ArrayList(imuTime.numCols() + gpsTime.numCols())
    for (i in 0..imuTime.numCols() - 1) {
        // Add IMU measurement every step after the start time
        if (i != 0) {
            measurements.add(hstack(mat[i * imuDeltaTime], staticDv, staticDth))
        }
        // Add GPS position measurement after every 100th imu measurement.
        // IMU must be at 100 Hz for this simple indexing to work.
        if (i % 100 == 0 && i != 0) {
            var noiseNorth = northToDeltaLat(ran.nextGaussian() * GPS_NOISE_SIGMA_METERS, truth.truthPos[i / 100, 0])
            var noiseEast = eastToDeltaLon(ran.nextGaussian() * GPS_NOISE_SIGMA_METERS, truth.truthPos[i / 100, 0])
            var noiseDown = ran.nextGaussian() * GPS_NOISE_SIGMA_METERS
            var measNoise = mat[noiseNorth, noiseEast, noiseDown]
            measurements.add(hstack(mat[i * imuDeltaTime], truth.truthPos[i / 100, 0..2] + measNoise))
        }
    }
    return measurements
}


/**
 * Loops over a list containing imu measurements and position updates,
 * propagating and updating as appropriate and recording filter estimated
 * position solutions.
 *
 * @param measurements: An array list of 1x7 (imu) or 1x4 (gps)
 * Matrix<Double>. Imu must be in the format of [time(s), dv_x, dv_y, dv_z,
 * dth_x, dth_y, dth_z]. Gps must be [time(s), latitude(rad), longitude(rad),
 * altitude(m)].
 * @return posOut: Matrix of corrected inertial solutions over time. Each row
 * is formatted as [time(s), latitude(rad), longitude(rad), altitude(m)].
 */
fun runFilter(measurements: ArrayList<Matrix<Double>>, filter: StandardSensorEKF,
              ins: InertialNED, master: MasterNavNED): Matrix<Double> {

    // For this example, we will record the corrected Lat, Lon and Alt values
    // to plot against the truth. Solutions will be recorded after every
    // propagate step.
    var posOut = hstack(mat[0.0], truth.truthPos0)

    /******** FILTER LOOP *********/
    for (meas in measurements) {
        var t = Time(meas[0])
        if (meas.numCols() == 7) {
            // Mechanize the inertial measurements
            ins.mechanize(meas[0, 1..3], meas[0, 4..6], t)

            // Provide the Pinson15NED block with the AuxData that contains the
            // current mechanized solution so it can propagate the states.
            var insSol = ins.getSolution(t)
            filter.giveStateBlockAuxData(PINSON_LABEL,
                                         Pinson15AuxData(insSol.navSolution, insSol.force))
            filter.propagate(t)

            // Record the propagated states with MasterNav
            master.giveErrorState(t, filter.getStateBlockEstimate(PINSON_LABEL),
                                  sqrt(filter.getStateBlockCovariance(PINSON_LABEL).diag()))
        } else {
            // Calculate the error measurement using the current GPS measurement
            // and the inertial solution at the time of the GPS measurement.
            var thisGPSMeas = meas[0, 1..3]
            var insSol = ins.getSolution(t)

            // Pass the measurement into the filter. AuxData is current
            // inertial solution, and the measurementCovariance matrix
            // (R) is in m^2.
            filter.update(Measurement(GPS_LABEL, t, t, thisGPSMeas,
                                      insSol.navSolution, eye(3) * pow(GPS_NOISE_SIGMA_METERS, 2)))

            // Record the error states using MasterNav
            master.giveErrorState(t, filter.getStateBlockEstimate(PINSON_LABEL),
                                  sqrt(filter.getStateBlockCovariance(PINSON_LABEL).diag()))
        }

        // Record the current solution for plotting
        var insSol = ins.getSolution(filter.curTime)
        var sol = master.getSolution(insSol)
        posOut = vstack(posOut, hstack(mat[t.time], sol.pose.pos.T))
    }
    return posOut
}

/**
 * Generates plots of estimated vs. true position.
 * @param results: A Nx4 matrix where each row contains
 * [time (s), latitude (rad), longitude (rad), altitude (m)]
 */
fun plotResults(results: Matrix<Double>) {
    figure(1)
    plot(truth.truthTime, truth.truthPos.getCol(0), "k", "Truth Lat")
    plot(results.getCol(0), results.getCol(1), "b", "Estimated Lat")
    xlabel("Time (s)")
    ylabel("Latitude (rad)")
    title("True Latitude Vs. Estimated Latitude")

    figure(2)
    plot(truth.truthTime, truth.truthPos.getCol(1), "k", "Truth Lon")
    plot(results.getCol(0), results.getCol(2), "b", "Estimated Lon")
    xlabel("Time (s)")
    ylabel("Longitude (rad)")
    title("True Longitude Vs. Estimated Longitude")

    figure(3)
    plot(truth.truthTime, truth.truthPos.getCol(2), "k", "Truth Altitude")
    plot(results.getCol(0), results.getCol(3), "b", "Estimated Altitude")
    xlabel("Time (s)")
    ylabel("Altitude (m)")
    title("True Altitude Vs. Estimated Altitude")
}

/**
 * Object that contains simulated truth PVA at time. The data generated is
 * for a vehicle moving north at a constant velocity. Used in the GpsInsExample.
 */
object simulatedTruthForGpsInsExample {
    // Generate 0 referenced time vector based on delta time
    val truthDeltaTime = 1.0
    val truthTime = arange(0.0, 60.0, truthDeltaTime)

    // Initial position, velocity and attitude
    val truthPos0 = mat[0.5, -1.0, 1000.0]  // Lat (rad), Lon (rad), Alt (m)
    val truthVel0 = mat[10.0, 0.0, 0.0]  // Velocity NED (m/s)
    val truthRot0 = eye(3)  // Cbn (unitless)

    // Add appropriate change in latitude (in rad) based upon the north
    // velocity of the vehicle over the time step.
    val dLat = northToDeltaLat(truthDeltaTime * truthVel0[0], truthPos0[0])
    val truthPos = zeros(truthTime.numCols(), 3).fill { row, col ->
        if (col == 0)
            truthPos0[col] + dLat * row else truthPos0[col]
    }
}

