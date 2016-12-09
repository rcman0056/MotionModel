import golem.*
import golem.containers.Time
import golem.containers.TimeStandard
import golem.matrix.Matrix
import modules.master_navs.MasterNavNED
import modules.stateblocks.ins.Pinson15AuxData
import modules.stateblocks.ins.Pinson15NEDBlock
import navutils.containers.Pose
import navutils.containers.Vector3
import scorpion.filters.sensor.StandardSensorEKF
import scorpion.filters.sensor.containers.Measurement
import insect.InertialNED
import modules.aux_data.PosVelAuxData
import modules.measurements.BiasedAltitudeMeasurementProcessor
import modules.measurements.PosVelMeasurementProcessor
import modules.stateblocks.common.FOGMBlock
import modules.stateblocks.ins.getImuModelHG1700
import navutils.*
import modules.preprocessors.Preprocessor
import scorpion.buffers.Buffer
import java.util.ArrayList

/**
 * Complete GPS/INS example using simulated getData.
 */

const val PINSON_LABEL = "pinson15"  // Name of Pinson15NED block in filter
const val FOGM1_LABEL = "fogm1"  // Name of first bias in velocity measurement block in filter
const val FOGM2_LABEL = "fogm2"  // Name of second bias in velocity measurement block in filter
const val FOGM3_LABEL = "fogm3"  // Name of third bias in velocity measurement block in filter
const val LAT0 = 0.0
const val LON0 = 0.0
const val ALT0 = 684.0
const val TERRAIN_ALT = 273.671 - 30

const val VO_ALGORITHM = "OF" //SVO, OF (optical flow)
const val USE_MEAS = true
const val RECORD_RESULTS = true
const val PERIODIC_RESETS = true
const val RESET_PERIOD = 100
const val NUM_MEAS_AVG = 1
const val SOURCE = "ASPN1" //example, ASPN1, ASPN2
const val RUN_TIME = 40 //seconds to run for 950 max for ASPN1
const val START_TIME = 0//130//3 //the EKF solution that the truth data comes from isn't accurate at first
var CAMERA_DT = 0.2 //for ASPN 0.2
const val INS_DT = 0.01
const val TRUTH_DT = 0.1

//val MEAS_COV = eye(3)*1
//val MEAS_COV = mat[10, 0, 0 end 0, 1, 0 end 0, 0, 1] * 30 * 30
val MEAS_COV = mat[100, 0, 0 end 0, 100, 0 end 0, 0, 100] * 30 * 30 * 100

val truth = initalTruthValues  //  getTruth container
var svoMeas = avgSVOMeasurementBuffer
val data = getMeasurements()
val temp = getTruth((START_TIME / TRUTH_DT).toInt())


/**
 * Simulates imu and GPS measurement vectors based upon getTruth getData.
 * IMU measurements assume a constant velocity and no rotation. Random
 * noise is added to the GPS measurements based upon the GPS_NOISE_SIGMA_METERS
 * constant.
 * @return: An array list of 1x7 (imu) or 1x4 (gps) Matrix<Double>. Imu
 * is in the format of [time(s), dv_x, dv_y, dv_z, dth_x, dth_y, dth_z].
 * Gps is in [time(s), latitude(rad), longitude(rad), altitude(m)].
 *
 */
fun getMeasurements(): ArrayList<Matrix<Double>> {
    var measurements: ArrayList<Matrix<Double>>
    val stopIndex = (RUN_TIME / INS_DT).toInt()
    val startIndex = (START_TIME / INS_DT).toInt()
    if (SOURCE == "example") {
        CAMERA_DT = 0.02
        val DATA_PATH = "/mnt/hgfs/Shared Folder/SVO_data/insData.txt"
        val delimiter = "\t"
        measurements = readTdf(DATA_PATH, delimiter, intArrayOf(7, 0, 0))
    } else {
        CAMERA_DT = 0.2
        val DATA_PATH = "/mnt/hgfs/Shared Folder/ASPNFlightForCarson/Data/13_1303_imu_itar.tdf"
        val delimiter = ","
        measurements = readTdf(DATA_PATH, delimiter, intArrayOf(7, stopIndex, startIndex))
    }

    return measurements
}

fun getTruth(startIndex: Int) {
    var META_PATH: String
    if (SOURCE == "example") {
        CAMERA_DT = 0.02
        META_PATH = "/mnt/hgfs/Shared Folder/SVO_data/metaData.txt"
    } else {
        CAMERA_DT = 0.2
        META_PATH = "/mnt/hgfs/Shared Folder/ASPNFlightForCarson/Data/meta_data.tdf"
    }

    val delimiter = ","
    val meta = readTdf(META_PATH, delimiter, intArrayOf(7, startIndex, startIndex))
    val metaData = meta.get(0)
    val rpy = mat[metaData[3], metaData[4], metaData[5]]

    truth.truthVel0 = mat[metaData[0], metaData[1], metaData[2]]
    truth.truthRot0 = rpyToDcm(rpy)
    truth.truthPos0[2] = metaData[6]
}

/**
 * Main function. Sets up the lateinit variables, generates faked getData,
 * runs the filter, and plots results
 */
fun main(args: Array<String>) {
    var filter = setupFilter()
    var ins = setupIns()
    var master = setupMasterNav(filter)
    val stateblock = filter.getStateBlockEstimate(PINSON_LABEL)

    val results = runFilter(data, filter, ins, master)

    if (RECORD_RESULTS) {
        var filename = ""
        if (USE_MEAS) {
            //filename = "out/resultsWMeasurements.txt"
            filename = "/mnt/hgfs/Shared Folder/resultsWMeasurements.txt"
        } else {
            //filename = "out/resultsWOMeasurements.txt"
            filename = "/mnt/hgfs/Shared Folder/resultsWOMeasurements.txt"
        }

        //writeToFileASCII(results, filename)
        writeToFileBinary(results, filename)
    }
    //plotResults(results)
}

/**
 * Initializes an EKF with a Pinson15NED state block and
 * DirectPositionMeasurementProcessor MeasurementProcessor for position
 * updates.
 */
fun setupFilter(): StandardSensorEKF {
    // Set up a filter that has a Pinson15NED state block and accepts GPS measurements.
    var filter = StandardSensorEKF(curTime = Time(data[0].get(0)), buffer = Buffer(standard = TimeStandard.WALL))

    // Create the desired state block- requires an IMU model for propagation noise parameters.
    //var model = getImuModelCommercial()
    //var model = getImuModelHG9900()
    var model = getImuModelHG1700()
    val scaleFactor = 1
    model.accelRandomWalkSigma = model.accelRandomWalkSigma * scaleFactor * 10
    model.gyroRandomWalkSigma = model.gyroRandomWalkSigma * scaleFactor //* 0
    model.accelBiasSigma = model.accelBiasSigma * scaleFactor * 10
    model.gyroBiasSigma = model.gyroBiasSigma * scaleFactor //* 0

    val fogmTau = mat[1, 1, 1]
    val fogmSigma = mat[10, 10, 10]
    var block = Pinson15NEDBlock(label = PINSON_LABEL, imuModel = model)
    filter.addStateBlock(block)
    //var biasBlockVel1 = FOGMBlock
    var altBiasBlock = FOGMBlock("altBias", 1.0, 1.0, 1)
    filter.addStateBlock(altBiasBlock)

    // Set the initial covariance on this state block
    var initCov = zeros(15, 15)
    initCov[0..2, 0..2] = mat[1, 0, 0 end 0, 1, 0 end 0, 0, 10] * 3 * 3
    initCov[3..5, 3..5] = eye(3) * 0.1
    initCov[6..8, 6..8] = eye(3) * .001
    initCov[9..11, 9..11] = eye(3) * pow(model.accelBiasSigma, 2)
    initCov[12..14, 12..14] = eye(3) * pow(model.gyroBiasSigma, 2)
    filter.setStateBlockCovariance(label = PINSON_LABEL, covariance = initCov)

    filter.addMeasurementProcessor(PosVelMeasurementProcessor("vo", pinsonLabel = "pinson15"))
    filter.addMeasurementProcessor(BiasedAltitudeMeasurementProcessor("alt", "pinson15", "altBias"))
    return filter
}

/**
 * Initializes an INS class from getTruth for mechanizing IMU measurements.
 * using the first time and PVA found in [truth].
 */
/**
 * Initializes an INS class from getTruth for mechanizing IMU measurements.
 * using the first time and PVA found in [truth].
 */
fun setupIns(): InertialNED {
    // Initialize the inertial that will be used to mechanize the IMU
    // measurements
    val tempTime = data[0].get(0) * 2 - data[1].get(0) //first time - dt
    val pose = Pose(rotMat = truth.truthRot0, pos = Vector3(truth.truthPos0), time = Time(tempTime))
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


    var rawCov = filter.getStateBlockCovariance(PINSON_LABEL)
    var cov = sqrt(rawCov.diag())
    var voVel = zeros(3, 1)
    var voAtt = dcmToRpy(truth.truthRot0)
    var voPos = truth.truthPos0
    var attitude = dcmToRpy(truth.truthRot0)
    var posOut = zeros(measurements.size + 1, 25)
    var tempRow = zeros(1, posOut.numCols())
    tempRow.set(0, data[0].get(0) - INS_DT)
    tempRow.set(0, 1..3, truth.truthPos0)
    tempRow.set(0, 4..6, mat[cov[0], cov[1], cov[2]])
    tempRow.set(0, 7..9, truth.truthVel0)
    tempRow.set(0, 10..12, svoMeas.avg)
    tempRow.set(0, 13..15, attitude)
    tempRow.set(0, 16..18, voAtt)
    tempRow.set(0, 19..21, voPos)
    tempRow.set(0, 22..24, mat[cov[3], cov[4], cov[5]])
    posOut.setRow(0, tempRow)


    val preprocessSVO = SVO_Carson()
    val preprocessOF = OpticalFlow_Carson()


    var timeStep: Double

    val camCal = fillCamCal(SOURCE)

    var iteration = (START_TIME / CAMERA_DT).toInt()
    var numSkipped = 0

    //if (VO_ALGORITHM == "SVO") {
    val camCalArray = doubleArrayOf(camCal.width, camCal.height, camCal.fx, camCal.fy, camCal.cx, camCal.cy)
    if (VO_ALGORITHM == "SVO")
        preprocessSVO.start(camCalArray) //start SVO
    var warmUpIteration = 0;

    var numTimesReset = 0
    var numRepeatVel = 0
    var measNum = 1

    val stepsToWarmUp: Int //first few measurements are inaccurate
    if (SOURCE == "example") {
        stepsToWarmUp = 3
    } else {
        stepsToWarmUp = 7
    }

    var svoRot0 = zeros(3, 3)
    //}

    var lastCamToNav = zeros(3, 3)
    var lastImage = intArrayOf(0)
    var measReady = false

    val aux = PosVelAuxData(PosVelAuxData.PosVelMeasurementTypes.VELOCITY)
    /******** FILTER LOOP *********/
    for (meas in measurements) {
        var t = Time(meas[0])
        if (meas.numCols() == 7) {
            // Mechanize the inertial measurements
            ins.mechanize(meas[0, 1..3], mat[meas[0, 4], meas[0, 5], meas[0, 6]], t)
            // Provide the Pinson15NED block with the AuxData that contains the
            // current mechanized solution so it can propagate the states.
            var insSol = ins.getSolution(t)
            filter.giveStateBlockAuxData(PINSON_LABEL,
                    Pinson15AuxData(insSol.navSolution, insSol.force))
            filter.propagate(t)

            // Record the propagated states with MasterNav
            master.giveErrorState(t, filter.getStateBlockEstimate(PINSON_LABEL),
                    sqrt(filter.getStateBlockCovariance(PINSON_LABEL).diag()))

            if (abs(t.time % 0.2) < 0.01) {
                if (USE_MEAS) {
                    val progress = ((t.time - START_TIME) / (RUN_TIME - START_TIME)) * 100
                    val progressString = String.format("%.2f", progress)
                    System.out.println(progressString + "% done\t")

                    //grab attitude and altitude
                    var sol = master.getSolution(insSol)
                    attitude = dcmToRpy(sol.pose.rotMat)

                    //make DCM to rotate velocities from the camera frame to the nav frame
                    var camToBodyRpy = zeros(3, 1)
                    if (SOURCE == "ASPN1")
                    //camToBodyRpy = mat[0.0039, 0.0789, -1.5587]
                        camToBodyRpy = mat[0.0039, 0.0789, -1.5587]
                    else if (SOURCE == "ASPN2")
                    //camToBodyRpy = mat[-0.0092, 0.0517, -3.1250]
                        camToBodyRpy = mat[0.0092, 0.0517, -3.1250]

                    //val camToBodyDcm = rpyToDcm(camToBodyRpy)
                    var insToBody = eye(3)

                    val camToASPNBodyDcm =
                            mat[0.999923, 0.011805, 0.003917 end
                                    0.012077, -0.996814, -0.078842 end
                                    0.002974, 0.078883, -0.996879]
                    val levelArm = mat[-0.9950 ,  -5.0080 ,  -0.9660].T;
                    /*
                    //These kinda work for both SVO and optical flow
                    var nwdToNed =
                            mat[-1, 0, 0 end
                                    0, 1, 0 end
                                    0, 0, 1]
                    if (VO_ALGORITHM == "SVO") {
                        insToBody =
                                mat[0, -1, 0 end
                                    1, 0, 0 end
                                    0, 0, 1]
                    } else {
                        insToBody =
                                mat[0, -1, 0 end
                                    1, 0, 0 end
                                    0, 0, 1]
                    }
                    */
                    /*
                    var ASPNBodyToNRDBody =
                    mat[-1, 0, 0 end
                            0, 1, 0 end
                            0, 0, 1]
                            */
                    insToBody =
                            mat[0, -1, 0 end
                                    1, 0, 0 end
                                    0, 0, 1]

                    val ASPNBodyToNRDBody =
                            mat[0, 1, 0 end
                                    1, 0, 0 end
                                    0, 0, -1]

                    //val camToNav = nwdToNed * sol.pose.rotMat * insToBody *

                    val camToNav = sol.pose.rotMat.T * ASPNBodyToNRDBody * camToASPNBodyDcm
                    val camToBodyDcm = ASPNBodyToNRDBody * camToASPNBodyDcm

                    //val camToNav = rpyToDcm(attitude).T * camToBodyDcm

                    var svoMeasurement = Measurement("", t, t, zeros(3, 1), null, eye(3))
                    var opticalFlowMeasurement = Measurement("", t, t, zeros(3, 1), null, eye(3))

                    val dt = CAMERA_DT * (numSkipped + 1)

                    //read in image
                    var path: String
                    var image: IntArray
                    if (SOURCE == "ASPN1") {
                        path = buildPathASPN1(iteration+4) //build path string
                    } else if (SOURCE == "ASPN2") {
                        path = buildPathASPN2(iteration) //build path string
                    } else {
                        //if(source == "example"){
                        path = buildPathSVOExample(iteration) //build path string
                    }
                    image = readImage.getImage(path)
                    //System.out.println(path+"\t"+t.time.toString())

                    //check if image was read in
                    if (image.size == 1) {
                        warmUpIteration = stepsToWarmUp - 3

                    } else {
                        var vTemp = zeros(1, 9)
                        if (VO_ALGORITHM == "SVO") {
                            svoMeasurement = preprocessSVO.addFrame(dt, image, (truth.truthPos0[2] - TERRAIN_ALT) / 1.1875, camToNav, t, sol.vel, "vo", MEAS_COV)
                            vTemp = preprocessSVO.getDebugData()
                        }
                        voVel = mat[vTemp[0], vTemp[1], vTemp[2]].T
                        voAtt = mat[vTemp[3], vTemp[4], vTemp[5]] //for debugging
                        voPos = mat[vTemp[6], vTemp[7], vTemp[8]] //for debugging

                        if (svoRot0 == zeros(3))
                            svoRot0 = rpyToDcm(voAtt)

                        voAtt = dcmToRpy(truth.truthRot0 * camToBodyDcm * rpyToDcm(voAtt).T * camToBodyDcm.T)

                        //if both this image and last image were read in correctly, then run optical flow
                        if (lastImage.size > 1) {
                            val height = truth.truthPos0[2] - TERRAIN_ALT
                            val f = doubleArrayOf(camCal.fx, camCal.fy)
                            val c = doubleArrayOf(camCal.cx, camCal.cy)
                            if (VO_ALGORITHM == "OF") {
                                opticalFlowMeasurement = preprocessOF.runOpticalFlow(CAMERA_DT, lastImage, image,
                                        height, f, c, lastCamToNav, camToNav, t, sol.vel, "vo", MEAS_COV)
                                measReady = true
                                voVel = preprocessOF.getDebugData()
                                //System.out.println(voVel[0])
                                if (t.time % RESET_PERIOD < 0.01)
                                    getTruth((t.time / TRUTH_DT).toInt())
                            }
                        }
                    }

                    iteration++
                    warmUpIteration++

                    val tempSol = master.getSolution(insSol)

                    if (VO_ALGORITHM == "SVO") {
                        val repeatVelocity = abs(voVel[0]) < 0.0001 && abs(voVel[1]) < 0.0001 && abs(voVel[2]) < 0.0001
                        //add measurement if filter is warmed up and was able to get an image and measurement is ready
                        if (warmUpIteration > stepsToWarmUp && numSkipped == 0) {

                            svoMeas.collected[svoMeas.count, 0..2] = voVel.T
                            svoMeas.count += 1
                            if (svoMeas.count == NUM_MEAS_AVG) {
                                svoMeas.count = 0
                                svoMeas.avg[0] = svoMeas.collected[0..NUM_MEAS_AVG - 1, 0].sum() / NUM_MEAS_AVG
                                svoMeas.avg[1] = svoMeas.collected[0..NUM_MEAS_AVG - 1, 1].sum() / NUM_MEAS_AVG
                                svoMeas.avg[2] = svoMeas.collected[0..NUM_MEAS_AVG - 1, 2].sum() / NUM_MEAS_AVG
                            }
                            //reset filter if SVO measurement did not update or is huge or SVO has been running for too long
                            val staleSVO = t.time % RESET_PERIOD < 0.01
                            val reset_filter = repeatVelocity || (staleSVO && PERIODIC_RESETS)
                            if (reset_filter) {
                                warmUpIteration = 0
                                numTimesReset++
                                if (repeatVelocity)
                                    numRepeatVel++
                                voVel = zeros(3, 1)
                                svoMeas.count = 0
                                svoMeas.collected = zeros(svoMeas.collected.numRows(), svoMeas.collected.numCols())
                                svoMeas.avg = zeros(1, 3)
                                truth.truthRot0 = insSol.navSolution.pose.rotMat
                                System.out.println("-----------------RESTARTING------------------")
                                System.out.println("repeatVelocity=" + repeatVelocity.toString())
                                preprocessSVO.stop()
                                getTruth((t.time / TRUTH_DT).toInt())
                                preprocessSVO.start(camCalArray)
                            } else if (svoMeas.count == 0) {
                                //add measurement update to filter
                                //val velMeas = makeSvoMeasurement(voVel,tempSol.vel,t)
                                //filter.update(Measurement("svo", Time(t.time), Time(t.time), velErr, aux, MEAS_COV))
                                if (VO_ALGORITHM == "SVO")
                                    filter.update(svoMeasurement)
                            }
                        } else {
                            //set SVO velocities to 0 for posterity
                            voVel = zeros(3, 1)
                            numSkipped = 0
                        }
                    } else if (VO_ALGORITHM == "OF") {
                        if (measReady) {
                            filter.update(opticalFlowMeasurement)
                            measReady = false
                        }
                    }

                    lastImage = image
                    lastCamToNav = camToNav

                } else {
                    iteration++
                    val progress = ((t.time - START_TIME) / (RUN_TIME - START_TIME)) * 100
                    val progressString = String.format("%.2f", progress)
                    System.out.println(progressString + "% done\t")
                    System.out.print(t.time.toString() + "s\t")


                }
            }


        } else {

            // Record the error states using MasterNav
            master.giveErrorState(t, filter.getStateBlockEstimate(PINSON_LABEL),
                    sqrt(filter.getStateBlockCovariance(PINSON_LABEL).diag()))
        }

        // Record the current solution for plotting
        var insSol = ins.getSolution(filter.curTime)
        var sol = master.getSolution(insSol)


        //reset INS altitude every INS_RESET_PERIOD seconds
        val INS_RESET_PERIOD = 100
        if ((t.time % INS_RESET_PERIOD < 0.001) && t.time > 0.01) {
            ins.solution = sol
            filter.setStateBlockEstimate(PINSON_LABEL, zeros(15, 1))
        }

        rawCov = filter.getStateBlockCovariance(PINSON_LABEL)
        cov = sqrt(rawCov.diag())
        if (!USE_MEAS) {
            for (i in 0..rawCov.numRows() - 1) {
                for (j in 0..rawCov.numCols() - 1) {
                    if (abs(rawCov[i, j]) > pow(10, 3)) {
                        rawCov[i, j] *= 0.01
                    }
                }
            }
            filter.setStateBlockCovariance(PINSON_LABEL, rawCov)
        }
        val attitude = dcmToRpy(sol.pose.rotMat)

        tempRow.set(0, t.time)
        tempRow.set(0, 1..3, sol.pose.pos.T)
        tempRow.set(0, 4..6, mat[cov[0], cov[1], cov[2]])
        tempRow.set(0, 7..9, sol.vel.T)
        if (VO_ALGORITHM == "SVO")
            tempRow.set(0, 10..12, svoMeas.avg)
        else
            tempRow.set(0, 10..12, voVel.T)
        tempRow.set(0, 13..15, attitude)
        tempRow.set(0, 16..18, dcmToRpy(lastCamToNav))
        tempRow.set(0, 19..21, voPos)
        tempRow.set(0, 22..24, mat[cov[3], cov[4], cov[5]])
        posOut.setRow((measNum).toInt(), tempRow)
        measNum++

    }
    if (VO_ALGORITHM == "SVO") {
        System.out.println("Reset " + numTimesReset.toString() + " time(s)")
        System.out.println(numRepeatVel.toString() + " time(s) due to not enough tracked features")
        preprocessSVO.stop()
    }

    val deltaLat = posOut.getCol(1) - LAT0
    val deltaLon = (posOut.getCol(2) - LON0)
    var lat = zeros(deltaLat.numRows(), 1)
    var lon = zeros(deltaLon.numRows(), 1)

    for (i in 0..deltaLat.numRows() - 1) {
        lat[i] = deltaLatToNorth(deltaLat[i], LAT0, ALT0)
        lon[i] = deltaLonToEast(deltaLon[i], LAT0, ALT0)
    }
    posOut.setCol(1, lat)
    posOut.setCol(2, lon)
    return posOut
}