import com.jcg.*
import golem.*
import golem.containers.Time
import insect.InertialNED
import modules.master_navs.MasterNavNED
import modules.measurements.DGPSPositionMeasurementProcessor
import modules.measurements.DGPSRelativeMeasurementProcessor
import modules.measurements.StereoAttitudeMeasurementProcessor
import modules.measurements.StereoPositionMeasurementProcessor
import modules.stateblocks.common.FOGMState
import modules.stateblocks.common.StereoFOGMBias
import modules.stateblocks.ins.ImuModel
import modules.stateblocks.ins.Pinson15NEDBlock
import modules.stateblocks.ins.getImuModelHG9900
import navutils.containers.Pose
import navutils.containers.Vector3
import navutils.rpyToDcm
import scorpion.filters.sensor.StandardSensorEKF

/**
 * Created by ucav on 9/9/2016.
 */

fun setupFilter(): StandardSensorEKF {
    // Set up a filter that has a Pinson15NED state block
    val filter = StandardSensorEKF(curTime = Time(startTime))

    // Create the state blocks
    var modelL = getImuModelKVH1750(1)
    var modelR = getImuModelKVH1750(1)
    var blockL = Pinson15NEDBlock(label = LEADER_LABEL, imuModel = modelL)
    var blockR = Pinson15NEDBlock(label = RECEIVER_LABEL, imuModel = modelL)
    filter.addStateBlock(blockL)
    filter.addStateBlock(blockR)

    var BiasSigma=mat[8,8,8].T
    var BiasTau=mat[1.0,1.0,1.0]
    var BiasCov=mat[0.0,0.0,0.0]
    var stereoBiasNorth= StereoFOGMBias(BIASNORTH,BiasSigma[0],BiasTau[0])
    var stereoBiasEast= StereoFOGMBias(BIASEAST,BiasSigma[1],BiasTau[1])
    var stereoBiasDown= StereoFOGMBias(BIASDOWN,BiasSigma[2],BiasTau[2])

    filter.addStateBlock(stereoBiasNorth)
    filter.addStateBlock(stereoBiasEast)
    filter.addStateBlock(stereoBiasDown)



    var scale=0
    // Set the initial covariance on this state block
    var initCovL = zeros(15, 15)
    initCovL[0..2, 0..2] = eye(3) * pow(2, 2)*scale
    initCovL[3..5, 3..5] = eye(3) * 0.1*scale
    initCovL[6..8, 6..8] = eye(3) * pow(1e-4, 2)*scale
    initCovL[9..11, 9..11] = eye(3) * pow(modelL.accelBiasSigma, 2)
    initCovL[12..14, 12..14] = eye(3) * pow(modelL.gyroBiasSigma, 2)

    var initCovR = zeros(15, 15)
    initCovR[0..2, 0..2] = eye(3) * pow(2, 2)*scale
    initCovR[3..5, 3..5] = eye(3) * 0.1*scale
    initCovR[6..8, 6..8] = eye(3) * pow(1e-4, 2)*scale
    initCovR[9..11, 9..11] = eye(3) * pow(modelR.accelBiasSigma, 2)
    initCovR[12..14, 12..14] = eye(3) * pow(modelR.gyroBiasSigma, 2)

    filter.setStateBlockCovariance(label = LEADER_LABEL, covariance = initCovL)
    filter.setStateBlockCovariance(label = RECEIVER_LABEL, covariance = initCovR)
    filter.setStateBlockCovariance(label = BIASNORTH, covariance = mat[BiasCov[0]])
    filter.setStateBlockCovariance(label = BIASEAST, covariance = mat[BiasCov[1]])
    filter.setStateBlockCovariance(label = BIASDOWN, covariance = mat[BiasCov[2]])

    filter.addMeasurementProcessor(StereoPositionMeasurementProcessor(STEREO_POSITION, arrayOf(LEADER_LABEL, RECEIVER_LABEL,BIASNORTH,BIASEAST,BIASDOWN)))
    filter.addMeasurementProcessor(StereoAttitudeMeasurementProcessor(STEREO_ATTITUDE, arrayOf(LEADER_LABEL, RECEIVER_LABEL)))
    filter.addMeasurementProcessor(DGPSPositionMeasurementProcessor(DGPS_POSITION,arrayOf(LEADER_LABEL,RECEIVER_LABEL)))
    filter.addMeasurementProcessor(DGPSRelativeMeasurementProcessor(DGPS_RELATIVE, arrayOf(LEADER_LABEL, RECEIVER_LABEL)))
    return filter
}

fun setupIns(TRUTH_FILE:String,START_FILE:String ): InertialNED {
    //Initialize INS class from truth data used to mechanize measurements
    var truth= truthR
    if(TRUTH_FILE== LEADER_TRUTH_FILE){
        truth= truthL
    }

    var rowTruth=truth.getRow(0)
    var start= readTdf(START_FILE, "\t", false)
    var rowStart=start.get(0)

    var vel0=rowStart[0,3..5] //Starting velocity comes from meta data

    var lat=rowTruth[0,1]* dtr //Starting position comes from truth converted to radians
    var lon=rowTruth[0,2]* dtr
    var pos0= zeros(3)
    pos0[0]=lat
    pos0[1]=lon
    pos0[2]=rowTruth[0,3]

    var rpy=rowStart[0,6..8] //Starting rpy comes from meta data
    var rot = rpyToDcm(rpy)
    var pose= Pose(rot, Vector3(pos0), Time(startTime))
    return InertialNED(pose, vel0)
}

fun setupMasterNav(filter: StandardSensorEKF, PINSON_LABEL:String): MasterNavNED {
    var master = MasterNavNED(10.0)
    master.giveErrorState(filter.curTime, filter.getStateBlockEstimate(PINSON_LABEL),
            sqrt(filter.getStateBlockCovariance(PINSON_LABEL).diag()))
    return master
}

fun getImuModelKVH1750(scale:Int) = ImuModel(accelRandomWalkSigma = .23*.3048/60*scale, // m/s^(3/2)   (0.0143 m/s/hr^(1/2))
        gyroRandomWalkSigma = .012*pi/180/60*scale, // rad/s^(1/2)   (0.002 deg/hr^(1/2))
        accelBiasSigma = .05*.00980665*scale, // m/s^2
        accelBiasTau = 3600.0, // sec
        gyroBiasSigma = .05*pi/180/3600*scale, // rad/s  (0.0015 deg/hr))
        gyroBiasTau = 3600.0) // sec

fun getImuModelMG362(scale:Int) = ImuModel(accelRandomWalkSigma = .0095, // m/s^(3/2)   (0.0143 m/s/hr^(1/2))
        gyroRandomWalkSigma = .0000873, // rad/s^(1/2)   (0.002 deg/hr^(1/2))
        accelBiasSigma = .0098, // m/s^2
        accelBiasTau = 3600.0, // sec
        gyroBiasSigma = 4.8481e-6, // rad/s  (0.0015 deg/hr))
        gyroBiasTau = 3600.0) // sec