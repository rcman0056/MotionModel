import com.jcg.*
import golem.*
import golem.containers.Time
import golem.matrix.Matrix
import golem.util.logging.log
import golem.util.logging.setLogLevel
import insect.InertialNED
import modules.master_navs.MasterNavNED
import modules.stateblocks.ins.Pinson15AuxData
import navutils.*
import navutils.containers.NavSolution
import scorpion.filters.sensor.StandardSensorEKF
import scorpion.filters.sensor.containers.Measurement
import java.util.*

/**
 * Created by ucav on 9/9/2016.
 */
//new sjgvbjksrbuewor
fun runFilter(filter: StandardSensorEKF,
              insL: InertialNED, insR: InertialNED, masterL: MasterNavNED, masterR: MasterNavNED,
              PINSON_LABELL:String, PINSON_LABELR: String,includeMeasurements: Boolean,
              includeStereo:Boolean,includeGPS:Boolean): ArrayList<Output> {


    setLogLevel("scorpion.filters.raw.StandardEKF", "OFF")//Prevents display of filter data every time it propagates

    //Setting up arraylist for data to be stored in.  It is much faster to build as an array list and convert to matrix
    //at the end
    var filterOut: ArrayList<Output> = ArrayList()

    //Initializing output vector with starting conditions
    var posL=insL.solution.pose.pos.get(0,0..2)
    var velL=insL.solution.vel
    var rpyL= dcmToRpy(insL.solution.pose.rotMat)
    var covL= sqrt(filter.getStateBlockCovariance(LEADER_LABEL).diag())
    var posR=insR.solution.pose.pos.get(0,0..2)
    var velR=insR.solution.vel
    var rpyR= dcmToRpy(insR.solution.pose.rotMat)
    var covR= sqrt(filter.getStateBlockCovariance(RECEIVER_LABEL).diag())
    var P=filter.filter.P

    var biasX=filter.getStateBlockEstimate(BIASNORTH)
    var biasCovX=filter.getStateBlockCovariance(BIASNORTH)
    var biasY=filter.getStateBlockEstimate(BIASEAST)
    var biasCovY=filter.getStateBlockCovariance(BIASEAST)
    var biasZ=filter.getStateBlockEstimate(BIASDOWN)
    var biasCovZ=filter.getStateBlockCovariance(BIASDOWN)

    filterOut.add(Output(startTime, posL,velL,rpyL,covL,posR,velR,rpyR,covR,P,biasX,biasCovX,biasY,biasCovY,biasZ,biasCovZ))
    //filterOut.add(Output(startTime, posL,velL,rpyL,covL,posR,velR,rpyR,covR,P))

    var measCount=0
    for(row in measurements) {
        // Mechanize the inertial measurements
        var offset=0.01

        //right now there are two separate imu times but they mechanize as one (the times are identical)
        var tL = Time(row[0, 0] + offset)
        insL.mechanize(row[0, 1..3], row[0, 4..6], tL)
        var tR = Time(row[0, 7] + offset)
        insR.mechanize(row[0, 8..10], row[0, 11..13], tR)

        // Provide the Pinson15NED block with the AuxData that contains the
        // current mechanized solution so it can propagate the states.
        var insSolL = insL.getSolution(tL)
        var insSolR = insR.getSolution(tR)

        filter.giveStateBlockAuxData(LEADER_LABEL,
                Pinson15AuxData(insSolL.navSolution, insSolL.force))
        filter.giveStateBlockAuxData(RECEIVER_LABEL,
                Pinson15AuxData(insSolR.navSolution, insSolR.force))



        //this section is for incorporating measurements.  It looks to see if there is a measurement in the next time
        //step and if so performs an update
        if(includeMeasurements==true) {

            if(includeStereo==true) {
                //Stereo Measurement Here
                var measRow = getClosestStereoMeas(tL)
                var measRowData = stereoMeasArray.getRow(measRow)
                var gpsTime = measRowData[0, 0] + gpsStartTime//-.01
                //need to validate this will work (what if the measurement time is slightly behind the last propagate time)
                //var measTimeDiff = abs(gpsTime - tL.time)
                var measTimeDiff = gpsTime - tL.time
                /*if(gpsTime>=334909)
            {var test=0}*/

                if (/*measTimeDiff >= 0 && measTimeDiff <.01*/measTimeDiff > -.01 && measTimeDiff <= 0) {
                    if (measCount == 0) {
                        measCount = 1
                        //filter.setStateBlockCovariance(BIASDOWN,mat[4.0])
                    }
                    //these will need to change to fit real measurements
                    var t = Time(gpsTime)
                    var posMeas = mat[measRowData[0, 1], measRowData[0, 3], measRowData[0, 5]].T

                    var range = sqrt(pow(posMeas[0], 2) + pow(posMeas[1], 2) + pow(posMeas[2], 2))
                    var bias = FOGMParameterLookupY(range)
                    filter.giveStateBlockAuxData(BIASNORTH, bias)
                    filter.giveStateBlockAuxData(BIASEAST, bias)
                    filter.giveStateBlockAuxData(BIASDOWN, bias)

                    //Setting minimum measurement accuracy


                    var posMeasCov = eye(3)
                    var measScale = mat[3, 3, 3].T
                    posMeasCov[0, 0] = pow(measRowData[0, 2] * measScale[0], 2)
                    posMeasCov[1, 1] = pow(measRowData[0, 4] * measScale[1], 2)
                    posMeasCov[2, 2] = pow(measRowData[0, 6] * measScale[2], 2)
                    var aux = arrayOf(insSolL.navSolution, insSolR.navSolution)
                    var position = Measurement(STEREO_POSITION, t, t, posMeas, aux, posMeasCov)

                    if(gpsTime-gpsStartTime>257.7)
                    {
                        var a=1;
                    }

                    filter.update(position)
                }
            }
            //GPS measurement here
            if(includeGPS==true) {
                var measRowGPS = getClosestGPSMeas(tL)
                var measRowGPSData = DGPSMeasArray.getRow(measRowGPS)
                var DGPSTime = measRowGPSData[0, 0]
                var measTimeDiff = DGPSTime - tL.time


                if (measTimeDiff > -.01 && measTimeDiff <= 0) {

                    var t = Time(DGPSTime)
                    var posMeas = mat[measRowGPSData[0, 1], measRowGPSData[0, 3], measRowGPSData[0, 5]].T

                    var posMeasCov = eye(3)
                    var measScale = mat[1,1,1].T
                    posMeasCov[0, 0] = pow(measRowGPSData[0, 2] * measScale[0], 2)
                    posMeasCov[1, 1] = pow(measRowGPSData[0, 4] * measScale[1], 2)
                    posMeasCov[2, 2] = pow(measRowGPSData[0, 6] * measScale[2], 2)
                    var aux = arrayOf(insSolL.navSolution, insSolR.navSolution)
                    var position = Measurement(DGPS_POSITION, t, t, posMeas, aux, posMeasCov)
                    if(relativeProcessor==true){
                        position = Measurement(DGPS_RELATIVE, t, t, posMeas, aux, posMeasCov)
                    }
                    var rangeLimit=0.0

                    var gpsRange=calcGpsRange(posMeas,insSolL.navSolution)
                    if(limitGPS==true) {
                        rangeLimit = 50.0
                    }else{
                        rangeLimit = 0.0
                    }
                    if(gpsRange>rangeLimit) {
                        filter.update(position)
                    }

                }
            }
        }

        filter.propagate(tL)


        //position update test
        if(tL.time>139.63+ gpsStartTime &&tL.time<139.65+ gpsStartTime) {
            var t= Time(139.64 + gpsStartTime)
            var aux = arrayOf(insSolL.navSolution,insSolR.navSolution)
            var posMeasCov= eye(3) *.25
            var position= Measurement(STEREO_POSITION, t, t, mat[-43.91844, 0, -14.16849].T, aux, posMeasCov)
            //filter.update(position)
        }

        // Record progagated error with MasterNav
        masterL.giveErrorState(tL, filter.getStateBlockEstimate(LEADER_LABEL), sqrt(filter.getStateBlockCovariance(LEADER_LABEL).diag()))
        masterR.giveErrorState(tR, filter.getStateBlockEstimate(RECEIVER_LABEL), sqrt(filter.getStateBlockCovariance(RECEIVER_LABEL).diag()))


        // Record the current solution for plotting
        // May need to redeclare insSolL when measurements are incorporated

        insSolL = insL.getSolution(filter.curTime)
        var solL = masterL.getSolution(insSolL)
        posL = solL.pose.pos.asRowVector()
        velL = solL.vel.T

        rpyL = dcmToRpy(solL.pose.rotMat)
        covL = filter.getStateBlockCovariance(LEADER_LABEL).diag()


        // May need to redeclare insSolR when measurements are incorporated
        insSolR = insR.getSolution(filter.curTime)
        var solR = masterR.getSolution(insSolR)
        posR = solR.pose.pos.asRowVector()
        velR = solR.vel.T
        rpyR = dcmToRpy(solR.pose.rotMat)
        covR = filter.getStateBlockCovariance(RECEIVER_LABEL).diag()

        P=filter.filter.P

        biasX=filter.getStateBlockEstimate(BIASNORTH)
        biasCovX=filter.getStateBlockCovariance(BIASNORTH)
        biasY=filter.getStateBlockEstimate(BIASEAST)
        biasCovY=filter.getStateBlockCovariance(BIASEAST)
        biasZ=filter.getStateBlockEstimate(BIASDOWN)
        biasCovZ=filter.getStateBlockCovariance(BIASDOWN)

        //var test2= Output(tL.time, posL, velL, rpyL, covL, posR, velR, rpyR, covR, P)
        var test2= Output(tL.time, posL, velL, rpyL, covL, posR, velR, rpyR, covR, P,biasX,biasCovX,biasY,biasCovY,biasZ,biasCovZ)
        filterOut.add(test2)




    }

    var insPosR=insR.getSolution(filter.curTime).navSolution.pose.pos.asColVector()
    var insPosL=insL.getSolution(filter.curTime).navSolution.pose.pos.asColVector()
    var insPosDiff=insPosR-insPosL

    var dxL=filter.getStateBlockEstimate(LEADER_LABEL).get(0..2,0)
    var dxR=filter.getStateBlockEstimate(RECEIVER_LABEL).get(0..2,0)

    var dxDiff=dxR-dxL

    var factor=radToMeter(insPosL[0],insPosL[2])
     var lon_factor=factor.first
    var lat_factor=factor.second

    var factorR=radToMeter(insPosR[0],insPosR[2])
    var lon_factorR=factorR.first
    var lat_factorR=factorR.second

    insPosDiff[0]=insPosDiff[0]*lat_factor
    insPosDiff[1]=insPosDiff[1]*lon_factor
    insPosDiff[2]=insPosDiff[2]*-1

    var insCorrected=insPosDiff+dxDiff

    var masterPosL=masterL.getSolution(insL.getSolution(filter.curTime)).pose.pos.asColVector()
    var masterPosR=masterR.getSolution(insR.getSolution(filter.curTime)).pose.pos.asColVector()


    var masterPosDiff=masterPosR-masterPosL

    var masterPosDiffN= deltaLatToNorth(posR[0] - posL[0], posL[0], posL[2])
    var masterPosDiffE= deltaLonToEast(posR[1]-posL[1],posL[0],posL[2])
    var masterPosDiff2=mat[masterPosDiffN, masterPosDiffE,posL[2]-posR[2]].T

    masterPosDiff[0]=masterPosDiff[0]*lat_factor
    masterPosDiff[1]=masterPosDiff[1]*lon_factor
    masterPosDiff[2]=masterPosDiff[2]*-1

    var dxLRad=mat[northToDeltaLat(dxL[0],insPosL[0]),eastToDeltaLon(dxL[1],insPosL[0]),-dxL[2]].T
    var dxRRad=mat[northToDeltaLat(dxR[0],insPosR[0]),eastToDeltaLon(dxR[1],insPosR[0]),-dxR[2]].T

    var dxRRadb=mat[dxR[0]/lat_factorR,dxR[1]/lon_factorR,-dxR[2]].T
    var insPosCorrectedRb=insPosR+dxRRadb
    var insPosCorrectedR=insPosR+dxRRad
    var insPosCorrectedL=insPosL+dxLRad

    var testLRad=masterPosL-insPosCorrectedL
    var testRRad=masterPosR-insPosCorrectedR

    var masterErrorEstL=masterPosL-insPosL
    var masterErrorEstR=masterPosR-insPosR

    var testLm=Rad2Meter(testLRad,lat_factor,lon_factor)
    var testRm=Rad2Meter(testRRad,lat_factorR,lon_factorR)


    return filterOut
}

fun calcGpsRange(posMeas: Matrix<Double>,insSolL:NavSolution):Double{

    var posL=insSolL.pose.pos
    var deltaGPSNorth=deltaLatToNorth(posMeas[0],posL[0], posL[2])
    var deltaGPSEast=deltaLonToEast(posMeas[1],posL[0],posL[2])
    var deltaGPSDown=-posMeas[2]

    var measurement=mat[deltaGPSNorth,deltaGPSEast,deltaGPSDown].T;

    var range=measurement.norm();

    return range
}

//finds the closest  time to 0 and returns its row location
fun getClosestStereoMeas(time: Time): Int{
    var timeCol= stereoMeasArray.getCol(0)+ startTime +.01-time.time
    var row = abs(timeCol).argMin()
    return row
}

fun getClosestGPSMeas(time: Time): Int{
    var timeCol= DGPSMeasArray.getCol(0)-time.time
    var row = abs(timeCol).argMin()
    return row
}