/**
 * Created by ucav on 11/7/2016.
 */

/**
 * Created by ucav on 8/15/2016.
 */
package modules.measurements

import Rad2Meter
import golem.*
import golem.matrix.*
import navutils.containers.NavSolution
import navutils.deltaLatToNorth
import navutils.deltaLonToEast
import navutils.radToMeter
import scorpion.filters.sensor.containers.Measurement
import scorpion.filters.sensor.containers.MeasurementModel
import scorpion.filters.sensor.containers.MeasurementProcessor
import java.util.*

open class DGPSPositionMeasurementProcessor: MeasurementProcessor {
    override var label: String
    override var stateBlockLabels: Array<String>

    constructor(label: String,
                pinsonLabel: Array<String>){
        this.label=label
        //This reads in the stateblock labels so the xhat and P are pulled correctly
        this.stateBlockLabels=pinsonLabel
    }


    override fun generateModel(meas: Measurement, xhat: Matrix<Double>, P: Matrix<Double>): MeasurementModel {
        //preloading H matrix
        var H = zeros(3,30)

        //reading in the aux data.  If it is not an Array<NavSolution> it will error
        val a = meas.auxData as? Array<NavSolution>?: error("Type must be Array<NavSolution>")
        var insL=a[0]
        var insR=a[1]

        radToMeter(insL.pose.pos.get(0),insL.pose.pos.get(2))
        //putting measurement in correct body frame
        var Llh=meas.measurementData

        //Virtual world body is nose, left wing, up vs nose, right wing, down
        //NRD[1]=-NRD[1]
        //NRD[2]=-NRD[2]

        var measurement=determineMeasurement(Llh,insL,insR)

        H[0..2,0..2]=-eye(3)
        H[0..2,15..17]=eye(3)

        var cov=meas.measurementCov
        return MeasurementModel(measurement, // pass-through
                {H * it},
                H,
                cov)
    }
    //First call to receive AuxData must be auxData:Any
    override fun receiveAuxData(auxData: Any) {
        throw UnsupportedOperationException()
    }

    fun determineMeasurement(m:Matrix<Double>,insL:NavSolution,insR:NavSolution): Matrix<Double> {
        var meas=zeros(3,1)
        if(m.numRows()==1){
            meas=m.T
        }
        else{
            meas=m
        }
        // Need both current solutions
        var posL=insL.pose.pos
        var posR=insR.pose.pos

        var factors=radToMeter(posL[0],posL[2])

        var test=Rad2Meter(meas,factors.second,factors.first)
        var test2=Rad2Meter(posR-posL,factors.second,factors.first)
        //finding the DGPS measurement in NED
        var deltaGPSNorth=deltaLatToNorth(meas[0],posL[0], posL[2])
        var deltaGPSEast=deltaLonToEast(meas[1],posL[0],posL[2])
        var deltaGPSDown=-meas[2]

        var measNED=mat[deltaGPSNorth,deltaGPSEast,deltaGPSDown].T
        //finding the difference between the two aircraft solutions

        var deltaNorth=deltaLatToNorth(posR[0]-posL[0],posL[0], posL[2])
        var deltaEast=deltaLonToEast(posR[1]-posL[1],posL[0],posL[2])
        var deltaDown=-(posR[2]-posL[2])

        var relNED=mat[deltaNorth,deltaEast,deltaDown].T

        var measurements=measNED-relNED

        if (abs(measurements[2])>.05) {
            var a=1
        }

        return measurements
    }
}
