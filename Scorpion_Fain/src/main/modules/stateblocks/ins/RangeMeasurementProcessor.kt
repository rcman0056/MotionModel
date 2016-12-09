package main.modules.stateblocks.ins

import golem.*
import golem.matrix.Matrix
import modules.stateblocks.ins.FainMeasurements
import navutils.deltaLatToNorth
import navutils.deltaLonToEast
import scorpion.filters.sensor.containers.Measurement
import scorpion.filters.sensor.containers.MeasurementModel
import scorpion.filters.sensor.containers.MeasurementProcessor


/**
 *
 *
 */
class RangeMeasurementProcessor : MeasurementProcessor {
    override var label: String
    override var stateBlockLabels: Array<String>
    private var auxData: FainMeasurements? = null

    /**
     * This block receives Motion Model inputs data via calls to this function. The parameter must be of type
     * [MotionModelAuxData], which is a container passing in motion model inputs
     */
    override fun receiveAuxData(auxData: Any) {
        if (auxData is FainMeasurements) {
            this.auxData = auxData
        } else {
            throw UnsupportedOperationException("This class only accepts FainMeasurements instances")
        }
    }

    /**
     *  Constructor takes an identifying label for this processor.
     *  @param label The label uniquely identifying this particular processor.
     *  @param blockLabel the label of the StateBlock we are updating.
     *  @param HValue the value H in the measurement equation z = Hx
     */
    constructor (label: String,
            //HValue: Double,
                 blockLabel: String) {
        this.label = label
        this.stateBlockLabels = arrayOf(blockLabel)
        this.blockLabel = blockLabel
    }

    //val HValue: Double
    val blockLabel: String

    // Generates z, h, H, and R for the filter to use during updates.
    override fun generateModel(meas: Measurement, xhat: Matrix<Double>, P: Matrix<Double>): MeasurementModel {
        var Curr_Meas = meas.auxData as FainMeasurements
      //  var alt =0.0
       // with(Curr_Meas as FainMeasurements) {
            //alt=
            //Measurement from LCm message
            var Corrected_range_lon_2pi = -(2 * Math.PI - Curr_Meas.range_Lon)
            var range_Lat_neu = deltaLatToNorth((Curr_Meas.range_Lat - (Curr_Meas.GPS_origin_lat * Math.PI / 180)), Curr_Meas.range_Lat, xhat[7])
            var range_Lon_neu = deltaLonToEast((Corrected_range_lon_2pi - (Curr_Meas.GPS_origin_lon * Math.PI / 180)), Curr_Meas.range_Lat, xhat[7])

            var z = meas.measurementData

            var North_delta1 = xhat[0] - range_Lat_neu
            var East_delta1 = xhat[1] - range_Lon_neu
            var Alt_delta1 = xhat[7] - Curr_Meas.range_alt
            // h = sqrt((North Diff^2)+(East Diff^2)+(Alt Diff^2))
            var h1 = pow(pow(North_delta1, 2) + pow(East_delta1, 2) + pow(Alt_delta1, 2), .5)

           // println("z=" + z.toString() + "/t" + "h1=" + h1.toString())

            var H = mat[North_delta1 / h1, East_delta1 / h1, 0, 0, 0, 0, 0, Alt_delta1 / h1, 0] //linearized measurement
            // Use the measurement container's covariance
            var R = meas.measurementCov
            //println("z=" + z + "\n" + "xhat=" + xhat[6] * (180 / Math.PI) + "\n" + "meas=" + meas.measurementData)
            // Make a inner function "h" to return to the filter
            fun h(x: Matrix<Double>): Matrix<Double> {
                var North_delta = x[0] - range_Lat_neu
                var East_delta = x[1] - range_Lon_neu
                var Alt_delta = x[7] - Curr_Meas.range_alt
                // h = sqrt((North Diff^2)+(East Diff^2)+(Alt Diff^2))


                var h = pow(pow(North_delta, 2) + pow(East_delta, 2) + pow(Alt_delta, 2), .5)

                return mat[h]
            }


            // ::h is a reference to the function "h"
            return MeasurementModel(z, ::h, H, R)
       //  }
    }
}
        /*
            //var Test_Point = mat[39.773692, -84.107681, 127.0] //Lat Lon AGL
            var Test_Point = mat[39.779692, -84.107681, 127.0] //Lat Lon AGL
            var range_Lat_neu_truth = deltaLatToNorth((Test_Point[0] - Curr_Meas.GPS_lat)* Math.PI / 180, Test_Point[0]*Math.PI/180, Test_Point[2])
            var range_Lon_neu_Truth = deltaLonToEast((Test_Point[1] - Curr_Meas.GPS_lon)* Math.PI / 180, Test_Point[0]*Math.PI/180, Test_Point[2])

            var North_delta_truth = range_Lat_neu_truth
            var East_delta_truth  = range_Lon_neu_Truth
            var Alt_delta_truth   = Test_Point[2]-Curr_Meas.GPS_height_agl

            var z  = mat[pow(pow(North_delta_truth,2)+pow(East_delta_truth,2)+pow(Alt_delta_truth,2),.5)]

            var range_Lat_neu = deltaLatToNorth((Test_Point[0] - Curr_Meas.GPS_origin_lat)* Math.PI / 180, Test_Point[0]*Math.PI/180, Test_Point[2]).toDouble()
            var range_Lon_neu =  deltaLonToEast((Test_Point[1] - Curr_Meas.GPS_origin_lon)* Math.PI / 180, Test_Point[0]*Math.PI/180, Test_Point[2]).toDouble()

            var North_delta = xhat[0] - range_Lat_neu
            var East_delta  = xhat[1] - range_Lon_neu
            var Alt_delta   = xhat[7] - Test_Point[2]
            var h1 =  pow(pow(North_delta,2)+pow(East_delta,2)+pow(Alt_delta,2),.5)
            println( "z=" + z.toString() + '\t' + "h1=" + h1.toString())

            var H = mat[North_delta/h1, East_delta/h1, 0, 0, 0, 0, 0, Alt_delta/h1, 0] //linearized measurement w/ Alt Update

            //var H = mat[North_delta/h1, East_delta/h1, 0, 0, 0, 0, 0, 0, 0] //linearized measurement no ALt Update
            // Use the measurement container's covariance
            var R = meas.measurementCov
            // println("z=" + z + "\n" + "xhat=" + xhat[6] * (180 / Math.PI) + "\n" + "meas=" + meas.measurementData)
            // Make a inner function "h" to return to the filter
            fun h(x: Matrix<Double>): Matrix<Double> {

                var North_delta = x[0] - range_Lat_neu
                var East_delta  = x[1] - range_Lon_neu
                var Alt_delta   = x[7] - Test_Point[2]
                // h = sqrt((North Diff^2)+(East Diff^2)+(Alt Diff^2))


                var h =  pow(pow(North_delta,2)+pow(East_delta,2)+pow(Alt_delta,2),.5)

                return mat[h]


                // ::h is a reference to the function "h"
            }

        return MeasurementModel(z, ::h, H, R)}
    }

    // Unused for this example, so throw an error if someone tries to use it

}
*/