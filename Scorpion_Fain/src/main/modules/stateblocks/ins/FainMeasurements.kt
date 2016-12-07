package modules.stateblocks.ins

import golem.matrix.*
//import navutils.containers.NavSolution

/**
 * Passes into a [Pinson15Block] the set of current values from a running INS mechanization to linearize about.
 *
 * @param force NED specific force vector
 * @param navSolution the current solution from the INS mechanization
 */
data class FainMeasurements(  var range: Double,
                              var range_time: Double,
                              var range_Lat: Double,
                              var range_Lon: Double,
                              var range_alt: Double,
                              var pix2_alt: Double,
                              var pix2_alt_time:Double,
                              var heading_time: Double,
                              var heading: Double,
                              var GPS_Linux_time: Double, //This is not GPS time just linux time the GPS was saved
                              var GPS_lat: Double,
                              var GPS_lon: Double,
                              var GPS_height_agl: Double,
                              var GPS_origin_lat: Double,
                              var GPS_origin_lon: Double,
                              var GPS_origin_alt: Double,
                              var GPS_origin_received: Boolean){



}