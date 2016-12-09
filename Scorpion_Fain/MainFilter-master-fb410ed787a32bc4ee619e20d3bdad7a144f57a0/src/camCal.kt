/**
 * Created by daniel on 07/12/16.
 */
fun fillCamCal(source: String): cameraCalibrationParams {
    val camCal = cameraCalibrationParams
    if (source == "ASPN1") {
        //For ASPN dataset low res cam
        camCal.width = 1360.0
        camCal.height = 1024.0
        camCal.fx = 1335.02770000000
        camCal.fy = 1334.80410000000
        camCal.cx = 718.161400000000
        camCal.cy = 492.175800000000

    } else if (source == "ASPN2") {
        //For ASPN dataset high res cam
        camCal.width = 4008.0
        camCal.height = 2672.0
        camCal.fx = 3995.38409142526
        camCal.fy = 3962.28371938875
        camCal.cx = 1989.75117773715
        camCal.cy = 1352.21244751804
    } else {
        //if (source == "example"){
        //For SVO example dataset
        camCal.width = 752.0
        camCal.height = 480.0
        camCal.fx = 315.5
        camCal.fy = 315.5
        camCal.cx = 376.0
        camCal.cy = 240.0
    }
    return camCal
}