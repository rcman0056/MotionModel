package modules.stateblocks.ins

import golem.matrix.Matrix


data class FainImagePreMeasurements(

                              var New_Image: IntArray,
                              var Old_Image: IntArray,
                              var New_Image_Time_Valid: Double,
                              var Old_Image_Time_Valid: Double, //This is not used
                              var New_Image_DCM: Matrix<Double>,
                              var Old_Image_DCM: Matrix<Double>,
                              var Image_Received_Flag: Boolean,
                              var First_Image_Received: Boolean,
                              var Cam_To_Body_DCM: Matrix<Double>){



}