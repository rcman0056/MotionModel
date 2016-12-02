
// finish this first
import exlcm.pixhawk

import golem.*
import golem.containers.Time
import golem.matrix.Matrix
import kotlin.system.exitProcess
import lcm.lcm.LCM
import lcm.lcm.LCMDataInputStream
import lcm.lcm.LCMSubscriber
import main.modules.stateblocks.ins.HeadingMeasurementProcessor
import modules.stateblocks.ins.FainMeasurements
import modules.stateblocks.ins.MotionModelAuxData
import modules.stateblocks.ins.MotionModelBlock
import navutils.deltaLatToNorth
import navutils.deltaLonToEast
import scorpion.buffers.Buffer
import java.io.*
import java.util.*
import java.nio.ByteBuffer
import scorpion.filters.sensor.StandardSensorEKF
import scorpion.filters.sensor.containers.Measurement

//Made as a global variable
var Input_LCM_Time = InputLCMTimeCheck  //used to check for time of first LCm message
var Export_Data = zeros(1,14) //used to export the filter output data
var Export_Pixhawk = zeros(1,6) // used to export Pixhawk data. Size depends on data wanted
var HeadingUpdateOn = false
var SavePixhawkData = false



// Main Function
object FAIN_Main {

    @JvmStatic
    fun main(args: Array<String>) {

        var LCMMeasurements = FainMeasurements(0.0,0.0,0.0,0.0,0.0,0.0)
        val filter = StandardSensorEKF(Time(0.0), //Set time filter start here at 0.0
                buffer = Buffer())

        var LCMChannels = LCM.getSingleton()
        LCMChannels.subscribe("PIXHAWK2", Subscribe_Pixhawk2(filter,LCMMeasurements)) //This subscribes to LCM message is kept open as long as main is func is running
        LCMChannels.subscribe("PIXHAWK1", Subscribe_Pixhawk1(filter,LCMMeasurements))



        val block = MotionModelBlock(label = "motionmodel")
            filter.addStateBlock(block)
        //Add measurement Processors

        //Create HeadingUpdate Processor
        val HeadingUpdate = "HeadingUpdate"
        filter.addMeasurementProcessor(HeadingMeasurementProcessor(HeadingUpdate,"motionmodel"))



        //Set Intial Cov
        var initCov = zeros(9, 9)
        var tau_vv:Double = 2.0//time constant on alt_vv ... also set in MotionModelBlock
        var sigma_vv = 5.0 //sigma on alt_vv ... also set in MotionModelBlock
        //initCov[0..7,0..7]=eye(8,8)*.001    //add noise to states for now need to calculate input noise
        initCov[6,6]= 8*Math.PI/180
        initCov[8, 8] = 2*pow(sigma_vv,2)/tau_vv
        filter.setStateBlockCovariance(label = "motionmodel",
                    covariance = initCov)

        //Set initial states but they are not really used as LCM over writes them
        var initStates = zeros(9,1)
        initStates[0] = 0
        initStates[1] = 0
        initStates[2] = 1
        initStates[3] = 1
        initStates[4] = 1
        initStates[5] = 1
        initStates[6] = 1
        initStates[7] = 1
        initStates[8] = 1
        filter.setStateBlockEstimate("motionmodel", initStates )
        Export_Data[0,1..9]=initStates.T
        while(true){

           //Need to include error
        }
    }
}

class Subscribe_Pixhawk1(var filter:StandardSensorEKF, var LCMMeasurements: FainMeasurements): LCMSubscriber {

    // comments
    override fun messageReceived(p0: LCM, channel: String, p2: LCMDataInputStream) {
        var pixhawk1 = (pixhawk(p2))

        LCMMeasurements.GPS_Linux_time = pixhawk1.global_relative_frame[0]
        LCMMeasurements.GPS_lat = pixhawk1.global_relative_frame[1]
        LCMMeasurements.GPS_lon = pixhawk1.global_relative_frame[2]
        LCMMeasurements.GPS_height_agl = pixhawk1.global_relative_frame[3]

        //save off origin
        if (GPS_origin_vars.GPS_origin_recieved == false){
            GPS_origin_vars.GPS_origin_recieved == true
            GPS_origin_vars.GPS_origin = mat[LCMMeasurements.GPS_lat,LCMMeasurements.GPS_lon,LCMMeasurements.GPS_height_agl]
        }

    }
}

//Subscriber for Pixhawk2 with filter updates
class Subscribe_Pixhawk2(var filter:StandardSensorEKF, var LCMMeasurements: FainMeasurements): LCMSubscriber {

    // comments
    override fun messageReceived(p0: LCM, channel: String, p2: LCMDataInputStream){
        var pixhawk2 = (pixhawk(p2))

        var pixhawk2_lcm_message_aux = MotionModelAuxData(pixhawk2.airspeed[1],
                                     pixhawk2.airspeed[0],
                                     pixhawk2.raw_rate[3],
                                     pixhawk2.raw_rate[4],
                                     pixhawk2.raw_rate[0],
                                     pixhawk2.attitude[1],
                                     pixhawk2.attitude[2],
                                     pixhawk2.attitude[0],
                                     pixhawk2.heading[0],
                                     pixhawk2.heading[1])
        if (SavePixhawkData == true){
            var PixhawkRow =  SavePixhawkData(pixhawk2)
            Export_Pixhawk = vstack(Export_Pixhawk, PixhawkRow)}
        /*
             var airspeed: Double,
             var airspeed_time: Double,
             var pitchrate: Double,
             var yawrate: Double,
             var yaw_pitch_rate_time: Double,
             var roll: Double,
             var pitch :Double,
             var roll_pitch_time: Double
              */

        //Check to see if individual time stamps have updated

        //If incoming message time is greater then stored time
            //Set time to new time
            //Set flag to high
        if (pixhawk2_lcm_message_aux.heading_time > Input_LCM_Time.heading_time){
            Input_LCM_Time.heading_time = pixhawk2_lcm_message_aux.heading_time
            Input_LCM_Time.heading_time_flag = true
        }
        if (pixhawk2_lcm_message_aux.airspeed_time > Input_LCM_Time.airspeed_time) {
            Input_LCM_Time.airspeed_time = pixhawk2_lcm_message_aux.airspeed_time
            Input_LCM_Time.airspeed_time_flag = true
        }
        if (pixhawk2_lcm_message_aux.yaw_pitch_rate_time > Input_LCM_Time.raw_rate_time) {
            Input_LCM_Time.raw_rate_time = pixhawk2_lcm_message_aux.yaw_pitch_rate_time
            Input_LCM_Time.raw_rate_time_flag = true
        }
        if (pixhawk2_lcm_message_aux.roll_pitch_time > Input_LCM_Time.attitude_time) {
            Input_LCM_Time.attitude_time = pixhawk2_lcm_message_aux.roll_pitch_time
            Input_LCM_Time.attitude_time_flag = true
        }

        if (Input_LCM_Time.airspeed_time_flag == true && Input_LCM_Time.raw_rate_time_flag == true && Input_LCM_Time.attitude_time_flag == true) {
            var time = 0.0
            if     (Input_LCM_Time.attitude_time > filter.curTime.time) {time = Input_LCM_Time.attitude_time}
            else if(Input_LCM_Time.raw_rate_time > filter.curTime.time) {time = Input_LCM_Time.raw_rate_time}
            else if(Input_LCM_Time.airspeed_time > filter.curTime.time) {time = Input_LCM_Time.airspeed_time}
            else {println('\n' + "Error trying to propagate prior to current filter time" + '\n'); exitProcess(0)}

            //set flags to false to reset time check
            Input_LCM_Time.airspeed_time_flag = false
            Input_LCM_Time.raw_rate_time_flag = false
            Input_LCM_Time.attitude_time_flag = false

            //Sets filter to initial LCM attitude time and initial state values
            if (Input_LCM_Time.LCM_start_time_flag == false){
                Input_LCM_Time.LCM_start_time_flag = true
                filter.curTime = Time(time)


                var initStates_LCM = zeros(9,1)
                initStates_LCM[0] = 0
                initStates_LCM[1] = 0
                initStates_LCM[2] = pixhawk2.airspeed[1]  //Set Ground speed guess to Air Speed
                initStates_LCM[3] = pixhawk2.heading[1]*Math.PI/180 //Set Course Angle to Yaw Angle and assume no wind
                initStates_LCM[4] = 0
                initStates_LCM[5] = 0
                initStates_LCM[6] = pixhawk2.heading[1]*Math.PI/180 //Set Yaw Angle
                initStates_LCM[7] = pixhawk2.global_relative_frame[3]
                initStates_LCM[8] = .01
                filter.setStateBlockEstimate("motionmodel", initStates_LCM )
            }



            filter.giveStateBlockAuxData("motionmodel", pixhawk2_lcm_message_aux)

            filter.propagate(Time(time))





            var time_filter = filter.curTime
            var X = filter.getStateBlockEstimate("motionmodel").asRowVector()
            var P = filter.getStateBlockCovariance("motionmodel").diag().asRowVector()
            println(time_filter.toString() + '\n' +
                    X[0].toString() + '\t' + P[0].toString() + '\n' +
                    X[1].toString() + '\t' + P[1].toString() + '\n' +
                    X[2].toString() + '\t' + P[2].toString() + '\n' +
                    X[3].toString() + '\t' + P[3].toString() + '\n' +
                    X[4].toString() + '\t' + P[4].toString() + '\n' +
                    X[5].toString() + '\t' + P[5].toString() + '\n' +
                    X[6].toString() + '\t' + P[6].toString() + '\n' +
                    X[7].toString() + '\t' + P[7].toString() + '\n' +
                    X[8].toString() + '\t' + P[8].toString() + '\n'
                    )
            //Get current GPS value:Note this is at 4hz while the Filter propagates at ~10Hz
            var current_gps = mat[LCMMeasurements.GPS_Linux_time,LCMMeasurements.GPS_lat,LCMMeasurements.GPS_lon,LCMMeasurements.GPS_height_agl]
            //Convert to NEU using first received GPS value as origin
            var current_gps_NE_AGL = mat[current_gps[0],
                                       deltaLatToNorth(current_gps[1]-GPS_origin_vars.GPS_origin[0],GPS_origin_vars.GPS_origin[2]),
                                        deltaLonToEast(current_gps[2]-GPS_origin_vars.GPS_origin[1],GPS_origin_vars.GPS_origin[2]),
                                        current_gps[3]]

           // var current_data = mat[time_filter.time, X,GPS _Data]
            var current_data = hstack(mat[time_filter.time], X, current_gps_NE_AGL)
            Export_Data = vstack(Export_Data, current_data)
            println(Export_Data.numRows().toString() + "--------------------#For Output Above----------------------------" + '\n')
            if (Export_Data.numRows() >1000){

                WriteToFileBinary(Export_Data, "/home/suas/IdeaProjects/MotionModel/Scorpion_Fain/Filter_Output/SampleRun.txt")

                if (SavePixhawkData == true){
                        WriteToFileBinary(Export_Pixhawk, "/home/suas/IdeaProjects/MotionModel/Scorpion_Fain/Filter_Output/Pixhawk_Data.txt")}

                exitProcess(0)
            }
        }

//If the model does not propagate or the measurement time is after the filter time then propagate

       if (Input_LCM_Time.heading_time_flag == true && Input_LCM_Time.heading_time > filter.curTime.time && Input_LCM_Time.LCM_start_time_flag==true && HeadingUpdateOn == true){
            Input_LCM_Time.heading_time_flag = false


            //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            //filter.giveMeasurementProcessorAuxData("HeadingUpdate", pixhawk2_lcm_message_aux)

            val HeadingMeasurement = Measurement(processorLabel = "HeadingUpdate",
                    timeReceived = Time(Input_LCM_Time.heading_time),
                    timeValidity = Time(Input_LCM_Time.heading_time),
                    measurementData = mat[pixhawk2_lcm_message_aux.heading*(Math.PI/180)],
                    auxData = null,
                    measurementCov = mat[20*(Math.PI/180)])

            filter.update(HeadingMeasurement)
            var X = filter.getStateBlockEstimate("motionmodel").asRowVector()
            var P = filter.getStateBlockCovariance("motionmodel").diag().asRowVector()
            println('\n' + "An Update happened for the Heading to" + '\n'+ filter.curTime.time + '\n'
            + X[6].toString() + '\t' + P[6].toString() + '\n' )


        }




        //flags for aux update. take time of last flag. make sure to update and not propagate past on an old time. skip aux data for that time?


     //println(pixhawk1.airspeed[1].toString()+ '\t' + pixhawk1.airspeed[0].toString())
        }
    }
object InputLCMTimeCheck {
    var airspeed_time = 0.0
    var airspeed_time_flag = false
    var raw_rate_time = 0.0
    var raw_rate_time_flag = false
    var attitude_time = 0.0
    var attitude_time_flag = false
    var LCM_start_time = 0.0
    var LCM_start_time_flag = false
    var heading_time = 0.0
    var heading_time_flag = false
}
object GPS_origin_vars{
    var GPS_origin = mat[0.0,0.0,0.0]
    var GPS_origin_recieved = false
}


fun WriteToFileBinary(results: Matrix<Double>, fileName:String){
    System.out.println("Writing results to binary file...")
    var doubleArray = DoubleArray(results.numRows()*results.numCols())
    for(i in 0..results.numRows()-1){
        for(j in 0..results.numCols()-1){
            doubleArray.set(j+results.numCols()*i,results.get(i,j))
        }
    }
    val byteArray = ToByteArray(doubleArray)
    val fileTarget = File(fileName)
    fileTarget.writeBytes(byteArray)
    System.out.println("...results written")
}

fun ToByteArray(doubleArray: DoubleArray): ByteArray{
    val times = 8 //doubleSize/byteSize
    var bytes = ByteArray(doubleArray.size*times)
    for(i in 0..doubleArray.size-1){
        ByteBuffer.wrap(bytes, i*times, times).putDouble(doubleArray[i])
    }
    return bytes
}

fun SavePixhawkData(pixhawk2:pixhawk): Matrix<Double>{
    var SaveData = mat[pixhawk2.raw_mag[0],pixhawk2.raw_mag[2],pixhawk2.raw_mag[3],pixhawk2.raw_mag[4],pixhawk2.heading[0],pixhawk2.heading[1]]
return SaveData
}