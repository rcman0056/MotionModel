
// finish this first
import exlcm.pixhawk

import golem.*
import golem.containers.Time
import golem.matrix.Matrix

import lcm.lcm.LCM
import lcm.lcm.LCMDataInputStream
import lcm.lcm.LCMSubscriber
import modules.stateblocks.ins.MotionModelAuxData
import modules.stateblocks.ins.MotionModelBlock
import scorpion.buffers.Buffer
import java.io.*
import java.util.*
import java.nio.ByteBuffer
import scorpion.filters.sensor.StandardSensorEKF
import kotlin.system.exitProcess

var Input_LCM_Time = InputLCMTimeCheck  //Made as a global variable
var Export_Data = zeros(1,10) //used to export the data

// Main Function
object FAIN_Main {

    @JvmStatic
    fun main(args: Array<String>) {


        val filter = StandardSensorEKF(Time(0.0), //Set time filter start here at 0.0
                buffer = Buffer())

        var pixhawklcm = LCM.getSingleton()
        pixhawklcm.subscribe("PIXHAWK2", FAIN_Subscribe(filter)) //This subscribes to LCM message is kept open as long as main is func is running


        val block = MotionModelBlock(label = "motionmodel")
            filter.addStateBlock(block)



        //Set Intial Cov
        val initCov = zeros(9, 9)
        var tau_vv:Double = 2.0//time constant on alt_vv ... also set in MotionModelBlock
        var sigma_vv = 5.0 //sigma on alt_vv ... also set in MotionModelBlock
        //initCov[0..7,0..7]=eye(8,8)*.001    //add noise to states for now need to calculate input noise
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

//Subscriber
class FAIN_Subscribe(var filter:StandardSensorEKF ): LCMSubscriber {

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
                                     pixhawk2.attitude[0])
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

            var time = Input_LCM_Time.attitude_time
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
                initStates_LCM[4] = 0.001
                initStates_LCM[5] = 0.001
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
           /* println(time_filter.toString() + '\n' +
                    X[0].toString() + '\t' + P[0].toString() + '\n' +
                    X[1].toString() + '\t' + P[1].toString() + '\n' +
                    X[2].toString() + '\t' + P[2].toString() + '\n' +
                    X[3].toString() + '\t' + P[3].toString() + '\n' +
                    X[4].toString() + '\t' + P[4].toString() + '\n' +
                    X[5].toString() + '\t' + P[5].toString() + '\n' +
                    X[6].toString() + '\t' + P[6].toString() + '\n' +
                    X[7].toString() + '\t' + P[7].toString() + '\n' +
                    X[8].toString() + '\t' + P[8].toString() + '\n')
            */
           // var current_data = mat[time_filter.time, X]
            var current_data = hstack(mat[time_filter.time], X)
            Export_Data = vstack(Export_Data, current_data)
            println(Export_Data.numRows().toString() + '\n')
            if (Export_Data.numRows() >1000){
                WriteToFileBinary(Export_Data, "/home/suas/IdeaProjects/MotionModel/Scorpion_Fain/Filter_Output/SampleRun.txt")
                exitProcess(0)
            }
        }


        //flags for aux update. take time of last flag. make sure to update and not propogate past on an old time. skip aux data for that time?


     //println(pixhawk1.airspeed[1].toString()+ '\t' + pixhawk1.airspeed[0].toString())
        }
    }
object InputLCMTimeCheck{
    var airspeed_time = 0.0
    var airspeed_time_flag = false
    var raw_rate_time = 0.0
    var raw_rate_time_flag = false
    var attitude_time = 0.0
    var attitude_time_flag = false
    var LCM_start_time = 0.0
    var LCM_start_time_flag = false
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