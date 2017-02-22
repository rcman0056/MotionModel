import exlcm.pixhawk
import exlcm.range
import exlcm.range_sensor
import datasources.lcm.messages.aspn.rawopticalcameraimage
import golem.*
import golem.containers.Time
import golem.matrix.Matrix
import kotlin.system.exitProcess
import lcm.lcm.LCM
import lcm.lcm.LCMDataInputStream
import lcm.lcm.LCMSubscriber
import main.modules.stateblocks.ins.*
import modules.stateblocks.ins.FainImagePreMeasurements
import modules.stateblocks.ins.FainMeasurements
import modules.stateblocks.ins.MotionModelAuxData
import modules.stateblocks.ins.MotionModelBlock
import navutils.deltaLatToNorth
import navutils.deltaLonToEast
import navutils.rpyToDcm
import scorpion.buffers.Buffer
import java.io.*
import java.util.*
import java.nio.ByteBuffer
import scorpion.filters.sensor.StandardSensorEKF
import scorpion.filters.sensor.containers.Measurement

//Made as a global variable
var Input_LCM_Time = InputLCMTimeCheck  //used to check for time of first LCM message


var Export_Data = zeros(1, 34) //used to export the filter output data
var Export_Pixhawk = zeros(1, 6) // used to export Pixhawk data. Size depends on data wanted
var HeadingUpdateOn =true
var AltitudeUpdateOn = true
var VOUpdateOn = true
var VOUpdateSimOn = false  // set meas.cov to 5 in VO update at bottom
var GPS_SKIP = 0


var Num_Images_skipped = 0//Number of images to skip IE 2 means it will skip two images before processing. it will compare image 1 and 4.
var RangeUpdateOn = false
var SimulatedRangeUpdateOn = true
var SimulatedRangeUpdateTwoOn = false
var SavePixhawkData = true //used to plot True Heading not true GPS data
var Save_Name = "oneloopmmVORR_2"

// Main Function
object FAIN_Main {
    @JvmStatic
    fun main(args: Array<String>) {
        var P_count = 0.0
        var Length_Of_Run = 1000 //1000 FOR ONELOOP 1700 for longloop
//70


        var LCMMeasurements = FainMeasurements(0.0, 0.0, 0.0,0.0, 0.0, 0.0,0.0,0.0,0.0,0.0,zeros(1,3),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,zeros(1,3),0.0,0.0,0.0, 0.0, 0.0, 0.0, zeros(1, 4),zeros(1, 5), 0.0, zeros(1, 4),zeros(1, 4),zeros(1, 4),
                zeros(1, 4),zeros(1, 4), 0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, false,zeros(1,3),zeros(1,3), GPS_SKIP)

        //Set simulated range values for an aircraft that circles clockwise at a point starting at 1,0 on the unit circle with a given radius and ground speed.
        LCMMeasurements.simulated_range_CenterNEU = mat[0,3000,2000] //X Y Z 10K ft = 3048m
        LCMMeasurements.simulated_range_ground_speed = 120.0 // m/s  76=170 MPH   stall speed C-130 =115mph  120m/s =270mph
        LCMMeasurements.simulated_range_radius = 1000.0 // m


        //Set simulated range values for an aircraft that circles at a point with a given radius and ground speed.
        // When trying to have two aircraft fly the same path but at different locations around the radius set this value below
        LCMMeasurements.Circumference_Offset_rads = 0.0 //Math.PI/2  //Must be a double in radians. Math.PI/2 = plane starts at 0,1 on the unit circle flying clockwise
        LCMMeasurements.simulated_range_CenterNEU_Two = mat[3000,0,2000] //X Y Z 10K ft = 3048m
        LCMMeasurements.simulated_range_ground_speed_Two = 120.0 // m/s  76=170 MPH   stall speed C-130 =115mph
        LCMMeasurements.simulated_range_radius_Two = 1000.0 // m

        var Input_LCM_Time = InputLCMTimeCheck  //used to check for time of first LCM message
        var Image_data = FainImagePreMeasurements(Num_Images_skipped,Num_Images_skipped,IntArray((1280 * 960 + 2).toInt()), IntArray((1280 * 960 + 2).toInt()), 0.0, 0.0,
                zeros(3, 3), zeros(3, 3), false, false, zeros(3, 3))
        //DCM from a Perfect Cam to Body Frame
        var Cam_To_Body_DCM = mat[0, -1, 0 end 1, 0, 0 end 0, 0, 1]
        var Body_To_Corrected_Body = rpyToDcm( mat[-1.263,0.456,0]*Math.PI/180).T //Adjust for camera misalignment using the body frame
        //Cam to Body * Cam to Corrected Cam
        Image_data.Cam_To_Body_DCM = Body_To_Corrected_Body*Cam_To_Body_DCM
        val filter = StandardSensorEKF(Time(0.0), //Set time filter start here at 0.0
                buffer = Buffer())


        //For OF Pre processor
        var PreprocessorOptical_Flow = VO_Carson()


        var LCMChannels = LCM.getSingleton()
        LCMChannels.subscribe("PIXHAWK2", Subscribe_Pixhawk2(filter, LCMMeasurements, Input_LCM_Time, P_count,Length_Of_Run, Save_Name)) //This subscribes to LCM message is kept open as long as main is func is running
        LCMChannels.subscribe("PIXHAWK1", Subscribe_Pixhawk1(filter, LCMMeasurements))
        LCMChannels.subscribe("RANGE", Subscribe_Range(filter, LCMMeasurements, Input_LCM_Time))
        LCMChannels.subscribe("CAM", Subscribe_Cam(filter, LCMMeasurements, Input_LCM_Time, Image_data, PreprocessorOptical_Flow))

        val block = MotionModelBlock(label = "motionmodel")
        filter.addStateBlock(block)
        //Add measurement Processors

        //Create HeadingUpdate Processor
        val HeadingUpdate = "HeadingUpdate"
        filter.addMeasurementProcessor(HeadingMeasurementProcessor(HeadingUpdate, "motionmodel"))

        //Create RangeUpdate Processor
        val RangeUpdate = "RangeUpdate"
        filter.addMeasurementProcessor(RangeMeasurementProcessor(RangeUpdate, "motionmodel"))

        //Create AltitudeUpdate Processor
        val AltitudeUpdate = "AltitudeUpdate"
        filter.addMeasurementProcessor(AltitudeFainMeasurementProcessor(AltitudeUpdate, "motionmodel"))

        //Create SimulatedRangeUpdate Processor
        val SimulatedRangeUpdate = "SimulatedRangeUpdate"
        filter.addMeasurementProcessor(SimulatedRangeMeasurementProcessor(SimulatedRangeUpdate, "motionmodel"))

        //Create SimulatedRangeUpdate Processor Two
        val SimulatedRangeUpdateTwo = "SimulatedRangeUpdateTwo"
        filter.addMeasurementProcessor(SimulatedRangeMeasurementProcessorTwo(SimulatedRangeUpdateTwo, "motionmodel"))

        //Create VOUpdate Processor
        val VOUpdate = "VOUpdate"
        filter.addMeasurementProcessor(VOMeasurementProcessor(VOUpdate, "motionmodel"))

        //Set Intial Cov
        var initCov = zeros(9, 9)
        var tau_vv: Double = 2.0//time constant on alt_vv ... also set in MotionModelBlock
        var sigma_vv = 5.0 //sigma on alt_vv ... also set in MotionModelBlock
        initCov[0, 0] = 5                         //North (m)
        initCov[1, 1] = 5                         //East  (m)
        initCov[2, 2] = 5                         //Vg Ground Speed (m/s)
        initCov[3, 3] = 100 * Math.PI / 180            //chi Course Angle (rads)
        initCov[4, 4] = 3                         //Wind North (m/s)
        initCov[5, 5] = 3                         //Wind East  (m/s)
        initCov[6, 6] = 64 * Math.PI / 180         //Yaw (rads)
        initCov[7, 7] = 3                         //Alt (m)
        initCov[8, 8] = 2 * pow(sigma_vv, 2) / tau_vv  //Alt_VV
        filter.setStateBlockCovariance(label = "motionmodel",
                covariance = initCov)

        //Set initial states but they are not really used as LCM over writes them
        var initStates = zeros(9, 1)
        initStates[0] = 0
        initStates[1] = 0
        initStates[2] = 1
        initStates[3] = 20 * Math.PI / 180
        initStates[4] = 1
        initStates[5] = 1
        initStates[6] = 1
        initStates[7] = 1
        initStates[8] = 1
        filter.setStateBlockEstimate("motionmodel", initStates)
        Export_Data[0, 1..9] = initStates.T
        while (true) {

            //Need to include error
        }
    }
}

class Subscribe_Cam(var filter: StandardSensorEKF, var LCMMeasurements: FainMeasurements, var Input_LCM_Time: InputLCMTimeCheck,
                    var Image_data: FainImagePreMeasurements, var PreprocessorOptical_Flow: VO_Carson) : LCMSubscriber {

    // comments
    override fun messageReceived(p0: LCM, channel: String, p2: LCMDataInputStream) {

        //handle the simulated VO
        if (VOUpdateSimOn == true && LCMMeasurements.VelocityTruth_Time_Vx_Vy[2] != 0.0 && -30*Math.PI/180 < LCMMeasurements.Image_TimeRPY_current[1] &&  LCMMeasurements.Image_TimeRPY_current[1]< 30*Math.PI/180){
            LCMMeasurements.Image_dt_velocityXYZ = mat[0,LCMMeasurements.VelocityTruth_Time_Vx_Vy[1],LCMMeasurements.VelocityTruth_Time_Vx_Vy[2],0]
            Input_LCM_Time.image_update_time_flag = true
            LCMMeasurements.Image_time_dt_VxVyVz = mat[0,0,LCMMeasurements.VelocityTruth_Time_Vx_Vy[1],LCMMeasurements.VelocityTruth_Time_Vx_Vy[2],0]
        }





        if (VOUpdateOn == true &&  -10*Math.PI/180 < LCMMeasurements.Image_TimeRPY_current[1] &&  LCMMeasurements.Image_TimeRPY_current[1]< 10*Math.PI/180) {
//var Roll_prt = LCMMeasurements.Image_TimeRPY_current[1]*180/Math.PI
           // println("Roll = " + Roll_prt)
            //handle first image received and store to first image
            if(Image_data.First_Image_Received == false){
                var Camera = (rawopticalcameraimage(p2))
                var Image = Camera.data

                var Valid_Time_TAI = Camera.valid_t_sec + Camera.valid_t_nsec * pow(10, -9)//Add seconds and nano secs fields
                // var Valid_Time2 = Camera.arrival_t_sec + Camera.arrival_t_nsec * pow(10, -9)

                //Difference between TAI(PTP) and UTC(UNIX) is 36 Seconds  http://tycho.usno.navy.mil/leapsec.html
                //As of June 30 2015, and until the leap second of December 31 2016 TAI is ahead of UTC by 36 seconds.
                var Valid_Time_UTC = Valid_Time_TAI + 36



                Image_data.New_Image_Time_Valid = Valid_Time_UTC
                //Calculate the interpolated RPY where Y is just the current Filter Yaw set in FAIN_Main Pixhawk2 LCM Recieve
                if(Image_data.New_Image_Time_Valid > LCMMeasurements.Image_TimeRPY_current[0] && Image_data.New_Image_Time_Valid < LCMMeasurements.Image_TimeRPY_delayed_1[0]) {

                    var Time_Ratio = (Image_data.New_Image_Time_Valid-LCMMeasurements.Image_TimeRPY_current[0])/(LCMMeasurements.Image_TimeRPY_delayed_1[0]-LCMMeasurements.Image_TimeRPY_current[0])
                    var Roll_interpolated =  LCMMeasurements.Image_TimeRPY_current[1] + (LCMMeasurements.Image_TimeRPY_delayed_1[1]-LCMMeasurements.Image_TimeRPY_current[1])*Time_Ratio
                    var Pitch_interpolated =  LCMMeasurements.Image_TimeRPY_current[2] + (LCMMeasurements.Image_TimeRPY_delayed_1[2]-LCMMeasurements.Image_TimeRPY_current[2])*Time_Ratio
                    LCMMeasurements.Image_TimeRPY_delayed[1] = Roll_interpolated
                    LCMMeasurements.Image_TimeRPY_delayed[2] = Pitch_interpolated
                }
                else if(Image_data.New_Image_Time_Valid > LCMMeasurements.Image_TimeRPY_delayed_1[0] && Image_data.New_Image_Time_Valid < LCMMeasurements.Image_TimeRPY_delayed_2[0]){
                    var Time_Ratio = (Image_data.New_Image_Time_Valid-LCMMeasurements.Image_TimeRPY_delayed_1[0])/(LCMMeasurements.Image_TimeRPY_delayed_2[0]-LCMMeasurements.Image_TimeRPY_delayed_1[0])
                    var Roll_interpolated =  LCMMeasurements.Image_TimeRPY_delayed_1[1] + (LCMMeasurements.Image_TimeRPY_delayed_2[1]-LCMMeasurements.Image_TimeRPY_delayed_1[1])*Time_Ratio
                    var Pitch_interpolated =  LCMMeasurements.Image_TimeRPY_delayed_1[2] + (LCMMeasurements.Image_TimeRPY_delayed_2[2]-LCMMeasurements.Image_TimeRPY_delayed_1[2])*Time_Ratio
                    LCMMeasurements.Image_TimeRPY_delayed[1] = Roll_interpolated
                    LCMMeasurements.Image_TimeRPY_delayed[2] = Pitch_interpolated
                }
                else if(Image_data.New_Image_Time_Valid > LCMMeasurements.Image_TimeRPY_delayed_2[0] && Image_data.New_Image_Time_Valid < LCMMeasurements.Image_TimeRPY_delayed_3[0]){
                    var Time_Ratio = (Image_data.New_Image_Time_Valid-LCMMeasurements.Image_TimeRPY_delayed_2[0])/(LCMMeasurements.Image_TimeRPY_delayed_3[0]-LCMMeasurements.Image_TimeRPY_delayed_2[0])
                    var Roll_interpolated =  LCMMeasurements.Image_TimeRPY_delayed_2[1] + (LCMMeasurements.Image_TimeRPY_delayed_3[1]-LCMMeasurements.Image_TimeRPY_delayed_2[1])*Time_Ratio
                    var Pitch_interpolated =  LCMMeasurements.Image_TimeRPY_delayed_2[2] + (LCMMeasurements.Image_TimeRPY_delayed_3[2]-LCMMeasurements.Image_TimeRPY_delayed_2[2])*Time_Ratio
                    LCMMeasurements.Image_TimeRPY_delayed[1] = Roll_interpolated
                    LCMMeasurements.Image_TimeRPY_delayed[2] = Pitch_interpolated
                }
                else{LCMMeasurements.Image_TimeRPY_delayed=LCMMeasurements.Image_TimeRPY_delayed_2}

                var width = Camera.width
                var height = Camera.height

                //Create Data type for Image processor
                //Byte Array [height width row0 row1 row2....row(height)]
                var NewImage = IntArray(((1280 * 960) + 2).toInt())

                NewImage[0] = height
                NewImage[1] = width
                for (i in 0..height - 1) {
                    var big_index = 2 + (width * i)
                    for (a in 0..width - 1) {

                        NewImage[big_index + a] = Image[i][a].toInt()
                    }
                }

                //Calculate DCMs The Image received is on average .1 seconds behind(diff btwn Valid time and received time) so the current RPY is stored as the RPY updates at .1 secs
                var X = filter.getStateBlockEstimate("motionmodel").asRowVector()
                var Body_To_Nav_DCM = rpyToDcm(mat[LCMMeasurements.Image_TimeRPY_current[1], LCMMeasurements.Image_TimeRPY_current[2],  X[6]]).T //LCMMeasurements.Image_TimeRPY_current[3]]).T //Roll, Pitch, Not Filter Yaw
                //var Body_To_Nav_DCM = rpyToDcm(mat[LCMMeasurements.Image_TimeRPY_current[1], 0,  270*Math.PI/180]).T
                Image_data.New_Image_DCM = Body_To_Nav_DCM * Image_data.Cam_To_Body_DCM
                //Handle the case of the first image
                Image_data.Old_Image = NewImage
                Image_data.Old_Image_Time_Valid = Image_data.New_Image_Time_Valid
                Image_data.Old_Image_DCM = Image_data.New_Image_DCM
                Image_data.First_Image_Received = true
            }

            if(Image_data.Skip_Images_Count == 0 && Image_data.First_Image_Received == true){
                Image_data.Skip_Images_Count = Image_data.Skip_Images
            var Camera = (rawopticalcameraimage(p2))
            var Image = Camera.data

            var Valid_Time_TAI = Camera.valid_t_sec + Camera.valid_t_nsec * pow(10, -9)//Add seconds and nano secs fields
            // var Valid_Time2 = Camera.arrival_t_sec + Camera.arrival_t_nsec * pow(10, -9)

            //Difference between TAI(PTP) and UTC(UNIX) is 36 Seconds  http://tycho.usno.navy.mil/leapsec.html
            //As of June 30 2015, and until the leap second of December 31 2016 TAI is ahead of UTC by 36 seconds.
            var Valid_Time_UTC = Valid_Time_TAI + 36
           // var Valid_RPY_dt = Valid_Time_UTC -  LCMMeasurements.Image_TimeRPY_delayed[0]

                //println("Dt btwn RPY and Image Valid = " + Valid_RPY_dt.toString())
            var dt = Valid_Time_UTC - Image_data.Old_Image_Time_Valid

            Image_data.New_Image_Time_Valid = Valid_Time_UTC

                //Calculate the interpolated RPY where Y is just the current Filter Yaw set in FAIN_Main Pixhawk2 LCM Recieve
                if(Image_data.New_Image_Time_Valid < LCMMeasurements.Image_TimeRPY_current[0] && Image_data.New_Image_Time_Valid > LCMMeasurements.Image_TimeRPY_delayed_1[0]) {

                    var Time_Ratio = (Image_data.New_Image_Time_Valid-LCMMeasurements.Image_TimeRPY_current[0])/(LCMMeasurements.Image_TimeRPY_delayed_1[0]-LCMMeasurements.Image_TimeRPY_current[0])
                    var Roll_interpolated =  LCMMeasurements.Image_TimeRPY_current[1] + (LCMMeasurements.Image_TimeRPY_delayed_1[1]-LCMMeasurements.Image_TimeRPY_current[1])*Time_Ratio
                    var Pitch_interpolated =  LCMMeasurements.Image_TimeRPY_current[2] + (LCMMeasurements.Image_TimeRPY_delayed_1[2]-LCMMeasurements.Image_TimeRPY_current[2])*Time_Ratio
                    LCMMeasurements.Image_TimeRPY_delayed[1] = Roll_interpolated
                    LCMMeasurements.Image_TimeRPY_delayed[2] = Pitch_interpolated
                }
                else if(Image_data.New_Image_Time_Valid < LCMMeasurements.Image_TimeRPY_delayed_1[0] && Image_data.New_Image_Time_Valid > LCMMeasurements.Image_TimeRPY_delayed_2[0]){
                    var Time_Ratio = (Image_data.New_Image_Time_Valid-LCMMeasurements.Image_TimeRPY_delayed_1[0])/(LCMMeasurements.Image_TimeRPY_delayed_2[0]-LCMMeasurements.Image_TimeRPY_delayed_1[0])
                    var Roll_interpolated =  LCMMeasurements.Image_TimeRPY_delayed_1[1] + (LCMMeasurements.Image_TimeRPY_delayed_2[1]-LCMMeasurements.Image_TimeRPY_delayed_1[1])*Time_Ratio
                    var Pitch_interpolated =  LCMMeasurements.Image_TimeRPY_delayed_1[2] + (LCMMeasurements.Image_TimeRPY_delayed_2[2]-LCMMeasurements.Image_TimeRPY_delayed_1[2])*Time_Ratio
                    LCMMeasurements.Image_TimeRPY_delayed[1] = Roll_interpolated
                    LCMMeasurements.Image_TimeRPY_delayed[2] = Pitch_interpolated
                }
                else if(Image_data.New_Image_Time_Valid < LCMMeasurements.Image_TimeRPY_delayed_2[0] && Image_data.New_Image_Time_Valid > LCMMeasurements.Image_TimeRPY_delayed_3[0]){
                    var Time_Ratio = (Image_data.New_Image_Time_Valid-LCMMeasurements.Image_TimeRPY_delayed_2[0])/(LCMMeasurements.Image_TimeRPY_delayed_3[0]-LCMMeasurements.Image_TimeRPY_delayed_2[0])
                    var Roll_interpolated =  LCMMeasurements.Image_TimeRPY_delayed_2[1] + (LCMMeasurements.Image_TimeRPY_delayed_3[1]-LCMMeasurements.Image_TimeRPY_delayed_2[1])*Time_Ratio
                    var Pitch_interpolated =  LCMMeasurements.Image_TimeRPY_delayed_2[2] + (LCMMeasurements.Image_TimeRPY_delayed_3[2]-LCMMeasurements.Image_TimeRPY_delayed_2[2])*Time_Ratio
                    LCMMeasurements.Image_TimeRPY_delayed[1] = Roll_interpolated
                    LCMMeasurements.Image_TimeRPY_delayed[2] = Pitch_interpolated
                }
                else{LCMMeasurements.Image_TimeRPY_delayed=LCMMeasurements.Image_TimeRPY_delayed_2}

            var width = Camera.width
            var height = Camera.height

            //Create Data type for Image processor
            //Byte Array [height width row0 row1 row2....row(height)]
            var NewImage = IntArray(((1280 * 960) + 2).toInt())

            NewImage[0] = height
            NewImage[1] = width
            for (i in 0..height - 1) {
                var big_index = 2 + (width * i)
                for (a in 0..width - 1) {

                    NewImage[big_index + a] = Image[i][a].toInt()
                }
            }

            //Calculate DCMs The Image received is on average .1 seconds behind(diff btwn Valid time and received time) so the current RPY is stored as the RPY updates at .1 secs
            var X = filter.getStateBlockEstimate("motionmodel").asRowVector()
            //var Body_To_Nav_DCM = rpyToDcm(mat[LCMMeasurements.Image_TimeRPY_delayed[1], LCMMeasurements.Image_TimeRPY_delayed[2], LCMMeasurements.Image_TimeRPY_delayed[3]]).T //Roll, Pitch, Not Filter Yaw
                var Body_To_Nav_DCM = rpyToDcm(mat[LCMMeasurements.Image_TimeRPY_delayed[1], LCMMeasurements.Image_TimeRPY_delayed[2],  LCMMeasurements.Image_TimeRPY_delayed[3]]).T
                Image_data.New_Image_DCM = Body_To_Nav_DCM * Image_data.Cam_To_Body_DCM

                var Roll_1 =LCMMeasurements.Image_TimeRPY_delayed[1]*(180/Math.PI)
                var Pitch_1 = LCMMeasurements.Image_TimeRPY_delayed[2]*(180/Math.PI)
                var Yaw_1 = LCMMeasurements.Image_TimeRPY_delayed[3]*(180/Math.PI)

                println("DATASIMVO   " + Roll_1.toString() + "," + Pitch_1.toString() + "," + Yaw_1.toString() + ":")
                //var YawVoInput = LCMMeasurements.Image_TimeRPY_delayed[3]*180/Math.PI

               // println("YawVoInput = " + YawVoInput.toString())

                //Run Image Processor     h= agl [fx fy], focal center [cx cy]
                //fc = [ 998.09342   1005.01966 ];
                //cc = [ 670.90144   466.79380 ];


                //fun runOpticalFlow(timeStep: Double, firstImage: IntArray current image, secondImage previous: IntArray, h: Double, f: DoubleArray,
                //                  c: DoubleArray, firstCamToNav: Matrix<Double>, secondCamToNav: Matrix<Double>, t: Time,
                //                  filterVel: Matrix<Double>, processor: String, measCov: Matrix<Double>): Measurement {

                var Height_AGL = X[7]
                //var fc = doubleArrayOf(998.09342, 1005.01966)
                //var cc = doubleArrayOf(670.90144, 466.79380)

                var fc = doubleArrayOf(1026.30, 1023.331)
                var cc = doubleArrayOf(678.015, 463.498)



                var useFeatures = true //=> AKAZE
                //useFeatures = false //=> optical flow

                var inertialAiding = false //=> use both provided DCM's
                //inertialAiding = false => calculate the rotation between frames
                val params = booleanArrayOf(useFeatures,inertialAiding)
                println("dt=" + dt.toString()+" " +"Height=" + Height_AGL.toString())
                PreprocessorOptical_Flow.runVO(dt, Image_data.Old_Image, NewImage, Height_AGL, fc, cc, Image_data.Old_Image_DCM,
                        Image_data.New_Image_DCM, filter.curTime, zeros(3, 1), "Blank", zeros(3, 3),params)

                //LCMMeasurements.Image_dt_velocityXYZ = [dt,Vx,Vy,Vz]

                var VxVyVz = PreprocessorOptical_Flow.getDebugData().asRowVector()
                LCMMeasurements.Image_dt_velocityXYZ = mat[dt,VxVyVz[0],VxVyVz[1],VxVyVz[2]]
                LCMMeasurements.Image_time_dt_VxVyVz = mat[Image_data.Old_Image_Time_Valid,dt,VxVyVz[0],VxVyVz[1],VxVyVz[2]]

                //println(LCMMeasurements.Image_dt_velocityXYZ.toString())

                //Set Time of measurement to last Image received time...Ideally it would be dt/2 but the filter time is usually already past that from other measurements
                LCMMeasurements.Image_velocity_time = Image_data.New_Image_Time_Valid

                //Set Current Image Data to Old Image
                Image_data.Old_Image = NewImage

                //Set current time to Old Time
                Image_data.Old_Image_Time_Valid = Image_data.New_Image_Time_Valid

                Input_LCM_Time.image_update_time = Image_data.New_Image_Time_Valid

                LCMMeasurements.Image_velocity_time = Image_data.New_Image_Time_Valid //This time is not used but to keep consistency of times in LCMMeasurements

                var Vx = LCMMeasurements.Image_time_dt_VxVyVz[2]
                var Vy = LCMMeasurements.Image_time_dt_VxVyVz[3]
                var dt_1 = LCMMeasurements.Image_time_dt_VxVyVz[1]
                var Grd_spd = pow((Vx * Vx) + (Vy * Vy), .5) / dt_1
                if (Grd_spd < 32){
                Input_LCM_Time.image_update_time_flag = true}
                else{println("VO Skipped due to Grd Speed to high")}
            } else {
               Image_data.Skip_Images_Count = Image_data.Skip_Images_Count -1
                println("Image Skipped")
            }


        }
        else{println("VO Skipped due to Roll or VO Sim is set")}
    }
}


class Subscribe_Pixhawk1(var filter: StandardSensorEKF, var LCMMeasurements: FainMeasurements) : LCMSubscriber {

    // comments
    override fun messageReceived(p0: LCM, channel: String, p2: LCMDataInputStream) {
        var pixhawk1 = (pixhawk(p2))

        LCMMeasurements.GPS_Linux_time = pixhawk1.global_relative_frame[0]
        LCMMeasurements.GPS_lat = pixhawk1.global_relative_frame[1]
        LCMMeasurements.GPS_lon = pixhawk1.global_relative_frame[2]
        LCMMeasurements.GPS_height_agl = pixhawk1.global_relative_frame[3]
        LCMMeasurements.GPS_ground_speed = pixhawk1.groundspeed[1]



        //save off origin
        if (LCMMeasurements.GPS_origin_received == false) {
            LCMMeasurements.GPS_origin_received = true
            LCMMeasurements.GPS_origin_lat = LCMMeasurements.GPS_lat
            LCMMeasurements.GPS_origin_lon = LCMMeasurements.GPS_lon
            LCMMeasurements.GPS_origin_alt = LCMMeasurements.GPS_height_agl


            var current_gps = mat[LCMMeasurements.GPS_Linux_time, LCMMeasurements.GPS_lat, LCMMeasurements.GPS_lon, LCMMeasurements.GPS_height_agl]
            //Convert to NEU using first received GPS value as origin
            var current_gps_NE_AGL = mat[current_gps[0],
                    deltaLatToNorth((current_gps[1] - LCMMeasurements.GPS_origin_lat) * Math.PI / 180, current_gps[1] * Math.PI / 180, current_gps[3]), //Diff in rads,Current lat,Estimated ALT
                    deltaLonToEast((current_gps[2] - LCMMeasurements.GPS_origin_lon) * Math.PI / 180, current_gps[1] * Math.PI / 180, current_gps[3]),
                    current_gps[3]]


            LCMMeasurements.Velocity_GPS_Time_Lat_Lon = mat[current_gps_NE_AGL[0], current_gps_NE_AGL[1], current_gps_NE_AGL[2]]
        }


        //Calculate Estimated Vx and Vy based on GPS by comparing past NED with current NED GPS coords
//if (LCMMeasurements.Velocity_GPS_Time_Lat_Lon[0] < LCMMeasurements.GPS_Linux_time){

//    if (LCMMeasurements.Velocity_GPS_skip == 0){
//        var current_gps = mat[LCMMeasurements.GPS_Linux_time, LCMMeasurements.GPS_lat, LCMMeasurements.GPS_lon, LCMMeasurements.GPS_height_agl]
        //Convert to NEU using first received GPS value as origin
//        var current_gps_NE_AGL = mat[current_gps[0],
//                deltaLatToNorth((current_gps[1] - LCMMeasurements.GPS_origin_lat) * Math.PI / 180, current_gps[1] * Math.PI / 180, current_gps[3]), //Diff in rads,Current lat,Estimated ALT
//                deltaLonToEast((current_gps[2] - LCMMeasurements.GPS_origin_lon) * Math.PI / 180, current_gps[1] * Math.PI / 180, current_gps[3]),
//                current_gps[3]]

        //Calculate Estimated Vx and Vy based on GPS
//        var dt = LCMMeasurements.GPS_Linux_time - LCMMeasurements.Velocity_GPS_Time_Lat_Lon[0]
//        var Vx = (current_gps_NE_AGL[1] - LCMMeasurements.Velocity_GPS_Time_Lat_Lon[1])/dt
//        var Vy = (current_gps_NE_AGL[2] - LCMMeasurements.Velocity_GPS_Time_Lat_Lon[2])/dt
        var Vy = LCMMeasurements.GPS_ground_speed
        var Vx = 0
//    LCMMeasurements.Velocity_GPS_Time_Lat_Lon = mat[LCMMeasurements.GPS_Linux_time,current_gps_NE_AGL[1],current_gps_NE_AGL[2]]
    LCMMeasurements.VelocityTruth_Time_Vx_Vy = mat[LCMMeasurements.GPS_Linux_time,Vx,Vy]
//LCMMeasurements.Velocity_GPS_skip = GPS_SKIP

 //   }
 //       else {LCMMeasurements.Velocity_GPS_skip = LCMMeasurements.Velocity_GPS_skip - 1}}
//}
}}

class Subscribe_Range(var filter: StandardSensorEKF, var LCMMeasurements: FainMeasurements, Input_LCM_Time: InputLCMTimeCheck) : LCMSubscriber {

    // comments
    override fun messageReceived(p0: LCM, channel: String, p2: LCMDataInputStream) {
        //var range_data = (range(p2)) //Use this for the Telemaster data that uses the real ranging data and need to include a subscribe to Pixhawk 3
        var range_data = (range_sensor(p2))

        LCMMeasurements.range_time = range_data.timestamp
        LCMMeasurements.range = range_data.range_mag.toDouble()
        LCMMeasurements.range_Lat = range_data.mav_position[0].toDouble()
        LCMMeasurements.range_Lon = range_data.mav_position[1].toDouble()
        LCMMeasurements.range_alt = range_data.mav_position[2].toDouble()


        if (Input_LCM_Time.LCM_start_time_flag == true && RangeUpdateOn == true) {
            Input_LCM_Time.range_time = LCMMeasurements.range_time
            Input_LCM_Time.range_time_flag = true
        }

        if (Input_LCM_Time.LCM_start_time_flag == true && SimulatedRangeUpdateOn == true){
            Input_LCM_Time.simulated_range_time = LCMMeasurements.range_time //Set equal to time range on LCM was received
            Input_LCM_Time.simulated_range_time_flag = true

            //This if statement handles the fist SimRange update as the range is calculated using a dt. Only 1 LCM msg means only one time
            if(LCMMeasurements.simulated_range_dt == 0.0 && LCMMeasurements.simulated_range_total_dt == 0.0){
                LCMMeasurements.simulated_range_dt = LCMMeasurements.range_time - filter.curTime.time
                LCMMeasurements.simulated_range_total_dt = LCMMeasurements.simulated_range_total_dt + LCMMeasurements.simulated_range_dt
                LCMMeasurements.simulated_range_time_old = LCMMeasurements.range_time
            }
            //handle creation of dt and total_dt used to calculate MAVs position while flying a simulated circle.
            else{
                LCMMeasurements.simulated_range_dt = LCMMeasurements.range_time - LCMMeasurements.simulated_range_time_old
                LCMMeasurements.simulated_range_total_dt = LCMMeasurements.simulated_range_total_dt + LCMMeasurements.simulated_range_dt
                LCMMeasurements.simulated_range_time_old = LCMMeasurements.range_time
            }
        }

        //Handle second simulated range separately

        if (Input_LCM_Time.LCM_start_time_flag == true && SimulatedRangeUpdateTwoOn == true){
            Input_LCM_Time.simulated_range_time_Two = LCMMeasurements.range_time //Set equal to time range on LCM was received
            Input_LCM_Time.simulated_range_time_Two_flag = true

            //This if statement handles the fist SimRange update as the range is calculated using a dt. Only 1 LCM msg means only one time
            if(LCMMeasurements.simulated_range_dt_Two == 0.0 && LCMMeasurements.simulated_range_total_dt_Two == 0.0 && SimulatedRangeUpdateOn == false){
                LCMMeasurements.simulated_range_dt_Two = LCMMeasurements.range_time - filter.curTime.time
                LCMMeasurements.simulated_range_total_dt_Two = LCMMeasurements.simulated_range_total_dt_Two + LCMMeasurements.simulated_range_dt_Two
                LCMMeasurements.simulated_range_time_old_Two = LCMMeasurements.range_time
            }
            else if (LCMMeasurements.simulated_range_dt_Two == 0.0 && LCMMeasurements.simulated_range_total_dt_Two == 0.0 && SimulatedRangeUpdateOn == true){
                LCMMeasurements.simulated_range_total_dt_Two = LCMMeasurements.simulated_range_total_dt
                LCMMeasurements.simulated_range_time_old_Two = LCMMeasurements.simulated_range_time_old
            }
            //handle creation of dt and total_dt used to calculate MAVs position while flying a simulated circle.
            else{
                LCMMeasurements.simulated_range_dt_Two = LCMMeasurements.range_time - LCMMeasurements.simulated_range_time_old
                LCMMeasurements.simulated_range_total_dt_Two = LCMMeasurements.simulated_range_total_dt_Two + LCMMeasurements.simulated_range_dt_Two
                LCMMeasurements.simulated_range_time_old = LCMMeasurements.range_time
            }
        }

    }
}

//Subscriber for Pixhawk2 with filter updates
class Subscribe_Pixhawk2(var filter: StandardSensorEKF, var LCMMeasurements: FainMeasurements, var Input_LCM_Time: InputLCMTimeCheck, var P_count: Double, var Length_Of_Run: Int,Save_Name: String) : LCMSubscriber {

    // comments
    override fun messageReceived(p0: LCM, channel: String, p2: LCMDataInputStream) {
        var pixhawk2 = (pixhawk(p2))

        var pixhawk2_lcm_message_aux = MotionModelAuxData(pixhawk2.airspeed[1],
                pixhawk2.airspeed[0],
                pixhawk2.raw_rate[3],
                pixhawk2.raw_rate[4],
                pixhawk2.raw_rate[0],
                pixhawk2.attitude[1],
                pixhawk2.attitude[2],
                pixhawk2.attitude[0])
        //Set Heading and Altitude Measurements
        LCMMeasurements.heading = pixhawk2.heading[1]
        LCMMeasurements.heading_time = pixhawk2.heading[0]
        LCMMeasurements.pix2_alt = pixhawk2.global_relative_frame[3]
        LCMMeasurements.pix2_alt_time = pixhawk2.global_relative_frame[0]

        //Set RPY for VO
        LCMMeasurements.Image_TimeRPY_current = mat[pixhawk2.attitude[0], pixhawk2.attitude[1], pixhawk2.attitude[2], pixhawk2.attitude[3]]

if (LCMMeasurements.Image_TimeRPY_current[0] != LCMMeasurements.Image_TimeRPY_delayed_1[0] ) { /////////////////////////////////////////////////
    //LCMMeasurements.Image_TimeRPY_delayed = mat[pixhawk2.attitude[0], pixhawk2.attitude[1], pixhawk2.attitude[2], pixhawk2.attitude[3]]
    var K = filter.getStateBlockEstimate("motionmodel").asRowVector()
    LCMMeasurements.Image_TimeRPY_delayed_3 = mat[LCMMeasurements.Image_TimeRPY_delayed_2[0], LCMMeasurements.Image_TimeRPY_delayed_2[1], LCMMeasurements.Image_TimeRPY_delayed_2[2], LCMMeasurements.Image_TimeRPY_delayed_2[3]]
    LCMMeasurements.Image_TimeRPY_delayed_2 = mat[LCMMeasurements.Image_TimeRPY_delayed_1[0], LCMMeasurements.Image_TimeRPY_delayed_1[1], LCMMeasurements.Image_TimeRPY_delayed_1[2], LCMMeasurements.Image_TimeRPY_delayed_1[3]]
    LCMMeasurements.Image_TimeRPY_delayed_1 = mat[pixhawk2.attitude[0], pixhawk2.attitude[1], pixhawk2.attitude[2], pixhawk2.attitude[3]]

}
        if (SavePixhawkData == true) {
            var PixhawkRow = SavePixhawkData(pixhawk2)
            Export_Pixhawk = vstack(Export_Pixhawk, PixhawkRow)
        }
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
        //Upon getting an Image update just apply it to the current filter time. Otherwise need a delayed state filter
        if (Input_LCM_Time.LCM_start_time_flag == true) {

            if (Input_LCM_Time.image_update_time_flag == true && (VOUpdateOn == true || VOUpdateSimOn == true)) {
                //Set VOUpdate time to current filter time. Averages a .1~.25 second delay of when the image is recieved and the valid time for the images.
                Input_LCM_Time.image_update_time = filter.curTime.time
                filter.giveStateBlockAuxData("motionmodel", pixhawk2_lcm_message_aux)
                Input_LCM_Time = VOUpdate(filter, Input_LCM_Time, LCMMeasurements, Console_Output = true)
            }}

        //Check to see if individual time stamps have updated

        //If incoming message time is greater then stored time
        //Set time to new time
        //Set flag to high

        if (LCMMeasurements.heading_time > Input_LCM_Time.heading_time && Input_LCM_Time.LCM_start_time_flag == true) {
            Input_LCM_Time.heading_time = LCMMeasurements.heading_time
            Input_LCM_Time.heading_time_flag = true
        }

        if (LCMMeasurements.pix2_alt_time > Input_LCM_Time.altitude_time && Input_LCM_Time.LCM_start_time_flag == true) {
            Input_LCM_Time.altitude_time = LCMMeasurements.pix2_alt_time
            Input_LCM_Time.altitude_time_flag = true
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
            if (Input_LCM_Time.attitude_time > filter.curTime.time) {
                time = Input_LCM_Time.attitude_time
            } else if (Input_LCM_Time.raw_rate_time > filter.curTime.time) {
                time = Input_LCM_Time.raw_rate_time
            } else if (Input_LCM_Time.airspeed_time > filter.curTime.time) {
                time = Input_LCM_Time.airspeed_time
            } else {
                println('\n' + "Error trying to propagate prior to current filter time" + '\n'); exitProcess(0)
            }

            //set flags to false to reset time check
            Input_LCM_Time.airspeed_time_flag = false
            Input_LCM_Time.raw_rate_time_flag = false
            Input_LCM_Time.attitude_time_flag = false

            //Sets filter to initial LCM attitude time and initial state values
            if (Input_LCM_Time.LCM_start_time_flag == false) {
                Input_LCM_Time.LCM_start_time_flag = true
                Input_LCM_Time.LCM_start_time = time
                filter.curTime = Time(time)


                var initStates_LCM = zeros(9, 1)
                initStates_LCM[0] = 0
                initStates_LCM[1] = 0
                initStates_LCM[2] = pixhawk2.airspeed[1] //Set Ground speed guess to Air Speed
                initStates_LCM[3] = pixhawk2.heading[1] * Math.PI / 180 //Set Course Angle to Yaw Angle and assume no wind
                initStates_LCM[4] = 0
                initStates_LCM[5] = 0
                initStates_LCM[6] = pixhawk2.heading[1] * Math.PI / 180 //Set Yaw Angle
                initStates_LCM[7] = pixhawk2.global_relative_frame[3]
                initStates_LCM[8] = .01
                filter.setStateBlockEstimate("motionmodel", initStates_LCM)
            }




            // Set P to always be symmetric..This will likely be fixed in future Scorpion Jar Files
            //         if (P_count > 200){
            //                P_count = 0.0
            //          var P_1 = filter.getStateBlockCovariance("motionmodel")
            //         P_1 = (P_1 + P_1.T) / 2
            //       filter.setStateBlockCovariance("motionmodel", P_1)
            //        }
            //     else{P_count = P_count + 1}

            //Give Current Aux Data to the Filter
            filter.giveStateBlockAuxData("motionmodel", pixhawk2_lcm_message_aux)
            //Propagate to the time based on the LCM message time from Input_LCM_Time
            filter.propagate(Time(time))


            var time_filter = filter.curTime
            var X = filter.getStateBlockEstimate("motionmodel").asRowVector()
            var P = filter.getStateBlockCovariance("motionmodel").diag().asRowVector()
/*
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
*/
            //Get current GPS value:Note this is at 4hz while the Filter propagates at ~10Hz

            if (LCMMeasurements.GPS_origin_received == true) {
                var current_gps = mat[LCMMeasurements.GPS_Linux_time, LCMMeasurements.GPS_lat, LCMMeasurements.GPS_lon, LCMMeasurements.GPS_height_agl]
                //Convert to NEU using first received GPS value as origin
                var current_gps_NE_AGL = mat[current_gps[0],
                        deltaLatToNorth((current_gps[1] - LCMMeasurements.GPS_origin_lat) * Math.PI / 180, current_gps[1] * Math.PI / 180, current_gps[3]), //Diff in rads,Current lat,Estimated ALT
                        deltaLonToEast((current_gps[2] - LCMMeasurements.GPS_origin_lon) * Math.PI / 180, current_gps[1] * Math.PI / 180, current_gps[3]),
                        current_gps[3]]

                // var current_data = mat[time_filter.time, X States,GPS _Data,P Covariance after sqrt taken]
                var current_data = hstack(mat[time_filter.time], X, current_gps_NE_AGL, P,mat[LCMMeasurements.heading],mat[LCMMeasurements.GPS_ground_speed],LCMMeasurements.Image_time_dt_VxVyVz,LCMMeasurements.Image_TimeRPY_current)
                Export_Data = vstack(Export_Data, current_data)
                println(Export_Data.numRows().toString() + "--------------------#For Output Above----------------------------" + '\n')



                if (Export_Data.numRows() > Length_Of_Run) {

                    WriteToFileBinary(Export_Data, "/home/suas/IdeaProjects/MotionModel/Scorpion_Fain/Filter_Output/SampleRun_"+ Save_Name+".txt")

                    if (SavePixhawkData == true) {
                        WriteToFileBinary(Export_Pixhawk, "/home/suas/IdeaProjects/MotionModel/Scorpion_Fain/Filter_Output/Pixhawk_Data_"+ Save_Name+".txt")
                    }

                    exitProcess(0)
                }
            }
        }

        //If the model does not propagate or the measurement time is after the filter time then propagate based on the measurements available

        //Make sure initial LCM values are set
        if (Input_LCM_Time.LCM_start_time_flag == true) {

            ////Not used as image times are prior to the curr filter time due to delay in receiving the images
            //if (Input_LCM_Time.image_update_time_flag == true && Input_LCM_Time.image_update_time >= filter.curTime.time && VOUpdateOn == true) {
            //    filter.giveStateBlockAuxData("motionmodel", pixhawk2_lcm_message_aux)
            //    Input_LCM_Time = VOUpdate(filter, Input_LCM_Time, LCMMeasurements, Console_Output = false)
            //}


            if (Input_LCM_Time.simulated_range_time_flag == true && Input_LCM_Time.simulated_range_time >= filter.curTime.time && SimulatedRangeUpdateOn == true) {
                filter.giveStateBlockAuxData("motionmodel", pixhawk2_lcm_message_aux)
                Input_LCM_Time = SimulatedRangeUpdate(filter, Input_LCM_Time, LCMMeasurements, Console_Output = false)
            }

            if (Input_LCM_Time.simulated_range_time_Two_flag == true && Input_LCM_Time.simulated_range_time_Two >= filter.curTime.time && SimulatedRangeUpdateTwoOn == true) {
                filter.giveStateBlockAuxData("motionmodel", pixhawk2_lcm_message_aux)
                Input_LCM_Time = SimulatedRangeUpdateTwo(filter, Input_LCM_Time, LCMMeasurements, Console_Output = false)
            }

            if (Input_LCM_Time.range_time_flag == true && Input_LCM_Time.range_time >= filter.curTime.time && RangeUpdateOn == true) {
                filter.giveStateBlockAuxData("motionmodel", pixhawk2_lcm_message_aux)
                Input_LCM_Time = RangeUpdate(filter, Input_LCM_Time, LCMMeasurements, Console_Output = false)
            }

            if (Input_LCM_Time.heading_time_flag == true && Input_LCM_Time.heading_time >= filter.curTime.time && HeadingUpdateOn == true) {
                filter.giveStateBlockAuxData("motionmodel", pixhawk2_lcm_message_aux)
                Input_LCM_Time = HeadingUpdate(filter, Input_LCM_Time, LCMMeasurements, Console_Output = false)
            }


            if (Input_LCM_Time.altitude_time_flag == true && Input_LCM_Time.altitude_time >= filter.curTime.time && AltitudeUpdateOn == true) {
                filter.giveStateBlockAuxData("motionmodel", pixhawk2_lcm_message_aux)
                Input_LCM_Time = AltitudeUpdate(filter, Input_LCM_Time, LCMMeasurements, Console_Output = false)
            }
        }
        //Handle flagging for times of all 3 measurements
        /*
          if(Input_LCM_Time.heading_time_flag == true && Input_LCM_Time.heading_time > filter.curTime.time && HeadingUpdateOn == true &&
                  Input_LCM_Time.range_time_flag == true && Input_LCM_Time.range_time > filter.curTime.time && RangeUpdateOn == true &&
                  Input_LCM_Time.altitude_time_flag == true && Input_LCM_Time.altitude_time > filter.curTime.time && AltitudeUpdateOn == true){
              //handle range>heading>altitude times
              if(Input_LCM_Time.range_time > Input_LCM_Time.heading_time && Input_LCM_Time.heading_time > Input_LCM_Time.altitude_time) {
                  //Update Heading then Range if the Heading update has an earlier time
                  Input_LCM_Time = AltitudeUpdate(filter, Input_LCM_Time, LCMMeasurements, Console_Output = true)
                  Input_LCM_Time = HeadingUpdate(filter, Input_LCM_Time, LCMMeasurements, Console_Output = true)
                  Input_LCM_Time = RangeUpdate(filter, Input_LCM_Time, LCMMeasurements, Console_Output = true)
              }
              //handle heading>range>altitude times
              if(Input_LCM_Time.heading_time > Input_LCM_Time.range_time && Input_LCM_Time.range_time > Input_LCM_Time.altitude_time) {
                  //Update Heading then Range if the Heading update has an earlier time
                  Input_LCM_Time = AltitudeUpdate(filter, Input_LCM_Time, LCMMeasurements, Console_Output = true)
                  Input_LCM_Time = RangeUpdate(filter, Input_LCM_Time, LCMMeasurements, Console_Output = true)
                  Input_LCM_Time = HeadingUpdate(filter, Input_LCM_Time, LCMMeasurements, Console_Output = true)
              }
              //Handle
              if(Input_LCM_Time.range_time > Input_LCM_Time.heading_time && Input_LCM_Time.heading_time > Input_LCM_Time.altitude_time) {
                  //Update Heading then Range if the Heading update has an earlier time
                  Input_LCM_Time = AltitudeUpdate(filter, Input_LCM_Time, LCMMeasurements, Console_Output = true)
                  Input_LCM_Time = HeadingUpdate(filter, Input_LCM_Time, LCMMeasurements, Console_Output = true)
                  Input_LCM_Time = RangeUpdate(filter, Input_LCM_Time, LCMMeasurements, Console_Output = true)
              }
              else {Input_LCM_Time = RangeUpdate(filter, Input_LCM_Time, LCMMeasurements, Console_Output = true)
                  Input_LCM_Time = HeadingUpdate(filter, Input_LCM_Time, LCMMeasurements, Console_Output = true)}


          */
    }


    fun HeadingUpdate(filter: StandardSensorEKF,
                      Input_LCM_Time: InputLCMTimeCheck,
                      LCMMeasurements: FainMeasurements,
                      Console_Output: Boolean): InputLCMTimeCheck {
        Input_LCM_Time.heading_time_flag = false


        val HeadingMeasurement = Measurement(processorLabel = "HeadingUpdate",
                timeReceived = Time(Input_LCM_Time.heading_time),
                timeValidity = Time(Input_LCM_Time.heading_time),
                measurementData = mat[LCMMeasurements.heading * (Math.PI / 180)],
                auxData = null,
                measurementCov = mat[20 * 20 * (Math.PI / 180)])

        filter.update(HeadingMeasurement)
        var X = filter.getStateBlockEstimate("motionmodel").asRowVector()
        var P = filter.getStateBlockCovariance("motionmodel").diag().asRowVector()
        if (Console_Output) {
            println('\n' + "An Update happened for the Heading to" + '\n' + filter.curTime.time + '\n'
                    + X[6].toString() + '\t' + P[6].toString() + '\n')
        }
        return Input_LCM_Time
    }

    fun RangeUpdate(filter: StandardSensorEKF,
                    Input_LCM_Time: InputLCMTimeCheck,
                    LCMMeasurements: FainMeasurements,
                    Console_Output: Boolean): InputLCMTimeCheck {

        Input_LCM_Time.range_time_flag = false
        val RangeMeasurement = Measurement(processorLabel = "RangeUpdate",
                timeReceived = Time(Input_LCM_Time.range_time),
                timeValidity = Time(Input_LCM_Time.range_time),
                measurementData = mat[LCMMeasurements.range],
                auxData = LCMMeasurements,
                measurementCov = mat[18 * 18])

        filter.update(RangeMeasurement)
        var X = filter.getStateBlockEstimate("motionmodel").asRowVector()
        var P = filter.getStateBlockCovariance("motionmodel").diag().asRowVector()
        if (Console_Output) {
            println('\n' + "An Update happened for the Range to" + '\n' + filter.curTime.time + '\n'
                    + "Pn=" + X[0].toString() + '\t' + P[0].toString() + '\n'
                    + "Pe=" + X[1].toString() + '\t' + P[1].toString() + '\n'
                    + "Alt=" + X[7].toString() + '\t' + P[7].toString() + '\n')
        }
        return Input_LCM_Time
    }

    fun SimulatedRangeUpdate(filter: StandardSensorEKF,
                    Input_LCM_Time: InputLCMTimeCheck,
                    LCMMeasurements: FainMeasurements,
                    Console_Output: Boolean): InputLCMTimeCheck {

        Input_LCM_Time.simulated_range_time_flag = false
        val SimulatedRangeMeasurement = Measurement(processorLabel = "SimulatedRangeUpdate",
                timeReceived = Time(Input_LCM_Time.simulated_range_time),
                timeValidity = Time(Input_LCM_Time.simulated_range_time),
                measurementData = mat[LCMMeasurements.simulated_range],//not used as z is calculated in the processor
                auxData = LCMMeasurements,
                measurementCov = mat[5 * 5])

        filter.update(SimulatedRangeMeasurement)
        var X = filter.getStateBlockEstimate("motionmodel").asRowVector()
        var P = filter.getStateBlockCovariance("motionmodel").diag().asRowVector()
        if (Console_Output) {
            println('\n' + "An Update happened for the Range to" + '\n' + filter.curTime.time + '\n'
                    + "Pn=" + X[0].toString() + '\t' + P[0].toString() + '\n'
                    + "Pe=" + X[1].toString() + '\t' + P[1].toString() + '\n'
                    + "Alt=" + X[7].toString() + '\t' + P[7].toString() + '\n')
        }
        return Input_LCM_Time
    }

    fun SimulatedRangeUpdateTwo(filter: StandardSensorEKF,
                             Input_LCM_Time: InputLCMTimeCheck,
                             LCMMeasurements: FainMeasurements,
                             Console_Output: Boolean): InputLCMTimeCheck {

        Input_LCM_Time.simulated_range_time_Two_flag = false
        val SimulatedRangeMeasurementTwo = Measurement(processorLabel = "SimulatedRangeUpdateTwo",
                timeReceived = Time(Input_LCM_Time.simulated_range_time_Two),
                timeValidity = Time(Input_LCM_Time.simulated_range_time_Two),
                measurementData = mat[LCMMeasurements.simulated_range],//not used as z is calculated in the processor
                auxData = LCMMeasurements,
                measurementCov = mat[5 * 5])

        filter.update(SimulatedRangeMeasurementTwo)
        var X = filter.getStateBlockEstimate("motionmodel").asRowVector()
        var P = filter.getStateBlockCovariance("motionmodel").diag().asRowVector()
        if (Console_Output) {
            println('\n' + "An Update happened for the RangeTwo to" + '\n' + filter.curTime.time + '\n'
                    + "Pn=" + X[0].toString() + '\t' + P[0].toString() + '\n'
                    + "Pe=" + X[1].toString() + '\t' + P[1].toString() + '\n'
                    + "Alt=" + X[7].toString() + '\t' + P[7].toString() + '\n')
        }
        return Input_LCM_Time
    }

    fun VOUpdate(filter: StandardSensorEKF,
                    Input_LCM_Time: InputLCMTimeCheck,
                    LCMMeasurements: FainMeasurements,
                    Console_Output: Boolean): InputLCMTimeCheck {

        Input_LCM_Time.image_update_time_flag = false
        val VOMeasurement = Measurement(processorLabel = "VOUpdate",
                timeReceived = Time(Input_LCM_Time.image_update_time),
                timeValidity = Time(Input_LCM_Time.image_update_time),
                measurementData = LCMMeasurements.Image_dt_velocityXYZ,//Not Used as measurement is calculated from LCMMeasurements
                auxData = null,
                measurementCov = mat[10*10])//, 0 end 0, 360*360*(Math.PI/2)])

        filter.update(VOMeasurement)
        var X = filter.getStateBlockEstimate("motionmodel").asRowVector()
        var P = filter.getStateBlockCovariance("motionmodel").diag().asRowVector()
        if (Console_Output) {
            println('\n' + "An Update happened for the VO to" + '\n' + filter.curTime.time + '\n'
                    + "GrdSpeed=" + X[2].toString() + '\t' + P[2].toString() + '\n'
                    + "CourseAng=" + X[4].toString() + '\t' + P[4].toString() + '\n'
                    )
        }
        return Input_LCM_Time
    }

    fun AltitudeUpdate(filter: StandardSensorEKF,
                       Input_LCM_Time: InputLCMTimeCheck,
                       LCMMeasurements: FainMeasurements,
                       Console_Output: Boolean): InputLCMTimeCheck {

        Input_LCM_Time.altitude_time_flag = false
        val AltitudeMeasurement = Measurement(processorLabel = "AltitudeUpdate",
                timeReceived = Time(Input_LCM_Time.altitude_time),
                timeValidity = Time(Input_LCM_Time.altitude_time),
                measurementData = mat[LCMMeasurements.pix2_alt],
                auxData = null,
                measurementCov = mat[64])

        filter.update(AltitudeMeasurement)
        var X = filter.getStateBlockEstimate("motionmodel").asRowVector()
        var P = filter.getStateBlockCovariance("motionmodel").diag().asRowVector()
        if (Console_Output) {
            println('\n' + "An Update happened for the Altitiude to" + '\n' + filter.curTime.time + '\n'
                    + "Alt=" + X[7].toString() + '\t' + P[7].toString() + '\n')
        }
        return Input_LCM_Time
    }

}

fun WriteToFileBinary(results: Matrix<Double>, fileName: String) {
    System.out.println("Writing results to binary file...")
    var doubleArray = DoubleArray(results.numRows() * results.numCols())
    for (i in 0..results.numRows() - 1) {
        for (j in 0..results.numCols() - 1) {
            doubleArray.set(j + results.numCols() * i, results.get(i, j))
        }
    }
    val byteArray = ToByteArray(doubleArray)
    val fileTarget = File(fileName)
    fileTarget.writeBytes(byteArray)
    System.out.println("...results written")
}

fun ToByteArray(doubleArray: DoubleArray): ByteArray {
    val times = 8 //doubleSize/byteSize
    var bytes = ByteArray(doubleArray.size * times)
    for (i in 0..doubleArray.size - 1) {
        ByteBuffer.wrap(bytes, i * times, times).putDouble(doubleArray[i])
    }
    return bytes
}

fun SavePixhawkData(pixhawk2: pixhawk): Matrix<Double> {
    var SaveData = mat[pixhawk2.raw_mag[0], pixhawk2.raw_mag[2], pixhawk2.raw_mag[3], pixhawk2.raw_mag[4], pixhawk2.heading[0], pixhawk2.heading[1]]
    return SaveData
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
    var range_time = 0.0
    var range_time_flag = false
    var simulated_range_time = 0.0
    var simulated_range_time_flag = false
    var simulated_range_time_Two = 0.0
    var simulated_range_time_Two_flag = false
    var altitude_time_flag = false
    var altitude_time = 0.0
    var image_update_time = 0.0
    var image_update_time_flag = false
}