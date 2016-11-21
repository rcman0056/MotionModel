
// finish this first
import exlcm.pixhawk

import golem.*

import lcm.lcm.LCM
import lcm.lcm.LCMDataInputStream
import lcm.lcm.LCMSubscriber

import scorpion.filters.sensor.StandardSensorEKF


// Main Function
object FAIN_Main {

    @JvmStatic
    fun main(args: Array<String>) {
        var pixhawklcm = LCM.getSingleton()
        pixhawklcm.subscribe("PIXHAWK2", FAIN_Subscribe()) //This subscribes to LCM message is kept open as long as main is func is running



        val block = MotionModelBlock(label = "motionmodel")

        val filter = StandardSensorEKF(Time(0.0, TimeStandard.WALL), //What to do for time????????????????????? my time is seconds since epoch
                    buffer = Buffer(standard = TimeStandard.WALL))


        filter.addStateBlock(block)



        val initCov = zeros(9, 9)
        var tau_vv:Double = 2.0//time constant on alt_vv ... also set in MotionModelBlock
        var sigma_vv = 5.0 //sigma on alt_vv ... also set in MotionModelBlock
        initCov[0..7,0..7]=eye(8)*.01    //add noise to states for now need to calculate input noise
        initCov[8, 8] = 2*pow(sigma_vv,2)/tau_vv
        filter.setStateBlockCovariance(label = "motionmodel",
                    covariance = initCov)
        //while(true){}
    }
}

//Subscriber
class FAIN_Subscribe(): LCMSubscriber {

    // comments
    override fun messageReceived(p0: LCM, channel: String, p2: LCMDataInputStream){
        var pixhawk2 = (pixhawk(p2))

        var aux = MotionModelAuxData(pixhawk2.airspeed[1],
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

     //println(pixhawk1.airspeed[1].toString()+ '\t' + pixhawk1.airspeed[0].toString())
        }
    }

