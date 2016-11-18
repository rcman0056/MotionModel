/**
 * Created by ucav on 8/2/2016.
 */
package com.jcg;

import golem.*
import golem.matrix.Matrix
import navutils.*
import plotResults
import convertArrayListToArray
import readImus
import readTdf
import runFilter
import setupFilter
import setupIns
import setupMasterNav
import java.util.*
import functionTime
import writeArrayToBinary
import writeToFileASCII
import writeToFileBinary
import java.io.File


/*
 * Attempting to build a filter that tracks both aircraft simultaneously
 */
var runtime: ArrayList<functionTime> = ArrayList(20)
var beginTime=System.currentTimeMillis()
const val LEADER_LABEL="pinson15L"  //Name of the PINSON15NED block for Leader in filter
const val RECEIVER_LABEL="pinson15R"//Name of the PINSON15NED block for Receiver in filter
const val BIASNORTH="stereonorthbias"
const val BIASEAST="stereoeastbias"
const val BIASDOWN="stereodownbias"
const val DGPS_POSITION="dgpsposition"
const val DGPS_RELATIVE="dgpsrelative"
const val STEREO_POSITION="stereoposition"//Name of the Stereo Position measurement processor
const val STEREO_ATTITUDE="stereoattitude"//Name of the Attitude measurement processor
const val DATADATE="1107"
const val LABELIN=DATADATE+"_KVH"
      var LABELOUT=""

// Leader data labels
const val LEADER_IMU_FILE ="../Kotlin/data/Imu_Tdf/Tanker_".plus(LABELIN).plus("_imu")
//const val LEADER_IMU_FILE ="C:/AFIT/aar/students/dan.johnson/Relative_Filter/Kotlin/data/Imu_Tdf/Tanker_".plus(LABELIN).plus("_imu")
const val LEADER_TRUTH_FILE ="../Kotlin/data/Truth/BlueMax/Tanker_".plus(DATADATE).plus(".tdf")

//const val LEADER_IMU_FILE ="C:/AFIT/aar/students/dan.johnson/Relative_Filter/Kotlin/tanker_0912_imu"
//const val LEADER_TRUTH_FILE ="C:/AFIT/aar/students/dan.johnson/Relative_Filter/Kotlin/Tanker.tdf"

// Meta data from fly required (mechanization needs a start point other than truth
const val LEADER_START_FILE ="../Kotlin/data/Imu_Meta/Tanker_".plus(LABELIN).plus("_meta")
//const val LEADER_START_FILE ="C:/AFIT/aar/students/dan.johnson/Relative_Filter/Kotlin/tanker_0912_meta"

//Receiver data labels
const val RECEIVER_IMU_FILE ="../Kotlin/data/Imu_Tdf/receiver_".plus(LABELIN).plus("_imu")
//const val RECEIVER_IMU_FILE ="C:/AFIT/aar/students/dan.johnson/Relative_Filter/Kotlin/data/Imu_Tdf/Receiver_".plus(LABELIN).plus("_imu")
const val RECEIVER_TRUTH_FILE ="../Kotlin/data/Truth/BlueMax/Receiver_".plus(DATADATE).plus(".tdf")
//const val RECEIVER_IMU_FILE ="C:/AFIT/aar/students/dan.johnson/Relative_Filter/Kotlin/receiver_0912_imu"
//const val RECEIVER_TRUTH_FILE ="C:/AFIT/aar/students/dan.johnson/Relative_Filter/Kotlin/Receiver.tdf"

// Meta data from fly required (mechanization needs a start point other than truth
const val RECEIVER_START_FILE ="../Kotlin/data/Imu_Meta/Receiver_".plus(LABELIN).plus("_meta")
//const val RECEIVER_START_FILE ="C:/AFIT/aar/students/dan.johnson/Relative_Filter/Kotlin/receiver_0912_meta"

//File containing stereo measuremets
//const val STEREO_FILE = "../Kotlin/data/Measurements/1027_noboom_onespeed_finalPosMeas.tdf"
      val STEREO_LABEL="1109_absolute_noboom_bigterrain3900_onespeed"
      //val STEREO_LABEL="1109_absolute_noboom_noterrain_onespeed_ideal_renderfix"
      val STEREO_FILE= "C:/AFIT/aar/students/dan.johnson/Relative_Filter/Kotlin/data/StereoData/Stable_1104/".plus(STEREO_LABEL).plus("/").plus(STEREO_LABEL).plus("_finalPosMeas.tdf")
//const val STEREO_FILE = "../Kotlin/data/Measurements/Sample_Measurements_Short_1018.tdf"
//const val STEREO_FILE = "C:/AFIT/aar/students/dan.johnson/Relative_Filter/Kotlin/Sample_Measurements_Short.tdf"

//File containing DGP
const val DGPS_FILE="../Kotlin/data/DGPS_Tdf/".plus(DATADATE).plus("_DGPS")
//const val DGPS_FILE="../Kotlin/data/DGPS_Tdf/DGPS_relative.tdf"

      //Reading in the IMU measurements
      val measurements = readImus(LEADER_IMU_FILE,RECEIVER_IMU_FILE,"\t")

      val llhOffset= mat[-28.74,0,-12.29]


      //Reading in truth files and converting to a golem matrix
      val truthListL=readTdf(LEADER_TRUTH_FILE,"\t",true)
      val truthL=convertArrayListToArray(truthListL)
      val truthListR=readTdf(RECEIVER_TRUTH_FILE,"\t",true)
      var truthR=convertArrayListToArray(truthListR)


      //Reading in stereo measurements
      var stereoMeasList = readTdf(STEREO_FILE,"\t",false)
      var stereoMeasArray = convertArrayListToArray(stereoMeasList)

      //Reading in DGPS measurements
      var DGPSMeasList=readTdf(DGPS_FILE,"\t",false)
      var DGPSMeasArray = convertArrayListToArray(DGPSMeasList)




      //Constants for ease of use later
      val pi=Math.PI
      val dtr=pi/180
      val rtd=180/pi
      var rtm=radToMeter(truthL.get(1).times(dtr),truthL.get(3))

      //Generating filter start time and measurement start times (the simulation only outputs time starting at 0.0)
      val gpsStartTime=measurements.get(0).get(0,0)//-.01
      val startTime=gpsStartTime

      var write=true;
      var includeMeasurements=true
      var includeStereo=true
      var includeGPS=false
      var relativeProcessor=false
      var limitGPS=false


      /**
 * Main function
 */

fun main(args: Array<String>){
    var filter = setupFilter()
          runtime.add(functionTime("filterSetup",System.currentTimeMillis(),(System.currentTimeMillis()-beginTime).div(1000.0)))

    //This offsets the receiver by the tanker size.  Originaly flight paths didn't take into account aircraft size
    //offSetReceiver(llhOffset)
          runtime.add(functionTime("offSet",System.currentTimeMillis(),(System.currentTimeMillis()-runtime.last().stopTime).div(1000.0)))

    var InsL=setupIns(LEADER_TRUTH_FILE,LEADER_START_FILE)
    var InsR=setupIns(RECEIVER_TRUTH_FILE,RECEIVER_START_FILE)
          runtime.add(functionTime("insSetup",System.currentTimeMillis(),(System.currentTimeMillis()-runtime.last().stopTime).div(1000.0)))

    var masterL= setupMasterNav(filter,LEADER_LABEL)
    var masterR = setupMasterNav(filter,RECEIVER_LABEL)
          runtime.add(functionTime("masterSetup",System.currentTimeMillis(),(System.currentTimeMillis()-runtime.last().stopTime).div(1000.0)))


    var results=runFilter(filter,InsL,InsR,masterL,masterR, LEADER_LABEL,RECEIVER_LABEL,includeMeasurements,includeStereo,includeGPS)
          runtime.add(functionTime("filter",System.currentTimeMillis(),(System.currentTimeMillis()-runtime.last().stopTime).div(1000.0)))

    //writeToFileASCII(results,"Results.txt")
    //      runtime.add(functionTime("writeASCII",System.currentTimeMillis(),(System.currentTimeMillis()-runtime.last().stopTime).div(1000.0)))

          if(write==true) {
              var fileBase="C:/AFIT/aar/students/dan.johnson/Relative_Filter/Kotlin/data/Filter_Results/"
              var GPSLABEL="GPS"
              if(limitGPS==false){
                  GPSLABEL="GPSFULL"
              }

              if(includeMeasurements==false){
                LABELOUT="_INS"
              }
              else if(includeStereo==true&&includeGPS==true){
                  LABELOUT="_INS_STEREO_".plus(GPSLABEL)
              }
              else if(includeStereo==true){
                  LABELOUT="_INS_STEREO"
              }
              else if (includeGPS==true){
                  LABELOUT="_INS_".plus(GPSLABEL)
              }

              writeToFileBinary(results, fileBase.plus("Results_").plus(STEREO_LABEL).plus("_").plus(DATADATE).plus(LABELOUT).plus(".bin"))
              runtime.add(functionTime("writeBinary", System.currentTimeMillis(), (System.currentTimeMillis() - runtime.last().stopTime).div(1000.0)))

              writeArrayToBinary(truthR, fileBase.plus("truthR.bin"))
              runtime.add(functionTime("writeArrayBinary", System.currentTimeMillis(), (System.currentTimeMillis() - runtime.last().stopTime).div(1000.0)))
          }
    plotResults(results)
          runtime.add(functionTime("plotting",System.currentTimeMillis(),(System.currentTimeMillis()-runtime.last().stopTime).div(1000.0)))
          runtime.add(functionTime("total",System.currentTimeMillis(),(System.currentTimeMillis()-beginTime).div(1000.0)))

          for( i in 0..runtime.size-1) {
              println(runtime.get(i))
          }


      }





fun offSetReceiver(offset:Matrix<Double>){
    var offsetCol=zeros(3,1)
    //making sure the offset input is in a column
    if(offset.numRows()==1)
    {offsetCol=offset.T}
    else{offsetCol=offset}

    //changing offset vector to be NRD instead of NLU
    /*
    offsetCol[1]=offsetCol[1].times(-1)
    offsetCol[2]=offsetCol[2].times(-1)
    */
    //This is the initial longitude difference between receiver and tanker it should be zero
    var longDiff=(truthR.get(0,2)-truthL.get(0,2))

    //how many data points are there
    var size=truthR.numRows()
    var longDiffFinal=truthR.get(size-1,2)-truthL.get(size-1,2)

    //pulling out just the llh from truth
    var truthPosLlh=truthR[0..size-1,1..3]
    //correcting for the long offset if there is one
    //    truthPosLlh[0..size-1,1]=truthPosLlh[0..size-1,1].plus(longDiff)
    //converting lat lon to radians
        truthPosLlh[0..size-1,0..1]=truthPosLlh[0..size-1,0..1].times(dtr)
    //grabbing truth rpy
    var rpy=truthL.get(0,4..6).times(dtr)

    //creating dcm from body to nav frame
    var Cnb=rpyToDcm(rpy)
    //adjust the tanker frame offset to be in the nav frame
    var llhOffset=Cnb*offsetCol
    //apply offset to truth llh
    var row=truthPosLlh.getRow(0)
        row[0]=row[0]+llhOffset[0].times(1/rtm.second)
        row[1]=row[1]+llhOffset[1].times(1/rtm.first)
        row[2]=row[2]+llhOffset[2]

    var truthPosLlhOffset=row

    //repeat offSet for all rows in truth
    for(i in 1..size-1){
        row=truthPosLlh.getRow(i)
        rpy=truthL.get(i,4..6).times(dtr)
        Cnb=rpyToDcm(rpy)
        llhOffset=Cnb*offsetCol

        row[0]=row[0]+llhOffset[0].times(1/rtm.second)
        row[1]=row[1]+llhOffset[1].times(1/rtm.first)
        row[2]=row[2]+llhOffset[2]
        truthPosLlhOffset=vstack(truthPosLlhOffset,row)
    }

    //convert back to degrees for truth file
    truthPosLlhOffset[0..size-1,0..1]=truthPosLlhOffset[0..size-1,0..1].times(rtd)
    truthR[0..size-1,1..3]=truthPosLlhOffset

}






//Dataouputstream os=new dataoutputstream(new Fileoutputstream("")