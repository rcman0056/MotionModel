#include <jni.h>
#include <stdio.h>
#include "runOpticalFlow.h"
#include "opticalFlowVO.h"

JNIEXPORT jdoubleArray JNICALL Java_runOpticalFlow_opticalFlow
          (JNIEnv *env, jobject obj, jobject buffer) {
	//get buffer address and capacity
	jbyte* buf = (*env)->GetDirectBufferAddress(env,buffer);
	float capacity = (*env)->GetDirectBufferCapacity(env,buffer);
	//int imgHeight = (buf[0]<<8)|(buf[1]&0x0FF);
	//int imgWidth = (buf[2]<<8)|(buf[3]&0x0FF);
	unsigned char* imageCArray = buf;

	int LENGTH = 3;
	jdouble outCArray[LENGTH];
	double* temp;

	temp = opticalFlow(imageCArray);

	int i;
	for(i=0;i<LENGTH;i++)
	{
		outCArray[i]  = temp[i];
	}
 	
   // Convert the C's Native jdouble[] to JNI jdoublearray, and return
   jdoubleArray outJNIArray = (*env)->NewDoubleArray(env, LENGTH);  // allocate
   if (NULL == outJNIArray) return NULL;
   (*env)->SetDoubleArrayRegion(env, outJNIArray, 0 , LENGTH, outCArray);  // copy
   return outJNIArray;
}