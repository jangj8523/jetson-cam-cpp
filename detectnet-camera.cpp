/*
 * Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "gstCamera.h"

#include <iostream>
#include <mosquitto.h>
#include <exception>
#include <stdlib.h>
#include "SceneObject.h"
#include "Constant.h"
#include <thread>
#include <pthread.h>
#include "CentroidTracker.h"
#include <python3.6/Python.h>
#include <pybind11/pybind11.h>
#include "PyUtils.h"
#include <mutex>          // std::mutex
#include <condition_variable> // std::condition_variable

//#include "NRMKMqttWrapper.h"
//#include "paho.mqtt.cpp/src/mqtt/client.h"
//#include "paho.mqtt.cpp/src/mqtt/connect_options.h"

#include "glDisplay.h"
#include "glTexture.h"

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <cmath>

//#include <ctime>
#include <sys/time.h>


#include "cudaMappedMemory.h"
#include "cudaNormalize.h"
#include "cudaFont.h"
#include <string>
#include "detectNet.h"
#include <unordered_map>
#include <map>

namespace py = pybind11;
using namespace std;

#define DEFAULT_CAMERA -1	// -1 for onboard camera, or change to index of /dev/video V4L2 camera (>=0)




string hst = "192.168.1.247";
const char *host = hst.c_str();
int port = 1883;
string vhost = "yourvhost";
string usn = "username";
string vhusn = vhost + ":" + usn;
const char *username = vhusn.c_str();
string pwd = "password";
const char *password = pwd.c_str();
string tpc = "jetson";
const char *topic = tpc.c_str();

string receive_tpc = "intercept";
const char *received_topic = receive_tpc.c_str();


CentroidTracker ct;
bool signal_received = false;
gstCamera* camera = gstCamera::Create(DEFAULT_CAMERA);
void* imgCPU  = NULL;
void* imgCUDA = NULL;
void* imgRGBA = NULL;

float* bbCPU    = NULL;
float* bbCUDA   = NULL;
float* confCPU  = NULL;
float* confCUDA = NULL;
detectNet* net;
cudaFont * font;

void sig_handler(int signo)
{
	if( signo == SIGINT )
	{
		printf("received SIGINT\n");
		signal_received = true;
	}
}


double compute_lidar_angle (double x_coordinate) {
	double pivot = constants::HORIZONTAL_RANGE / 2;
  double x_1 = (x_coordinate < pivot) ? pivot - x_coordinate : x_coordinate - pivot;
  double theta = atan(constants::TAN_31_1 * x_1 / pivot) * 180.0 / M_PI;
  double angle = (x_coordinate < pivot) ? 360.0 - theta : theta;
  return angle;
}

void onConnect(struct mosquitto *mosq, void *userdata, int result) {
	if (!result) {
		try {
			mosquitto_subscribe(mosq, NULL, received_topic, 1);
		} catch (exception& e) {
			printf("Error: Failed to subscribe1!!!\n%s\n", e.what());
		}
	} else {
		printf("Error: Failed to connect TO SUBSCRIBE!!\n");
	}
}

void onMessage(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message) {
  if(message->payloadlen) {
    char *payload_string = new char [message->payloadlen];
    strcpy(payload_string, (char*) message->payload);

		int num_bounding_boxes = 1;
    float depth, x_left, y_left, x_right, y_right, unix_time;

		sscanf (payload_string, "%d;", &num_bounding_boxes);
		payload_string += 2;

		vector<vector<float>> bounding_box_list;
		for (int i = 0; i < num_bounding_boxes; i++) {
    	sscanf (payload_string, "%f,%f,%f,%f,%f,%f;", &depth, &x_left, &y_left, &x_right, &y_right, &unix_time);
			while (*payload_string != ';') {
				payload_string++;
			}
			payload_string++;
			printf ("===========\npayload from lidar: %f, %f, %f, %f, %f, %f\n===========\n\n", depth, x_left, y_left, x_right, y_right, unix_time);
			vector<float> boxes {x_left, y_left, x_right, y_right, depth, unix_time};
			bounding_box_list.push_back(boxes);
			payload_string++;
		}
    printf("Topic: %s, Message: %s\n", (char*)message->topic, (char*)message->payload);
    printf("Done\n\n");

		std::map<int, int> id_mapping = ct.update(bounding_box_list);


		int lastClass = 0;
		int lastStart = 0;
		for (int n = 0; n < num_bounding_boxes; n++) {
			const int nc = confCUDA[n*2+1];
			float* bb = bbCUDA + (n * 4);
			if( nc != lastClass || n == (bounding_box_list.size() - 1) )
			{
				string str = std::to_string(id_mapping[n]);
				const char * str_id = str.c_str();
				int x_left = bounding_box_list[n][0];
				int y_left = bounding_box_list[n][1];
				if( !net->DrawBoxes((float4*)imgRGBA, (float4*)imgRGBA, camera->GetWidth(), camera->GetHeight(),
																	bbCUDA + (lastStart * 4), (n - lastStart) + 1, lastClass) )
					printf("detectnet-console:  failed to draw boxes\n");

				if ( !font->RenderOverlay((float4*)imgRGBA, (float4*)imgRGBA, camera->GetWidth(), camera->GetHeight(),
							str_id, x_left, y_left) )
					printf("detectnet-console:  failed to draw font\n");



				lastClass = nc;
				lastStart = n;

				CUDA(cudaDeviceSynchronize());
			}
		}
	} else {
		printf("Topic: %s, Message: (null)\n", message->topic);
	}
	fflush(stdout);
}

void *functionCount1(void *exit_flag) {
  while (true) {
    char character = cin.get();
    fflush(stdin);
    if (character == 'n'){
        printf("Breaking;\n");
        break;
    }
    printf("runing\n");
    *(bool *)exit_flag = true;
  }
}



int main( int argc, char** argv )
{
	int keepalive = 60;
	bool clean_session = true;
	struct mosquitto *mosq = NULL;
	bool exit_flag = false;



  /*pthread_t thread1;
  int rc = pthread_create( &thread1, NULL, functionCount1, &exit_flag);
  printf("Initialized thread\n");*/
  // char *arg1="sir", *arg2="robin", *cstr;
  PyObject *pmod, *pclass, *pargs, *pinst, *pmeth, *pres;;
  Py_Initialize();
  printf ("Initialized Python integration\n");

  pmod   = PyImport_ImportModule("sayhi");
  pclass = PyObject_GetAttrString(pmod, "SayHi");
  Py_DECREF(pmod);

  pargs  = Py_BuildValue("(i)", 7);
  pinst  = PyEval_CallObject(pclass, pargs);
  Py_DECREF(pclass);
  Py_DECREF(pargs);

  //result = instance.method(x,y)

  pmeth  = PyObject_GetAttrString(pinst, "say_age");
  Py_DECREF(pinst);
  pargs  = Py_BuildValue("()");
  pres   = PyEval_CallObject(pmeth, pargs);
  Py_DECREF(pmeth);
  Py_DECREF(pargs);
  Py_DECREF(pres);


	//create producer and connect to broker
	mosquitto_lib_init();
	mosq = mosquitto_new(NULL, clean_session, NULL);
	//mosquitto_username_pw_set(mosq, username, password);
  mosquitto_connect_callback_set(mosq, onConnect);
	mosquitto_message_callback_set(mosq, onMessage);

	if(mosquitto_connect(mosq, host, port, keepalive)) {
		printf("Error: Failed to connect\n");
		return 1;
	}
	//usually start loop right after connecting
	mosquitto_loop_start(mosq);
	printf("detectnet-camera\n  args (%i):  ", argc);


	for( int i=0; i < argc; i++ )
		printf("%i [%s]  ", i, argv[i]);


	printf("\n\n");

  ct.print_test();
  vector <vector<float>> bounding_test;
  vector<float> sample {0.1, 0.1, 0.1, 0.1, 0.3};
  bounding_test.push_back(sample);
  ct.update(bounding_test);

  vector <vector<float>> bounding_test2;
  vector<float> sample2 {0.19, 0.2, 0.5, 0.7, 0.32};
  bounding_test2.push_back(sample2);
  ct.update(bounding_test2);
  printf("Test done");


	/*
	 * parse network type from CLI arguments
	 */
	detectNet::NetworkType networkType = detectNet::PEDNET_MULTI;

	/*if( argc > 1 )
	{
		if( strcmp(argv[1], "multiped") == 0 || strcmp(argv[1], "pednet") == 0 || strcmp(argv[1], "multiped-500") == 0 )
			networkType = detectNet::PEDNET_MULTI;
		else if( strcmp(argv[1], "ped-100") == 0 )
			networkType = detectNet::PEDNET;
		else if( strcmp(argv[1], "facenet") == 0 || strcmp(argv[1], "facenet-120") == 0 || strcmp(argv[1], "face-120") == 0 )
			networkType = detectNet::FACENET;
	}*/

	if( signal(SIGINT, sig_handler) == SIG_ERR ) {
		printf("\ncan't catch SIGINT\n");

	}
	/*
	 * create the camera device
	 */

	if( !camera )
	{
		printf("\ndetectnet-camera:  failed to initialize video device\n");
		return 0;
	}

	printf("\ndetectnet-camera:  successfully initialized video device\n");
	printf("    width:  %u\n", camera->GetWidth());
	printf("   height:  %u\n", camera->GetHeight());
	printf("    depth:  %u (bpp)\n\n", camera->GetPixelDepth());


	/*
	 * create detectNet
	 */
	net = detectNet::Create(argc, argv);
	font = cudaFont::Create();
	if( !net )
	{
		printf("detectnet-camera:   failed to initialize imageNet\n");
		return 0;
	}


	/*
	 * allocate memory for output bounding boxes and class confidence
	 */
	const uint32_t maxBoxes = net->GetMaxBoundingBoxes();
	const uint32_t classes  = net->GetNumClasses();



	if( !cudaAllocMapped((void**)&bbCPU, (void**)&bbCUDA, maxBoxes * sizeof(float4)) ||
	    !cudaAllocMapped((void**)&confCPU, (void**)&confCUDA, maxBoxes * classes * sizeof(float)) )
	{
		printf("detectnet-console:  failed to alloc output memory\n");
		return 0;
	}


	/*
	 * create openGL window
	 */
	glDisplay* display = glDisplay::Create();
	glTexture* texture = NULL;

	if( !display ) {
		printf("\ndetectnet-camera:  failed to create openGL display\n");
	}
	else
	{
		texture = glTexture::Create(camera->GetWidth(), camera->GetHeight(), GL_RGBA32F_ARB/*GL_RGBA8*/);

		if( !texture )
			printf("detectnet-camera:  failed to create openGL texture\n");
	}


	/*
	 * create font
	 */
	cudaFont* font = cudaFont::Create();


	/*
	 * start streaming
	 */
	if( !camera->Open() )
	{
		printf("\ndetectnet-camera:  failed to open camera for streaming\n");
		return 0;
	}

	printf("\ndetectnet-camera:  camera open for streaming\n");



	/*
	 * processing loop
	 */
	float confidence = 0.0f;

	while( !signal_received )
	{
		// get the latest frame
		if( !camera->Capture(&imgCPU, &imgCUDA, 1000) )
			printf("\ndetectnet-camera:  failed to capture frame\n");

		// convert from YUV to RGBA


		if( !camera->ConvertRGBA(imgCUDA, &imgRGBA) )
			printf("detectnet-camera:  failed to convert from NV12 to RGBA\n");

		// classify image with detectNet
		int numBoundingBoxes = maxBoxes;

		if( net->Detect((float*)imgRGBA, camera->GetWidth(), camera->GetHeight(), bbCUDA, &numBoundingBoxes, confGPU))
		{
      vector <vector<float>> bounding_box_list;
      time_t sec = time(NULL);

			for( int n=0; n < numBoundingBoxes; n++ )
			{

				const int nc = confGPU[n*2+1];
				float* bb = bbGPU + (n * 4);

				printf("detected obj %i  class #%u (%s)  confidence=%f\n", n, nc, net->GetClassDesc(nc), confGPU[n*2]);
				printf("bounding box %i  (%f, %f)  (%f, %f)  w=%f  h=%f\n", n, bb[0], bb[1], bb[2], bb[3], bb[2] - bb[0], bb[3] - bb[1]);
        printf("the current time is %ld\n", sec);
        printf("numBoundingBoxes: %d\n", numBoundingBoxes);
        time_t sec = time(NULL);
				printf("the current time is %ld\n", sec);
        vector <float> myBoxes {bb[0], bb[1], bb[2], bb[3], (float)sec};
        bounding_box_list.push_back(myBoxes);
			}



      if (numBoundingBoxes > 0) {

				char *message_to_mqtt = (char*) malloc(sizeof(char[300]));
				char *tracker = message_to_mqtt;
				int total = 0;

				for(int i=0; i < 300; i++) {
					 *tracker = '\n';
				}
				tracker = message_to_mqtt;

        for (std::vector<std::vector<float>>::iterator index = bounding_box_list.begin(); index != bounding_box_list.end(); ++index) {
          vector<float> curr = *index;
          for (std::vector<float>::const_iterator i = curr.begin(); i != curr.end(); ++i)
             std::cout << *i << " YEAH! \n";
		 			int count = sprintf(message_to_mqtt, "%.2f,%.2f,%.2f,%.2f,%.2f,%ld;", curr[0], curr[1], curr[2], curr[3], curr[0] + (curr[2] - curr[0])/2, sec);
		 			printf("printed value: %.2f,%.2f,%.2f,%.2f,%.2f,%ld;\n", curr[0], curr[1], curr[2], curr[3], curr[0] + (curr[2] - curr[0])/2, sec);
					tracker += count;
					total += count;

        }
				try {
						mosquitto_publish(mosq, NULL, topic, total, message_to_mqtt, 1, false);
				} catch(exception& e) {
					 printf("Error: Failed to publish message\n%s\n", e.what());
					 return 1;
				}
				free(message_to_mqtt);
        std::cout <<"\n";
      }


			if( display != NULL )
			{
				char str[256];
				sprintf(str, "TensorRT %i.%i.%i | %s | %04.1f FPS", NV_TENSORRT_MAJOR, NV_TENSORRT_MINOR, NV_TENSORRT_PATCH, precisionTypeToStr(net->GetPrecision()), display->GetFPS());
				display->SetTitle(str);
			}
		}


		// update display
		if( display != NULL )
		{
			display->UserEvents();
			display->BeginRender();

			if( texture != NULL )
			{
				// rescale image pixel intensities for display
				CUDA(cudaNormalizeRGBA((float4*)imgRGBA, make_float2(0.0f, 255.0f),
								   (float4*)imgRGBA, make_float2(0.0f, 1.0f),
		 						   camera->GetWidth(), camera->GetHeight()));

				// map from CUDA to openGL using GL interop
				void* tex_map = texture->MapCUDA();

				if( tex_map != NULL )
				{
					cudaMemcpy(tex_map, imgRGBA, texture->GetSize(), cudaMemcpyDeviceToDevice);
					texture->Unmap();
				}

				// draw the texture
				texture->Render(100,100);
			}

			display->EndRender();
		}
    if (exit_flag == true) {
			break;
		}
	}

	printf("\ndetectnet-camera:  un-initializing video device\n");


	/*
	 * shutdown the camera device
	 */
	if( camera != NULL )
	{
		delete camera;
		camera = NULL;
	}

	if( display != NULL )
	{
		delete display;
		display = NULL;
	}

	// pthread_join(rc, NULL);
  // pthread_exit(NULL);

	mosquitto_loop_stop(mosq, true);
	mosquitto_disconnect(mosq);
	mosquitto_destroy(mosq);
	mosquitto_lib_cleanup();

	printf("detectnet-camera:  video device has been un-initialized.\n");
	printf("detectnet-camera:  this concludes the test of the video device.\n");
	return 0;
}
