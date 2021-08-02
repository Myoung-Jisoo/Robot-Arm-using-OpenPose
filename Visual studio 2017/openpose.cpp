// ------------------------- OpenPose C++ API Tutorial - Example 11 - Custom Output -------------------------
// Asynchronous mode: ideal for fast prototyping when performance is not an issue.
// In this function, the user can implement its own way to render/display/storage the results.

// Third-party dependencies
#include <opencv2/opencv.hpp>
// Command-line user interface
#define OPENPOSE_FLAGS_DISABLE_DISPLAY
#include <openpose/flags.hpp>
// OpenPose dependencies
#include <openpose/headers.hpp>
#include <math.h>
#include <vector>
#include <ctime>
// Custom OpenPose flags
// Display
//////////////////////////////////////////////////////////////////////////////////////////////////////////
// File      : Comm.h
// Version   : 1.0.1
// Date      : 2019.08.21
// Writer   : Lee, Seungmin (CDSL)
//////////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef _CCOMM_H__
#define _CCOMM_H__


//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Class Name   : CComm
// Summury      : Port Handler
//////////////////////////////////////////////////////////////////////////////////////////////////////////
class CComm {

	// Define ////////////////////////////////////////////////////////
public:

protected:

private:
	class _CPortHandler;


	// Method ////////////////////////////////////////////////////////
public:
	////////////////////////////////////////////////////////////////////////////////////////////
	// Method   : CComm
	// Input   : None
	// Output   : None
	// Summury   : Standard constructor
	////////////////////////////////////////////////////////////////////////////////////////////
	CComm();



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method   : ~CComm
	// Input   : None
	// Output   : None
	// Summury   : Standard destructor
	////////////////////////////////////////////////////////////////////////////////////////////
	~CComm();



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method   : Open
	// Input   : port name(const char), baudrate(int)
	// Output   : Result(bool)
	// Summury   : Open port handler.
	////////////////////////////////////////////////////////////////////////////////////////////
	bool Open(const char* port, int baudrate);



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method   : Close
	// Input   : None
	// Output   : Result(bool)
	// Summury   : Close port handler.
	////////////////////////////////////////////////////////////////////////////////////////////
	bool Close();



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method   : isOpen
	// Input   : None
	// Output   : isopen(bool)
	// Summury   : The port is open or not
	////////////////////////////////////////////////////////////////////////////////////////////
	bool isOpen();



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method   : Write
	// Input   : data buffer(const char*), data length(int)
	// Output   : length of sended data(int)
	// Summury   : Write data to port.
	////////////////////////////////////////////////////////////////////////////////////////////
	int Write(const char* buf, int length);



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method   : Read
	// Input   : data buffer(char*), data length(int)
	// Output   : length of recved data(int)
	// Summury   : Read data from recv buffer.
	////////////////////////////////////////////////////////////////////////////////////////////
	int Read(char* buf, int length);



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method   : SetBufSize
	// Input   : buffer size(int)
	// Output   : None
	// Summury   : Set buffer size.
	////////////////////////////////////////////////////////////////////////////////////////////
	void SetBufSize(int size);



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method   : SetTimeOut
	// Input   : time(double)
	// Output   : None
	// Summury   : Set Time Out.
	////////////////////////////////////////////////////////////////////////////////////////////
	void SetTimeOut(double t);

protected:

private:


	// Member ////////////////////////////////////////////////////////
public:

protected:

private:

	_CPortHandler *_poHandler;
};

#endif

DEFINE_bool(no_display, false,
	"Enable to disable the visual display.");

int filter = 0;
double angle1, angle2;
double upLength = 0;
double upLength_Max;
double downLength = 0;
double downLength_Max;
double dx1, dy1, dx2, dy2;

cv::Point shoulder;
cv::Point elbow;
cv::Point wrist;
cv::Point f_shoulder;
cv::Point f_elbow;

std::vector<double> shoulderX;
std::vector<double> shoulderY;
std::vector<double> elbowX;
std::vector<double> elbowY;
std::vector<double> wristX;
std::vector<double> wristY;

int shoulder_count; // 사람이 가만히 있나
int elbow_count;

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// File      : Comm.cpp
// Version   : 1.0.1
// Date      : 2019.08.21
// Writer   : Lee, Seungmin (CDSL)
//////////////////////////////////////////////////////////////////////////////////////////////////////////

//include//
#include <chrono>
#include <thread>
#include <mutex>

//Windows Platform
#if defined(_WIN32) || defined(_WIN64)
#define __WINDOWS_PLATFORM_
#include <Windows.h>

//Linux Platform
#elif defined(__linux__)
#define __LINUX_PLATFORM_
#include <sys/time.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#endif


//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Class Name   : CComm::_CPortHandler
// Summury      : Private Class for Port Handler
//////////////////////////////////////////////////////////////////////////////////////////////////////////
class CComm::_CPortHandler {

	// Define ////////////////////////////////////////////////////////
public:

protected:

private:


	// Method ////////////////////////////////////////////////////////
public:

	////////////////////////////////////////////////////////////////////////////////////////////
	// Method   : _CPortHandler
	// Input   : None
	// Output   : None
	// Summury   : Standard constructor
	////////////////////////////////////////////////////////////////////////////////////////////
	_CPortHandler() :
		_timeout(0.001), _bufSize(4096), _isOpen(false) {

		memset(_portName, 0, 100);
	}



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method   : ~_CPortHandler
	// Input   : None
	// Output   : None
	// Summury   : Standard destructor
	////////////////////////////////////////////////////////////////////////////////////////////
	~_CPortHandler() {

	}



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method   : Open
	// Input   : port name(const char), baudrate(int)
	// Output   : Result(bool)
	// Summury   : Open port handler.
	////////////////////////////////////////////////////////////////////////////////////////////
	bool Open(const char* port, int baudrate) {

		bool result = false;

		sprintf_s(this->_portName, sizeof(_portName), "\\\\.\\%s", port);
		_baudrate = baudrate;

		if (!_open(_baudrate)) {
			result = false;
			_isOpen = false;
			return result;
		}

		result = true;
		_isOpen = true;
		return result;
	}



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method   : Close
	// Input   : None
	// Output   : Result(bool)
	// Summury   : Close port handler.
	////////////////////////////////////////////////////////////////////////////////////////////
	bool Close() {

		bool result = false;

		if (!_isOpen)
			return result;

		_close();

		result = true;
		_isOpen = false;
		return result;
	}



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method   : isOpen
	// Input   : None
	// Output   : isopen(bool)
	// Summury   : The port is open or not
	////////////////////////////////////////////////////////////////////////////////////////////
	bool isOpen() {

		return _isOpen;
	}



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method   : Write
	// Input   : data buffer(const char*), data length(int)
	// Output   : length of sended data(int)
	// Summury   : Write data to port.
	////////////////////////////////////////////////////////////////////////////////////////////
	int Write(const char* buf, int length) {

		DWORD dwWrite = 0;

		if (WriteFile(_handle, buf, (DWORD)length, &dwWrite, NULL) == FALSE)
			return -1;

		return (int)dwWrite;
	}



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method   : Read
	// Input   : data buffer(char*), data length(int)
	// Output   : length of recved data(int)
	// Summury   : Read data from recv buffer.
	////////////////////////////////////////////////////////////////////////////////////////////
	int Read(char* buf, int length) {

		DWORD dwRead = 0;

		if (ReadFile(_handle, buf, (DWORD)length, &dwRead, NULL) == FALSE)
			return -1;

		return dwRead;
	}



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method   : SetBufSize
	// Input   : buffer size(int)
	// Output   : None
	// Summury   : Set buffer size.
	////////////////////////////////////////////////////////////////////////////////////////////
	void SetBufSize(int size) {

		_bufSize = size;
	}



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method   : SetTimeOut
	// Input   : time(double)
	// Output   : None
	// Summury   : Set Time Out.
	////////////////////////////////////////////////////////////////////////////////////////////
	void SetTimeOut(double t) {

		_timeout = t;
	}

protected:

private:
	////////////////////////////////////////////////////////////////////////////////////////////
	// Method   : _open
	// Input   : baudrate(int)
	// Output   : Result(bool)
	// Summury   : Open port handler.
	////////////////////////////////////////////////////////////////////////////////////////////
	bool _open(int baudrate) {

		DCB dcb;
		COMMTIMEOUTS timeouts;
		DWORD dwError;

		_close();

		//Create Port Handle.
		_handle = CreateFileA(_portName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
		if (_handle == INVALID_HANDLE_VALUE)
		{
			printf("[PortHandlerWindows::SetupPort] Error opening serial port!\n");
			return false;
		}

		dcb.DCBlength = sizeof(DCB);
		if (GetCommState(_handle, &dcb) == FALSE) {
			_close();
			return false;
		}

		// Set baudrate
		dcb.BaudRate = (DWORD)baudrate;
		dcb.ByteSize = 8;               // Data bit = 8bit
		dcb.Parity = NOPARITY;            // No parity
		dcb.StopBits = ONESTOPBIT;          // Stop bit = 1
		dcb.fParity = NOPARITY;             // No Parity check
		dcb.fBinary = 1;                    // Binary mode
		dcb.fNull = 0;                  // Get Null byte
		dcb.fAbortOnError = 0;
		dcb.fErrorChar = 0;

		// Not using XOn/XOff
		dcb.fOutX = 0;
		dcb.fInX = 0;

		// Not using H/W flow control
		dcb.fDtrControl = DTR_CONTROL_DISABLE;
		dcb.fRtsControl = RTS_CONTROL_DISABLE;
		dcb.fDsrSensitivity = 0;
		dcb.fOutxDsrFlow = 0;
		dcb.fOutxCtsFlow = 0;

		if (SetCommState(_handle, &dcb) == FALSE) {
			_close();
			return false;
		}


		if (SetCommMask(_handle, 0) == FALSE ||
			SetupComm(_handle, 4096, 4096) == FALSE ||
			PurgeComm(_handle, PURGE_TXABORT | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_RXCLEAR) == FALSE ||
			ClearCommError(_handle, &dwError, NULL) == FALSE ||
			GetCommTimeouts(_handle, &timeouts) == FALSE
			) {

			_close();
			return false;
		}

		timeouts.ReadIntervalTimeout = 0;
		timeouts.ReadTotalTimeoutMultiplier = 0;
		timeouts.ReadTotalTimeoutConstant = 1; // must not be zero.
		timeouts.WriteTotalTimeoutMultiplier = 0;
		timeouts.WriteTotalTimeoutConstant = 0;
		if (SetCommTimeouts(_handle, &timeouts) == FALSE) {
			_close();
			return false;
		}

		return true;
	}



	////////////////////////////////////////////////////////////////////////////////////////////
	// Method   : _close
	// Input   : None
	// Output   : Result(bool)
	// Summury   : Close port handler.
	////////////////////////////////////////////////////////////////////////////////////////////
	bool _close() {

		if (_handle != INVALID_HANDLE_VALUE)
		{
			CloseHandle(_handle);
			_handle = INVALID_HANDLE_VALUE;

			return true;
		}

		return false;
	}


	// Member ////////////////////////////////////////////////////////
public:

protected:

private:
	double _timeout;
	int _bufSize;
	bool _isOpen;

	HANDLE _handle;
	char _portName[100];
	int _baudrate;
};




CComm::CComm()
{
	_poHandler = new _CPortHandler;
}



CComm::~CComm()
{
	delete _poHandler;
}



bool CComm::Open(const char* port, int baudrate) {

	std::cout << "open uart\n";
	return _poHandler->Open(port, baudrate);
}



bool CComm::Close() {

	return _poHandler->Close();
}



bool CComm::isOpen() {

	return _poHandler->isOpen();
}



int CComm::Write(const char* buf, int length) {

	return _poHandler->Write(buf, length);
}



int CComm::Read(char* buf, int length) {

	return _poHandler->Read(buf, length);
}



void CComm::SetBufSize(int size) {

	return _poHandler->SetBufSize(size);
}



void CComm::SetTimeOut(double t) {

	return _poHandler->SetTimeOut(t);
}



// openpose 시작
CComm uart;

void getAngleAndSend() {
	angle1 = acos(downLength / downLength_Max);

	
	//double dz = downLength * tan(angle1);
	//double a = sqrt(pow(dx2, 2) + pow(dz, 2));

	angle2 = asin((f_elbow.x - wrist.x) / downLength_Max);

	if ((elbow.y - wrist.y) > 0) angle1 = 3.14 - angle1; // 1.5708은 90도

	std::string angle = std::to_string((int)(angle1 * (180.0 / 3.14))) + "," + std::to_string((int)(angle2 * (180.0 / 3.14)));

	op::opLog("\n" + angle);

	angle = "s" + angle + "e";

	char Angle[100];
	strcpy(Angle, angle.c_str());

	if (uart.isOpen()) {
		op::opLog("connected!");
		uart.Write(Angle, angle.length());
	}
	else op::opLog("not connected :(");
}

void length() {
	upLength = sqrt(pow(dx1, 2) + pow(dy1, 2));
	if (upLength_Max < upLength) upLength_Max = upLength;
	downLength = sqrt(pow(dx2, 2) + pow(dy2, 2));
	if (downLength_Max < downLength) downLength_Max = downLength;

	op::opLog("\nLengthU_Max = " + std::to_string(upLength_Max));
	op::opLog("LengthD = " + std::to_string(downLength));

	getAngleAndSend();
}


// This worker will just read and return all the jpg files in a directory
class UserOutputClass
{
public:
	bool display(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr);
	void printKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr);
};

bool UserOutputClass::display(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
	// User's displaying/saving/other processing here
	   // datumPtr->cvOutputData: rendered frame with pose or heatmaps
	   // datumPtr->poseKeypoints: Array<float> with the estimated pose
	if (datumsPtr != nullptr && !datumsPtr->empty())
	{
		// Display image and sleeps at least 1 ms (it usually sleeps ~5-10 msec to display the image)
		const cv::Mat cvMat = OP_OP2CVCONSTMAT(datumsPtr->at(0)->cvOutputData);
		if (!cvMat.empty())
			cv::imshow(OPEN_POSE_NAME_AND_VERSION + " - Tutorial C++ API", cvMat);
		else
			op::opLog("Empty cv::Mat as output.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
	}
	else
		op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
	const auto key = (char)cv::waitKey(1);
	system("cls");
	return (key == 27);
}

void UserOutputClass::printKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr) // 좌표값 받아오는 부분 다시 보기
{
	int detecting[50] = { 0, }; // 감지된 신체 부위 수
	int b_detect = 0;
	int main_person = 0;
	// Example: How to use the pose keypoints
	if (datumsPtr != nullptr && !datumsPtr->empty())
	{
		op::opLog("\nKeypoints:");
		// Accesing each element of the keypoints
		const auto& poseKeypoints = datumsPtr->at(0)->poseKeypoints;
		op::opLog("Person pose keypoints:");
		for (auto person = 0; person < poseKeypoints.getSize(0); person++)
		{
			//op::opLog("Person " + std::to_string(person) + " (x, y, score):");

			for (auto bodyPart = 0; bodyPart < poseKeypoints.getSize(1); bodyPart++)
			{
				for (auto xyscore = 0; xyscore < poseKeypoints.getSize(2); xyscore++)
				{
					if ((0 < poseKeypoints[{person, xyscore, 0}]) && (0 < poseKeypoints[{person, xyscore, 1}]))
						detecting[person]++;
				}
			}

			if (b_detect < detecting[person]) {
				b_detect = detecting[person];
				main_person = person;
			}
		}

		if (poseKeypoints.getSize(0) > 0) { // 사람 수가 0명 이상

			shoulderX.push_back(poseKeypoints[{main_person, 2, 0}]);
			shoulderY.push_back(poseKeypoints[{main_person, 2, 1}]);
			elbowX.push_back(poseKeypoints[{main_person, 3, 0}]);
			elbowY.push_back(poseKeypoints[{main_person, 3, 1}]);
			wristX.push_back(poseKeypoints[{main_person, 4, 0}]);
			wristY.push_back(poseKeypoints[{main_person, 4, 1}]);

			filter++;

			// 필터
			if (filter > 5) {
				filter = 0;
				// 크기순 정렬
				sort(shoulderX.begin(), shoulderX.end());
				sort(shoulderY.begin(), shoulderY.end());
				sort(elbowX.begin(), elbowX.end());
				sort(elbowY.begin(), elbowY.end());
				sort(wristX.begin(), wristX.end());
				sort(wristY.begin(), wristY.end());
				// 중간값 담기
				if ((abs(shoulder.x - shoulderX.at(2)) < 10) && (abs(shoulder.y - shoulderY.at(2)) < 10)) shoulder_count++;
				if (shoulder_count > 4) {
					f_shoulder.x = shoulder.x;
					f_shoulder.y = shoulder.y;
				}
				else {
					shoulder.x = shoulderX.at(2);
					shoulder.y = shoulderY.at(2);
					//shoulder_count = 0;
				}

				if ((abs(elbow.x - elbowX.at(2)) < 20) && (abs(elbow.y - elbowY.at(2)) < 20)) elbow_count++;
				if (elbow_count > 4) {
					f_elbow.x = elbow.x;
					f_elbow.y = elbow.y;
				}
				else {
					elbow.x = elbowX.at(2);
					elbow.y = elbowY.at(2);
				}

				wrist.x = wristX.at(2);
				wrist.y = wristY.at(2);
				// vector 비우기
				shoulderX.clear();
				shoulderY.clear();
				elbowX.clear();
				elbowY.clear();
				wristX.clear();
				wristY.clear();
			}

			std::string shoulderLeft = std::to_string(shoulder.x) + " " + std::to_string(shoulder.y);
			std::string elbowLeft = std::to_string(elbow.x) + " " + std::to_string(elbow.y);
			std::string wristLeft = std::to_string(wrist.x) + " " + std::to_string(wrist.y);

			op::opLog("shoulder : " + shoulderLeft, op::Priority::High);
			op::opLog("elbow    : " + elbowLeft, op::Priority::High);
			op::opLog("wrist    : " + wristLeft, op::Priority::High);

			dx1 = f_shoulder.x - f_elbow.x;
			dy1 = f_shoulder.y - f_elbow.y;
			dx2 = f_elbow.x - wrist.x;
			dy2 = f_elbow.y - wrist.y;

			if ((f_shoulder.x > 0) && (f_shoulder.y > 0) && (f_elbow.x > 0) && (f_shoulder.y > 0)) length(); // 팔길이 구하기
			else std::cout << "움직이지 마세요." << std::endl;
		}
		else {
			op::opLog("no detect!", op::Priority::High);
			f_shoulder.x = 0; // 초기화
			f_elbow.x = 0;
			f_shoulder.y = 0;
			f_elbow.y = 0;
			upLength_Max = 0;
			downLength_Max = 0;
			angle1 = 0;
			angle2 = 0;
			shoulder_count = 0;
			elbow_count = 0;
			uart.Write("s0,0e", 5);
		}

		op::opLog(" ");
	}
	else
		op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
}

void configureWrapper(op::Wrapper& opWrapper) // 환경 설정
{
	try
	{
		// Configuring OpenPose

		// logging_level
		op::checkBool(
			0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.",
			__LINE__, __FUNCTION__, __FILE__);
		op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
		op::Profiler::setDefaultX(FLAGS_profile_speed);

		// Applying user defined configuration - GFlags to program variables
		// producerType=
		op::ProducerType producerType;
		op::String producerString;
		std::tie(producerType, producerString) = op::flagsToProducer(
			op::String(FLAGS_image_dir), op::String(FLAGS_video), op::String(FLAGS_ip_camera), FLAGS_camera,
			FLAGS_flir_camera, FLAGS_flir_camera_index);
		// cameraSize
		const auto cameraSize = op::flagsToPoint(op::String(FLAGS_camera_resolution), "-1x-1");
		// outputSize
		const auto outputSize = op::flagsToPoint(op::String(FLAGS_output_resolution), "-1x-1");
		// netInputSize
		const auto netInputSize = op::flagsToPoint(op::String(FLAGS_net_resolution), "-1x368");
		// faceNetInputSize
		const auto faceNetInputSize = op::flagsToPoint(op::String(FLAGS_face_net_resolution), "368x368 (multiples of 16)");
		// handNetInputSize
		const auto handNetInputSize = op::flagsToPoint(op::String(FLAGS_hand_net_resolution), "368x368 (multiples of 16)");
		// poseMode
		const auto poseMode = op::flagsToPoseMode(FLAGS_body);
		// poseModel
		const auto poseModel = op::flagsToPoseModel(op::String(FLAGS_model_pose));
		// JSON saving
		if (!FLAGS_write_keypoint.empty())
			op::opLog(
				"Flag `write_keypoint` is deprecated and will eventually be removed. Please, use `write_json`"
				" instead.", op::Priority::Max);
		// keypointScaleMode
		const auto keypointScaleMode = op::flagsToScaleMode(FLAGS_keypoint_scale);
		// heatmaps to add
		const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts, FLAGS_heatmaps_add_bkg,
			FLAGS_heatmaps_add_PAFs);
		const auto heatMapScaleMode = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);
		// >1 camera view?
		const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1 || FLAGS_flir_camera);
		// Face and hand detectors
		const auto faceDetector = op::flagsToDetector(FLAGS_face_detector);
		const auto handDetector = op::flagsToDetector(FLAGS_hand_detector);
		// Enabling Google Logging
		const bool enableGoogleLogging = true;

		// Pose configuration (use WrapperStructPose{} for default and recommended configuration)
		const op::WrapperStructPose wrapperStructPose{
		   poseMode, netInputSize, outputSize, keypointScaleMode, FLAGS_num_gpu, FLAGS_num_gpu_start,
		   FLAGS_scale_number, (float)FLAGS_scale_gap, op::flagsToRenderMode(FLAGS_render_pose, multipleView),
		   poseModel, !FLAGS_disable_blending, (float)FLAGS_alpha_pose, (float)FLAGS_alpha_heatmap,
		   FLAGS_part_to_show, op::String(FLAGS_model_folder), heatMapTypes, heatMapScaleMode, FLAGS_part_candidates,
		   (float)FLAGS_render_threshold, FLAGS_number_people_max, FLAGS_maximize_positives, FLAGS_fps_max,
		   op::String(FLAGS_prototxt_path), op::String(FLAGS_caffemodel_path),
		   (float)FLAGS_upsampling_ratio, enableGoogleLogging };
		opWrapper.configure(wrapperStructPose);
		// Face configuration (use op::WrapperStructFace{} to disable it)
		const op::WrapperStructFace wrapperStructFace{
		   FLAGS_face, faceDetector, faceNetInputSize,
		   op::flagsToRenderMode(FLAGS_face_render, multipleView, FLAGS_render_pose),
		   (float)FLAGS_face_alpha_pose, (float)FLAGS_face_alpha_heatmap, (float)FLAGS_face_render_threshold };
		opWrapper.configure(wrapperStructFace);
		// Hand configuration (use op::WrapperStructHand{} to disable it)
		const op::WrapperStructHand wrapperStructHand{
		   FLAGS_hand, handDetector, handNetInputSize, FLAGS_hand_scale_number, (float)FLAGS_hand_scale_range,
		   op::flagsToRenderMode(FLAGS_hand_render, multipleView, FLAGS_render_pose), (float)FLAGS_hand_alpha_pose,
		   (float)FLAGS_hand_alpha_heatmap, (float)FLAGS_hand_render_threshold };
		opWrapper.configure(wrapperStructHand);
		// Extra functionality configuration (use op::WrapperStructExtra{} to disable it)
		const op::WrapperStructExtra wrapperStructExtra{
		   FLAGS_3d, FLAGS_3d_min_views, FLAGS_identification, FLAGS_tracking, FLAGS_ik_threads };
		opWrapper.configure(wrapperStructExtra);
		// Producer (use default to disable any input)
		const op::WrapperStructInput wrapperStructInput{
		   producerType, producerString, FLAGS_frame_first, FLAGS_frame_step, FLAGS_frame_last,
		   FLAGS_process_real_time, FLAGS_frame_flip, FLAGS_frame_rotate, FLAGS_frames_repeat,
		   cameraSize, op::String(FLAGS_camera_parameter_path), FLAGS_frame_undistort, FLAGS_3d_views };
		opWrapper.configure(wrapperStructInput);
		// Output (comment or use default argument to disable any output)
		const op::WrapperStructOutput wrapperStructOutput{
		   FLAGS_cli_verbose, op::String(FLAGS_write_keypoint), op::stringToDataFormat(FLAGS_write_keypoint_format),
		   op::String(FLAGS_write_json), op::String(FLAGS_write_coco_json), FLAGS_write_coco_json_variants,
		   FLAGS_write_coco_json_variant, op::String(FLAGS_write_images), op::String(FLAGS_write_images_format),
		   op::String(FLAGS_write_video), FLAGS_write_video_fps, FLAGS_write_video_with_audio,
		   op::String(FLAGS_write_heatmaps), op::String(FLAGS_write_heatmaps_format), op::String(FLAGS_write_video_3d),
		   op::String(FLAGS_write_video_adam), op::String(FLAGS_write_bvh), op::String(FLAGS_udp_host),
		   op::String(FLAGS_udp_port) };
		opWrapper.configure(wrapperStructOutput);
		// No GUI. Equivalent to: opWrapper.configure(op::WrapperStructGui{});
		// Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
		if (FLAGS_disable_multi_thread)
			opWrapper.disableMultiThreading();
	}
	catch (const std::exception& e)
	{
		op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
	}
} 


int tutorialApiCpp()
{
	try
	{
		op::opLog("Starting OpenPose demo...", op::Priority::High);
		const auto opTimer = op::getTimerInit();

		// Configuring OpenPose
		op::opLog("Configuring OpenPose...", op::Priority::High);
		op::Wrapper opWrapper{ op::ThreadManagerMode::AsynchronousOut };
		configureWrapper(opWrapper);

		// Start, run, and stop processing - exec() blocks this thread until OpenPose wrapper has finished
		op::opLog("Starting thread(s)...", op::Priority::High);
		opWrapper.start();

		// User processing
		UserOutputClass userOutputClass;
		bool userWantsToExit = false;
		while (!userWantsToExit)
		{
			// Pop frame
			std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> datumProcessed;
			if (opWrapper.waitAndPop(datumProcessed))
			{
				if (!FLAGS_no_display)
					userWantsToExit = userOutputClass.display(datumProcessed);;
				userOutputClass.printKeypoints(datumProcessed);
			}
			// If OpenPose finished reading images
			else if (!opWrapper.isRunning())
				break;
			// Something else happened
			else
				op::opLog("Processed datum could not be emplaced.", op::Priority::High);
		}

		op::opLog("Stopping thread(s)", op::Priority::High);
		opWrapper.stop();

		// Measuring total time
		op::printTime(opTimer, "OpenPose demo successfully finished. Total time: ", " seconds.", op::Priority::High);

		// Return
		return 0;
	}
	catch (const std::exception&)
	{
		return -1;
	}
}

int main(int argc, char *argv[])
{
	// Parsing command line flags
	gflags::ParseCommandLineFlags(&argc, &argv, true);

	printf_s(" \x1b[31m%s\x1b[0m ", uart.Open("COM6", 115200) ? "OPEN" : "FALL");

	// Running tutorialApiCpp
	return tutorialApiCpp();
}