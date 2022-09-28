//
// Copyright 2022 Ting Cao <cao_ting@yahoo.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and

///
/// FLIR camera driver based on Spinnaker SDK
/// @remark The current implementation covers only GigEV camera
///
/// @Thanks Maxar. This code is done in Maxar time (ting.cao@maxar.com)
///

#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

#include <chrono>
#include <numeric>
#include <iomanip>
#include <memory>
#include <vector>
#include <string>

#include "spinnaker_driver/spinnaker_driver.hpp"

// TODO(tcao): frame rate adjust: https://github.com/ros-drivers/flir_camera_driver/blob/noetic-devel/spinnaker_camera_driver/src/cm3.cpp
namespace spinnaker_driver
{
#pragma region namespace spinnaker_driver utility
using namespace std::chrono_literals;
// TODO(tcao): This affects the streaming frame rate which can't be fast than kRunningThreadWakeup
static const std::chrono::milliseconds kRunningThreadWakeup(10);

/**
 * @brief Get the dotted address in text form from an integer
 *
 * @param value IP address in integer
 * @return std::string
 */
std::string get_dotted_address(int64_t value)
{
  // Helper function for formatting IP Address into the following format
  // x.x.x.x
  unsigned int inputValue = static_cast<unsigned int>(value);
  std::ostringstream convertValue;
  convertValue << ((inputValue & 0xFF000000) >> 24);
  convertValue << ".";
  convertValue << ((inputValue & 0x00FF0000) >> 16);
  convertValue << ".";
  convertValue << ((inputValue & 0x0000FF00) >> 8);
  convertValue << ".";
  convertValue << (inputValue & 0x000000FF);
  return convertValue.str().c_str();
}
#pragma endregion

#pragma region SpinnakerDriverImplementation internal class
AcquiredImage acquiredImage;
class SpinnakerDriverImplementation : public Spinnaker::ImageEventHandler
{
private:
  SpinnakerDriverImplementation()
  : cameraInstance_(nullptr),
    released_(false),
    acquiredImageCallback_(nullptr)
  {
    // Retrieve singleton reference to system object
    SpinnakerDriverImplementation::systemInstance_ = Spinnaker::System::GetInstance();
  }

public:
  /**
   * @brief Destroy the Spinnaker Driver Implementation object
   * @remark Destructor of this singleton may not be called by C++ runtime
   */
  virtual ~SpinnakerDriverImplementation()
  {
    std::cout << "SpinnakerDriverImplementation::~SpinnakerDriverImplementation()" << std::endl;
    release();
  }

  /**
   * @brief C++ runtime may or may not to call ~SpinnakerDriverImplementation
   *    So call release explicitly
   */
  void release()
  {
    std::cout << "SpinnakerDriverImplementation::release()" << std::endl;

    if (!released_ && SpinnakerDriverImplementation::systemInstance_) {
      //
      // Release system
      //
      // *** NOTES ***
      // The system should be released, but if it is not, it will do so itself.
      // It is often at the release of the system (whether manual or automatic)
      // that unreleased resources and still registered events will throw an
      // exception.
      //
      try {
        cameraInstance_ = nullptr;
        systemInstance_->ReleaseInstance();
        released_ = true;
        // @remark: set systemInstance_ to nullptr will kick off a new instantiation
      } catch (const Spinnaker::Exception & se) {
        std::cerr << "SpinnakerDriverImplementation::release exception: " << se.what() << '\n';
      }
    }
  }
  /**
   * @brief Interface ImageEventHandler's main function
   *
   * @param image SDK provied Image object and will be released by SDK after the event
   *  (it is a blocking camera buffer)
   * @remark image is reset after this call by SDK, so don't hold on it
   * @remark It is the callback acquiredImageCallback_ responsibility
   */
  void OnImageEvent(Spinnaker::ImagePtr image) override
  {
    if (nullptr != acquiredImageCallback_) {
      auto now = std::chrono::system_clock::now();
      acquiredImage.timestamp_ = now.time_since_epoch().count();
      if (!image->IsIncomplete()) {
        acquiredImage.width_ = image->GetWidth();
        acquiredImage.height_ = image->GetHeight();
        acquiredImage.x_padding_ = image->GetXPadding();
        acquiredImage.y_padding_ = image->GetYPadding();
        acquiredImage.stride_ = image->GetStride();
        acquiredImage.bits_per_pixel_ = image->GetBitsPerPixel();
        acquiredImage.channels_ = image->GetNumChannels();

        switch (image->GetPixelFormat()) {
          case Spinnaker::PixelFormatEnums::PixelFormat_Mono8:
            acquiredImage.pixel_format_ = PIXEL_FORMAT_MONO8;
            break;

          case Spinnaker::PixelFormatEnums::PixelFormat_BayerGB8:
            // PixelFormat_BayerGB8 is a 8-bit green/blue packed pixel format
            acquiredImage.pixel_format_ = PIXEL_FORMAT_GB8;
            break;

          case Spinnaker::PixelFormatEnums::PixelFormat_BayerRG8:
            // PixelFormat_BayerRG8 is a 8-bit red/green packed pixel format
            acquiredImage.pixel_format_ = PIXEL_FORMAT_RG8;
            break;

          case Spinnaker::PixelFormatEnums::PixelFormat_BGR8:
            acquiredImage.pixel_format_ = PIXEL_FORMAT_BGR8;
            break;

          default:
            // Convert other format to one of the above, for example:
            // ImagePtr converted = image->Convert(PixelFormat_RGB8)
            acquiredImage.pixel_format_ = PIXEL_FORMAT_INVALID;
            std::cerr << "Unsupported pixel format: " << image->GetPixelFormatName() << std::endl;
            break;
        }
        acquiredImage.data_ =
          (acquiredImage.pixel_format_ != PIXEL_FORMAT_INVALID) ? image->GetData() : nullptr;
      } else {
        // AcquiredImage.data_ nullptr is used as an indicator of incomplete issue
        acquiredImage.data_ = nullptr;
      }
      acquiredImageCallback_(&acquiredImage);
    }
  }

  /**
   * @brief Register callback for ImageEventHandler interface OnImageEvent
   *
   * @param callback
   * @remark Set it before camera start, reset it after camera stop
   */
  void registerCallback(AcquiredImageCallback callback)
  {
    acquiredImageCallback_ = callback;
  }

public:
  static std::shared_ptr<SpinnakerDriverImplementation> getInstance()
  {
    if (!driverImplementationInstance_) {
      // @remark: std::make_shared doesn't work when constructor is protected or private
      driverImplementationInstance_ = std::shared_ptr<SpinnakerDriverImplementation>(
        new SpinnakerDriverImplementation());
    }
    return driverImplementationInstance_;
  }

  /**
   * @brief Get the System Instance object
   *
   * @return Spinnaker::SystemPtr
   */
  Spinnaker::SystemPtr getSystemInstance()
  {
    return systemInstance_;
  }

  /**
   * @brief Get the Camera Instance object
   *
   * @return Spinnaker::CameraPtr
   * @remark Instance valid only after
   *  higher level driver connect is successful
   */
  Spinnaker::CameraPtr getCameraInstance()
  {
    return cameraInstance_;
  }

  /**
   * @brief Set the Camera Instance object
   *
   * @param camera
   */
  void setCameraInstance(Spinnaker::CameraPtr camera)
  {
    cameraInstance_ = camera;
  }

private:
  Spinnaker::SystemPtr systemInstance_;
  Spinnaker::CameraPtr cameraInstance_;
  bool released_;
  AcquiredImageCallback acquiredImageCallback_;
  static std::shared_ptr<SpinnakerDriverImplementation> driverImplementationInstance_;
};  // namespace spinnaker_driver_implement
#pragma endregion

#pragma region SpinnakerDriverImplementation static variables
std::shared_ptr<SpinnakerDriverImplementation>
SpinnakerDriverImplementation::driverImplementationInstance_ = nullptr;
#pragma endregion

#pragma region SpinnakerDriver implementation
SpinnakerDriver::SpinnakerDriver()
{
  SpinnakerDriverImplementation::getInstance();
}

SpinnakerDriver::~SpinnakerDriver()
{
  std::cout << "SpinnakerDriver::~SpinnakerDriver()" << std::endl;
}

SpinnakerDriverGigE::SpinnakerDriverGigE()
: cameraIndex_(0),
  acquisitionCallback_(nullptr),
  runningThread_(nullptr),
  running_(false)
{}

SpinnakerDriverGigE::~SpinnakerDriverGigE()
{
  std::cout << "SpinnakerDriverGigE::~SpinnakerDriverGigE()" << std::endl;
  if (SpinnakerDriverImplementation::getInstance()->getCameraInstance()) {
    try {
      SpinnakerDriverImplementation::getInstance()->getCameraInstance()->DeInit();
    } catch (const Spinnaker::Exception & se) {
      std::cerr << se.what() << std::endl;
    }
    if (runningThread_) {
      try {
        if (runningThread_->joinable()) {
          runningThread_->join();
        }
      } catch (const std::exception & e) {
        std::cerr << "spinnakerDriverGigE::~SpinnakerDriverGigE exception: " << e.what() <<
          std::endl;
      }
      runningThread_ = nullptr;
    }
  }
}

/**
 * @brief Connect the specified camera
 *
 * @param camera_index
 * @param camera_type
 * @param parameters - optional output parameters for CameraParameters
 * @return true when successful
 * @return false when fail
 * @remark camera instance is set when successful
 */
bool SpinnakerDriverGigE::connect(
  uint32_t camera_index, SupportedCameraType_e camera_type,
  CameraParameters * parameters)
{
  if (CAMERA_TYPE_GIGE != camera_type) {
    return false;
  }
  Spinnaker::SystemPtr system = SpinnakerDriverImplementation::getInstance()->getSystemInstance();
  if (nullptr == system) {
    return false;
  }
  Spinnaker::CameraList camera_list = system->GetCameras();
  unsigned int camera_count = camera_list.GetSize();
  if (camera_count <= camera_index) {
    std::cerr << "Camera index: " << camera_index << " is out of range of found camera(s): " <<
      camera_count << std::endl;
    return false;
  }
  Spinnaker::CameraPtr camera = camera_list.GetByIndex(camera_index);
  bool success = true;
  try {
    camera->Init();
    // Save the camera instance
    SpinnakerDriverImplementation::getInstance()->setCameraInstance(camera);

    // Fill current camera prime parameters
    if (nullptr != parameters) {
      retrieve_parameters(parameters);
    }
    // Camera is setup for further operation from other calls
    // so camera's DeInit will be called later
  } catch (const Spinnaker::Exception & se) {
    std::cerr << "Failed to initialize camera: " << se.what() << '\n';
    success = false;
  }

  camera_list.Clear();
  return success;
}

bool SpinnakerDriverGigE::start(AcquiredImageCallback acquisition_callback, float frame_rate)
{
  (void)frame_rate;   // May bring it back for convenience/flexibility

  if (Spinnaker::CameraPtr camera =
    SpinnakerDriverImplementation::getInstance()->getCameraInstance())
  {
    // Retrieve TL device nodemap; see NodeMapInfo example for
    // additional comments on transport layer nodemaps
    Spinnaker::GenApi::INodeMap & nodeMap = camera->GetNodeMap();

    //
    // Notice that both the enumeration and the entry nodes are checked for
    // availability and readability/writability. Enumeration nodes are
    // generally readable and writable whereas their entry nodes are only
    // ever readable.
    //

    // Retrieve enumeration node from nodemap
    Spinnaker::GenApi::CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
    if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode)) {
      std::cout << "Unable to set acquisition mode to continuous (enum retrieval). Aborting..." <<
        std::endl << std::endl;
      return false;
    }

    // Retrieve entry node from enumeration node
    Spinnaker::GenApi::CEnumEntryPtr ptrAcquisitionModeContinuous =
      ptrAcquisitionMode->GetEntryByName("Continuous");
    if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous)) {
      std::cout << "Unable to set acquisition mode to continuous (entry retrieval). Aborting..." <<
        std::endl << std::endl;
      return false;
    }

    // Retrieve integer value from entry node
    const int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();
    // Set integer value from entry node as new value of enumeration node
    ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

    // Register/unregister the event handler
    try {
      // @remark: Multiple event handlers can be registered for the same camera
      // @remark: Image event handlers must be unregistered manually. This must be done prior
      // to releasing the system and while the image event handlers are still in scope.
      camera->RegisterEventHandler(*SpinnakerDriverImplementation::getInstance());
    } catch (Spinnaker::Exception & e) {
      std::cerr << "Fail to register acquisition event: " << e.what() << std::endl;
      return false;
    }

    // Save the callback
    SpinnakerDriverImplementation::getInstance()->registerCallback(acquisition_callback);

    // Start running thread
    runningThread_ = std::make_shared<std::thread>(&SpinnakerDriverGigE::run, this);

    // Disable Heartbeat
    Spinnaker::GenApi::CBooleanPtr ptrDeviceHeartbeat = nodeMap.GetNode("GevGVCPHeartbeatDisable");
    if (IsAvailable(ptrDeviceHeartbeat) && IsWritable(ptrDeviceHeartbeat)) {
      // Disable heartbeat (true for GevGVCPHeartbeatDisable) when start,
      // and reneable it (false) when stop
      ptrDeviceHeartbeat->SetValue(true);
    } else {
      std::cout << "Unable to disable/enable heartbeat" << std::endl << std::endl;
      ptrDeviceHeartbeat = nullptr;
    }

    // Start acquistion
    camera->BeginAcquisition();

    // Spins till the running thread is terminated by stop operation
    if (runningThread_->joinable()) {
      // @remart start operation - execution path 1 - wait till thread terminates
      runningThread_->join();
      // @remark start operation - execution path 2 - thread terminates
      std::cout << "start (true) thread exits" << std::endl;
    }

    // Clean up
    runningThread_ = nullptr;
    // Stop acquistion
    camera->EndAcquisition();
    // Reset the callback
    SpinnakerDriverImplementation::getInstance()->registerCallback(nullptr);

    try {
      // @remark: It is important to unregister all image event handlers
      camera->UnregisterEventHandler(*SpinnakerDriverImplementation::getInstance());
    } catch (Spinnaker::Exception & e) {
      std::cerr << "Fail to unregister acquisition event: " << e.what() << std::endl;
      // don't exit such that it has chance to finish cleanup
    }

    // Renable Heartbeat
    if (nullptr != ptrDeviceHeartbeat) {
      ptrDeviceHeartbeat->SetValue(false);
      std::cout << "Heartbeat is reenabled" << std::endl;
    }

    std::cout << "camera->DeInit" << std::endl;
    camera->DeInit();

    std::cout << "start operation exits successfully" << std::endl;
  } else {
    std::cout << "Camera is not properly connected" << std::endl;
    return false;
  }

  return true;
}

bool SpinnakerDriverGigE::stop()
{
  running_ = false;
  return true;
}

bool SpinnakerDriverGigE::list(SupportedCameraType_e camera_type)
{
  // Support only CAMERA_TYPE_GIGE
  if (CAMERA_TYPE_GIGE != camera_type) {
    return false;
  }

  Spinnaker::SystemPtr system = SpinnakerDriverImplementation::getInstance()->getSystemInstance();
  if (nullptr == system) {
    return false;
  }

  Spinnaker::CameraList camera_list = system->GetCameras();
  unsigned int camera_count = camera_list.GetSize();
  Spinnaker::CameraPtr camera = nullptr;
  unsigned int gigev_camera_count = 0;
  std::cout << "Found " << camera_count << ((1 < camera_count) ? " cameras" : " camera") <<
    std::endl;
  if (0 != camera_count) {
    std::cout << "Use the following index to the GigEV camera list to connect/use" <<
      std::endl << std::endl;
  }
  for (unsigned int i = 0; i < camera_count; i++) {
    camera = camera_list.GetByIndex(i);
    // Retrieve TL device nodemap; see NodeMapInfo example for
    // additional comments on transport layer nodemaps
    Spinnaker::GenApi::INodeMap & nodeMapTLDevice = camera->GetTLDeviceNodeMap();

    // Skip if connected camera is GigEV
    Spinnaker::GenApi::CEnumerationPtr ptrDeviceType = nodeMapTLDevice.GetNode("DeviceType");
    if (Spinnaker::GenApi::IsAvailable(ptrDeviceType) &&
      Spinnaker::GenApi::IsReadable(ptrDeviceType))
    {
      if (Spinnaker::DeviceTypeEnum::DeviceType_GigEVision == ptrDeviceType->GetIntValue() ) {
        gigev_camera_count++;
      } else {
        std::cout << i << " : This is not a GigEV camera" << std::endl;
        continue;
      }
    } else {
      std::cout << i << " : DeviceType query not supported" << std::endl;
      continue;
    }

    // Retrieve camera vendor, model, S/N and IP
    //
    // *** NOTES ***
    // Grabbing node information requires first retrieving the node and
    // then retrieving its information. There are two things to keep in
    // mind. First, a node is distinguished by type, which is related
    // to its value's data type. Second, nodes should be checked for
    // availability and readability/writability prior to making an
    // attempt to read from or write to the node.
    //
    std::string vendor = "";
    Spinnaker::GenApi::CStringPtr ptrStringVendor = nodeMapTLDevice.GetNode("DeviceVendorName");
    if (Spinnaker::GenApi::IsAvailable(ptrStringVendor) &&
      Spinnaker::GenApi::IsReadable(ptrStringVendor))
    {
      vendor = ptrStringVendor->GetValue();
    }

    std::string model = "";
    Spinnaker::GenApi::CStringPtr ptrStringModel = nodeMapTLDevice.GetNode("DeviceModelName");
    if (Spinnaker::GenApi::IsAvailable(ptrStringModel) &&
      Spinnaker::GenApi::IsReadable(ptrStringModel))
    {
      model = ptrStringModel->GetValue();
    }

    std::string serial_number = "";
    Spinnaker::GenApi::CStringPtr ptrStringSerial = nodeMapTLDevice.GetNode("DeviceSerialNumber");
    if (Spinnaker::GenApi::IsAvailable(ptrStringSerial) &&
      Spinnaker::GenApi::IsReadable(ptrStringSerial))
    {
      serial_number = ptrStringSerial->GetValue();
    }

    std::string camera_ip = "";
    Spinnaker::GenApi::CIntegerPtr ptrIP = nodeMapTLDevice.GetNode("GevDeviceIPAddress");
    if (Spinnaker::GenApi::IsAvailable(ptrIP) &&
      Spinnaker::GenApi::IsReadable(ptrIP))
    {
      camera_ip = get_dotted_address(ptrIP->GetValue());
    }

    // Use the following camera index to connect
    std::cout << "GigEV Camera " << i << std::endl;

    std::cout << "Vendor: " << vendor << ", Model: " << model << ", S/N: " << serial_number <<
      ", IP: " << camera_ip << std::endl;

    // The following code applies only when camera is initialized
    try {
      // Initialize Camera
      camera->Init();
      // Retrieve device nodemap
      Spinnaker::GenApi::INodeMap & nodeMapDevice = camera->GetNodeMap();

      Spinnaker::GenApi::CIntegerPtr ptrCurrentIPAddress =
        nodeMapDevice.GetNode("GevPrimaryApplicationIPAddress");
      if (Spinnaker::GenApi::IsAvailable(ptrCurrentIPAddress) &&
        Spinnaker::GenApi::IsReadable(ptrCurrentIPAddress))
      {
        // This is important information,
        // and the only place to tell if the camera is in proper GigE ethernet adapter
        std::cout << "Host IP: " << get_dotted_address(ptrCurrentIPAddress->GetValue()) <<
          std::endl;
      }

      // Temporarily set camera instance, such that retrieve_parameters can use
      SpinnakerDriverImplementation::getInstance()->setCameraInstance(camera);
      CameraParameters parameters;
      if (retrieve_parameters(&parameters, true)) {
        std::cout << parameters.pixel_format << std::endl;
        // Skip std::endl
        std::cout << parameters.video_mode;

        std::cout << "Current, Maximum resolution: " <<
          parameters.image_dimension.width << "/" <<
          parameters.image_dimension.height << ", " <<
          parameters.max_image_dimension.width << "/" <<
          parameters.max_image_dimension.height << std::endl;

        std::cout << "Current/Max Frame rate: " << parameters.frame_rate << "/" <<
          parameters.max_frame_rate << std::endl;
      }
      std::cout << std::endl;
      SpinnakerDriverImplementation::getInstance()->setCameraInstance(nullptr);

      // Unmount camera since no other operation is expected
      camera->DeInit();
    } catch (const Spinnaker::Exception & se) {
      std::cerr << se.what() << std::endl;
    }
  }

  camera_list.Clear();

  return true;
}

bool SpinnakerDriverGigE::set_videomode(uint32_t camera_index, uint32_t video_mode)
{
  Spinnaker::SystemPtr system = SpinnakerDriverImplementation::getInstance()->getSystemInstance();
  if (nullptr == system) {
    return false;
  }

  Spinnaker::CameraList camera_list = system->GetCameras();
  unsigned int camera_count = camera_list.GetSize();
  if (camera_count <= camera_index) {
    std::cerr << "Camera index: " << camera_index << " is out of range of found camera(s): " <<
      camera_count << std::endl;
    return false;
  }

  Spinnaker::CameraPtr camera = camera_list.GetByIndex(camera_index);
  bool success = true;
  try {
    camera->Init();

    Spinnaker::GenApi::INodeMap & nodeMapDevice = camera->GetNodeMap();
    // Video Mode
    Spinnaker::GenApi::CEnumerationPtr ptrVideoMode = nodeMapDevice.GetNode("VideoMode");
    if (IsAvailable(ptrVideoMode) && IsWritable(ptrVideoMode)) {
      std::cout << "Current " << ptrVideoMode->GetDisplayName() << ": " <<
        ptrVideoMode->ToString() << " (" << ptrVideoMode->GetIntValue() << ")" << std::endl;
    } else {
      std::cerr << "Failed to retrieve Video Mode node in write mode" << std::endl;
      success = false;
    }

    if (success) {
      try {
        // Exception will throw if video_mode is invalid, or system doesn't take it
        ptrVideoMode->SetIntValue(video_mode);
      } catch (const Spinnaker::Exception & se) {
        std::cerr << "Failed to set Video Mode: " << se.what() << std::endl;
        success = false;
      }
      // Verify if the video mode is successfully set
      if (ptrVideoMode->GetIntValue() != static_cast<int64_t>(video_mode)) {
        std::cerr << "Video Mode is not set properly, sure about mode = " << video_mode <<
          " ? " << std::endl;
        success = false;
      }

      // Temporarily set camera instance, such that retrieve_parameters can use
      SpinnakerDriverImplementation::getInstance()->setCameraInstance(camera);
      // Used to show camera parameters after video mode is set
      // When it fails to set, show camera info in details
      CameraParameters parameters;
      if (retrieve_parameters(&parameters, !success)) {
        if (success) {
          std::cout << parameters.pixel_format << std::endl;
          std::cout << parameters.video_mode << std::endl;

          std::cout << "Current, Maximum resolution: " <<
            parameters.image_dimension.width << "/" <<
            parameters.image_dimension.height << ", " <<
            parameters.max_image_dimension.width << "/" <<
            parameters.max_image_dimension.height << std::endl;

          std::cout << "Current/Max Frame rate: " << parameters.frame_rate << "/" <<
            parameters.max_frame_rate << std::endl;
        } else {
          // Only show video mode details which should contain supported modes
          // Skip std::endl
          std::cerr << parameters.video_mode;
        }
      }
      SpinnakerDriverImplementation::getInstance()->setCameraInstance(nullptr);
    }
    std::cout << std::endl;

    // Unmount camera since no other operation is expected
    camera->DeInit();
  } catch (const Spinnaker::Exception & se) {
    std::cerr << "Failed to initialize camera: " << se.what() << std::endl;
    success = false;
  }

  camera_list.Clear();
  return success;
}

bool SpinnakerDriverGigE::set_configurable_parameters(
  uint32_t camera_index, std::vector<ConfigurableParameter> & parameters)
{
  Spinnaker::SystemPtr system = SpinnakerDriverImplementation::getInstance()->getSystemInstance();
  if (nullptr == system) {
    return false;
  }

  /* default parameter's index to invalid */
  int32_t video_mode_index = -1;
  int32_t frame_rate_index = -1;
  int32_t width_index = -1;
  int32_t height_index = -1;

  // Set index to desired parameters
  for (int i = 0; i < static_cast<int>(parameters.size()); i++) {
    // ConfigurableParameter * parameter = &parameter[i];
    switch (parameters[i].type) {
      case CONFIG_VIDEO_MODE:
        video_mode_index = i;
        break;

      case CONFIG_FRAME_RATE:
        frame_rate_index = i;
        break;

      case CONFIG_IMAGE_WIDTH:
        width_index = i;
        break;

      case CONFIG_IMAGE_HEIGHT:
        height_index = i;
        break;

      default:
        std::cerr << "Unknown configurable parameter type: " << parameters[i].type << std::endl;
        break;
    }
  }

  Spinnaker::CameraList camera_list = system->GetCameras();
  unsigned int camera_count = camera_list.GetSize();
  if (camera_count <= camera_index) {
    std::cerr << "Camera index: " << camera_index << " is out of range of found camera(s): " <<
      camera_count << std::endl;
    return false;
  }

  Spinnaker::CameraPtr camera = camera_list.GetByIndex(camera_index);
  bool success = true;
  try {
    camera->Init();

    Spinnaker::GenApi::INodeMap & nodeMap = camera->GetNodeMap();

    // Configure video mode
    if (0 <= video_mode_index) {
      bool good = true;
      Spinnaker::GenApi::CEnumerationPtr ptrVideoMode = nodeMap.GetNode("VideoMode");
      if (IsAvailable(ptrVideoMode) && IsWritable(ptrVideoMode)) {
        std::cout << "Current " << ptrVideoMode->GetDisplayName() << ": " <<
          ptrVideoMode->ToString() << " (" << ptrVideoMode->GetIntValue() << ")" << std::endl;
      } else {
        std::cerr << "Failed to retrieve Video Mode node in write mode" << std::endl;
        good = success = false;
      }
      if (good) {
        uint32_t video_mode = *(static_cast<uint32_t *>(parameters[video_mode_index].data));
        try {
          // Exception will throw if video_mode is invalid, or system doesn't take it
          ptrVideoMode->SetIntValue(video_mode);
        } catch (const Spinnaker::Exception & se) {
          std::cerr << "Failed to set Video Mode: " << se.what() << std::endl;
          good = success = false;
        }
        // Verify if the video mode is successfully set
        if (good && (ptrVideoMode->GetIntValue() != static_cast<int64_t>(video_mode))) {
          std::cerr << "Video Mode is not set properly, sure about mode = " << video_mode <<
            " ? " << std::endl;
          good = success = false;
        }
      }
    }

    // Configure frame rate
    if (0 <= frame_rate_index) {
      bool good = true;
      // enable frame rate control
      Spinnaker::GenApi::CBooleanPtr ptrFrameRateEnable = nodeMap.GetNode
        // @remark SDK (AcquisitionFrameRateEnable) and
        // actual code is out of sync (AcquisitionFrameRateEnabled)
        // ("AcquisitionFrameRateEnable");
          ("AcquisitionFrameRateEnabled");
      if (IsAvailable(ptrFrameRateEnable) && IsWritable(ptrFrameRateEnable)) {
        ptrFrameRateEnable->SetValue(true);

        Spinnaker::GenApi::CFloatPtr ptrFrameRate = nodeMap.GetNode("AcquisitionFrameRate");
        if (IsAvailable(ptrFrameRate) && IsWritable(ptrFrameRate)) {
          float frame_rate = *(static_cast<float *>(parameters[frame_rate_index].data));
          double max = ptrFrameRate->GetMax();
          double min = ptrFrameRate->GetMin();
          double desired = static_cast<double>(frame_rate);

          if ((min > desired) || (max < desired)) {
            if (min > desired) {
              std::cout << "Minimum frame rate: " << min <<
                " is used, instead of: " << desired << std::endl;
              desired = min;
            } else {
              std::cout << "Maximum frame rate: " << max <<
                " is used, instead of: " << desired << std::endl;
              desired = max;
            }
          }
          // Perform frame rate set
          good = true;
          try {
            ptrFrameRate->SetValue(desired);
            if (desired != ptrFrameRate->GetValue()) {
              std::cerr << "Failed to set frame rate: " << desired <<
                " instead use : " << ptrFrameRate->GetValue() << std::endl;
              good = success = false;
            }
          } catch (const Spinnaker::Exception & se) {
            std::cerr << "Failed to set Frame Rate: " << se.what() << std::endl;
            good = success = false;
          }

          if (good) {
            std::cout << "Frame rate is set to: " << ptrFrameRate->GetValue() <<
              " (min=" << min << ", max=" << max << ")" << std::endl;
          }
          // else error message should already be given out
        } else {
          std::cerr << "AcquisitionFrameRate is not available/writable" << std::endl;
          good = success = false;
        }
        // Configure is done here, success or not
      } else {
        std::cerr << "AcquisitionFrameRateEnable is not available/writable" << std::endl;
        good = success = false;
      }
    }

    // Configure image dimension
    if ((0 <= width_index) || (0 <= height_index)) {
      bool good = true;
      if (0 <= width_index) {
        success = true;
        Spinnaker::GenApi::CIntegerPtr ptrWidth = nodeMap.GetNode("Width");
        if (IsAvailable(ptrWidth) && IsWritable(ptrWidth)) {
          uint32_t max = static_cast<uint32_t>(ptrWidth->GetMax());
          uint32_t min = static_cast<uint32_t>(ptrWidth->GetMin());
          uint32_t width = *(static_cast<uint32_t *>(parameters[width_index].data));
          if ((min > width) || (max < width)) {
            if (min > width) {
              std::cout << "Minimum width " << min <<
                " is used, instead of: " << width << std::endl;
              width = min;
            }
            if (max < width) {
              std::cout << "Maximum width " << max <<
                " is used, instead of: " << width << std::endl;
              width = max;
            }
          }
          // Perform fwidth set
          good = true;
          try {
            ptrWidth->SetValue(width);
          } catch (const Spinnaker::Exception & se) {
            std::cerr << "Failed to set width: " << se.what() << std::endl;
            good = success = false;
          }
          // Verify if the width is successfully set
          if (good && (ptrWidth->GetValue() != static_cast<int64_t>(width))) {
            std::cerr << "Width is not set properly, sure about width = " << width <<
              " ? " << std::endl;
            good = success = false;
          }
        } else {
          std::cerr << "Width is not available/writable" << std::endl;
          good = success = false;
        }
      }

      if (0 <= height_index) {
        good = true;
        Spinnaker::GenApi::CIntegerPtr ptrHeight = nodeMap.GetNode("Height");
        if (IsAvailable(ptrHeight) && IsWritable(ptrHeight)) {
          uint32_t max = static_cast<uint32_t>(ptrHeight->GetMax());
          uint32_t min = static_cast<uint32_t>(ptrHeight->GetMin());
          uint32_t height = *(static_cast<uint32_t *>(parameters[height_index].data));
          if ((min > height) || (max < height)) {
            if (min > height) {
              std::cout << "Minimum height " << min <<
                " is used, instead of: " << height << std::endl;
              height = min;
            }
            if (max < height) {
              std::cout << "Maximum height " << max <<
                " is used, instead of: " << height << std::endl;
              height = max;
            }
          }
          // Perform height set
          good = true;
          try {
            ptrHeight->SetValue(height);
          } catch (const Spinnaker::Exception & se) {
            std::cerr << "Failed to set height: " << se.what() << std::endl;
            good = success = false;
          }
          // Verify if the height is successfully set
          if (good && (ptrHeight->GetValue() != static_cast<int64_t>(height))) {
            std::cerr << "Hidth is not set properly, sure about height = " << height <<
              " ? " << std::endl;
            good = success = false;
          }
        } else {
          std::cerr << "Height is not available/writable" << std::endl;
          good = success = false;
        }
      }
    }

    // Unmount camera since no other operation is expected
    camera->DeInit();
  } catch (const Spinnaker::Exception & se) {
    std::cerr << "Failed to initialize camera: " << se.what() << std::endl;
    success = false;
  }

  camera_list.Clear();
  return success;
}

void SpinnakerDriverGigE::release()
{
  std::cout << "SpinnakerDriverGigE::release()" << std::endl;
  try {
    auto driver = SpinnakerDriverImplementation::getInstance();
    driver->release();
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl << "use 0 for camera index" << std::endl;
  }
}

/** protected */
bool SpinnakerDriverGigE::retrieve_parameters(CameraParameters * parameters, bool extra_details)
{
  bool success = true;
  Spinnaker::CameraPtr camera = SpinnakerDriverImplementation::getInstance()->getCameraInstance();
  // Retrieve device nodemap
  Spinnaker::GenApi::INodeMap & nodeMapDevice = camera->GetNodeMap();
  // Pixel format
  Spinnaker::GenApi::CEnumerationPtr ptrPixelFormatMode = nodeMapDevice.GetNode("PixelFormat");
  if (IsAvailable(ptrPixelFormatMode) && IsReadable(ptrPixelFormatMode)) {
    parameters->pixel_format = ptrPixelFormatMode->GetDisplayName() + ": " +
      ptrPixelFormatMode->ToString();
  } else {
    success = false;
  }

  // Video Mode
  Spinnaker::GenApi::CEnumerationPtr ptrVideoMode = nodeMapDevice.GetNode("VideoMode");
  if (IsAvailable(ptrVideoMode) && IsReadable(ptrVideoMode)) {
    parameters->video_mode = ptrVideoMode->GetDisplayName() + ": " + ptrVideoMode->ToString();
    parameters->video_mode += " (" + std::to_string(ptrVideoMode->GetIntValue()) + ")";
    if (extra_details) {
      std::string details("\nSupported Video Mode:\n");
      // Add supported video modes
      Spinnaker::GenApi::StringList_t entries;
      ptrVideoMode->GetSymbolics(entries);
      for (size_t i = 0; i < entries.size(); i++) {
        Spinnaker::GenApi::CEnumEntryPtr selectorEntry = ptrVideoMode->GetEntryByName(entries[i]);
        details += "  " + entries[i] + " (" + selectorEntry->ToString() + ")\n";
      }
      parameters->video_mode += details;
    }
  } else {
    success = false;
  }

  // Max width/height
  Spinnaker::GenApi::CIntegerPtr ptrWidth = nodeMapDevice.GetNode("Width");
  if (IsAvailable(ptrWidth) && IsReadable(ptrWidth)) {
    int64_t width = ptrWidth->GetMax();
    int64_t height = 0;
    Spinnaker::GenApi::CIntegerPtr ptrHeight = nodeMapDevice.GetNode("Height");
    if (IsAvailable(ptrHeight) && IsReadable(ptrHeight)) {
      height = ptrHeight->GetMax();
    } else {
      success = false;
    }
    parameters->max_image_dimension.width = width;
    parameters->max_image_dimension.height = height;
    parameters->image_dimension.width = ptrWidth->GetValue();
    parameters->image_dimension.height = ptrHeight->GetValue();
  } else {
    success = false;
  }

  // Max frame rate
  Spinnaker::GenApi::CFloatPtr ptrFrameRate = nodeMapDevice.GetNode("AcquisitionFrameRate");
  if (IsAvailable(ptrFrameRate) && IsReadable(ptrFrameRate)) {
    parameters->frame_rate = static_cast<float>(ptrFrameRate->GetValue());
    parameters->max_frame_rate = static_cast<float>(ptrFrameRate->GetMax());
  } else {
    success = false;
  }

  return success;
}

/** protected */
void SpinnakerDriverGigE::run()
{
  uint32_t running_count = 0;
  running_ = true;
  while (running_) {
    std::this_thread::sleep_for(kRunningThreadWakeup);
    running_count++;
  }
}

#pragma endregion SpinnakerDriver implementation
}  // namespace spinnaker_driver
