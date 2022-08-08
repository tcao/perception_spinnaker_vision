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
#include <string>

#include "spinnaker_driver/spinnaker_driver.hpp"

// TODO(tcao): frame rate adjust: https://github.com/ros-drivers/flir_camera_driver/blob/noetic-devel/spinnaker_camera_driver/src/cm3.cpp
namespace spinnaker_driver
{
#pragma region namespace spinnaker_driver utility
using namespace std::chrono_literals;
static const std::chrono::milliseconds kRunningThreadWakeup(100);

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
   * @param image SDK provied Image object and will be released by SDK
   * @remark image is reset after this call by SDK
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
            acquiredImage.pixel_format_ = PIXEL_FORMAT_GB8;
            break;

          case Spinnaker::PixelFormatEnums::PixelFormat_BayerRG8:
            acquiredImage.pixel_format_ = PIXEL_FORMAT_RG8;
            break;

          default:
            acquiredImage.pixel_format_ = PIXEL_FORMAT_INVALID;
            break;
        }
        acquiredImage.data_ = image->GetData();
      } else {
        // AcquiredImage.data_ nullptr is used as an indicator of incomplete issue
        // acquiredImage.data_ = nullptr;
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
 * @return true when successful
 * @return false when fail
 * @remark camera instance is set when successful
 */
bool SpinnakerDriverGigE::connect(
  uint32_t camera_index, SupportedCameraType camera_type)
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
    std::cout << "Camera index: " << camera_index << " is out of range of found camera(s): " <<
      camera_count << std::endl;
    return false;
  }
  Spinnaker::CameraPtr camera = camera_list.GetByIndex(camera_index);
  bool success = true;
  try {
    camera->Init();
    // Save the camera instance
    SpinnakerDriverImplementation::getInstance()->setCameraInstance(camera);

    // camera's DeInit will be called from the destructor
  } catch (const Spinnaker::Exception & se) {
    std::cerr << "Failed to initialize camera: " << se.what() << '\n';
    success = false;
  }

  camera_list.Clear();
  return success;
}

bool SpinnakerDriverGigE::start(AcquiredImageCallback acquisition_callback, float frame_rate)
{
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
      std::cout << "Fail to register acquisition event: " << e.what() << std::endl;
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

    bool use_default_frame_rate = true;
    if (1.0f != frame_rate) {
      std::cout << "desired frame rate: " << frame_rate << std::endl;
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
          double max = ptrFrameRate->GetMax();
          double min = ptrFrameRate->GetMin();
          double desired = static_cast<double> frame_rate;
          if ( (min > desired) || (max < desired) ) {
            if (min > desired) {
              std::cout << "minimum frame rate: " << min <<
                " is used, instead of: " << desired << std::endl;
              desired = min;
            } else {
              std::cout << "maximum frame rate: " << max <<
                " is used, instead of: " << desired << std::endl;
              desired = max;
            }
          }
          ptrFrameRate->SetValue(desired);
          if (desired != ptrFrameRate->GetValue()) {
            std::cout << "Failed to set frame rate: " << desired <<
              " instead use : " << ptrFrameRate->GetValue() << std::endl;
          }
          std::cout << "Frame rate: " << ptrFrameRate->GetValue() <<
            " (" << min << "-" << max << ")" << std::endl;
          use_default_frame_rate = false;
        } else {
          std::cout << "AcquisitionFrameRate is not available/writable" << std::endl;
        }
      } else {
        std::cout << "AcquisitionFrameRateEnable is not available/writable" << std::endl;
      }
    }
    if (use_default_frame_rate) {
      std::cout << "Default frame rate is used" << std::endl;
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
      std::cout << "Fail to unregister acquisition event: " << e.what() << std::endl;
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

bool SpinnakerDriverGigE::list(SupportedCameraType camera_type)
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
          std::endl << std::endl;
      }
      camera->DeInit();
    } catch (const Spinnaker::Exception & se) {
      std::cerr << se.what() << std::endl;
    }
  }

  camera_list.Clear();

  return true;
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
