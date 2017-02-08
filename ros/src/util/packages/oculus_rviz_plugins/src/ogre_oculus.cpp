/// Copyright (C) 2013 Kojack
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), 
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
/// DEALINGS IN THE SOFTWARE.

#include "oculus_rviz_plugins/ogre_oculus.h"
#include "OGRE/OgreSceneManager.h"
#include "OGRE/OgreRenderWindow.h"
#include "OGRE/OgreCompositorManager.h"
#include "OGRE/OgreCompositorInstance.h"
#include "OGRE/OgreCompositionTargetPass.h"
#include "OGRE/OgreCompositionPass.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
using namespace OVR;

namespace
{
const float g_defaultNearClip = 0.01f;
const float g_defaultFarClip = 10000.0f;
const float g_defaultIPD = 0.064f;
const Ogre::ColourValue g_defaultViewportColour(97 / 255.0f, 97 / 255.0f, 200 / 255.0f);
const float g_defaultProjectionCentreOffset = 0.14529906f;
const float g_defaultDistortion[4] = {1.0f, 0.22f, 0.24f, 0.0f};
const float g_defaultChromAb[4] = {0.996, -0.004, 1.014, 0.0f};
ros::Publisher oculus_pose_pub;
}

namespace oculus_rviz_plugins
{

Oculus::Oculus(void) :
/*
    m_sensorFusion(0), 
    m_stereoConfig(0), 
    m_hmd(0), 
    m_deviceManager(0), */
    m_oculusReady(false), m_ogreReady(false), //m_sensor(0), 
    m_centreOffset(g_defaultProjectionCentreOffset), m_window(0), m_sceneManager(0), m_cameraNode(0)
{
  for (int i = 0; i < 2; ++i)
  {
    m_cameras[i] = 0;
    m_viewports[i] = 0;
    m_compositors[i] = 0;
  }

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  oculus_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/oculus_pose",1000);

}

Oculus::~Oculus(void)
{
  shutDownOgre();
  shutDownOculus();
}

void Oculus::shutDownOculus()
{

  if (m_hmd){
    ovrHmd_Destroy(m_hmd);
    ovr_Shutdown();
  }
  /*
  delete m_stereoConfig;
  m_stereoConfig = 0;
  delete m_sensorFusion;
  m_sensorFusion = 0;

  if (m_sensor)
  {
    m_sensor->Release();
  }
  if (m_hmd)
  {
    m_hmd->Release();
    m_hmd = 0;
  }
  if (m_deviceManager)
  {
    m_deviceManager->Release();
    m_deviceManager = 0;
  }
*/
  if ( m_oculusReady)
  {

  }

  m_oculusReady = false;
//  System::Destroy();
}

void Oculus::shutDownOgre()
{
  m_ogreReady = false;
  for (int i = 0; i < 2; ++i)
  {
    if (m_compositors[i])
    {
      Ogre::CompositorManager::getSingleton().removeCompositor(m_viewports[i], "Oculus");
      m_compositors[i] = 0;
    }
    if (m_viewports[i])
    {
      m_window->removeViewport(i);
      m_viewports[i] = 0;
    }
    if (m_cameras[i])
    {
      m_cameras[i]->getParentSceneNode()->detachObject(m_cameras[i]);
      m_sceneManager->destroyCamera(m_cameras[i]);
      m_cameras[i] = 0;
    }
  }
  if (m_cameraNode)
  {
    m_cameraNode->getParentSceneNode()->removeChild(m_cameraNode);
    m_sceneManager->destroySceneNode(m_cameraNode);
    m_cameraNode = 0;
  }
  m_window = 0;
  m_sceneManager = 0;
}

bool Oculus::isOculusReady() const
{
  return m_oculusReady;
}

bool Oculus::isOgreReady() const
{
  return m_ogreReady;
}

bool Oculus::setupOculus()
{
  if (m_oculusReady)
  {
    Ogre::LogManager::getSingleton().logMessage("Oculus: Already Initialised");
    return true;
  }
  ovrInitParams params = {0, 0, NULL, 0};
  bool success = ovr_Initialize(&params);
  // initialzes the OVR

  m_hmd = ovrHmd_Create(0);

  if (!success){
      Ogre::LogManager::getSingleton().logMessage("Oculus: Initialization of OVR failed.");
      // no return here; the next step finds out why
  }

  // attempts to detect the HMD and dies if it doesn't.
  int numDevices = ovrHmd_Detect();

  // special steps for Dk1?

  if (numDevices < 1){
      Ogre::LogManager::getSingleton().logMessage("Oculus: Hmd not detected.");
      switch(numDevices){
        case 0:
           Ogre::LogManager::getSingleton().logMessage("Check if the Oculus is plugged in / turned on.");
          return false;

        case -1:
          Ogre::LogManager::getSingleton().logMessage("Oculus: Please run ovrd in a seperate terminal.");
          return false;

        default:
          char msgstr[10];
          sprintf(msgstr, "Unknown Error Code: %d",numDevices);
          Ogre::LogManager::getSingleton().logMessage(msgstr);
          return false;

      }
    }  



  if (!m_hmd){   
    Ogre::LogManager::getSingleton().logMessage("Oculus:Could not set up virtual HMD.");
    return false;    
  }

  ovrHmd_RecenterPose(m_hmd);

  // sets up tracking

  if (!ovrHmd_ConfigureTracking(m_hmd,
                                ovrTrackingCap_Orientation | ovrTrackingCap_MagYawCorrection | ovrTrackingCap_Position,
                                0))
  {
    Ogre::LogManager::getSingleton().logMessage("Oculus: Cannot configure OVR Tracking.");
    return false;
  } 



  // sets up rendering information



  if (!ovrHmd_ConfigureRendering(m_hmd, 0, 0, 0, m_eyeRenderDescOut)){
    Ogre::LogManager::getSingleton().logMessage("Oculus: Cannot configure OVR rendering.");
    return false;
  }  

  unsigned int hmdCaps = ovrHmdCap_DynamicPrediction;
  ovrHmd_SetEnabledCaps(m_hmd, hmdCaps);

  // Set up whatever data structures are necessary.

  // Prints the version number
  Ogre::LogManager::getSingleton().logMessage(ovr_GetVersionString());
  // Prints success
  Ogre::LogManager::getSingleton().logMessage("Oculus: Oculus setup completed successfully.");

  // TODO: figure out if additional calibration is needed.

  /*
  Ogre::LogManager::getSingleton().logMessage("Oculus: Initialising system");
  System::Init(Log::ConfigureDefaultLog(LogMask_All));
  m_deviceManager = DeviceManager::Create();
  if (!m_deviceManager)
  {
    Ogre::LogManager::getSingleton().logMessage("Oculus: Failed to create Device Manager");
    return false;
  }
  Ogre::LogManager::getSingleton().logMessage("Oculus: Created Device Manager");
  m_stereoConfig = new Util::Render::StereoConfig();
  if (!m_stereoConfig)
  {
    Ogre::LogManager::getSingleton().logMessage("Oculus: Failed to create StereoConfig");
    return false;
  }
  m_centreOffset = m_stereoConfig->GetProjectionCenterOffset();
  Ogre::LogManager::getSingleton().logMessage("Oculus: Created StereoConfig");
  m_hmd = m_deviceManager->EnumerateDevices<HMDDevice>().CreateDevice();
  if (!m_hmd)
  {
    Ogre::LogManager::getSingleton().logMessage("Oculus: Failed to create HMD");
    return false;
  }
  Ogre::LogManager::getSingleton().logMessage("Oculus: Created HMD");
  HMDInfo devinfo;
  m_hmd->GetDeviceInfo(&devinfo);
  m_stereoConfig->SetHMDInfo(devinfo);

  m_sensor = m_hmd->GetSensor();
  if (!m_sensor)
  {
    Ogre::LogManager::getSingleton().logMessage("Oculus: Failed to create sensor");
    return false;
  }
  Ogre::LogManager::getSingleton().logMessage("Oculus: Created sensor");

  m_sensorFusion = new SensorFusion();
  m_sensorFusion->AttachToSensor(m_sensor);
  Ogre::LogManager::getSingleton().logMessage("Oculus: Created SensorFusion");

  m_magCalibration = new Util::MagCalibration();
  m_magCalibration->BeginAutoCalibration( *m_sensorFusion );
  Ogre::LogManager::getSingleton().logMessage("Oculus: Created MagCalibration");
*/
  m_oculusReady = true;
  Ogre::LogManager::getSingleton().logMessage("Oculus: Oculus setup completed successfully");
  return true;
}

// Currently, only the DK2 is supported. Each DK has it's own resolution
// per eye. Further revisions will include a way to account for other dk versions
float Oculus::getAspectRatio(){
	switch (m_hmd->Type){
		case ovrHmd_DK1:
			return 640.0/800.0;		
		case ovrHmd_DK2:
			return 960.0/1080.0;
		default: // release version
			return 2160.0/1200.0;
	}
}

bool Oculus::setupOgre(Ogre::SceneManager *sm, Ogre::RenderWindow *win, Ogre::SceneNode *parent)
{
  m_window = win;
  m_sceneManager = sm;
  Ogre::LogManager::getSingleton().logMessage("Oculus: Setting up Ogre");
  if (parent)
    m_cameraNode = parent->createChildSceneNode("StereoCameraNode");
  else
    m_cameraNode = sm->getRootSceneNode()->createChildSceneNode("StereoCameraNode");

  m_cameras[0] = sm->createCamera("CameraLeft");
  m_cameras[1] = sm->createCamera("CameraRight");

  Ogre::MaterialPtr matLeft = Ogre::MaterialManager::getSingleton().getByName("Ogre/Compositor/Oculus");
  Ogre::MaterialPtr matRight = matLeft->clone("Ogre/Compositor/Oculus/Right");
  Ogre::GpuProgramParametersSharedPtr pParamsLeft =
      matLeft->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
  Ogre::GpuProgramParametersSharedPtr pParamsRight =
      matRight->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
  Ogre::Vector4 hmdwarp;

  /*
  if (m_stereoConfig)
  {
    hmdwarp = Ogre::Vector4(m_stereoConfig->GetDistortionK(0), m_stereoConfig->GetDistortionK(1),
                            m_stereoConfig->GetDistortionK(2), m_stereoConfig->GetDistortionK(3));
  }
  else
  {
    hmdwarp = Ogre::Vector4(g_defaultDistortion[0], g_defaultDistortion[1], g_defaultDistortion[2],
                            g_defaultDistortion[3]);
  }

  pParamsLeft->setNamedConstant("HmdWarpParam", hmdwarp);
  pParamsRight->setNamedConstant("HmdWarpParam", hmdwarp);

  Ogre::Vector4 hmdchrom;
  if (m_stereoConfig)
  {
    hmdchrom = Ogre::Vector4(m_stereoConfig->GetHMDInfo().ChromaAbCorrection);
  }
  else
  {
    hmdchrom = Ogre::Vector4(g_defaultChromAb);
  }
  pParamsLeft->setNamedConstant("ChromAbParam", hmdchrom);
  pParamsRight->setNamedConstant("ChromAbParam", hmdchrom);

  pParamsLeft->setNamedConstant("LensCenter", 0.5f + (m_stereoConfig->GetProjectionCenterOffset() / 2.0f));
  pParamsRight->setNamedConstant("LensCenter", 0.5f - (m_stereoConfig->GetProjectionCenterOffset() / 2.0f));

  Ogre::CompositorPtr comp = Ogre::CompositorManager::getSingleton().getByName("OculusRight");
  comp->getTechnique(0)->getOutputTargetPass()->getPass(0)->setMaterialName("Ogre/Compositor/Oculus/Right");

  */
  for (int i = 0; i < 2; ++i)
  {
    m_cameraNode->attachObject(m_cameras[i]);

    // sets camera options
      m_cameras[i]->setNearClipDistance(g_defaultNearClip);
      m_cameras[i]->setFarClipDistance(g_defaultFarClip);

//      m_cameras[i]->setFarClipDistance(m_hmd->CameraFrustumFarZInMeters);

//      m_cameras[i]->setNearClipDistance(m_hmd->CameraFrustumNearZInMeters);
//      m_cameras[i]->setFarClipDistance(m_hmd->CameraFrustumFarZInMeters);
      m_cameras[i]->setPosition((i * 2 - 1) * OVR_DEFAULT_IPD * 0.5f, 0, 0);
      m_cameras[i]->setFOVy(Ogre::Radian(m_hmd->CameraFrustumVFovInRadians*2.));
      // aspect ratio for DK2. Add in a more encapsulated way of setting this


      m_cameras[i]->setAspectRatio(getAspectRatio());      
    /*
    if (m_stereoConfig)
    {
      // Setup cameras.
      m_cameras[i]->setNearClipDistance(m_stereoConfig->GetEyeToScreenDistance());
      m_cameras[i]->setFarClipDistance(g_defaultFarClip);
      m_cameras[i]->setPosition((i * 2 - 1) * m_stereoConfig->GetIPD() * 0.5f, 0, 0);
      m_cameras[i]->setAspectRatio(m_stereoConfig->GetAspect());
      m_cameras[i]->setFOVy(Ogre::Radian(m_stereoConfig->GetYFOVRadians()));
    }
    else
    {
      m_cameras[i]->setNearClipDistance(g_defaultNearClip);
      m_cameras[i]->setFarClipDistance(g_defaultFarClip);
      m_cameras[i]->setPosition((i * 2 - 1) * g_defaultIPD * 0.5f, 0, 0);
    }
  */    
    m_viewports[i] = win->addViewport(m_cameras[i], i, 0.5f * i, 0, 0.5f, 1.0f);
    m_viewports[i]->setBackgroundColour(g_defaultViewportColour);
    m_compositors[i] = Ogre::CompositorManager::getSingleton().addCompositor(m_viewports[i],
                                                                             i == 0 ? "OculusLeft" : "OculusRight");
    m_compositors[i]->setEnabled(true);
  }

  updateProjectionMatrices();

  m_ogreReady = true;
  Ogre::LogManager::getSingleton().logMessage("Oculus: Oculus setup completed successfully");
  return true;
}

void Oculus::updateProjectionMatrices()
{



//  if (m_stereoConfig)
//  {

    ovrHmd_BeginFrameTiming(m_hmd, 0);


    for (int i = 0; i < 2; ++i)
    {

      Ogre::Matrix4 proj = Ogre::Matrix4::IDENTITY;
      m_cameras[i]->setCustomProjectionMatrix(false);
      ovrEyeType eye;

      switch(i){
        case 0:
          eye = ovrEye_Left;
          break;
        case 1:
          eye = ovrEye_Right;
          break;
      }
      ovrEyeRenderDesc temp = ovrHmd_GetRenderDesc(m_hmd,eye,m_hmd->DefaultEyeFov[i]);
      //ovrPosef temp = ovrHmd_GetHmdPosePerEye(m_hmd, eye);

      //float temp = m_stereoConfig->GetProjectionCenterOffset();
//      proj.setTrans(Ogre::Vector3(-m_stereoConfig->GetProjectionCenterOffset() * (2 * i - 1), 0, 0));
//      proj.setTrans(Ogre::Vector3(temp.Orientation.x, temp.Orientation.y, temp.Orientation.z));

      proj.setTrans(Ogre::Vector3(temp.HmdToEyeViewOffset.x, temp.HmdToEyeViewOffset.y, temp.HmdToEyeViewOffset.z));

//    char msgstring[1024];      
//    sprintf(msgstring, "Orientation: %.2f %.2f %.2f %.2f", temp.HmdToEyeViewOffset.x,temp.HmdToEyeViewOffset.y,temp.HmdToEyeViewOffset.z);

//    Ogre::LogManager::getSingleton().logMessage(msgstring);

      m_cameras[i]->setCustomProjectionMatrix(true, proj * m_cameras[i]->getProjectionMatrix());
    }

    ovrHmd_EndFrameTiming(m_hmd);


  //
  
}

void Oculus::update()
{
  if (m_ogreReady)
  {
    Ogre::Quaternion orient = getOrientation();

//    char msgstring[1024];
//    sprintf(msgstring, "Orientation: %.2f %.2f %.2f %.2f", orient.x,orient.y,orient.z, orient.w);
//    Ogre::LogManager::getSingleton().logMessage(msgstring);

    ovrPosef currentPose = getPosition();

    m_cameraNode->setPosition(currentPose.Position.x, currentPose.Position.y, currentPose.Position.z); //
    m_cameraNode->setOrientation(getOrientation());
    //    ROS_INFO("%.2f %.2f %.2f %.2f %.2f %.2f %.2f",currentPose.Position.x, currentPose.Position.y, currentPose.Position.z, orient.x,orient.y,orient.z, orient.w); 

    geometry_msgs::PoseStamped oculus_pose_msg;
    //	oculu_pose_msg.header.frame_id = ;
    //	oculu_pose_msg.header.stamp = ;
    oculus_pose_msg.pose.position.x = currentPose.Position.x;
    oculus_pose_msg.pose.position.y = currentPose.Position.y;
    oculus_pose_msg.pose.position.z = currentPose.Position.z;
    oculus_pose_msg.pose.orientation.x = orient.x;
    oculus_pose_msg.pose.orientation.y = orient.y;
    oculus_pose_msg.pose.orientation.z = orient.z;
    oculus_pose_msg.pose.orientation.w = orient.w;
    
    //    oculus_pose_pub.publish(oculus_pose_msg);

/*
    if (m_magCalibration->IsAutoCalibrating())
    {
      m_magCalibration->UpdateAutoCalibration( *m_sensorFusion );
      if (m_magCalibration->IsCalibrated())
      {
        m_sensorFusion->SetYawCorrectionEnabled(true);
      }
      
    }
    */
    
  }
  
}

bool Oculus::isMagCalibrated()
{

  //TODO: Figure out how to calibrate mag stuff if necessary.
  return true;

  /*
  return m_oculusReady && m_magCalibration->IsCalibrated();
  */
}

Ogre::SceneNode* Oculus::getCameraNode()
{
  return m_cameraNode;
}

void Oculus::setPredictionDt(float dt)
{
  if (m_oculusReady)
  {

//    m_sensorFusion->SetPrediction( dt, dt > 0.0f );
  }
}

Ogre::Quaternion Oculus::getOrientation() const
{
  if (m_oculusReady)
  {
   // get orientation
   
   ovrTrackingState state = ovrHmd_GetTrackingState(m_hmd, 0.0);
   if (!&state){
      Ogre::LogManager::getSingleton().logMessage("Oculus: Sensor not found.");
   } else {
    ovrQuatf q = state.HeadPose.ThePose.Orientation;// get pose

    return Ogre::Quaternion(q.w, q.x, q.y, q.z);
   }

//    Quatf q = m_sensorFusion->GetPredictedOrientation();
//    return Ogre::Quaternion(q.w, q.x, q.y, q.z);
  }
  else
  {
  }

    return Ogre::Quaternion::IDENTITY;  
}

ovrPosef Oculus::getPosition() const
{
  ovrEyeType eye; 
  ovrPosef pose = ovrHmd_GetHmdPosePerEye(m_hmd, eye);

  return pose;
}

Ogre::CompositorInstance *Oculus::getCompositor(unsigned int i)
{
  return m_compositors[i];
}

float Oculus::getCentreOffset() const
{
  return m_centreOffset;
}

void Oculus::resetOrientation()
{
//  if (m_sensorFusion)
//    m_sensorFusion->Reset();

  if (m_hmd){
    ovrHmd_RecenterPose(m_hmd);
  }
}
}
