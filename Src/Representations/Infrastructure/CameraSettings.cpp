/**
 * @file CameraSettings.cpp
 * Implementation of CameraSettings.
 */

#include "CameraSettings.h"
#include "Platform/BHAssert.h"
#include "Platform/Camera.h"

CameraSettings::CameraSettingsCollection::CameraSettingsCollection()
{
  settings.fill(-1000);
}

void CameraSettings::CameraSettingsCollection::serialize(In* in, Out* out)
{
  int& autoFocus = settings[CameraSettings::autoFocus];
  int& autoExposure = settings[CameraSettings::autoExposure];
  int& autoHue = settings[CameraSettings::autoHue];
  int& autoWhiteBalance = settings[CameraSettings::autoWhiteBalance];

  int& brightness = settings[CameraSettings::brightness];
  int& contrast = settings[CameraSettings::contrast];
  int& saturation = settings[CameraSettings::saturation];
  int& hue = settings[CameraSettings::hue];
  int& gain = settings[CameraSettings::gain];
  int& sharpness = settings[CameraSettings::sharpness];
  int& whiteBalanceTemperature = settings[CameraSettings::whiteBalanceTemperature];
  int& focusAbsolute = settings[CameraSettings::focusAbsolute];
  int& exposureAbsolute = settings[CameraSettings::exposureAbsolute];

  STREAM(autoFocus);
  STREAM(autoExposure);
  STREAM(autoHue);
  STREAM(autoWhiteBalance);

  STREAM(brightness);
  STREAM(contrast);
  STREAM(saturation);
  STREAM(hue);
  STREAM(gain);
  STREAM(sharpness);
  STREAM(whiteBalanceTemperature);
  STREAM(focusAbsolute);
  STREAM(exposureAbsolute);
  
  if(in)
  {
    settings[CameraSettings::brightness] = static_cast<int>(brightness);
    settings[CameraSettings::contrast] = static_cast<int>(contrast);
    settings[CameraSettings::saturation] = static_cast<int>(saturation);
    settings[CameraSettings::hue] = static_cast<int>(hue);
    settings[CameraSettings::gain] = static_cast<int>(gain);
    settings[CameraSettings::sharpness] = static_cast<int>(sharpness);
    settings[CameraSettings::whiteBalanceTemperature] = static_cast<int>(whiteBalanceTemperature);
    settings[CameraSettings::focusAbsolute] = static_cast<int>(focusAbsolute);
    settings[CameraSettings::exposureAbsolute] = static_cast<int>(exposureAbsolute);
    settings[CameraSettings::autoFocus] = static_cast<int>(autoFocus);
    settings[CameraSettings::autoExposure] = static_cast<int>(autoExposure);
    settings[CameraSettings::autoHue] = static_cast<int>(autoHue);
    settings[CameraSettings::autoWhiteBalance] = static_cast<int>(autoWhiteBalance);
  }
}

void CameraSettings::CameraSettingsCollection::reg()
{

  PUBLISH(reg);
  REG_CLASS(CameraSettingsCollection);
  REG(int, autoFocus);
  REG(int, autoExposure);
  REG(int, autoHue);
  REG(int, autoWhiteBalance);
  REG(int, brightness);
  REG(int, contrast);
  REG(int, saturation);
  REG(int, hue);
  REG(int, gain);
  REG(int, sharpness);
  REG(int, whiteBalanceTemperature);
  REG(int, focusAbsolute);
  REG(int, exposureAbsolute);


}
