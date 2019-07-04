/**
 * @file CameraSettings.h
 * Declaration of a struct representing the settings of the PDA camera.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Math/FixedPoint.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include <array>

/*
                   brightness (int)    : min=-255 max=255 step=1 default=0 value=35
                         contrast (int)    : min=0 max=255 step=1 default=32 value=64
                       saturation (int)    : min=0 max=255 step=1 default=64 value=160
                              hue (int)    : min=-180 max=180 step=1 default=0 value=0
   white_balance_temperature_auto (bool)   : default=1 value=1
                             gain (int)    : min=0 max=1023 step=1 default=16 value=160
                         hue_auto (bool)   : default=0 value=0
        white_balance_temperature (int)    : min=2500 max=6500 step=500 default=2500 value=0 flags=inactive
                        sharpness (int)    : min=0 max=9 step=1 default=4 value=2
                    exposure_auto (menu)   : min=0 max=3 default=0 value=0
                exposure_absolute (int)    : min=0 max=1048575 step=1 default=512 value=512 flags=inactive
                   focus_absolute (int)    : min=0 max=250 step=25 default=0 value=0 flags=inactive
                       focus_auto (bool)   : default=0 value=1
*/
STREAMABLE(CameraSettings,
{
  ENUM(CameraSetting,
  {,
    // NEW
  autoFocus,
  autoExposure,
  autoHue,
  autoWhiteBalance,
  
  brightness,
  contrast,
  saturation,
  hue,
  gain,
  sharpness,
  whiteBalanceTemperature,
  focusAbsolute,
  exposureAbsolute,
 
 
  });

  ENUM(ExposureAlgorithm,
  {,
    averageY, /* Average scene brightness. */
    weighted, /* Weighted average brightness (the weights can be controlled by the autoExposureWeightTable). */
    highlights, /* Adaptive weighted auto exposure for highlights. */
    lowlights, /* Adaptive weighted auto exposure for lowlights. */
  });

  ENUM(PowerLineFrequency,
  {,
    _50Hz,
    _60Hz,
  });

  struct CameraSettingsCollection : public Streamable
  {
    std::array<int COMMA numOfCameraSettings> settings;

    CameraSettingsCollection();

  protected:
    void serialize(In* in, Out* out) override;

  private:
    static void reg();
  };

  CameraSettingsCollection& operator[](const CameraInfo::Camera camera)
  {
    return camera == CameraInfo::upper ? upper : lower;
  },

  (CameraSettingsCollection) upper,
  (CameraSettingsCollection) lower,
});
