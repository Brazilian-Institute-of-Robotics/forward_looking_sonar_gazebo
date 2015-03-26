#ifndef SIMULATION_NORMAL_DEPTH_MAP_SRC_TOOLS_HPP_
#define SIMULATION_NORMAL_DEPTH_MAP_SRC_TOOLS_HPP_

namespace normal_depth_map {

  /**
   * @brief compute Underwater Signal Attenuation coefficient
   *
   *  This method is based on paper "A simplified formula for viscous and
   *  chemical absorption in sea water". The method computes the attenuation
   *  coefficient that will be used on shader normal intensite return.
   *
   *  @param double frequency: sound frequency in kHz.
   *  @param double temperature: water temperature in Celsius degrees.
   *  @param double depth: distance from water surface in meters.
   *  @param double salinity: amount of salt dissolved in a body of water in ppt.
   *  @param double acidity: pH water value.
   *
   *  @return double coefficient attenuation value
   */

  double underwaterSignalAttenuation( const double frequency,
                                      const double temperature,
                                      const double depth,
                                      const double salinity,
                                      const double acidity);
}

#endif
