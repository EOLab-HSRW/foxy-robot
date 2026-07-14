#ifndef FOXY_GZ_PLUGINS__LED_SYSTEM_HH_
#define FOXY_GZ_PLUGINS__LED_SYSTEM_HH_

#include <memory>

#include <gz/sim/System.hh>
#include <sdf/Element.hh>

namespace foxy_gz_plugins
{
class LedSystemPrivate;

/// \brief Sensor-attached LED actuator renderer for Gazebo Harmonic.
///
/// Attach this system plugin to an SDFormat custom sensor:
///
/// \code{.xml}
/// <sensor name="leds" type="custom" gz:type="led">
///   <plugin filename="foxy_gz_led_system"
///           name="foxy_gz_plugins::LedSystem">
///     ...
///   </plugin>
/// </sensor>
/// \endcode
///
/// For every configured <led>, the system constructs and owns one sphere visual
/// and one point light from compact LED parameters. Callers do not provide raw
/// <visual> or <light> SDF blocks. Both entities are parented directly to the link that
/// owns the sensor. LED poses are expressed in the sensor frame.
///
/// Channel command and optional state topics use gz.msgs.Color. RGB stores the
/// requested color and alpha stores drive intensity in [0, 1]. The default
/// topic layout remains:
///
/// \verbatim
/// /model/<model_name>/led/<channel_name>/set
/// /model/<model_name>/led/<channel_name>/get
/// \endverbatim
class LedSystem:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemReset
{
  public: LedSystem();
  public: ~LedSystem() override;

  public: void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr) override;

  public: void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override;

  public: void Reset(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override;

  private: std::unique_ptr<LedSystemPrivate> dataPtr;
};
}  // namespace foxy_gz_plugins

#endif  // FOXY_GZ_PLUGINS__LED_SYSTEM_HH_
