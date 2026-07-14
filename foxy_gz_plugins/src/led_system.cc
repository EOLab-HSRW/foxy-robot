#include "foxy_gz_plugins/led_system.hh"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Color.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <gz/msgs/color.pb.h>
#include <gz/msgs/light.pb.h>
#include <gz/msgs/visual.pb.h>

#include <gz/plugin/Register.hh>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>

#include <gz/sensors/Util.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/LightCmd.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/VisualCmd.hh>

#include <gz/transport/Node.hh>
#include <gz/transport/TopicUtils.hh>

#include <sdf/Element.hh>
#include <sdf/Geometry.hh>
#include <sdf/Light.hh>
#include <sdf/Material.hh>
#include <sdf/Sensor.hh>
#include <sdf/Sphere.hh>
#include <sdf/Types.hh>
#include <sdf/Visual.hh>

using namespace gz;
using namespace gz::sim;

namespace foxy_gz_plugins
{
namespace
{
//////////////////////////////////////////////////
double Clamp01(double _value)
{
  return std::clamp(_value, 0.0, 1.0);
}

//////////////////////////////////////////////////
bool Near(double _a, double _b, double _eps = 1e-6)
{
  return std::abs(_a - _b) <= _eps;
}

//////////////////////////////////////////////////
math::Color ClampColor(const math::Color &_color)
{
  return math::Color(
    Clamp01(_color.R()),
    Clamp01(_color.G()),
    Clamp01(_color.B()),
    Clamp01(_color.A()));
}

//////////////////////////////////////////////////
msgs::Color ToMsg(const math::Color &_state)
{
  msgs::Color msg;
  msg.set_r(static_cast<float>(Clamp01(_state.R())));
  msg.set_g(static_cast<float>(Clamp01(_state.G())));
  msg.set_b(static_cast<float>(Clamp01(_state.B())));
  msg.set_a(static_cast<float>(Clamp01(_state.A())));
  return msg;
}

//////////////////////////////////////////////////
math::Color FromMsg(const msgs::Color &_msg, const math::Color &_defaultColor)
{
  const double intensity = Clamp01(_msg.a());

  double r = Clamp01(_msg.r());
  double g = Clamp01(_msg.g());
  double b = Clamp01(_msg.b());

  // Proto3 scalar presence is unavailable. RGB=0 with nonzero intensity means
  // "use the channel default color". Black is physically equivalent to off.
  if (intensity > 0.0 && Near(r, 0.0) && Near(g, 0.0) && Near(b, 0.0))
  {
    r = Clamp01(_defaultColor.R());
    g = Clamp01(_defaultColor.G());
    b = Clamp01(_defaultColor.B());
  }

  return math::Color(r, g, b, intensity);
}

//////////////////////////////////////////////////
math::Color EffectiveVisualColor(
  const math::Color &_state,
  const math::Vector3d &_colorMultiplier,
  double _gain)
{
  const double intensity = Clamp01(_state.A() * _gain);
  return math::Color(
    Clamp01(_state.R() * _colorMultiplier.X() * intensity),
    Clamp01(_state.G() * _colorMultiplier.Y() * intensity),
    Clamp01(_state.B() * _colorMultiplier.Z() * intensity),
    1.0);
}

//////////////////////////////////////////////////
math::Color EffectiveLightColor(
  const math::Color &_state,
  const math::Vector3d &_colorMultiplier)
{
  return math::Color(
    Clamp01(_state.R() * _colorMultiplier.X()),
    Clamp01(_state.G() * _colorMultiplier.Y()),
    Clamp01(_state.B() * _colorMultiplier.Z()),
    1.0);
}

//////////////////////////////////////////////////
bool VisualMsgsEqual(const msgs::Visual &_a, const msgs::Visual &_b)
{
  const auto &aMaterial = _a.material();
  const auto &bMaterial = _b.material();

  return _a.id() == _b.id() &&
    math::equal(aMaterial.ambient().r(), bMaterial.ambient().r(), 1e-6f) &&
    math::equal(aMaterial.ambient().g(), bMaterial.ambient().g(), 1e-6f) &&
    math::equal(aMaterial.ambient().b(), bMaterial.ambient().b(), 1e-6f) &&
    math::equal(aMaterial.ambient().a(), bMaterial.ambient().a(), 1e-6f) &&
    math::equal(aMaterial.diffuse().r(), bMaterial.diffuse().r(), 1e-6f) &&
    math::equal(aMaterial.diffuse().g(), bMaterial.diffuse().g(), 1e-6f) &&
    math::equal(aMaterial.diffuse().b(), bMaterial.diffuse().b(), 1e-6f) &&
    math::equal(aMaterial.diffuse().a(), bMaterial.diffuse().a(), 1e-6f) &&
    math::equal(aMaterial.specular().r(), bMaterial.specular().r(), 1e-6f) &&
    math::equal(aMaterial.specular().g(), bMaterial.specular().g(), 1e-6f) &&
    math::equal(aMaterial.specular().b(), bMaterial.specular().b(), 1e-6f) &&
    math::equal(aMaterial.specular().a(), bMaterial.specular().a(), 1e-6f) &&
    math::equal(aMaterial.emissive().r(), bMaterial.emissive().r(), 1e-6f) &&
    math::equal(aMaterial.emissive().g(), bMaterial.emissive().g(), 1e-6f) &&
    math::equal(aMaterial.emissive().b(), bMaterial.emissive().b(), 1e-6f) &&
    math::equal(aMaterial.emissive().a(), bMaterial.emissive().a(), 1e-6f);
}

//////////////////////////////////////////////////
bool LightMsgsEqual(const msgs::Light &_a, const msgs::Light &_b)
{
  return _a.id() == _b.id() &&
    math::equal(_a.intensity(), _b.intensity(), 1e-6f) &&
    math::equal(_a.diffuse().r(), _b.diffuse().r(), 1e-6f) &&
    math::equal(_a.diffuse().g(), _b.diffuse().g(), 1e-6f) &&
    math::equal(_a.diffuse().b(), _b.diffuse().b(), 1e-6f) &&
    math::equal(_a.diffuse().a(), _b.diffuse().a(), 1e-6f) &&
    math::equal(_a.specular().r(), _b.specular().r(), 1e-6f) &&
    math::equal(_a.specular().g(), _b.specular().g(), 1e-6f) &&
    math::equal(_a.specular().b(), _b.specular().b(), 1e-6f) &&
    math::equal(_a.specular().a(), _b.specular().a(), 1e-6f) &&
    math::equal(_a.range(), _b.range(), 1e-6f) &&
    math::equal(
      _a.attenuation_constant(), _b.attenuation_constant(), 1e-6f) &&
    math::equal(_a.attenuation_linear(), _b.attenuation_linear(), 1e-6f) &&
    math::equal(
      _a.attenuation_quadratic(), _b.attenuation_quadratic(), 1e-6f);
}

//////////////////////////////////////////////////
std::string ModelTopicPrefix(
  std::string _configuredPrefix,
  const std::string &_modelName)
{
  while (!_configuredPrefix.empty() && _configuredPrefix.back() == '/')
  {
    _configuredPrefix.pop_back();
  }

  if (_configuredPrefix.empty())
  {
    return "/model/" + _modelName;
  }

  if (_configuredPrefix.front() != '/')
  {
    _configuredPrefix.insert(_configuredPrefix.begin(), '/');
  }

  return _configuredPrefix + "/model/" + _modelName;
}

//////////////////////////////////////////////////
std::string MaybeDefaultTopic(
  const std::string &_configured,
  const std::string &_modelTopicPrefix,
  const std::string &_channelName,
  const std::string &_suffix)
{
  if (!_configured.empty())
  {
    return _configured;
  }

  return _modelTopicPrefix + "/led/" + _channelName + "/" + _suffix;
}

//////////////////////////////////////////////////
Entity FindOwningModel(
  Entity _entity,
  const EntityComponentManager &_ecm)
{
  Entity current = _entity;
  for (std::size_t depth = 0; depth < 256u; ++depth)
  {
    if (_ecm.Component<components::Model>(current))
    {
      return current;
    }

    const auto parent = _ecm.Component<components::ParentEntity>(current);
    if (!parent || parent->Data() == kNullEntity)
    {
      return kNullEntity;
    }

    current = parent->Data();
  }

  return kNullEntity;
}

//////////////////////////////////////////////////
math::Vector3d ClampVector01(const math::Vector3d &_value)
{
  return math::Vector3d(
    Clamp01(_value.X()),
    Clamp01(_value.Y()),
    Clamp01(_value.Z()));
}

//////////////////////////////////////////////////
bool IsPositiveFinite(double _value)
{
  return std::isfinite(_value) && _value > 0.0;
}
}  // namespace

//////////////////////////////////////////////////
struct Led
{
  std::string name;
  std::string visualName;
  std::string lightName;

  sdf::Visual visualSdf;
  sdf::Light lightSdf;

  Entity visualEntity{kNullEntity};
  Entity lightEntity{kNullEntity};

  math::Vector3d colorMultiplier{1.0, 1.0, 1.0};
  double gain{1.0};
};

//////////////////////////////////////////////////
struct LedChannel
{
  std::string name;
  std::string commandTopic;
  std::string stateTopic;

  math::Color defaultState{1.0, 1.0, 1.0, 0.0};
  math::Color commandedState{1.0, 1.0, 1.0, 0.0};
  math::Color appliedState{1.0, 1.0, 1.0, 0.0};

  transport::Node::Publisher statePublisher;
  bool dirty{true};
  std::vector<Led> leds;
};

//////////////////////////////////////////////////
class LedSystemPrivate
{
  public: std::optional<Led> ParseLed(
    sdf::ElementConstPtr _sdf,
    const math::Pose3d &_sensorPose,
    const std::string &_sensorName,
    std::unordered_set<std::string> &_usedLedNames) const;

  public: std::optional<LedChannel> ParseChannel(
    sdf::ElementConstPtr _sdf,
    const std::string &_modelTopicPrefix,
    const math::Pose3d &_sensorPose,
    const std::string &_sensorName,
    std::unordered_set<std::string> &_usedLedNames) const;

  public: bool CreateEntities(
    EntityComponentManager &_ecm,
    EventManager &_eventManager);

  public: void OnCommand(
    const std::string &_channelName,
    const msgs::Color &_msg);

  public: void ApplyChannel(
    LedChannel &_channel,
    EntityComponentManager &_ecm);

  public: void SetVisualProperties(
    Entity _visualEntity,
    EntityComponentManager &_ecm,
    const math::Color &_materialColor) const;

  public: void SetLightProperties(
    Entity _lightEntity,
    EntityComponentManager &_ecm,
    const math::Color &_lightColor,
    double _lightIntensity) const;

  public: void ResetChannels();

  public: Entity sensorEntity{kNullEntity};
  public: Entity parentLinkEntity{kNullEntity};
  public: Entity modelEntity{kNullEntity};
  public: std::string sensorName;
  public: std::string modelName;

  public: transport::Node node;
  public: std::vector<LedChannel> channels;
  public: bool publishState{true};
  public: bool configured{false};
  public: std::mutex mutex;
};

//////////////////////////////////////////////////
LedSystem::LedSystem()
  : System(), dataPtr(std::make_unique<LedSystemPrivate>())
{
}

//////////////////////////////////////////////////
LedSystem::~LedSystem() = default;

//////////////////////////////////////////////////
void LedSystem::Configure(
  const Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  EntityComponentManager &_ecm,
  EventManager &_eventManager)
{
  this->dataPtr->sensorEntity = _entity;

  if (!_ecm.Component<components::Sensor>(_entity))
  {
    gzerr << "[Foxy LED System] Plugin must be attached to a sensor entity."
          << std::endl;
    return;
  }

  const auto customSensor = _ecm.Component<components::CustomSensor>(_entity);
  if (!customSensor || customSensor->Data().Type() != sdf::SensorType::CUSTOM)
  {
    gzerr << "[Foxy LED System] Plugin must be attached to an SDFormat "
          << "custom sensor (type=\"custom\", gz:type=\"led\")."
          << std::endl;
    return;
  }

  const std::string customType = gz::sensors::customType(customSensor->Data());
  if (customType != "led")
  {
    gzerr << "[Foxy LED System] Unsupported custom sensor type ["
          << customType << "]. Expected gz:type=\"led\"." << std::endl;
    return;
  }

  const auto sensorNameComp = _ecm.Component<components::Name>(_entity);
  if (!sensorNameComp || sensorNameComp->Data().empty())
  {
    gzerr << "[Foxy LED System] Sensor entity has no valid name." << std::endl;
    return;
  }
  this->dataPtr->sensorName = sensorNameComp->Data();

  const auto parentComp = _ecm.Component<components::ParentEntity>(_entity);
  if (!parentComp || parentComp->Data() == kNullEntity)
  {
    gzerr << "[Foxy LED System] Sensor [" << this->dataPtr->sensorName
          << "] has no parent entity." << std::endl;
    return;
  }

  this->dataPtr->parentLinkEntity = parentComp->Data();
  if (!_ecm.Component<components::Link>(this->dataPtr->parentLinkEntity))
  {
    gzerr << "[Foxy LED System] Sensor [" << this->dataPtr->sensorName
          << "] must be attached to a link. Parent entity ["
          << this->dataPtr->parentLinkEntity << "] is not a link."
          << std::endl;
    return;
  }

  this->dataPtr->modelEntity =
    FindOwningModel(this->dataPtr->parentLinkEntity, _ecm);
  if (this->dataPtr->modelEntity == kNullEntity)
  {
    gzerr << "[Foxy LED System] Could not find the model that owns sensor ["
          << this->dataPtr->sensorName << "]." << std::endl;
    return;
  }

  const auto modelNameComp =
    _ecm.Component<components::Name>(this->dataPtr->modelEntity);
  if (!modelNameComp || modelNameComp->Data().empty())
  {
    gzerr << "[Foxy LED System] Owning model has no valid name." << std::endl;
    return;
  }
  this->dataPtr->modelName = modelNameComp->Data();

  math::Pose3d sensorPose = math::Pose3d::Zero;
  const auto sensorPoseComp = _ecm.Component<components::Pose>(_entity);
  if (sensorPoseComp)
  {
    sensorPose = sensorPoseComp->Data();
  }

  if (_sdf->HasElement("publish_state"))
  {
    this->dataPtr->publishState = _sdf->Get<bool>("publish_state");
  }

  std::string configuredTopicPrefix;
  if (_sdf->HasElement("topic_prefix"))
  {
    configuredTopicPrefix = _sdf->Get<std::string>("topic_prefix");
  }

  std::string modelTopicPrefix =
    ModelTopicPrefix(configuredTopicPrefix, this->dataPtr->modelName);
  modelTopicPrefix = transport::TopicUtils::AsValidTopic(modelTopicPrefix);
  if (modelTopicPrefix.empty())
  {
    gzerr << "[Foxy LED System] Invalid model topic prefix." << std::endl;
    return;
  }

  if (!_sdf->HasElement("channel"))
  {
    gzerr << "[Foxy LED System] No <channel> elements found for sensor ["
          << this->dataPtr->sensorName << "]." << std::endl;
    return;
  }

  std::unordered_set<std::string> usedLedNames;
  std::unordered_set<std::string> usedChannelNames;
  auto channelElem = _sdf->FindElement("channel");
  while (channelElem)
  {
    auto channel = this->dataPtr->ParseChannel(
      channelElem,
      modelTopicPrefix,
      sensorPose,
      this->dataPtr->sensorName,
      usedLedNames);

    if (!channel.has_value())
    {
      gzerr << "[Foxy LED System] Failed to parse LED sensor ["
            << this->dataPtr->sensorName << "]." << std::endl;
      this->dataPtr->channels.clear();
      return;
    }

    if (!usedChannelNames.insert(channel->name).second)
    {
      gzerr << "[Foxy LED System][Channel " << channel->name
            << "] Channel names must be unique within one sensor."
            << std::endl;
      this->dataPtr->channels.clear();
      return;
    }

    this->dataPtr->channels.push_back(std::move(*channel));
    channelElem = channelElem->GetNextElement("channel");
  }

  if (this->dataPtr->channels.empty())
  {
    gzerr << "[Foxy LED System] No valid channels were created." << std::endl;
    return;
  }

  if (!this->dataPtr->CreateEntities(_ecm, _eventManager))
  {
    gzerr << "[Foxy LED System] Failed to create LED visual/light entities."
          << std::endl;
    this->dataPtr->channels.clear();
    return;
  }

  for (auto &channel : this->dataPtr->channels)
  {
    if (this->dataPtr->publishState)
    {
      channel.statePublisher =
        this->dataPtr->node.Advertise<msgs::Color>(channel.stateTopic);
      if (!channel.statePublisher)
      {
        gzerr << "[Foxy LED System] Failed to advertise state topic ["
              << channel.stateTopic << "] for channel [" << channel.name
              << "]." << std::endl;
        return;
      }
    }

    const std::string channelName = channel.name;
    std::function<void(const msgs::Color &)> callback =
      [this, channelName](const msgs::Color &_msg)
      {
        this->dataPtr->OnCommand(channelName, _msg);
      };

    if (!this->dataPtr->node.Subscribe<msgs::Color>(
        channel.commandTopic, callback))
    {
      gzerr << "[Foxy LED System] Failed to subscribe channel ["
            << channel.name << "] to topic [" << channel.commandTopic
            << "]." << std::endl;
      return;
    }

    gzmsg << "[Foxy LED System] Channel [" << channel.name
          << "] command topic: [" << channel.commandTopic << "]"
          << std::endl;
    if (this->dataPtr->publishState)
    {
      gzmsg << "[Foxy LED System] Channel [" << channel.name
            << "] state topic:   [" << channel.stateTopic << "]"
            << std::endl;
    }
  }

  this->dataPtr->configured = true;
  gzmsg << "[Foxy LED System] Loaded sensor [" << this->dataPtr->sensorName
        << "] with " << this->dataPtr->channels.size()
        << " channel(s) for model [" << this->dataPtr->modelName << "]."
        << std::endl;
}

//////////////////////////////////////////////////
void LedSystem::PreUpdate(const UpdateInfo &, EntityComponentManager &_ecm)
{
  if (!this->dataPtr->configured)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  for (auto &channel : this->dataPtr->channels)
  {
    if (!channel.dirty)
    {
      continue;
    }

    this->dataPtr->ApplyChannel(channel, _ecm);
    channel.appliedState = channel.commandedState;
    channel.dirty = false;

    if (this->dataPtr->publishState && channel.statePublisher)
    {
      channel.statePublisher.Publish(ToMsg(channel.appliedState));
    }
  }
}

//////////////////////////////////////////////////
void LedSystem::Reset(const UpdateInfo &, EntityComponentManager &_ecm)
{
  if (!this->dataPtr->configured)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->ResetChannels();

  for (auto &channel : this->dataPtr->channels)
  {
    this->dataPtr->ApplyChannel(channel, _ecm);
    channel.appliedState = channel.commandedState;
    channel.dirty = false;

    if (this->dataPtr->publishState && channel.statePublisher)
    {
      channel.statePublisher.Publish(ToMsg(channel.appliedState));
    }
  }
}

//////////////////////////////////////////////////
std::optional<Led> LedSystemPrivate::ParseLed(
  sdf::ElementConstPtr _sdf,
  const math::Pose3d &_sensorPose,
  const std::string &_sensorName,
  std::unordered_set<std::string> &_usedLedNames) const
{
  if (!_sdf->HasAttribute("name"))
  {
    gzerr << "[Foxy LED System][LED] Missing required name attribute."
          << std::endl;
    return std::nullopt;
  }

  Led led;
  led.name = _sdf->Get<std::string>("name");
  if (led.name.empty())
  {
    gzerr << "[Foxy LED System][LED] Empty LED name." << std::endl;
    return std::nullopt;
  }

  if (!_usedLedNames.insert(led.name).second)
  {
    gzerr << "[Foxy LED System][LED " << led.name
          << "] LED names must be unique within one sensor." << std::endl;
    return std::nullopt;
  }

  led.visualName = _sensorName + "_" + led.name + "_visual";
  led.lightName = _sensorName + "_" + led.name + "_light";

  math::Pose3d ledPose = math::Pose3d::Zero;
  if (_sdf->HasElement("pose"))
  {
    ledPose = _sdf->Get<math::Pose3d>("pose");
  }
  const math::Pose3d linkRelativePose = _sensorPose * ledPose;

  std::string shape = "sphere";
  if (_sdf->HasElement("shape"))
  {
    shape = _sdf->Get<std::string>("shape");
  }
  if (shape != "sphere")
  {
    gzerr << "[Foxy LED System][LED " << led.name
          << "] Unsupported shape [" << shape
          << "]. Gazebo Harmonic implementation supports sphere LEDs."
          << std::endl;
    return std::nullopt;
  }

  double radius = 0.005;
  if (_sdf->HasElement("radius"))
  {
    radius = _sdf->Get<double>("radius");
  }
  if (!IsPositiveFinite(radius))
  {
    gzerr << "[Foxy LED System][LED " << led.name
          << "] <radius> must be a positive finite value." << std::endl;
    return std::nullopt;
  }

  if (_sdf->HasElement("color") && _sdf->HasElement("color_multiplier"))
  {
    gzerr << "[Foxy LED System][LED " << led.name
          << "] Specify either <color> or legacy <color_multiplier>, not both."
          << std::endl;
    return std::nullopt;
  }
  if (_sdf->HasElement("color"))
  {
    led.colorMultiplier = ClampVector01(
      _sdf->Get<math::Vector3d>("color"));
  }
  else if (_sdf->HasElement("color_multiplier"))
  {
    led.colorMultiplier = ClampVector01(
      _sdf->Get<math::Vector3d>("color_multiplier"));
  }

  if (_sdf->HasElement("gain"))
  {
    led.gain = _sdf->Get<double>("gain");
  }
  if (!std::isfinite(led.gain) || led.gain < 0.0)
  {
    gzerr << "[Foxy LED System][LED " << led.name
          << "] <gain> must be a non-negative finite value." << std::endl;
    return std::nullopt;
  }

  double lightRange = 2.0;
  if (_sdf->HasElement("range"))
  {
    lightRange = _sdf->Get<double>("range");
  }
  if (!IsPositiveFinite(lightRange))
  {
    gzerr << "[Foxy LED System][LED " << led.name
          << "] <range> must be a positive finite value." << std::endl;
    return std::nullopt;
  }

  double attenuationConstant = 0.6;
  double attenuationLinear = 0.1;
  double attenuationQuadratic = 0.01;
  if (_sdf->HasElement("attenuation"))
  {
    const auto attenuationElem = _sdf->FindElement("attenuation");
    if (attenuationElem->HasElement("constant"))
    {
      attenuationConstant = attenuationElem->Get<double>("constant");
    }
    if (attenuationElem->HasElement("linear"))
    {
      attenuationLinear = attenuationElem->Get<double>("linear");
    }
    if (attenuationElem->HasElement("quadratic"))
    {
      attenuationQuadratic = attenuationElem->Get<double>("quadratic");
    }
  }

  if (!std::isfinite(attenuationConstant) || attenuationConstant < 0.0 ||
      !std::isfinite(attenuationLinear) || attenuationLinear < 0.0 ||
      !std::isfinite(attenuationQuadratic) || attenuationQuadratic < 0.0)
  {
    gzerr << "[Foxy LED System][LED " << led.name
          << "] Attenuation values must be non-negative and finite."
          << std::endl;
    return std::nullopt;
  }

  bool castShadows = false;
  if (_sdf->HasElement("cast_shadows"))
  {
    castShadows = _sdf->Get<bool>("cast_shadows");
  }

  bool visualizeLight = false;
  if (_sdf->HasElement("visualize"))
  {
    visualizeLight = _sdf->Get<bool>("visualize");
  }

  // Build the visual internally. The caller supplies only compact LED
  // parameters, never a raw <visual> element.
  sdf::Sphere sphere;
  sphere.SetRadius(radius);

  sdf::Geometry geometry;
  geometry.SetType(sdf::GeometryType::SPHERE);
  geometry.SetSphereShape(sphere);

  const math::Color offColor(0.0, 0.0, 0.0, 1.0);
  sdf::Material material;
  material.SetAmbient(offColor);
  material.SetDiffuse(offColor);
  material.SetSpecular(offColor);
  material.SetEmissive(offColor);

  led.visualSdf.SetName(led.visualName);
  led.visualSdf.SetRawPose(linkRelativePose);
  led.visualSdf.SetGeom(geometry);
  led.visualSdf.SetMaterial(material);
  led.visualSdf.SetCastShadows(castShadows);

  // Build the point light internally. Its initial intensity is zero; the
  // channel command updates both the visual material and physical light.
  const math::Color hardwareColor(
    led.colorMultiplier.X(),
    led.colorMultiplier.Y(),
    led.colorMultiplier.Z(),
    1.0);

  led.lightSdf.SetName(led.lightName);
  led.lightSdf.SetType(sdf::LightType::POINT);
  led.lightSdf.SetRawPose(linkRelativePose);
  led.lightSdf.SetLightOn(true);
  led.lightSdf.SetVisualize(visualizeLight);
  led.lightSdf.SetCastShadows(castShadows);
  led.lightSdf.SetIntensity(0.0);
  led.lightSdf.SetDiffuse(hardwareColor);
  led.lightSdf.SetSpecular(hardwareColor);
  led.lightSdf.SetAttenuationRange(lightRange);
  led.lightSdf.SetConstantAttenuationFactor(attenuationConstant);
  led.lightSdf.SetLinearAttenuationFactor(attenuationLinear);
  led.lightSdf.SetQuadraticAttenuationFactor(attenuationQuadratic);

  return led;
}

//////////////////////////////////////////////////
std::optional<LedChannel> LedSystemPrivate::ParseChannel(
  sdf::ElementConstPtr _sdf,
  const std::string &_modelTopicPrefix,
  const math::Pose3d &_sensorPose,
  const std::string &_sensorName,
  std::unordered_set<std::string> &_usedLedNames) const
{
  if (!_sdf->HasAttribute("name"))
  {
    gzerr << "[Foxy LED System][Channel] Missing required name attribute."
          << std::endl;
    return std::nullopt;
  }

  LedChannel channel;
  channel.name = _sdf->Get<std::string>("name");
  if (channel.name.empty())
  {
    gzerr << "[Foxy LED System][Channel] Empty channel name." << std::endl;
    return std::nullopt;
  }

  std::string commandTopic;
  if (_sdf->HasElement("topic"))
  {
    commandTopic = _sdf->Get<std::string>("topic");
  }
  commandTopic = MaybeDefaultTopic(
    commandTopic, _modelTopicPrefix, channel.name, "set");
  channel.commandTopic = transport::TopicUtils::AsValidTopic(commandTopic);
  if (channel.commandTopic.empty())
  {
    gzerr << "[Foxy LED System][Channel " << channel.name
          << "] Invalid command topic [" << commandTopic << "]."
          << std::endl;
    return std::nullopt;
  }

  std::string stateTopic;
  if (_sdf->HasElement("state_topic"))
  {
    stateTopic = _sdf->Get<std::string>("state_topic");
  }
  stateTopic = MaybeDefaultTopic(
    stateTopic, _modelTopicPrefix, channel.name, "get");
  channel.stateTopic = transport::TopicUtils::AsValidTopic(stateTopic);
  if (channel.stateTopic.empty())
  {
    gzerr << "[Foxy LED System][Channel " << channel.name
          << "] Invalid state topic [" << stateTopic << "]."
          << std::endl;
    return std::nullopt;
  }

  if (_sdf->HasElement("default_state"))
  {
    const auto defaultStateElem = _sdf->FindElement("default_state");
    if (defaultStateElem->HasElement("color"))
    {
      channel.defaultState =
        ClampColor(defaultStateElem->Get<math::Color>("color"));
    }
  }

  channel.commandedState = channel.defaultState;
  channel.appliedState = channel.defaultState;

  if (!_sdf->HasElement("led"))
  {
    gzerr << "[Foxy LED System][Channel " << channel.name
          << "] No <led> elements found." << std::endl;
    return std::nullopt;
  }

  auto ledElem = _sdf->FindElement("led");
  while (ledElem)
  {
    auto led = this->ParseLed(
      ledElem, _sensorPose, _sensorName, _usedLedNames);
    if (!led.has_value())
    {
      return std::nullopt;
    }

    channel.leds.push_back(std::move(*led));
    ledElem = ledElem->GetNextElement("led");
  }

  return channel;
}

//////////////////////////////////////////////////
bool LedSystemPrivate::CreateEntities(
  EntityComponentManager &_ecm,
  EventManager &_eventManager)
{
  SdfEntityCreator creator(_ecm, _eventManager);

  for (auto &channel : this->channels)
  {
    for (auto &led : channel.leds)
    {
      led.visualEntity = creator.CreateEntities(&led.visualSdf);
      if (led.visualEntity == kNullEntity)
      {
        gzerr << "[Foxy LED System][LED " << led.name
              << "] Failed to create visual entity." << std::endl;
        return false;
      }
      creator.SetParent(led.visualEntity, this->parentLinkEntity);

      led.lightEntity = creator.CreateEntities(&led.lightSdf);
      if (led.lightEntity == kNullEntity)
      {
        gzerr << "[Foxy LED System][LED " << led.name
              << "] Failed to create light entity." << std::endl;
        return false;
      }
      creator.SetParent(led.lightEntity, this->parentLinkEntity);

      gzmsg << "[Foxy LED System][LED " << led.name
            << "] Created visual [" << led.visualName << "] entity ["
            << led.visualEntity << "] and light [" << led.lightName
            << "] entity [" << led.lightEntity << "] on parent link entity ["
            << this->parentLinkEntity << "]." << std::endl;
    }
  }

  return true;
}

//////////////////////////////////////////////////
void LedSystemPrivate::OnCommand(
  const std::string &_channelName,
  const msgs::Color &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  auto channelIt = std::find_if(
    this->channels.begin(), this->channels.end(),
    [&_channelName](const LedChannel &_channel)
    {
      return _channel.name == _channelName;
    });

  if (channelIt == this->channels.end())
  {
    gzwarn << "[Foxy LED System] Received command for unknown channel ["
           << _channelName << "]." << std::endl;
    return;
  }

  channelIt->commandedState = FromMsg(_msg, channelIt->defaultState);
  channelIt->dirty = true;
}

//////////////////////////////////////////////////
void LedSystemPrivate::ApplyChannel(
  LedChannel &_channel,
  EntityComponentManager &_ecm)
{
  for (const auto &led : _channel.leds)
  {
    const math::Color visualColor = EffectiveVisualColor(
      _channel.commandedState, led.colorMultiplier, led.gain);
    this->SetVisualProperties(led.visualEntity, _ecm, visualColor);

    const math::Color lightColor = EffectiveLightColor(
      _channel.commandedState, led.colorMultiplier);
    const double lightIntensity =
      Clamp01(_channel.commandedState.A()) * led.gain;
    this->SetLightProperties(
      led.lightEntity, _ecm, lightColor, lightIntensity);
  }
}

//////////////////////////////////////////////////
void LedSystemPrivate::SetVisualProperties(
  Entity _visualEntity,
  EntityComponentManager &_ecm,
  const math::Color &_materialColor) const
{
  msgs::Visual visualMsg;
  visualMsg.set_id(_visualEntity);

  auto setColor = [&_materialColor](msgs::Color *_color)
  {
    _color->set_r(_materialColor.R());
    _color->set_g(_materialColor.G());
    _color->set_b(_materialColor.B());
    _color->set_a(_materialColor.A());
  };

  setColor(visualMsg.mutable_material()->mutable_ambient());
  setColor(visualMsg.mutable_material()->mutable_diffuse());
  setColor(visualMsg.mutable_material()->mutable_specular());
  setColor(visualMsg.mutable_material()->mutable_emissive());

  auto visualCmdComp = _ecm.Component<components::VisualCmd>(_visualEntity);
  if (!visualCmdComp)
  {
    _ecm.CreateComponent(_visualEntity, components::VisualCmd(visualMsg));
    return;
  }

  const auto state = visualCmdComp->SetData(visualMsg, VisualMsgsEqual) ?
    ComponentState::OneTimeChange : ComponentState::NoChange;
  _ecm.SetChanged(_visualEntity, components::VisualCmd::typeId, state);
}

//////////////////////////////////////////////////
void LedSystemPrivate::SetLightProperties(
  Entity _lightEntity,
  EntityComponentManager &_ecm,
  const math::Color &_lightColor,
  double _lightIntensity) const
{
  const auto lightComp = _ecm.Component<components::Light>(_lightEntity);
  if (!lightComp)
  {
    gzerr << "[Foxy LED System] Light entity [" << _lightEntity
          << "] has no Light component." << std::endl;
    return;
  }

  msgs::Light lightMsg;
  lightMsg.set_id(_lightEntity);
  lightMsg.set_intensity(_lightIntensity);

  auto setColor = [&_lightColor](msgs::Color *_color)
  {
    _color->set_r(_lightColor.R());
    _color->set_g(_lightColor.G());
    _color->set_b(_lightColor.B());
    _color->set_a(_lightColor.A());
  };

  setColor(lightMsg.mutable_diffuse());
  setColor(lightMsg.mutable_specular());

  lightMsg.set_range(lightComp->Data().AttenuationRange());
  lightMsg.set_attenuation_constant(
    lightComp->Data().ConstantAttenuationFactor());
  lightMsg.set_attenuation_linear(
    lightComp->Data().LinearAttenuationFactor());
  lightMsg.set_attenuation_quadratic(
    lightComp->Data().QuadraticAttenuationFactor());

  auto lightCmdComp = _ecm.Component<components::LightCmd>(_lightEntity);
  if (!lightCmdComp)
  {
    _ecm.CreateComponent(_lightEntity, components::LightCmd(lightMsg));
    return;
  }

  const auto state = lightCmdComp->SetData(lightMsg, LightMsgsEqual) ?
    ComponentState::OneTimeChange : ComponentState::NoChange;
  _ecm.SetChanged(_lightEntity, components::LightCmd::typeId, state);
}

//////////////////////////////////////////////////
void LedSystemPrivate::ResetChannels()
{
  for (auto &channel : this->channels)
  {
    channel.commandedState = channel.defaultState;
    channel.appliedState = channel.defaultState;
    channel.dirty = true;
  }
}
}  // namespace foxy_gz_plugins

GZ_ADD_PLUGIN(
  foxy_gz_plugins::LedSystem,
  gz::sim::System,
  foxy_gz_plugins::LedSystem::ISystemConfigure,
  foxy_gz_plugins::LedSystem::ISystemPreUpdate,
  foxy_gz_plugins::LedSystem::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(
  foxy_gz_plugins::LedSystem,
  "foxy_gz_plugins::LedSystem")
