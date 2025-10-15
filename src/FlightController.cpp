#include "FlightController.hpp"
#include <iostream>

namespace fcu {

FlightController::FlightController() :
    msp_version_(2), control_source_(ControlSource::NONE) {}

FlightController::~FlightController() { disconnect(); }

bool FlightController::connect(const std::string &device, const size_t baudrate,
                               const double &timeout, const bool print_info) {
    if(!client_.start(device, baudrate)) return false;

    msp::msg::FcVariant fcvar(fw_variant_);
    if(client_.sendMessage(fcvar, 1.0) && !fcvar.identifier().empty()) {
        fw_variant_ = msp::variant_map.at(fcvar.identifier());
        if(print_info) std::cout << fcvar;
    }
    else {
        std::cerr
            << "cannot determine the flight controller variant (MSP_FC_VARIANT)"
            << std::endl;
        return false;
    }

    if(fw_variant_ != msp::FirmwareVariant::MWII) {
        msp::msg::ApiVersion api_version(fw_variant_);
        if(client_.sendMessage(api_version, timeout)) {
            if(print_info) std::cout << api_version;
        }
    }

    if(print_info) {
        msp::msg::FcVersion fcver(fw_variant_);
        if(client_.sendMessage(fcver, timeout)) {
            std::cout << fcver;
        }
    }

    msp::msg::BoardInfo boardinfo(fw_variant_);
    if(client_.sendMessage(boardinfo, timeout)) {
        if(print_info) std::cout << boardinfo;
        board_name_ = boardinfo.name();
    }

    if(print_info) {
        msp::msg::BuildInfo buildinfo(fw_variant_);
        if(client_.sendMessage(buildinfo, timeout)) std::cout << buildinfo;
    }

    msp::msg::Status status(fw_variant_);
    client_.sendMessage(status, timeout);
    if(print_info) std::cout << status;
    sensors_ = status.sensors;

    // get boxes
    initBoxes();

    // determine channel mapping
    if(getFwVariant() == msp::FirmwareVariant::MWII) {
        // default mapping
        for(uint8_t i(0); i < msp::msg::MAX_MAPPABLE_RX_INPUTS; ++i) {
            channel_map_[i] = i;
        }
    }
    else {
        // get channel mapping from MSP_RX_MAP
        msp::msg::RxMap rx_map(fw_variant_);
        client_.sendMessage(rx_map, timeout);
        if(print_info) std::cout << rx_map;
        channel_map_ = rx_map.map;
    }

    return true;
}

bool FlightController::disconnect() { return client_.stop(); }

bool FlightController::isConnected() const { return client_.isConnected(); }

void FlightController::setLoggingLevel(const msp::client::LoggingLevel &level) {
    client_.setLoggingLevel(level);
}

void FlightController::setFlightMode(FlightMode) {
    // unimplemented
    assert(false);
}

FlightMode FlightController::getFlightMode() const { return flight_mode_; }

void FlightController::setControlSource(ControlSource source) {
    if(source == control_source_) return;

    msp::msg::RxConfig rxConfig(fw_variant_);
    if(!client_.sendMessage(rxConfig))
        std::cout << "client_.sendMessage(rxConfig) failed" << std::endl;
    std::cout << rxConfig;

    msp::msg::SetRxConfig setRxConfig(fw_variant_);
    static_cast<msp::msg::RxConfigSettings &>(setRxConfig) =
        static_cast<msp::msg::RxConfigSettings &>(rxConfig);

    if(source == ControlSource::SBUS) {
        setRxConfig.receiverType      = 3;
        setRxConfig.serialrx_provider = 2;
    }
    else if(source == ControlSource::MSP) {
        setRxConfig.receiverType = 4;
    }
    client_.sendMessage(setRxConfig);

    control_source_ = source;
}

ControlSource FlightController::getControlSource() {
    msp::msg::RxConfig rxConfig(fw_variant_);
    client_.sendMessage(rxConfig);

    if(rxConfig.receiverType.set() && rxConfig.receiverType() == 4)
        return ControlSource::MSP;
    else if(rxConfig.serialrx_provider() == 2)
        return ControlSource::SBUS;
    return ControlSource::OTHER;
}

void FlightController::setRPYT(std::array<double, 4> &&) {
    // unimplemented
    assert(false);
}

bool FlightController::saveSettings() {
    msp::msg::WriteEEPROM writeEEPROM(fw_variant_);
    return client_.sendMessage(writeEEPROM);
}

bool FlightController::reboot() {
    msp::msg::Reboot reboot(fw_variant_);
    return client_.sendMessage(reboot);
}

msp::FirmwareVariant FlightController::getFwVariant() const {
    return fw_variant_;
}

int FlightController::getProtocolVersion() const { return msp_version_; }

std::string FlightController::getBoardName() const { return board_name_; }

void FlightController::initBoxes() {
    // get box names
    msp::msg::BoxNames box_names(fw_variant_);
    if(!client_.sendMessage(box_names))
        throw std::runtime_error("Cannot get BoxNames!");

    // get box IDs
    msp::msg::BoxIds box_ids(fw_variant_);
    if(!client_.sendMessage(box_ids))
        throw std::runtime_error("Cannot get BoxIds!");
    assert(box_names.box_names.size() == box_ids.box_ids.size());

    box_name_ids_.clear();
    for(size_t ibox(0); ibox < box_names.box_names.size(); ibox++) {
        if(getFwVariant() == msp::FirmwareVariant::CLFL) {
            // workaround for wrong box ids in cleanflight
            // cleanflight's box ids are in order of the box names
            // https://github.com/cleanflight/cleanflight/issues/2606
            box_name_ids_[box_names.box_names[ibox]] = ibox;
        }
        else {
            box_name_ids_[box_names.box_names[ibox]] = box_ids.box_ids[ibox];
        }
    }
}

bool FlightController::isStatusActive(const std::string &status_name,
                                      const double &timeout) {
    if(box_name_ids_.count(status_name) == 0) {
        // box ids have not been initialised or requested status is unsupported
        // by FC
        throw std::runtime_error(
            "Box ID of " + status_name +
            " is unknown! You need to call 'initBoxes()' first.");
    }

    msp::msg::Status status(fw_variant_);
    client_.sendMessage(status, timeout);

    // check if ARM box id is amongst active box IDs
    return status.box_mode_flags.count(box_name_ids_.at(status_name));
}

bool FlightController::setRc(const uint16_t roll, const uint16_t pitch,
                             const uint16_t yaw, const uint16_t throttle,
                             const uint16_t aux1, const uint16_t aux2,
                             const uint16_t aux3, const uint16_t aux4) {
    msp::msg::SetRawRc rc(fw_variant_);
    // insert mappable channels
    rc.channels[channel_map_[0]] = roll;
    rc.channels[channel_map_[1]] = pitch;
    rc.channels[channel_map_[2]] = yaw;
    rc.channels[channel_map_[3]] = throttle;

    rc.channels[4] = aux1;
    rc.channels[5] = aux2;
    rc.channels[6] = aux3;
    rc.channels[7] = aux4;

    return client_.sendMessageNoWait(rc);
}

bool FlightController::setRc(
    std::array<uint16_t, msp::msg::MAX_SUPPORTED_RC_CHANNEL_COUNT> channels) {
    msp::msg::SetRawRc rc(fw_variant_);
    rc.channels = channels;
    return client_.sendMessageNoWait(rc);
}

bool FlightController::setRxMap(
    std::array<uint8_t, msp::msg::MAX_MAPPABLE_RX_INPUTS> map) {
    msp::msg::SetRxMap rxMap(fw_variant_);
    rxMap.map      = map;
    const auto res = client_.sendMessageNoWait(rxMap);
    if(res) {
        channel_map_ = map;
    }
    return res;
}

bool FlightController::setMotors(
    const std::array<uint16_t, msp::msg::N_MOTOR> &motor_values) {
    msp::msg::SetMotor motor(fw_variant_);
    motor.motor = motor_values;
    return client_.sendMessage(motor);
}

int FlightController::updateFeatures(const std::set<std::string> &add,
                                     const std::set<std::string> &remove) {
    // get original feature configuration
    msp::msg::Feature feature_in(fw_variant_);
    if(!client_.sendMessage(feature_in)) return -1;

    // update feature configuration
    msp::msg::SetFeature feature_out(fw_variant_);
    feature_out.features = feature_in.features;
    // enable features
    for(const std::string &a : add) {
        feature_out.features.insert(a);
    }
    // disable features
    for(const std::string &rem : remove) {
        feature_out.features.erase(rem);
    }

    // check if feature configuration changed
    if(feature_out.features == feature_in.features) return 0;

    if(!client_.sendMessage(feature_out)) return -1;

    // make settings permanent and reboot
    if(!saveSettings()) return -1;
    if(!reboot()) return -1;

    return 1;
}

}  // namespace fcu
