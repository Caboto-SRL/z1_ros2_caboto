/*
 * Copyright 2025 IDRA, University of Trento
 * Author: Matteo Dalle Vedove (matteodv99tn@gmail.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "z1_hardware_interface/z1_hardware_interface.hpp"

#include <algorithm>
#include <ctime>
#include <fmt/format.h>
#include <memory>
#include <stdexcept>
#include <unitree_arm_sdk/control/unitreeArm.h>
#include <chrono>
#include <thread>
#include <unitree_arm_sdk/message/arm_common.h>

#include <rclcpp/duration.hpp>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

//  ____            _                 _   _
// |  _ \  ___  ___| | __ _ _ __ __ _| |_(_) ___  _ __  ___
// | | | |/ _ \/ __| |/ _` | '__/ _` | __| |/ _ \| '_ \/ __|
// | |_| |  __/ (__| | (_| | | | (_| | |_| | (_) | | | \__ \
// |____/ \___|\___|_|\__,_|_|  \__,_|\__|_|\___/|_| |_|___/
//

using unitree::z1::HardwareInterface;

static void to_lower_string(std::string& str);

template <typename Iterable>
static std::string
pretty_vector(const Iterable& vec) {
    return fmt::format("({})", fmt::join(vec, ", "));
}

static std::pair<std::string, std::string> split_interface(const std::string&);

//  ____   ____ _     ____ ____  ____    _     _  __       ____           _
// |  _ \ / ___| |   / ___|  _ \|  _ \  | |   (_)/ _| ___ / ___|   _  ___| | ___
// | |_) | |   | |  | |   | |_) | |_) | | |   | | |_ / _ \ |  | | | |/ __| |/ _ \
// |  _ <| |___| |__| |___|  __/|  __/  | |___| |  _|  __/ |__| |_| | (__| |  __/
// |_| \_\\____|_____\____|_|   |_|     |_____|_|_|  \___|\____\__, |\___|_|\___|
//                                                             |___/
hardware_interface::CallbackReturn
HardwareInterface::on_configure(const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(get_logger(), "calling on_configure()");
    if (hardware_interface::SystemInterface::on_configure(prev_state)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "parent on_configure() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }

#ifdef SHOW_DEBUG_MESSAGES
    rclcpp::Logger logger = get_logger();
    logger.set_level(rclcpp::Logger::Level::Debug);
#endif

    if (with_gripper()) RCLCPP_INFO(get_logger(), "Gripper is enabled");
    else RCLCPP_INFO(get_logger(), "Gripper is disabled");

    // TODO: load torque limits from URDF
    RCLCPP_INFO(
            get_logger(),
            "Joint torque limits: %s",
            pretty_vector(_arm_max_torque).c_str()
    );
    RCLCPP_INFO(get_logger(), "Gripper torque limit: %lf", _gripper_max_torque);

    RCLCPP_INFO(get_logger(), "Establishing connection to the ARM through SDK");
    _arm = std::make_unique<UNITREE_ARM::unitreeArm>(with_gripper());
    RCLCPP_INFO(get_logger(), "Connection established!");
    // Le transizioni FSM bloccanti dell'SDK (setFsm, backToStart) si appoggiano al
    // suo thread interno a 500 Hz. Lo usiamo SOLO per quelle e lo fermiamo prima di
    // ogni I/O sincrono: due chiamanti sullo stesso socket UDP si rubano le risposte
    // e chi resta a mani vuote paga il timeout di 20 ms della recv.
    if (!fsm_transition(UNITREE_ARM::ArmFSMState::PASSIVE, "PASSIVE")) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    read(rclcpp::Time(0), rclcpp::Duration(0, 0));

    // clang-format off
    RCLCPP_INFO(get_logger(), "Current joints configuration: %s", pretty_vector(_arm_state.q).c_str());
    RCLCPP_INFO(get_logger(), "Current joints velocity: %s", pretty_vector(_arm_state.qd).c_str());
    RCLCPP_INFO(get_logger(), "Measured joint torque: %s", pretty_vector(_arm_state.tau).c_str());
    RCLCPP_INFO(get_logger(), "Position-proportional gains: %s", pretty_vector(_default_gains.kp).c_str());
    RCLCPP_INFO(get_logger(), "Velocity-proportional gains: %s", pretty_vector(_default_gains.kd).c_str());
    // clang-format on

    // Comando iniziale = stato misurato, velocita' e coppia nulle (hold), pinza inclusa
    hold_current_state();
    if (!fsm_transition(UNITREE_ARM::ArmFSMState::LOWCMD, "LOWCMD")) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_logger(), "SDK in low-level control; the ros2_control loop is the only sendRecv caller");
    diag_start();

    RCLCPP_DEBUG(get_logger(), "on_configure() completed successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
HardwareInterface::on_cleanup(const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(get_logger(), "calling on_cleanup()");
    if (hardware_interface::SystemInterface::on_cleanup(prev_state)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "parent on_cleanup() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }
    // TODO
    RCLCPP_DEBUG(get_logger(), "on_cleanup() completed successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
HardwareInterface::on_shutdown(const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(get_logger(), "calling on_shutdown()");
    if (hardware_interface::SystemInterface::on_shutdown(prev_state)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "parent on_shutdown() failed");
    }
    if (_arm) {
        // Le transizioni FSM bloccanti si appoggiano al thread dell'SDK; sequenza
        // dell'esempio Unitree lowcmd_development.cpp: JOINTCTRL -> backToStart -> PASSIVE.
        _arm->sendRecvThread->start();
        if (_arm->setFsm(UNITREE_ARM::ArmFSMState::JOINTCTRL)) {
            RCLCPP_INFO(get_logger(), "Going back to start");
            _arm->backToStart();
        } else {
            // Senza JOINTCTRL confermato l'homing non e' sicuro: si va direttamente in PASSIVE.
            RCLCPP_WARN(get_logger(), "FSM transition to JOINTCTRL not acknowledged: skipping homing");
        }
        RCLCPP_INFO(get_logger(), "Setting arm into passive state");
        if (_arm->setFsm(UNITREE_ARM::ArmFSMState::PASSIVE))
            RCLCPP_INFO(get_logger(), "Arm PASSIVE confirmed: clean shutdown");   // riga cercata da compass_stop.sh
        else
            RCLCPP_WARN(get_logger(), "FSM transition to PASSIVE not acknowledged");
        RCLCPP_INFO(get_logger(), "Closing SDK connection");
        diag_stop();
        _arm->sendRecvThread->shutdown();
    }
    RCLCPP_DEBUG(get_logger(), "on_shutdown() completed successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
HardwareInterface::on_activate(const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(get_logger(), "calling on_activate()");
    if (hardware_interface::SystemInterface::on_activate(prev_state)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "parent on_activate() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }
    hold_current_state();  // riparte tenendo la posa misurata, senza comandi stantii
    // Verifica che il link SDK sia ancora in LOWCMD (z1_ctrl puo' essere decaduto
    // durante una lunga attesa in INACTIVE); se no, prova a rientrare.
    // Limiti noti: l'SDK precompilato non espone la freschezza della ricezione, quindi
    // recvState puo' essere stantio se la recv e' andata in timeout; e l'hold riporta
    // tutti i giunti ai guadagni di posizione, quindi un controller in velocita'/coppia
    // che sopravvivesse alla riattivazione dell'hardware dovrebbe essere riavviato.
    _arm->sendRecv();
    if (_arm->_ctrlComp->recvState.state != UNITREE_ARM::ArmFSMState::LOWCMD) {
        RCLCPP_WARN(get_logger(), "SDK not in LOWCMD at activation: re-entering");
        if (!fsm_transition(UNITREE_ARM::ArmFSMState::LOWCMD, "LOWCMD")) {
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    _active.store(true);
    RCLCPP_DEBUG(get_logger(), "on_activate() completed successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * This function should deactivate the hardware.
 */
hardware_interface::CallbackReturn
HardwareInterface::on_deactivate(const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(get_logger(), "calling on_deactivate()");
    if (hardware_interface::SystemInterface::on_deactivate(prev_state)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "parent on_deactivate() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }
    _active.store(false);
    // INACTIVE: read() continua a girare e trasmette il comando in cache -> hold esplicito
    hold_current_state();
    RCLCPP_INFO(get_logger(), "Arm deactivated: holding measured pose (qd = 0, tau = 0)");
    // TODO
    RCLCPP_DEBUG(get_logger(), "on_deactivate() completed successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * This function should handle errors.
 */
hardware_interface::CallbackReturn
HardwareInterface::on_error(const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(get_logger(), "called on_error()");
    if (hardware_interface::SystemInterface::on_error(prev_state)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "parent on_error() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (_arm) {
        // Arresto sicuro: tiene la posa (niente homing, niente PASSIVE che farebbe
        // cadere il braccio). Dopo il ritorno le read() cessano, quindi l'hold va
        // trasmesso qui, con uno scambio limitato (10 cicli da 2 ms).
        hold_current_state();
        for (int i = 0; i < 10; ++i) {
            _arm->sendRecv();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    // TODO
    RCLCPP_DEBUG(get_logger(), "on_error() processed correctly");
    return hardware_interface::CallbackReturn::SUCCESS;
}

//  _   ___        __  ___       _             __
// | | | \ \      / / |_ _|_ __ | |_ ___ _ __ / _| __ _  ___ ___
// | |_| |\ \ /\ / /   | || '_ \| __/ _ \ '__| |_ / _` |/ __/ _ \
// |  _  | \ V  V /    | || | | | ||  __/ |  |  _| (_| | (_|  __/
// |_| |_|  \_/\_/    |___|_| |_|\__\___|_|  |_|  \__,_|\___\___|
//

void HardwareInterface::hold_current_state() {
    _arm_cmd.q = _arm_state.q;
    _arm_cmd.qd.setZero();
    _arm_cmd.tau.setZero();
    _gripper_cmd.q   = _gripper_state.q;
    _gripper_cmd.qd  = 0.0;
    _gripper_cmd.tau = 0.0;
    _current_gains = _default_gains;  // guadagni di posizione: senza kp non c'e' hold
    if (!_arm) return;
    _arm->lowcmd->setControlGain(_current_gains.kp, _current_gains.kd);
    _arm->lowcmd->setGripperGain(_current_gains.kp[6], _current_gains.kd[6]);
    _arm->setArmCmd(_arm_cmd.q, _arm_cmd.qd, _arm_cmd.tau);
    if (with_gripper()) _arm->setGripperCmd(_gripper_cmd.q, _gripper_cmd.qd, _gripper_cmd.tau);
}

bool HardwareInterface::fsm_transition(UNITREE_ARM::ArmFSMState state, const char* name) {
    // setFsm() e' bloccante e si appoggia al thread interno dell'SDK: lo avviamo solo
    // per la transizione e lo fermiamo subito dopo, cosi' il loop resta l'unico
    // chiamante di sendRecv().
    _arm->sendRecvThread->start();
    const bool ok = _arm->setFsm(state);
    _arm->sendRecvThread->shutdown();
    if (!ok) RCLCPP_ERROR(get_logger(), "FSM transition to %s failed", name);
    return ok;
}

std::vector<hardware_interface::StateInterface>
HardwareInterface::export_state_interfaces() {
    using hardware_interface::HW_IF_EFFORT;
    using hardware_interface::HW_IF_POSITION;
    using hardware_interface::HW_IF_VELOCITY;

    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.reserve(21);  // NOLINT: 7 joints * 3 states
    for (long i = 0; i < 6; ++i) {
        const std::string jnt_name = joints()[i].name;
        state_interfaces.emplace_back(jnt_name, HW_IF_POSITION, &_arm_state.q(i));
        state_interfaces.emplace_back(jnt_name, HW_IF_VELOCITY, &_arm_state.qd(i));
        state_interfaces.emplace_back(jnt_name, HW_IF_EFFORT, &_arm_state.tau(i));
    }
    if (with_gripper()) {
        const std::string jnt_name = joints()[6].name;
        state_interfaces.emplace_back(jnt_name, HW_IF_POSITION, &_gripper_state.q);
        state_interfaces.emplace_back(jnt_name, HW_IF_VELOCITY, &_gripper_state.qd);
        state_interfaces.emplace_back(jnt_name, HW_IF_EFFORT, &_gripper_state.tau);
    }
    return state_interfaces;
};

std::vector<hardware_interface::CommandInterface>
HardwareInterface::export_command_interfaces() {
    using hardware_interface::HW_IF_EFFORT;
    using hardware_interface::HW_IF_POSITION;
    using hardware_interface::HW_IF_VELOCITY;

    std::vector<hardware_interface::CommandInterface> cmd_interfaces;
    cmd_interfaces.reserve(21);  // NOLINT: 7 joints * 3 cmd interfaces
    for (long i = 0; i < 6; ++i) {
        const std::string jnt_name = joints()[i].name;
        cmd_interfaces.emplace_back(jnt_name, HW_IF_POSITION, &_arm_cmd.q(i));
        cmd_interfaces.emplace_back(jnt_name, HW_IF_VELOCITY, &_arm_cmd.qd(i));
        cmd_interfaces.emplace_back(jnt_name, HW_IF_EFFORT, &_arm_cmd.tau(i));
    }
    if (with_gripper()) {
        const std::string jnt_name = joints()[6].name;
        cmd_interfaces.emplace_back(jnt_name, HW_IF_POSITION, &_gripper_cmd.q);
        cmd_interfaces.emplace_back(jnt_name, HW_IF_VELOCITY, &_gripper_cmd.qd);
        cmd_interfaces.emplace_back(jnt_name, HW_IF_EFFORT, &_gripper_cmd.tau);
    }
    return cmd_interfaces;
}

hardware_interface::return_type
HardwareInterface::
        read(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
    if (_recovering.load()) return hardware_interface::return_type::OK;   // il thread di recupero usa il socket
    _arm->sendRecv();
    // Sorveglianza del firmware: con l'hardware attivo lo stato riportato deve essere
    // LOWCMD. Se non lo e' per 100 cicli (0.2 s) e non c'e' una disconnessione in corso,
    // si chiede il riaggancio (rate-limited nel thread di diagnostica).
    if (_active.load() && !_recovering.load()) {
        const bool disc = _arm->_ctrlComp && _arm->_ctrlComp->udp && _arm->_ctrlComp->udp->isDisConnect;
        const bool lowcmd = _arm->_ctrlComp && _arm->_ctrlComp->recvState.state == UNITREE_ARM::ArmFSMState::LOWCMD;
        if (!disc && !lowcmd) { if (++_bad_state_cycles >= 100) { _bad_state_cycles = 0; request_recover("stato FSM del braccio non LOWCMD"); } }
        else _bad_state_cycles = 0;
    }
    for (long i = 0; i < 6; ++i) {
        _arm_state.q(i)   = _arm->lowstate->q[i];
        _arm_state.qd(i)  = _arm->lowstate->dq[i];
        _arm_state.tau(i) = _arm->lowstate->tau[i];
    }
    if (++_diag_counter >= 100) { _diag_counter = 0; diag_sample(); }
    if (with_gripper()) {
        _gripper_state.q   = _arm->lowstate->q[6];
        _gripper_state.qd  = _arm->lowstate->dq[6];
        _gripper_state.tau = _arm->lowstate->tau[6];
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
HardwareInterface::
        write(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
    if (_recovering.load()) return hardware_interface::return_type::OK;
    saturate_torque();
    _arm->setArmCmd(_arm_cmd.q, _arm_cmd.qd, _arm_cmd.tau);
    if (with_gripper()) _arm->setGripperCmd(_gripper_cmd.q, _gripper_cmd.qd, _gripper_cmd.tau);
    // Nessun sendRecv qui: il comando parte nel read() del ciclo successivo
    // (latenza di un periodo, 2 ms a 500 Hz) e il socket ha un solo chiamante.
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
HardwareInterface::perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces
) {
    using hardware_interface::HW_IF_EFFORT;
    using hardware_interface::HW_IF_POSITION;
    using hardware_interface::HW_IF_VELOCITY;

    RCLCPP_INFO(get_logger(), "Switching control mode");
    // Interfacce rilasciate da un controller che si ferma: quel giunto torna in hold
    // di posizione (i guadagni di velocita'/coppia lascerebbero kp = 0).
    for (const std::string& interface : stop_interfaces) {
        const auto [name, type] = split_interface(interface);
        auto idx                = get_joint_id(name);
        _current_gains.kp[idx]  = _default_gains.kp[idx];
        _current_gains.kd[idx]  = _default_gains.kd[idx];
    }
    for (const std::string& interface : start_interfaces) {
        const auto [name, type] = split_interface(interface);
        auto idx                = get_joint_id(name);

        if (type == HW_IF_POSITION) {
            _current_gains.kp[idx] = _default_gains.kp[idx];
            _current_gains.kd[idx] = _default_gains.kd[idx];
        } else if (type == HW_IF_VELOCITY) {
            _current_gains.kp[idx] = 0.0;
            _current_gains.kd[idx] = _default_gains.kd[idx];
        } else if (type == HW_IF_EFFORT) {
            _current_gains.kp[idx] = 0.0;
            _current_gains.kd[idx] = 0.0;
        } else {
            RCLCPP_ERROR(
                    get_logger(),
                    "Don't know how to configure interface '%s'",
                    interface.c_str()
            );
            return hardware_interface::return_type::ERROR;
        }
    }

    // clang-format off
    RCLCPP_INFO(get_logger(), "Updated proportional gains: %s", pretty_vector(_current_gains.kp).c_str());
    RCLCPP_INFO(get_logger(), "Updated derivative gains: %s", pretty_vector(_current_gains.kd).c_str());
    // clang-format on

    _arm->lowcmd->setControlGain(_current_gains.kp, _current_gains.kd);
    _arm->lowcmd->setGripperGain(_current_gains.kp[6], _current_gains.kd[6]);

    // Comando = posizione misurata, velocita' e coppia nulle: nessun comando stantio
    _arm_cmd.q = _arm_state.q;
    _arm_cmd.qd.setZero();
    _arm_cmd.tau.setZero();
    _gripper_cmd.q   = _gripper_state.q;
    _gripper_cmd.qd  = 0.0;
    _gripper_cmd.tau = 0.0;
    return hardware_interface::return_type::OK;
}

//  ____       _            _
// |  _ \ _ __(_)_   ____ _| |_ ___
// | |_) | '__| \ \ / / _` | __/ _ \
// |  __/| |  | |\ V / (_| | ||  __/
// |_|   |_|  |_| \_/ \__,_|\__\___|
//

void
HardwareInterface::saturate_torque() {
    const Vec6 original_tau = _arm_cmd.tau;
    _arm_cmd.tau = original_tau.cwiseMin(_arm_max_torque).cwiseMax(-_arm_max_torque);
    _gripper_cmd.tau =
            std::clamp(_gripper_cmd.tau, -_gripper_max_torque, _gripper_max_torque);

    if (original_tau != _arm_cmd.tau)
        RCLCPP_WARN(get_logger(), "Saturating input torque");
}

bool
HardwareInterface::with_gripper() const {
    std::string gripper_param = info_.hardware_parameters.at("gripper");
    to_lower_string(gripper_param);
    return gripper_param == "true";
}

long
HardwareInterface::get_joint_id(const std::string& joint_name) const {
    for (long i = 0; i < joints().size(); ++i) {
        if (joints()[i].name == joint_name) return i;
    }
    throw std::out_of_range(
            fmt::format("Unable to find joint '{}' with the joints of the robot")
    );
}

//  ____  _        _   _
// / ___|| |_ __ _| |_(_) ___ ___
// \___ \| __/ _` | __| |/ __/ __|
//  ___) | || (_| | |_| | (__\__ \
// |____/ \__\__,_|\__|_|\___|___/
//

/**
 * @bried Convert in-place a string to lower case.
 *
 * @param[in,out] str       The string to be converted.
 */
static void
to_lower_string(std::string& str) {
    std::transform(str.begin(), str.end(), str.begin(), [](unsigned char c) {
        return std::tolower(c);
    });
}

static std::pair<std::string, std::string>
split_interface(const std::string& in) {
    const auto sep_id = in.find('/');
    return std::make_pair(
            in.substr(0, sep_id), in.substr(sep_id + 1, in.size() - sep_id - 1)
    );
}

//  _____                       _
// | ____|_  ___ __   ___  _ __| |_
// |  _| \ \/ / '_ \ / _ \| '__| __|
// | |___ >  <| |_) | (_) | |  | |_
// |_____/_/\_\ .__/ \___/|_|   \__|
//            |_|
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
        unitree::z1::HardwareInterface, hardware_interface::SystemInterface
);


// ---- diagnostica motori -------------------------------------------------------------
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>

void HardwareInterface::diag_sample() {
    if (!_arm || !_arm->lowstate) return;
    std::unique_lock<std::mutex> lk(_diag_mtx, std::try_to_lock);
    if (!lk.owns_lock()) return;   // il thread di pubblicazione sta leggendo: si riprova al giro dopo
    _diag_temperature = _arm->lowstate->temperature;
    _diag_errorstate  = _arm->lowstate->errorstate;
}

void HardwareInterface::diag_start() {
    if (_diag_run.exchange(true)) return;
    _diag_thread = std::thread([this]() {
        auto node = std::make_shared<rclcpp::Node>("z1_motor_diagnostics");
        auto pub = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/z1/diagnostics", rclcpp::QoS(5));
        auto srv = node->create_service<std_srvs::srv::Trigger>("/z1/rehandshake",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                request_recover("richiesta dell'operatore"); res->success = true; res->message = "riaggancio richiesto"; });
        rclcpp::executors::SingleThreadedExecutor exec; exec.add_node(node);
        auto last_warn = std::chrono::steady_clock::now() - std::chrono::seconds(10);
        while (_diag_run.load() && rclcpp::ok()) {
            std::vector<int> temp; std::vector<uint8_t> err;
            { std::lock_guard<std::mutex> lk(_diag_mtx); temp = _diag_temperature; err = _diag_errorstate; }
            diagnostic_msgs::msg::DiagnosticArray arr;
            arr.header.stamp = node->now();
            bool any_error = false; std::string summary;
            for (size_t i = 0; i < temp.size() && i < 7; ++i) {
                diagnostic_msgs::msg::DiagnosticStatus st;
                st.name = "z1/motor" + std::to_string(i + 1);
                st.hardware_id = "unitree_z1";
                const uint8_t e = i < err.size() ? err[i] : 0;
                st.level = (e & 0x04) ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                         : (e != 0)  ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                                     : diagnostic_msgs::msg::DiagnosticStatus::OK;
                st.message = (e & 0x04) ? "avvolgimenti surriscaldati (0x04)" : (e != 0) ? "errore motore 0x" + std::to_string(e) : "ok";
                diagnostic_msgs::msg::KeyValue kt; kt.key = "temperature_C"; kt.value = std::to_string(temp[i]);
                diagnostic_msgs::msg::KeyValue ke; ke.key = "errorstate"; ke.value = std::to_string(e);
                st.values = {kt, ke};
                arr.status.push_back(st);
                if (e != 0) { any_error = true; summary += " motore" + std::to_string(i + 1) + "=0x" + std::to_string(e) + "(" + std::to_string(temp[i]) + "C)"; }
            }
            pub->publish(arr);
            exec.spin_some(std::chrono::milliseconds(50));
            if (_recover_request.load()) do_recover();
            const auto now = std::chrono::steady_clock::now();
            if (any_error && now - last_warn > std::chrono::seconds(5)) {
                last_warn = now;
                RCLCPP_WARN(get_logger(), "errore motori:%s", summary.c_str());
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });
}

void HardwareInterface::diag_stop() {
    if (!_diag_run.exchange(false)) return;
    if (_diag_thread.joinable()) _diag_thread.join();
}

void HardwareInterface::request_recover(const char* why) {
    { std::lock_guard<std::mutex> lk(_recover_mtx); _recover_why = why; }
    _recover_request.store(true);
}

void HardwareInterface::do_recover() {
    _recover_request.store(false);
    const auto now = std::chrono::steady_clock::now();
    if (now - _last_recover < std::chrono::seconds(5)) return;   // al massimo uno ogni 5 s
    _last_recover = now;
    std::string why; { std::lock_guard<std::mutex> lk(_recover_mtx); why = _recover_why; }
    RCLCPP_WARN(get_logger(), "riaggancio del braccio (%s): il ciclo real-time sospende sendRecv, transizione a LOWCMD", why.c_str());
    _recovering.store(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));   // il ciclo RT vede il flag
    const bool ok = fsm_transition(UNITREE_ARM::ArmFSMState::LOWCMD, "LOWCMD");
    if (ok) hold_current_state();   // comando = stato misurato: nessuno scatto alla ripresa
    _recovering.store(false);
    if (ok) RCLCPP_WARN(get_logger(), "riaggancio riuscito: braccio di nuovo in LOWCMD, hold sulla posa misurata");
    else RCLCPP_ERROR(get_logger(), "riaggancio FALLITO: il braccio non risponde alla transizione (spento? acceso da poco?)");
}
