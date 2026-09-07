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
#ifndef UNITREE_Z1_HW_INTERFACE_HPP__
#define UNITREE_Z1_HW_INTERFACE_HPP__

#include <Eigen/Dense>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "unitree_arm_sdk/control/unitreeArm.h"

namespace unitree::z1 {

class HardwareInterface : public hardware_interface::SystemInterface {
public:
    using Vec6   = Eigen::Vector<double, 6>;  // NOLINT: magic number
    using ArmPtr = std::unique_ptr<UNITREE_ARM::unitreeArm>;

    RCLCPP_SHARED_PTR_DEFINITIONS(HardwareInterface)

    HardwareInterface()           = default;
HardwareInterface(const HardwareInterface&)             = delete;
    HardwareInterface(const HardwareInterface&&)            = delete;
    HardwareInterface& operator=(const HardwareInterface&)  = delete;
    HardwareInterface& operator=(const HardwareInterface&&) = delete;


    hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State& prev_state
    ) override;

    hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State& prev_state
    ) override;

    hardware_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State& prev_state
    ) override;

    hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State& prev_state
    ) override;

    hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State& prev_state
    ) override;

    hardware_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State& prev_state
    ) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface>

    export_command_interfaces() override;

    hardware_interface::return_type read(
            const rclcpp::Time& time, const rclcpp::Duration& period
    ) override;

    hardware_interface::return_type write(
            const rclcpp::Time& time, const rclcpp::Duration& period
    ) override;

    hardware_interface::return_type perform_command_mode_switch(
            const std::vector<std::string>& start_interfaces,
            const std::vector<std::string>& stop_interfaces
    ) override;

    // clang-format off
    [[nodiscard]] std::vector<hardware_interface::ComponentInfo>& joints() { return info_.joints; }
    [[nodiscard]] const std::vector<hardware_interface::ComponentInfo>& joints() const { return info_.joints; }

    [[nodiscard]] rclcpp::Logger& get_logger() { return _logger; }
    
    [[nodiscard]] bool with_gripper() const;
    /// Comando = stato misurato, qd = 0, tau = 0 (braccio e pinza); inoltra all'SDK se connesso.
    void hold_current_state();
    /// Transizione FSM bloccante dell'SDK, con il suo thread avviato solo per la durata della chiamata.
    bool fsm_transition(UNITREE_ARM::ArmFSMState state, const char* name);
    // clang-format on


private:
    rclcpp::Logger _logger = rclcpp::get_logger("z1_hardware_interface");

    // Diagnostica motori (temperatura ed errorstate dell'SDK) pubblicata da un nodo
    // separato su un thread proprio: il ciclo real-time copia i valori sotto mutex
    // ogni ~100 cicli, il thread li pubblica a 1 Hz su /z1/diagnostics e avvisa
    // (limitato) quando un motore segnala un errore (0x04 = surriscaldamento).
    ~HardwareInterface() override;
    void diag_start();
    void diag_stop();
    void diag_sample();
    // Riaggancio automatico, TUTTO nel ciclo real-time (nessun thread tocca l'SDK):
    // se, con l'hardware attivo, lo stato FSM riportato dal braccio non e' LOWCMD
    // (firmware ripartito in PASSIVE dopo una perdita UDP o uno spegnimento), read()
    // chiede LOWCMD nel comando UDP (come fa setFsm dell'SDK) finche' il braccio non
    // lo conferma, poi il comando riparte dalla posa MISURATA. Il servizio
    // /z1/rehandshake alza solo un flag. Il limitatore di passo in write() rende
    // comunque limitato qualunque salto fra comando e posa reale.
    void request_recover(const char* why);
    void recover_step();            // chiamata da read() con il socket in mano
    void slew_limit_cmd();          // chiamata da write(): passo massimo per ciclo
    std::atomic<bool> _active{false};
    std::atomic<bool> _recover_request{false};
    bool _recovering = false;                       // solo dal ciclo RT
    unsigned _recover_cycles = 0;
    std::atomic<int> _recover_backoff{0};           // cicli di attesa dopo un tentativo fallito
    std::string _recover_why;
    std::mutex _recover_mtx;
    unsigned _bad_state_cycles = 0;
    Vec6 _last_sent_q = Vec6::Zero();
    bool _last_sent_valid = false;
    double _max_cmd_step_rad = 0.002;               // 1 rad/s a 500 Hz: mai raggiunto dai controller normali
    std::thread _diag_thread;
    std::atomic<bool> _diag_run{false};
    std::mutex _diag_mtx;
    std::vector<int> _diag_temperature;
    std::vector<uint8_t> _diag_errorstate;
    unsigned _diag_counter = 0;

    ArmPtr _arm = nullptr;

    Vec6   _arm_max_torque     = 20.0 * Vec6::Ones();
    double _gripper_max_torque = 20.0;

    struct GainsData {
        std::vector<double> kp = {20.0, 30.0, 30.0, 20.0, 15.0, 10.0, 20.0};
        std::vector<double> kd = {2000, 2000, 2000, 2000, 2000, 2000, 2000};
    };

    GainsData _default_gains;
    GainsData _current_gains;

    struct {
        Vec6 q   = Vec6::Zero();
        Vec6 qd  = Vec6::Zero();
        Vec6 tau = Vec6::Zero();
    } _arm_state;

    struct {
        double q   = 0;
        double qd  = 0;
        double tau = 0;
    } _gripper_state;

    struct {
        Vec6 q   = Vec6::Zero();
        Vec6 qd  = Vec6::Zero();
        Vec6 tau = Vec6::Zero();
    } _arm_cmd;

    struct {
        double q   = 0;
        double qd  = 0;
        double tau = 0;
    } _gripper_cmd;

    void saturate_torque();

    long get_joint_id(const std::string& joint_name) const;
};


}  // namespace unitree::z1

#endif  // UNITREE_Z1_HW_INTERFACE_HPP__
