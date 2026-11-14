// SuperMimic startup integration probe: real upstream control/estimation/inference plugins.
// No hardware plugin is instantiated; outputs are returned over a JSON-lines pipe.
#include <rmcs_executor/component.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <eigen3/Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <array>
#include <iostream>
#include <numbers>
#include <set>
using Json = nlohmann::json;
namespace rmcs_executor {
// The production Component exposes interface wiring to its Executor friend.
// This test executor binds the same declarations and performs the same topological ordering,
// while allowing the simulator to supply each sensor step without a real-time hardware thread.
class Executor {
public:
    void add(std::shared_ptr<Component> c) {
        components_.push_back(c);
        for (auto p:c->partner_component_list_) add(p);
    }
    void init() {
        Component::OutputInfoMap info;
        for (auto c:components_) for (auto& o:c->output_list_) {
            if (!outputs_.emplace(o.name,&o).second) throw std::runtime_error("Duplicate output: "+o.name);
            info.emplace(o.name,Component::OutputInfo{o.type,o.kind});
        }
        for (auto c:components_) c->before_pairing(info);
        for (auto c:components_) for (auto& i:c->input_list_) {
            auto it=outputs_.find(i.name);
            if (it==outputs_.end()) {
                if (i.required) throw std::runtime_error("Missing input: "+i.name);
                continue;
            }
            auto& o=*it->second;
            if (i.type!=o.type || i.kind!=o.kind) throw std::runtime_error("Type mismatch: "+i.name);
            i.bind(i.binding,o.binding);
            deps_[c.get()].insert(o.component);
        }
        std::set<Component*> visiting,done;
        std::function<void(Component*)> visit=[&](Component* c) {
            if (done.contains(c)) return;
            if (!visiting.insert(c).second) throw std::runtime_error("Dependency cycle");
            for (auto d:deps_[c]) visit(d);
            visiting.erase(c);done.insert(c);order_.push_back(c);
        };
        for (auto c:components_) visit(c.get());
        for (auto c:order_) c->before_updating();
    }
    void update() { for (auto c:order_) c->update(); }
    template<typename T> const T& read(const std::string& path) const {
        auto o=outputs_.at(path);
        if (o->type!=typeid(T)) throw std::runtime_error("Read type mismatch: "+path);
        return *static_cast<T*>(o->binding);
    }
private:
    std::vector<std::shared_ptr<Component>> components_;
    std::map<std::string,Component::OutputDeclaration*> outputs_;
    std::map<Component*,std::set<Component*>> deps_;
    std::vector<Component*> order_;
};
}
class SyntheticSensors final: public rmcs_executor::Component {
public:
    using Joints=std::array<double,6>;
    SyntheticSensors() {
        register_output("/predefined/update_count",tick,std::size_t{0});
        register_output("/predefined/update_rate",rate,1000.);
        register_output("/predefined/timestamp",stamp,std::chrono::steady_clock::now());
        register_output("/wheel_leg/feedback_available",available,true);
        register_output("/wheel_leg/feedback_fresh",fresh,true);
        register_output("/wheel_leg/dm_control_ready",drives,true);
        register_output("/wheel_leg/dr16_fresh",remote,true);
        register_output("/wheel_leg/imu/quaternion",quat,Eigen::Quaterniond::Identity());
        register_output("/wheel_leg/imu/angular_velocity",gyro,Eigen::Vector3d::Zero());
        register_output("/wheel_leg/imu/acceleration",acceleration,Eigen::Vector3d(0.,0.,9.81));
        register_output("/remote/switch/left",left,rmcs_msgs::Switch::DOWN);
        register_output("/remote/switch/right",right,rmcs_msgs::Switch::DOWN);
        register_output("/remote/joystick/right",stick,Eigen::Vector2d::Zero());
        register_output("/remote/rotary_knob",knob,0.);
        register_output("/remote/keyboard",keyboard,rmcs_msgs::Keyboard::zero());
        register_output("/gimbal/yaw/angle",gimbal,0.);
        for (int i=0;i<6;++i) {
            const auto prefix=std::string("/wheel_leg/")+names[i];
            register_output(prefix+"/angle",angle[i],0.);
            register_output(prefix+"/velocity",speed[i],0.);
            if (i<4)
                register_output(prefix+"/angle_wrap_span",wrap_span[i],2.*std::numbers::pi);
            register_output(prefix+"/max_torque",caps[i],i<4?40.:4.5);
        }
    }
    void set(const Json& j) {
        const Joints offsets{1.6,3.0282761689183353,-1.6,-3.0283102010168,0.,0.};
        const auto q=j.value("q",Joints{-.42,.13722330866417465,.42,-.13721605817245947,0.,0.});
        const auto dq=j.value("dq",Joints{});
        for (int i=0;i<6;++i) { *angle[i]=i<4?-q[i]+offsets[i]:q[i]; *speed[i]=i<4?-dq[i]:dq[i]; }
        auto rotation=j.value("quat",std::array{1.,0.,0.,0.});
        *quat=Eigen::Quaterniond(rotation[0],rotation[1],rotation[2],rotation[3]);
        auto w=j.value("gyro",std::array{0.,0.,0.});*gyro=Eigen::Vector3d(w[0],w[1],w[2]);
        *fresh=j.value("fresh",true);*available=j.value("available",true);
        *drives=j.value("drives",true);*remote=j.value("remote",true);
        *left=static_cast<rmcs_msgs::Switch>(j.value("left",3));
        *right=static_cast<rmcs_msgs::Switch>(j.value("right",3));
        auto js=j.value("stick",std::array{0.,0.});*stick=Eigen::Vector2d(js[0],js[1]);
        *knob=j.value("knob",0.);*gimbal=j.value("gimbal",0.);
        *keyboard=rmcs_msgs::Keyboard::zero();
        for (auto key:j.value("keys",std::vector<std::string>{})) {
            if(key=="q")keyboard->q=true;
            if(key=="e")keyboard->e=true;
            if(key=="c")keyboard->c=true;
            if(key=="w")keyboard->w=true;
            if(key=="s")keyboard->s=true;
            if(key=="a")keyboard->a=true;
            if(key=="d")keyboard->d=true;
        }
    }
    void update() override { ++*tick; *stamp=std::chrono::steady_clock::now(); }
    static constexpr std::array<const char*,6> names{"left_hip_joint","left_knee_joint","right_hip_joint","right_knee_joint","left_wheel","right_wheel"};
private:
    OutputInterface<std::size_t> tick;
    OutputInterface<double> rate,knob,gimbal;
    OutputInterface<std::chrono::steady_clock::time_point> stamp;
    OutputInterface<bool> available,fresh,drives,remote;
    OutputInterface<Eigen::Quaterniond> quat;
    OutputInterface<Eigen::Vector3d> gyro,acceleration;
    OutputInterface<Eigen::Vector2d> stick;
    OutputInterface<rmcs_msgs::Switch> left,right;
    OutputInterface<rmcs_msgs::Keyboard> keyboard;
    std::array<OutputInterface<double>,6> angle,speed,caps;
    std::array<OutputInterface<double>,4> wrap_span;
};
int main(int argc,char** argv) {
    rclcpp::init(argc,argv);
    try {
        pluginlib::ClassLoader<rmcs_executor::Component> loader("rmcs_executor","rmcs_executor::Component");
        rmcs_executor::Executor executor;
        rmcs_executor::Component::initializing_component_name="synthetic_sensors";
        auto sensors=std::make_shared<SyntheticSensors>();executor.add(sensors);
        for (auto [type,name]:std::array{
            std::pair{"rmcs_core::controller::chassis::WheelLegChassisController","wheel_leg_chassis_controller"},
            std::pair{"rmcs_core::controller::chassis::WheelLegStateEstimator","wheel_leg_state_estimator"},
            std::pair{"rmcs_core::controller::chassis::WheelLegRlController","wheel_leg_rl_controller"},
            std::pair{"rmcs_core::controller::chassis::WheelLegObservation","wheel_leg_observation"},
            std::pair{"rmcs_rl::RlBridge","rl_bridge"}}) {
            rmcs_executor::Component::initializing_component_name=name;
            executor.add(loader.createSharedInstance(type));
        }
        executor.init();
        std::cout<<"READY"<<std::endl;
        std::string line;
        while (std::getline(std::cin,line)) {
            Json j=Json::parse(line);sensors->set(j);
            for (int i=0;i<j.value("steps",5);++i) executor.update();
            using JV=std::array<double,6>;
            Json out;
            for (auto [key,path]:std::array{std::pair{"state","/wheel_leg/rl/state"},
                std::pair{"phase","/wheel_leg/recovery/phase"},
                std::pair{"recovery_fault","/wheel_leg/recovery/fault"},
                std::pair{"blockers","/wheel_leg/recovery/capture_blockers"}})
                out[key]=executor.read<int>(path);
            for (auto [key,path]:std::array{std::pair{"fault","/wheel_leg/rl/fault_latched"},
                std::pair{"enabled","/wheel_leg/enable_request"},std::pair{"policy","/wheel_leg/rl/input/enable"},
                std::pair{"armed","/chassis/control_enable"},std::pair{"valid","/wheel_leg/state/valid"}})
                out[key]=executor.read<bool>(path);
            out["mode"]=static_cast<int>(executor.read<rmcs_msgs::ChassisMode>("/chassis/control_mode"));
            auto cmd=executor.read<Eigen::Vector3d>("/wheel_leg/rl/input/command");
            out["cmd"]=std::array{cmd.x(),cmd.y(),cmd.z()};
            out["height"]=executor.read<double>("/wheel_leg/rl/input/height");
            out["requested_height"]=executor.read<double>("/chassis/control_height");
            out["raw"]=executor.read<JV>("/wheel_leg/rl/applied_action");
            out["q"]=executor.read<JV>("/wheel_leg/state/position");
            out["dq"]=executor.read<JV>("/wheel_leg/state/velocity");
            out["frame_seq"]=executor.read<std::size_t>("/wheel_leg/rl/input/frame_sequence");
            out["action_seq"]=executor.read<std::size_t>("/wheel_leg/rl/action_seq");
            out["age"]=executor.read<double>("/wheel_leg/rl/action_age");
            out["inference_us"]=executor.read<double>("/wheel_leg/rl/performance/inference_us");
            out["torque"]=Json::array();
            for (auto name:SyntheticSensors::names) {
                const auto v=executor.read<double>(std::string("/wheel_leg/")+name+"/control_torque");
                out["torque"].push_back(v);
            }
            if(j.value("frame",false)) {
                out["frame"]=executor.read<std::vector<double>>("/wheel_leg/rl/input/frame");
                out["source_frame"]=executor.read<std::vector<double>>("/wheel_leg/rl/input/supermimic_frame");
            }
            out["mapped"]=Json::array();out["actor_raw"]=Json::array();
            out["scale"]=Json::array();out["offset"]=Json::array();
            for (auto name:SyntheticSensors::names) {
                const auto base=std::string("/wheel_leg/rl/raw/")+name;
                out["mapped"].push_back(executor.read<double>(base));
                out["actor_raw"].push_back(executor.read<double>(base+"/raw"));
                out["scale"].push_back(executor.read<double>(base+"/scale"));
                out["offset"].push_back(executor.read<double>(base+"/offset"));
            }
            std::cout<<out.dump()<<std::endl;
        }
        rclcpp::shutdown();
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1; }
}
