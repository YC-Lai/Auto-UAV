#include "auto_uav/gcsGui.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(gcsGui::gcsGui, rviz::Panel)

namespace gcsGui {
gcsGui::gcsGui(QWidget* parent)
    : rviz::Panel(parent),
      ui_(std::make_shared<Ui::GCS>()),
      logi_(std::make_unique<gcsLogi>()) {
    // Extend the widget with all attributes and children from UI file
    ui_->setupUi(this);

    QObject::connect(ui_->pb_connect, SIGNAL(clicked()), this, SLOT(connect()));
    QObject::connect(ui_->pb_arm, SIGNAL(clicked()), this, SLOT(arm()));
    QObject::connect(ui_->pb_disarm, SIGNAL(clicked()), this, SLOT(disarm()));
    QObject::connect(ui_->pb_takeoff, SIGNAL(clicked()), this, SLOT(takeoff()));
    QObject::connect(ui_->pb_land, SIGNAL(clicked()), this, SLOT(land()));
    QObject::connect(ui_->pb_keyboard, SIGNAL(clicked()), this, SLOT(keyboard()));
    QObject::connect(ui_->pb_planner, SIGNAL(clicked()), this, SLOT(planner()));
    QObject::connect(ui_->pb_kill, SIGNAL(clicked()), this, SLOT(kill()));

    // editing text board is not permitted
    ui_->textEdit_console->setReadOnly(true);
    textEdit_write("Welcome pilot. Have a safe flight. \n");
}

void gcsGui::connect() {
    auto_uav_msgs::Connect connSrv;
    bool is_connected = logi_->connect_client.call(connSrv) && connSrv.response.is_success;
    if (is_connected) {
        textEdit_write("connection estabilished");
        logi_->is_connected = true;
    } else {
        textEdit_write("connection failed");
    }
}

void gcsGui::arm() {
    if (logi_->is_connected) {
        mavros_msgs::CommandBool cmdSrv;
        mavros_msgs::SetMode modeSrv;
        cmdSrv.request.value = true;
        modeSrv.request.custom_mode = "OFFBOARD";

        bool is_armed = logi_->arm_client.call(cmdSrv) && cmdSrv.response.success;
        bool is_offboard = logi_->set_mode_client.call(modeSrv) && modeSrv.response.mode_sent;

        if (is_armed) {
            textEdit_write("Armed");
        } else {
            textEdit_write("Arming requested but service call failed");
        }

        if (is_offboard) {
            textEdit_write("Offboard enabled");
        } else {
            textEdit_write("requested offboard but failed");
        }
    } else {
        textEdit_write("please invoke setmode after ros connect");
    }
}

void gcsGui::disarm() {
    if (logi_->is_connected) {
        mavros_msgs::CommandBool cmdSrv;
        cmdSrv.request.value = false;

        bool is_disarmed = logi_->arm_client.call(cmdSrv) && cmdSrv.response.success;

        if (is_disarmed) {
            textEdit_write("Disarmed");
        } else {
            textEdit_write("Disarming requested but service call failed");
        }
    } else {
        textEdit_write("please invoke setmode after ros connect");
    }
}

void gcsGui::takeoff() {
    if (logi_->is_connected) {
        auto_uav_msgs::Takeoff takeoffSrv;
        takeoffSrv.request.height = logi_->altitude;
        double takeoffSpeed = logi_->speed;
        if (takeoffSpeed <= 0) {
            textEdit_write("takeoff speed is below zero. resetting 0.05 [m/s]");
            takeoffSpeed = 0.05;
        }

        takeoffSrv.request.speed = takeoffSpeed;
        bool is_takeoff = logi_->takeoff_client.call(takeoffSrv) && takeoffSrv.response.is_success;
        if (is_takeoff) {
            textEdit_write("Vehicle Takeoff");
        }
    } else {
        textEdit_write("please invoke takeoff after ros connect");
    }
}

void gcsGui::land() {
    if (logi_->is_connected) {
        auto_uav_msgs::Land landSrv;
        double landGround = -1;
        double landSpeed = logi_->speed;

        if (landGround > 0)
            textEdit_write("landing ground is larger than zero.");
        if (landSpeed <= 0) {
            textEdit_write("landing speed is below zero. resetting 0.05 [m/s]");
            landSpeed = 0.05;
        }

        landSrv.request.ground = landGround;
        landSrv.request.speed = landSpeed;
        bool is_land = logi_->land_client.call(landSrv) && landSrv.response.is_success;
        if (is_land) {
            textEdit_write("Vehicle land");
            logi_->is_connected = false;
        }
    } else {
        textEdit_write("please invoke land after ros connect");
    }
}

void gcsGui::keyboard() {
}

void gcsGui::planner() {
}

void gcsGui::kill() {
}

void gcsGui::textEdit_write(QString text) {
    ui_->textEdit_console->append(text);
}

/**
     *  Save all configuration data from this panel to the given
     *  Config object. It is important here that you call save()
     *  on the parent class so the class id and panel name get saved.
     */
void gcsGui::save(rviz::Config config) const {
    rviz::Panel::save(config);
}

/**
     *  Load all configuration data for this panel from the given Config object.
     */
void gcsGui::load(const rviz::Config& config) {
    rviz::Panel::load(config);
}
}  // namespace gcsGui
