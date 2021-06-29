#ifndef GCSGUI_GCSGUI_H
#define GCSGUI_GCSGUI_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#endif

/** 
 *  Include header generated from ui file
 *  Note that you will need to use add_library function first
 *  in order to generate the header file from ui.
 */
#include <ui_gcsGui.h>
#include <auto_uav/gcsLogi.hpp>

namespace gcsGui {
/**
     *  Here we declare our new subclass of rviz::Panel. Every panel which
     *  can be added via the Panels/Add_New_Panel menu is a subclass of
     *  rviz::Panel.
     */

class gcsGui : public rviz::Panel {
    /**
         * This class uses Qt slots and is a subclass of QObject, so it needs
         * the Q_OBJECT macro.
         */
    Q_OBJECT
   public:
    /**
             *  QWidget subclass constructors usually take a parent widget
             *  parameter (which usually defaults to 0).  At the same time,
             *  pluginlib::ClassLoader creates instances by calling the default
             *  constructor (with no arguments). Taking the parameter and giving
             *  a default of 0 lets the default constructor work and also lets
             *  someone using the class for something else to pass in a parent
             *  widget as they normally would with Qt.
             */
    gcsGui(QWidget* parent = 0);

    /**
             *  Now we declare overrides of rviz::Panel functions for saving and
             *  loading data from the config file.  Here the data is the topic name.
             */
    virtual void save(rviz::Config config) const;
    virtual void load(const rviz::Config& config);

    /**
         *  Next come a couple of public Qt Slots.
         */
   public Q_SLOTS:

    /**
         *  Here we declare some internal slots.
         */
   protected Q_SLOTS:

    void connect();
    void arm();
    void disarm();
    void takeoff();
    void land();
    void keyboard();
    void planner();
    void kill();

    /**
         *  Finally, we close up with protected member variables
         */
   protected:
    // UI pointer
    std::shared_ptr<Ui::GCS> ui_;
    std::unique_ptr<gcsLogi> logi_;
    void textEdit_write(QString text);
};
}  // namespace gcsGui
#endif
