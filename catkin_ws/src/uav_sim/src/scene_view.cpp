// Set only the spectator camera, once the actual model visuals have arrived.
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <ignition/math/Pose3.hh>
#include <fstream>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>

namespace gazebo {
class AI4SceneView : public GUIPlugin {
 public:
  AI4SceneView() {
    // GUIPlugin is a QWidget; constrain its otherwise 640x480 empty overlay.
    this->setMaximumSize(1, 1);
    this->resize(1, 1);
    this->hide();
  }
  void Load(sdf::ElementPtr) override {
    gzmsg << "AI4 spectator plugin loaded; status="
          << (std::getenv("AI4_VIEW_STATUS") ? std::getenv("AI4_VIEW_STATUS") : "<unset>") << "\n";
    connection_ = event::Events::ConnectPreRender([this]() { this->Apply(); });
  }
 private:
  void Apply() {
    if (applied_) return;
    auto camera = gui::get_active_camera();
    if (!camera || !camera->Initialized() || !camera->GetScene()) return;
    auto scene = camera->GetScene();
    if (!scene->Initialized() || !scene->GetVisual("static_uav") ||
        !scene->GetVisual("target_uav")) return;
    const ignition::math::Pose3d pose(2, -6, 4.2, 0, .43, 1.5707963267948966);
    if (!requested_) {
      camera->SetViewController("orbit");
      camera->SetUseSDFPose(true);
      camera->SetInitialPose(pose);
      camera->SetWorldPose(pose);
      requested_ = true;
      gzmsg << "AI4 spectator pose requested after camera, scene and both visuals initialized\n";
      return;  // Read back on a later rendering callback.
    }
    const auto actual = camera->WorldPose();
    if (!actual.Pos().Equal(pose.Pos(), 1e-6) ||
        !actual.Rot().Equal(pose.Rot(), 1e-6)) {
      requested_ = false;
      return;
    }
    const char *path = std::getenv("AI4_VIEW_STATUS");
    if (!path) {
      if (!reported_) gzerr << "AI4_VIEW_STATUS missing; spectator readiness cannot be written\n";
      reported_ = true;
      return;
    }
    const std::string temporary = std::string(path) + ".tmp";
    std::ofstream output(temporary);
    output << "spectator view verified after both model visuals arrived\n"
           << "pid=" << getpid() << "\n"
           << "position=" << actual.Pos() << "\n"
           << "rotation=" << actual.Rot() << "\n";
    output.close();
    if (!output || std::rename(temporary.c_str(), path) != 0) {
      if (!reported_) gzerr << "Cannot write spectator readiness to " << path << "; retrying\n";
      reported_ = true;
      return;
    }
    applied_ = true;
    gzmsg << "AI4 spectator view verified and readiness written: " << path << "\n";
  }
  event::ConnectionPtr connection_;
  bool applied_ = false;
  bool requested_ = false;
  bool reported_ = false;
};
GZ_REGISTER_GUI_PLUGIN(AI4SceneView)
}
