//
// Created by rolf on 07-03-21.
//

#ifndef RTT_ROBOTEAM_AI_SRC_AISETTINGS_H_
#define RTT_ROBOTEAM_AI_SRC_AISETTINGS_H_

#include <string>
namespace rtt {
  class AISettings {
   public:
    explicit AISettings(int id);
    enum CommunicationMode {
      SERIAL,
      GRSIM
    };

    [[nodiscard]] int getId() const;
    void setId(int id);

    [[nodiscard]] bool isYellow() const;
    void setYellow(bool isYellow);

    [[nodiscard]] bool isLeft() const;
    void setLeft(bool left);

    [[nodiscard]] CommunicationMode getCommunicationMode() const;
    void setCommunicationMode(CommunicationMode mode);

    [[nodiscard]] const std::string &getRobothubSendIp() const;
    void setRobothubSendIp(const std::string &ip);

    [[nodiscard]] int getRobothubSendPort() const;
    void setRobothubSendPort(int port);

    [[nodiscard]] bool isPaused() const;
    void setPause(bool paused);
   private:
    int id;
    bool is_yellow;
    bool is_left;
    CommunicationMode mode;
    std::string robothub_send_ip;
    int robothub_send_port;
    bool is_paused;

  };
}
#endif //RTT_ROBOTEAM_AI_SRC_AISETTINGS_H_