#ifndef AS5047_CLIENT_H
#define AS5047_CLIENT_H

#include "communication_interface.h"
#include "client_communication.hpp"

//TODO::Cleanup then include common_message_types and delete the below line
const uint8_t kTypeAs5047 = 56;

class As5047Client: public ClientAbstract{
  public:
    As5047Client(uint8_t obj_idn):
      ClientAbstract(   kTypeAs5047, obj_idn),
      angle_(           kTypeAs5047, obj_idn, kSubAngle),
      angle_source_(    kTypeAs5047, obj_idn, kSubAngleSource),
      settings1_(       kTypeAs5047, obj_idn, kSubSettings1),
      settings2_(       kTypeAs5047, obj_idn, kSubSettings2),
      diagnostic_(      kTypeAs5047, obj_idn, kSubDiagnostic),
      errfl_(           kTypeAs5047, obj_idn, kSubErrfl),
      one_time_program_(kTypeAs5047, obj_idn, kSubOneTimeProgram)
      {};

    // Client Entries
    ClientEntry<float>      angle_;
    ClientEntry<uint8_t>    angle_source_;
    ClientEntry<uint16_t>   settings1_;
    ClientEntry<uint16_t>   settings2_;
    ClientEntry<uint16_t>   diagnostic_;
    ClientEntry<uint16_t>   errfl_;
    ClientEntryVoid         one_time_program_;

    void ReadMsg(CommunicationInterface& com,
      uint8_t* rx_data, uint8_t rx_length)
    {
      static const uint8_t kEntryLength = kSubOneTimeProgram+1;
      ClientEntryAbstract* entry_array[kEntryLength] = {
        &angle_,          //0
        &angle_source_,   //1
        &settings1_,      //2
        &settings2_,      //3
        &diagnostic_,     //4
        &errfl_,          //5
        &one_time_program_//6
      };

      ParseMsg(rx_data, rx_length, entry_array, kEntryLength);
    }

  private:
    static const uint8_t kSubAngle          = 0;
    static const uint8_t kSubAngleSource    = 1;
    static const uint8_t kSubSettings1      = 2;
    static const uint8_t kSubSettings2      = 3;
    static const uint8_t kSubDiagnostic     = 4;
    static const uint8_t kSubErrfl          = 5;
    static const uint8_t kSubOneTimeProgram = 6;
};

#endif // AS5047_CLIENT_H
