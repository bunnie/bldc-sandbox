/*
 * step_direction_input_client.hpp
 *
 *  Created on: May 5, 2017
 *      Author: Matthew Piccoli
 */

#ifndef PENN_ECOSYSTEM_HAL_COMMON_CPP_INC_STEP_DIRECTION_INPUT_CLIENT_HPP_
#define PENN_ECOSYSTEM_HAL_COMMON_CPP_INC_STEP_DIRECTION_INPUT_CLIENT_HPP_


#include "communication_interface.h"
#include "client_communication.hpp"

//TODO::Cleanup then include common_message_types and delete the below line
const uint8_t kTypeStepDirInput = 58;

class StepDirectionInputClient: public ClientAbstract{
  public:
    StepDirectionInputClient(uint8_t obj_idn):
      ClientAbstract( kTypeStepDirInput, obj_idn),
      angle_(         kTypeStepDirInput, obj_idn, kSubAngle),
      angle_step_(    kTypeStepDirInput, obj_idn, kSubAngleStep)
      {};

    // Client Entries
    ClientEntry<float>      angle_;
    ClientEntry<float>      angle_step_;

    void ReadMsg(CommunicationInterface& com,
      uint8_t* rx_data, uint8_t rx_length)
    {
      static const uint8_t kEntryLength = kSubAngleStep+1;
      ClientEntryAbstract* entry_array[kEntryLength] = {
        &angle_,      // 0
        &angle_step_  // 1
      };

      ParseMsg(rx_data, rx_length, entry_array, kEntryLength);
    }

  private:
    static const uint8_t kSubAngle               = 0;
    static const uint8_t kSubAngleStep           = 1;
};



#endif /* PENN_ECOSYSTEM_HAL_COMMON_CPP_INC_STEP_DIRECTION_INPUT_CLIENT_HPP_ */
