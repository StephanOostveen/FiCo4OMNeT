//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef FRPORTINPUT_H_
#define FRPORTINPUT_H_

#include <omnetpp.h>
#include <string.h>
#include "FRFrame_m.h"
#include "FRScheduler.h"
#include "FRSync.h"

using namespace std;

/**
 * @brief Received messages are initially handled in this module.
 *
 * @ingroup Port
 *
 * @author Stefan Buschmann
 */
class FRPortInput: public cSimpleModule{
protected:
    /**
     *
     */
    virtual void initialize();

    /**
     * @brief Handles all received messages
     *
     * @param msg the incoming message.
     */
    virtual void handleMessage(cMessage *msg);

private:
    /**
     * @brief Bandwidth of the network.
     */
    int bandwidth;

    /**
     * @brief Handles the received message.
     */
    virtual void receivedExternMessage(FRFrame *msg);

    /**
     * @brief Calculates when the frame is ready to be forwarded based on the number of bits.
     */
    virtual double calculateScheduleTiming(int length);
};

#endif /* FRPORTINPUT_H_ */
