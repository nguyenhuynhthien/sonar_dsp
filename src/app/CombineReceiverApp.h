#ifndef COMBINE_RECEIVER_APP_H
#define COMBINE_RECEIVER_APP_H

#include <SharedSonarData.h>
#include "../service/ComManager.h"

class CombineReceiverApp {
public:
    CombineReceiverApp(SharedSonarData& sharedData);
    void begin();
    void run();
    void setComManager(ComManager& com) { _com = &com; }

private:
    SharedSonarData& _sharedData;
    ComManager* _com = nullptr;

    void sendSumWaveformFrame(int c);
    void processTargetAndVelocity();
};

#endif // COMBINE_RECEIVER_APP_H
